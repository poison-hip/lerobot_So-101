from __future__ import annotations

import base64
import os
import time
from typing import Protocol

import numpy as np

try:
    from fastapi import FastAPI
    import uvicorn
except ImportError as exc:  # pragma: no cover - runtime dependency check
    raise SystemExit(
        "Missing dependency for lerobot policy server. "
        "Please install `fastapi` and `uvicorn` in env_lerobot."
    ) from exc

from shared.schema import Action, Observation

app = FastAPI(title="LeRobot Policy Server", version="0.1.0")
_START_TIME = time.time()
_REQUEST_COUNT = 0
_BACKEND_NAME = os.getenv("LEROBOT_POLICY_BACKEND", "stub").strip().lower()
_ACTION_SCALE = float(os.getenv("LEROBOT_ACTION_SCALE", "0.1"))
_DEFAULT_ACTION_DIM = int(os.getenv("LEROBOT_DEFAULT_ACTION_DIM", "1"))
_MODEL_PATH = os.getenv("LEROBOT_MODEL_PATH", "")
_MODEL_DEVICE = os.getenv("LEROBOT_MODEL_DEVICE", "cuda").strip().lower()
_STRICT_MODEL_LOAD = os.getenv("LEROBOT_STRICT_MODEL_LOAD", "false").strip().lower() in {"1", "true", "yes"}


class PolicyBackend(Protocol):
    def act(self, obs: Observation) -> list[float]:
        ...


class StubBackend:
    def __init__(self, default_action_dim: int) -> None:
        self.default_action_dim = max(1, default_action_dim)

    def act(self, obs: Observation) -> list[float]:
        state = obs.policy_state()
        action_dim = len(state) if state else self.default_action_dim
        if "action_dim" in obs.metadata:
            try:
                action_dim = max(1, int(obs.metadata["action_dim"]))
            except ValueError:
                pass
        return [0.0] * action_dim


class SimplePDBackend:
    def __init__(self, action_scale: float, default_action_dim: int) -> None:
        self.action_scale = action_scale
        self.default_action_dim = max(1, default_action_dim)

    def act(self, obs: Observation) -> list[float]:
        state = obs.policy_state()
        if not state:
            action_dim = self.default_action_dim
            if "action_dim" in obs.metadata:
                try:
                    action_dim = max(1, int(obs.metadata["action_dim"]))
                except ValueError:
                    pass
            return [0.0] * action_dim
        return [float(-self.action_scale * x) for x in state]


class LerobotPlaceholderBackend:
    def __init__(self, model_path: str, fallback: PolicyBackend, model_device: str, strict_load: bool = False) -> None:
        self.model_path = model_path
        self.fallback = fallback
        self.model_device = model_device
        self.strict_load = strict_load
        self._model_loaded = False
        self._model_error = ""
        self._policy = None
        self._torch_device = None
        self._preprocess = None
        self._postprocess = None
        self._state_key = "observation.state"
        self._image_keys: list[str] = []
        self._last_inference_error = ""
        self._inference_ok_count = 0
        self._fallback_count = 0
        self._last_action_source = "init"
        self._load_model()

    def _load_model(self) -> None:
        if not self.model_path:
            self._model_error = "LEROBOT_MODEL_PATH is empty."
            return
        try:
            import torch
            from lerobot.configs.types import FeatureType
            from lerobot.policies.factory import make_pre_post_processors
            from lerobot.policies.pi0.modeling_pi0 import PI0Policy

            policy = PI0Policy.from_pretrained(self.model_path)
            device = self.model_device
            if device == "cuda" and not torch.cuda.is_available():
                device = "cpu"
            policy.to(device)
            policy.eval()
            preprocess, postprocess = make_pre_post_processors(
                policy.config,
                self.model_path,
                preprocessor_overrides={"device_processor": {"device": str(device)}},
            )

            state_keys = [
                k for k, v in policy.config.input_features.items() if getattr(v, "type", None) == FeatureType.STATE
            ]
            visual_keys = [
                k for k, v in policy.config.input_features.items() if getattr(v, "type", None) == FeatureType.VISUAL
            ]

            self._policy = policy
            self._preprocess = preprocess
            self._postprocess = postprocess
            self._torch_device = torch.device(device)
            if state_keys:
                self._state_key = state_keys[0]
            self._image_keys = visual_keys
            self._model_loaded = True
            self._model_error = ""
        except Exception as exc:
            self._model_loaded = False
            self._model_error = f"{type(exc).__name__}: {exc}"
            if self.strict_load:
                raise

    def act(self, obs: Observation) -> list[float]:
        if not self._model_loaded:
            self._fallback_count += 1
            self._last_action_source = "fallback_model_not_loaded"
            return self.fallback.act(obs)
        try:
            import torch
            from lerobot.policies.utils import prepare_observation_for_inference

            state = obs.policy_state()
            if not state:
                self._fallback_count += 1
                self._last_action_source = "fallback_empty_state"
                return self.fallback.act(obs)

            observation_dict: dict[str, np.ndarray] = {
                self._state_key: np.asarray(state, dtype=np.float32)
            }

            if self._image_keys:
                image_map = self._decode_image_map_from_obs(obs)
                if image_map:
                    for image_key in self._image_keys:
                        short_key = image_key.split(".")[-1]
                        if image_key in image_map:
                            observation_dict[image_key] = image_map[image_key]
                        elif short_key in image_map:
                            observation_dict[image_key] = image_map[short_key]
                elif obs.image_b64:
                    image = self._decode_image_from_obs(obs)
                    if image is not None:
                        # If only one camera is available, replicate it to all expected visual keys.
                        for image_key in self._image_keys:
                            observation_dict[image_key] = image

            prepared = prepare_observation_for_inference(
                observation=observation_dict,
                device=self._torch_device,
                task=obs.instruction or "",
                robot_type=obs.metadata.get("robot_type", ""),
            )
            processed_obs = self._preprocess(prepared) if self._preprocess is not None else prepared
            with torch.inference_mode():
                action = self._policy.select_action(processed_obs)
            action = self._postprocess(action) if self._postprocess is not None else action
            action_np = action.squeeze(0).detach().cpu().numpy().reshape(-1)
            self._last_inference_error = ""
            self._inference_ok_count += 1
            self._last_action_source = "pi0_select_action"
            return [float(v) for v in action_np]
        except Exception as exc:
            self._last_inference_error = f"{type(exc).__name__}: {exc}"
            self._fallback_count += 1
            self._last_action_source = "fallback_inference_exception"
            return self.fallback.act(obs)

    def _decode_image_from_obs(self, obs: Observation) -> np.ndarray | None:
        if not obs.image_b64:
            return None
        try:
            raw = base64.b64decode(obs.image_b64)
            h = int(obs.metadata.get("image_height", "0"))
            w = int(obs.metadata.get("image_width", "0"))
            encoding = obs.metadata.get("image_encoding", "rgb8")
            if h > 0 and w > 0 and encoding in {"rgb8", "bgr8"}:
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
                if encoding == "bgr8":
                    arr = arr[..., ::-1]
                return arr
            return None
        except Exception:
            return None

    def _decode_image_map_from_obs(self, obs: Observation) -> dict[str, np.ndarray]:
        image_map: dict[str, np.ndarray] = {}
        if not obs.image_b64_map:
            return image_map
        try:
            h = int(obs.metadata.get("image_height", "0"))
            w = int(obs.metadata.get("image_width", "0"))
            encoding = obs.metadata.get("image_encoding", "rgb8")
            for k, v in obs.image_b64_map.items():
                raw = base64.b64decode(v)  
                arr_u8 = np.frombuffer(raw, dtype=np.uint8)
                if h > 0 and w > 0 and encoding in {"rgb8", "bgr8"} and arr_u8.size == h * w * 3:
                    arr = arr_u8.reshape(h, w, 3)
                    if encoding == "bgr8":
                        arr = arr[..., ::-1]
                else:
                    # fallback: try infer HxW from a few common widths (no crash if metadata missing)
                    arr = None
                    for guess_w in (1280, 960, 848, 640, 424, 320):
                        if arr_u8.size % (guess_w * 3) == 0:
                            guess_h = arr_u8.size // (guess_w * 3)
                            arr = arr_u8.reshape(guess_h, guess_w, 3)
                            break
                    if arr is None:
                        continue
                image_map[k] = arr
        except Exception:
            return {}
        return image_map


def _build_backend() -> PolicyBackend:
    stub = StubBackend(default_action_dim=_DEFAULT_ACTION_DIM)
    simple_pd = SimplePDBackend(action_scale=_ACTION_SCALE, default_action_dim=_DEFAULT_ACTION_DIM)
    if _BACKEND_NAME == "stub":
        return stub
    if _BACKEND_NAME == "simple_pd":
        return simple_pd
    if _BACKEND_NAME == "lerobot":
        return LerobotPlaceholderBackend(
            model_path=_MODEL_PATH,
            fallback=simple_pd,
            model_device=_MODEL_DEVICE,
            strict_load=_STRICT_MODEL_LOAD,
        )
    return stub


_POLICY_BACKEND = _build_backend()


@app.get("/health")
def health() -> dict[str, str | int | float]:
    return {
        "status": "ok",
        "service": "lerobot_policy_server",
        "uptime_s": round(time.time() - _START_TIME, 3),
        "request_count": _REQUEST_COUNT,
        "backend": _BACKEND_NAME,
        "action_scale": _ACTION_SCALE,
        "model_path": _MODEL_PATH if _MODEL_PATH else "unset",
        "model_device": _MODEL_DEVICE,
        "strict_model_load": _STRICT_MODEL_LOAD,
        "model_loaded": bool(getattr(_POLICY_BACKEND, "_model_loaded", False)),
        "model_error": getattr(_POLICY_BACKEND, "_model_error", ""),
        "last_inference_error": getattr(_POLICY_BACKEND, "_last_inference_error", ""),
        "last_action_source": getattr(_POLICY_BACKEND, "_last_action_source", "unknown"),
        "inference_ok_count": getattr(_POLICY_BACKEND, "_inference_ok_count", 0),
        "fallback_count": getattr(_POLICY_BACKEND, "_fallback_count", 0),
    }


@app.post("/act", response_model=Action)
def act(obs: Observation) -> Action:
    global _REQUEST_COUNT
    _REQUEST_COUNT += 1
    action = _POLICY_BACKEND.act(obs)
    return Action(action=action)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
