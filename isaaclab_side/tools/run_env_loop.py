from __future__ import annotations

import argparse
import base64
import importlib
import os
import sys
import traceback
from typing import Any

import numpy as np

from shared.http_client import PolicyHTTPClient
from shared.schema import Observation


def _to_numpy_array(value: Any) -> np.ndarray:
    # Handle torch tensor from Isaac Lab (often on cuda:0).
    if hasattr(value, "detach") and hasattr(value, "cpu") and hasattr(value, "numpy"):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def _flatten_float_list(value: Any) -> list[float]:
    arr = _to_numpy_array(value).reshape(-1)
    return [float(v) for v in arr]


def encode_image_to_b64(image_array: np.ndarray) -> str:
    contiguous = np.ascontiguousarray(image_array)
    return base64.b64encode(contiguous.tobytes()).decode("ascii")


def extract_joint_state(observation: Any) -> list[float]:
    if isinstance(observation, dict):
        if "joint_state" in observation:
            return _flatten_float_list(observation["joint_state"])
        if "state" in observation:
            return _flatten_float_list(observation["state"])
        if "observation" in observation:
            return _flatten_float_list(observation["observation"])
        # For manager-based envs, policy obs is often nested under this key.
        if "policy" in observation:
            return _flatten_float_list(observation["policy"])
        return _flatten_float_list(list(observation.values())[0])
    return _flatten_float_list(observation)


def extract_image(observation: Any, image_key: str) -> np.ndarray | None:
    if not isinstance(observation, dict):
        return None
    if image_key not in observation:
        return None
    image = _to_numpy_array(observation[image_key])
    return image


def convert_action_for_env(action: list[float], action_space: Any, env: Any = None) -> Any:
    space_name = action_space.__class__.__name__
    if space_name == "Discrete":
        return int(round(action[0])) if action else 0
    if space_name == "MultiDiscrete":
        arr = np.asarray(action, dtype=np.int64)
        return arr[: len(action_space.nvec)]
    if space_name == "Box":
        shape_size = int(np.prod(action_space.shape))
        arr = np.asarray(action, dtype=np.float32).reshape(-1)
        if arr.size < shape_size:
            padded = np.zeros(shape_size, dtype=np.float32)
            padded[: arr.size] = arr
            arr = padded
        else:
            arr = arr[:shape_size]
        arr = arr.reshape(action_space.shape)
        arr = np.clip(arr, action_space.low, action_space.high)
        try:
            import torch

            device = getattr(env, "device", None)
            num_envs = int(getattr(env, "num_envs", 1))
            batched = np.broadcast_to(arr, (num_envs,) + tuple(action_space.shape)).copy()
            return torch.tensor(batched, dtype=torch.float32, device=device)
        except Exception:
            return arr
    return np.asarray(action, dtype=np.float32)


def _try_register_isaac_envs(target_env_id: str) -> None:
    if not target_env_id.startswith("Isaac-"):
        return

    import gymnasium as gym

    if target_env_id in gym.envs.registry:
        return

    search_roots = []
    isaaclab_root = os.environ.get("ISAACLAB_ROOT")
    if isaaclab_root:
        search_roots.extend(
            [
                os.path.join(isaaclab_root, "source"),
                os.path.join(isaaclab_root, "source", "isaaclab"),
                os.path.join(isaaclab_root, "source", "isaaclab_tasks"),
                os.path.join(isaaclab_root, "source", "isaaclab_assets"),
            ]
        )

    # Common default used in your setup.
    default_root = "/root/gpufree-data/tmp/projects/IsaacLab"
    search_roots.extend(
        [
            os.path.join(default_root, "source"),
            os.path.join(default_root, "source", "isaaclab"),
            os.path.join(default_root, "source", "isaaclab_tasks"),
            os.path.join(default_root, "source", "isaaclab_assets"),
        ]
    )

    for path in search_roots:
        if os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)

    try:
        importlib.import_module("isaaclab_tasks")
    except Exception as exc:
        raise RuntimeError(
            "Failed to import isaaclab_tasks for Isaac env registration. "
            "Run this script via isaaclab.sh and ensure IsaacLab source paths are available."
        ) from exc

    # Some layouts need importing the nested package to trigger side-effects.
    if target_env_id not in gym.envs.registry:
        try:
            importlib.import_module("isaaclab_tasks.isaaclab_tasks")
        except ModuleNotFoundError:
            pass
        except Exception as exc:
            raise RuntimeError("Failed while importing nested isaaclab_tasks package.") from exc

    if target_env_id not in gym.envs.registry:
        raise RuntimeError(
            f"Environment `{target_env_id}` is still not registered after importing isaaclab_tasks. "
            "Please run `./isaaclab.sh -p scripts/environments/list_envs.py` to verify registration."
        )


def make_env(env_id: str, args: argparse.Namespace) -> Any:
    import gymnasium as gym

    if env_id.startswith("Isaac-"):
        from isaaclab_tasks.utils import parse_env_cfg

        env_cfg = parse_env_cfg(
            env_id,
            device=getattr(args, "device", None),
            num_envs=args.num_envs,
            use_fabric=not args.disable_fabric,
        )
        return gym.make(env_id, cfg=env_cfg)

    return gym.make(env_id)


def main() -> None:
    parser = argparse.ArgumentParser(description="Isaac Lab loop -> LeRobot policy server")
    try:
        from isaaclab.app import AppLauncher
    except Exception as exc:
        raise RuntimeError(
            "Failed to import isaaclab.app.AppLauncher. "
            "Please run this script via Isaac Lab runtime: `./isaaclab.sh -p ...`"
        ) from exc

    AppLauncher.add_app_launcher_args(parser)
    parser.add_argument("--env-id", required=True, help="Gym/Isaac Lab environment id")
    parser.add_argument("--server-url", default="http://127.0.0.1:8000")
    parser.add_argument("--instruction", default=None)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--max-retries", type=int, default=1)
    parser.add_argument("--send-image", action="store_true")
    parser.add_argument("--image-key", default="image")
    parser.add_argument("--log-every", type=int, default=20)
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--disable-fabric", action="store_true")
    args = parser.parse_args()

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    env = None
    try:
        import gymnasium as gym

        print(f"[bridge] registering env id: {args.env_id}")
        _try_register_isaac_envs(args.env_id)

        print(f"[bridge] creating env: {args.env_id}")
        env = make_env(args.env_id, args)
        policy_client = PolicyHTTPClient(
            server_url=args.server_url,
            timeout_s=args.timeout_s,
            max_retries=args.max_retries,
            backoff_s=0.2,
        )
        try:
            print(f"Policy health: {policy_client.health()}")
        except Exception as exc:
            print(f"Policy health check failed: {exc}")

        for episode in range(args.episodes):
            print(f"[bridge] reset env for episode={episode}")
            observation, _ = env.reset()
            print(f"Episode {episode} started")
            for step in range(args.max_steps):
                joint_state = extract_joint_state(observation)
                image_b64 = None
                if args.send_image:
                    image_array = extract_image(observation, args.image_key)
                    if image_array is not None:
                        image_b64 = encode_image_to_b64(image_array)
                request_obs = Observation(
                    joint_state=joint_state,
                    image_b64=image_b64,
                    instruction=args.instruction,
                    metadata={"source": "isaaclab_env_loop", "episode": str(episode), "step": str(step)},
                )
                action = policy_client.act(request_obs)
                env_action = convert_action_for_env(action, env.action_space, env)
                step_result = env.step(env_action)
                if len(step_result) == 5:
                    observation, _, terminated, truncated, _ = step_result
                else:
                    observation, _, done, _ = step_result
                    terminated, truncated = bool(done), False

                if step % max(1, args.log_every) == 0:
                    print(
                        f"episode={episode} step={step} obs_dim={len(joint_state)} action_dim={len(action)} "
                        f"terminated={terminated} truncated={truncated}"
                    )

                if terminated or truncated:
                    print(
                        f"Episode {episode} ended at step {step} "
                        f"(terminated={terminated}, truncated={truncated})"
                    )
                    break
    except Exception:
        print("[bridge] run_env_loop failed with exception:")
        traceback.print_exc()
        raise
    finally:
        if env is not None:
            try:
                env.close()
            except Exception as exc:
                print(f"[bridge] env.close() raised: {exc}")
        simulation_app.close()


if __name__ == "__main__":
    main()
