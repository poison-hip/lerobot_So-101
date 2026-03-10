from __future__ import annotations

import time
from typing import Any

import requests

from shared.schema import Observation, model_dump_compat


class PolicyHTTPClient:
    def __init__(
        self,
        server_url: str,
        *,
        timeout_s: float = 5.0,
        max_retries: int = 2,
        backoff_s: float = 0.2,
    ) -> None:
        self.server_url = server_url.rstrip("/")
        self.timeout_s = timeout_s
        self.max_retries = max_retries
        self.backoff_s = backoff_s
        self.session = requests.Session()

    def health(self) -> dict[str, Any]:
        response = self.session.get(f"{self.server_url}/health", timeout=self.timeout_s)
        response.raise_for_status()
        return response.json()

    def act(self, observation: Observation) -> list[float]:
        payload = model_dump_compat(observation)
        last_error: Exception | None = None
        for attempt in range(self.max_retries + 1):
            try:
                response = self.session.post(
                    f"{self.server_url}/act",
                    json=payload,
                    timeout=self.timeout_s,
                )
                response.raise_for_status()
                body = response.json()
                action = body.get("action", [])
                return [float(v) for v in action]
            except (requests.RequestException, ValueError, TypeError) as exc:
                last_error = exc
                if attempt >= self.max_retries:
                    break
                time.sleep(self.backoff_s * (attempt + 1))
        assert last_error is not None
        raise RuntimeError(f"failed to query policy server at {self.server_url}: {last_error}") from last_error
