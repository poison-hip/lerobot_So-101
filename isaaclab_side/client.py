from __future__ import annotations

from shared.http_client import PolicyHTTPClient
from shared.schema import Observation

SERVER_URL = "http://127.0.0.1:8000"


def query_policy(
    joint_state: list[float],
    instruction: str | None = None,
    *,
    server_url: str = SERVER_URL,
) -> list[float]:
    client = PolicyHTTPClient(server_url=server_url, timeout_s=5.0, max_retries=2, backoff_s=0.2)
    obs = Observation(
        joint_state=joint_state,
        instruction=instruction,
        metadata={"source": "isaaclab_test"},
    )
    return client.act(obs)


if __name__ == "__main__":
    sample_joint_state = [0.1, 0.2, 0.3, 0.4]
    action = query_policy(sample_joint_state, instruction="reach target")
    print("joint_state =", sample_joint_state)
    print("action =", action)
