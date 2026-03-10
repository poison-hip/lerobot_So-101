from __future__ import annotations

import argparse

from shared.http_client import PolicyHTTPClient
from shared.schema import Observation


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe LeRobot policy server from Isaac Lab side")
    parser.add_argument("--server-url", default="http://127.0.0.1:8000")
    parser.add_argument("--state-dim", type=int, default=7)
    parser.add_argument("--instruction", default="probe")
    args = parser.parse_args()

    client = PolicyHTTPClient(server_url=args.server_url, timeout_s=3.0, max_retries=1, backoff_s=0.2)
    health = client.health()
    print("health:", health)

    obs = Observation(
        joint_state=[0.0] * args.state_dim,
        instruction=args.instruction,
        metadata={"source": "policy_probe", "action_dim": str(args.state_dim)},
    )
    action = client.act(obs)
    print("probe action:", action)
    print("probe action dim:", len(action))


if __name__ == "__main__":
    main()
