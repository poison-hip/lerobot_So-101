from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from shared.http_client import PolicyHTTPClient
from shared.schema import Observation


def safe_shutdown() -> None:
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


class JointStatePolicyClient(Node):
    def __init__(
        self,
        server_url: str,
        topic: str,
        *,
        instruction: str | None,
        timeout_s: float,
        max_retries: int,
        queue_size: int,
        log_every: int,
    ) -> None:
        super().__init__("lerobot_joint_state_client")
        self.topic = topic
        self.instruction = instruction
        self.log_every = max(1, log_every)
        self._message_count = 0
        self.client = PolicyHTTPClient(
            server_url=server_url,
            timeout_s=timeout_s,
            max_retries=max_retries,
            backoff_s=0.2,
        )
        self.subscription = self.create_subscription(JointState, topic, self._on_joint_state, queue_size)

    def _on_joint_state(self, msg: JointState) -> None:
        self._message_count += 1
        joint_state = [float(value) for value in msg.position]
        obs = Observation(
            joint_state=joint_state,
            instruction=self.instruction,
            metadata={
                "source": "ros2_joint_states",
                "topic": self.topic,
                "stamp_sec": str(msg.header.stamp.sec),
                "stamp_nanosec": str(msg.header.stamp.nanosec),
            },
        )
        try:
            action = self.client.act(obs)
            if self._message_count % self.log_every == 0:
                self.get_logger().info(
                    f"msg={self._message_count} joint_state_len={len(joint_state)} action_len={len(action)} action={action}"
                )
        except RuntimeError as exc:
            self.get_logger().error(f"policy request failed: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 /joint_states -> LeRobot /act bridge")
    parser.add_argument("--server-url", default="http://127.0.0.1:8000")
    parser.add_argument("--topic", default="/joint_states")
    parser.add_argument("--instruction", default=None)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument("--max-retries", type=int, default=1)
    parser.add_argument("--queue-size", type=int, default=10)
    parser.add_argument("--log-every", type=int, default=10, help="Print every N joint messages")
    args = parser.parse_args()

    rclpy.init()
    node = JointStatePolicyClient(
        server_url=args.server_url,
        topic=args.topic,
        instruction=args.instruction,
        timeout_s=args.timeout_s,
        max_retries=args.max_retries,
        queue_size=args.queue_size,
        log_every=args.log_every,
    )
    try:
        health = node.client.health()
        node.get_logger().info(f"policy server health: {health}")
    except Exception as exc:
        node.get_logger().warn(f"unable to fetch policy health: {exc}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        safe_shutdown()


if __name__ == "__main__":
    main()
