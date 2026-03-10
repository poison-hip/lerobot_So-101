from __future__ import annotations

import argparse
import base64
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

from shared.http_client import PolicyHTTPClient
from shared.schema import Observation


class Ros2ObsActionBridge(Node):
    def __init__(
        self,
        *,
        server_url: str,
        joint_topic: str,
        image_topic: str,
        image_topic_left: str,
        image_topic_right: str,
        action_topic: str,
        include_image: bool,
        include_multi_image: bool,
        instruction: str | None,
        rate_hz: float,
        timeout_s: float,
        max_retries: int,
    ) -> None:
        super().__init__("lerobot_obs_action_bridge")
        self.include_image = include_image
        self.include_multi_image = include_multi_image
        self.instruction = instruction
        self.client = PolicyHTTPClient(
            server_url=server_url,
            timeout_s=timeout_s,
            max_retries=max_retries,
            backoff_s=0.2,
        )

        self.latest_joint_state: list[float] | None = None
        self.latest_joint_stamp: tuple[int, int] | None = None
        self.latest_image_b64: str | None = None
        self.latest_image_meta: dict[str, str] = {}
        self.latest_image_b64_map: dict[str, str] = {}
        self._last_image_keys_log_count = 0
        self.req_count = 0

        self.joint_sub = self.create_subscription(JointState, joint_topic, self._on_joint, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self._on_image_base, 2) if include_image else None
        self.image_sub_left = (
            self.create_subscription(Image, image_topic_left, self._on_image_left, 2) if include_multi_image else None
        )
        self.image_sub_right = (
            self.create_subscription(Image, image_topic_right, self._on_image_right, 2)
            if include_multi_image
            else None
        )
        self.action_pub = self.create_publisher(Float64MultiArray, action_topic, 10)
        self.timer = self.create_timer(max(1e-3, 1.0 / rate_hz), self._tick)

    def _on_joint(self, msg: JointState) -> None:
        self.latest_joint_state = [float(v) for v in msg.position]
        self.latest_joint_stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))

    def _on_image_base(self, msg: Image) -> None:
        self.latest_image_b64 = base64.b64encode(bytes(msg.data)).decode("ascii")
        self.latest_image_b64_map["base_0_rgb"] = self.latest_image_b64
        self.latest_image_meta = {
            "image_encoding": str(msg.encoding),
            "image_height": str(msg.height),
            "image_width": str(msg.width),
            "image_step": str(msg.step),
        }

    def _on_image_left(self, msg: Image) -> None:
        self.latest_image_b64_map["left_wrist_0_rgb"] = base64.b64encode(bytes(msg.data)).decode("ascii")
        if not self.latest_image_meta:
            self.latest_image_meta = {
                "image_encoding": str(msg.encoding),
                "image_height": str(msg.height),
                "image_width": str(msg.width),
                "image_step": str(msg.step),
            }

    def _on_image_right(self, msg: Image) -> None:
        self.latest_image_b64_map["right_wrist_0_rgb"] = base64.b64encode(bytes(msg.data)).decode("ascii")
        if not self.latest_image_meta:
            self.latest_image_meta = {
                "image_encoding": str(msg.encoding),
                "image_height": str(msg.height),
                "image_width": str(msg.width),
                "image_step": str(msg.step),
            }

    def _tick(self) -> None:
        if not self.latest_joint_state:
            return

        metadata = {
            "source": "ros2_obs_action_bridge",
            "obs_ts_ms": str(int(time.time() * 1000)),
            "action_dim": str(len(self.latest_joint_state)),
        }
        if self.latest_joint_stamp is not None:
            metadata["joint_stamp_sec"] = str(self.latest_joint_stamp[0])
            metadata["joint_stamp_nanosec"] = str(self.latest_joint_stamp[1])
        metadata.update(self.latest_image_meta)

        obs = Observation(
            joint_state=self.latest_joint_state,
            image_b64=self.latest_image_b64 if self.include_image else None,
            image_b64_map=self.latest_image_b64_map if self.include_multi_image else None,
            instruction=self.instruction,
            metadata=metadata,
        )
        try:
            action = self.client.act(obs)
            self.req_count += 1
            msg = Float64MultiArray(data=[float(v) for v in action])
            self.action_pub.publish(msg)
            if self.req_count % 10 == 0:
                image_keys = sorted(self.latest_image_b64_map.keys()) if self.include_multi_image else []
                self.get_logger().info(
                    f"req={self.req_count} joint_dim={len(self.latest_joint_state)} action_dim={len(action)} "
                    f"image_keys={image_keys}"
                )
        except Exception as exc:
            self.get_logger().error(f"policy bridge request failed: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 observation -> LeRobot /act -> action topic bridge")
    parser.add_argument("--server-url", default="http://127.0.0.1:8000")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--image-topic", default="/sim/camera/image_raw")
    parser.add_argument("--image-topic-left", default="/sim/camera/image_raw_left")
    parser.add_argument("--image-topic-right", default="/sim/camera/image_raw_right")
    parser.add_argument("--action-topic", default="/policy_action")
    parser.add_argument("--include-image", action="store_true")
    parser.add_argument("--include-multi-image", action="store_true")
    parser.add_argument("--instruction", default=None)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--timeout-s", type=float, default=2.0)
    parser.add_argument("--max-retries", type=int, default=1)
    args = parser.parse_args()

    rclpy.init()
    node = Ros2ObsActionBridge(
        server_url=args.server_url,
        joint_topic=args.joint_topic,
        image_topic=args.image_topic,
        image_topic_left=args.image_topic_left,
        image_topic_right=args.image_topic_right,
        action_topic=args.action_topic,
        include_image=args.include_image,
        include_multi_image=args.include_multi_image,
        instruction=args.instruction,
        rate_hz=args.rate_hz,
        timeout_s=args.timeout_s,
        max_retries=args.max_retries,
    )
    try:
        try:
            health = node.client.health()
            node.get_logger().info(f"policy server health: {health}")
        except Exception as exc:
            node.get_logger().warn(f"health check failed: {exc}")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
