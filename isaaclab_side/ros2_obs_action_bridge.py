from __future__ import annotations

import argparse
import base64
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        image_stale_ms: int,
        multi_image_sync_ms: int,
        require_all_multi_images: bool,
        reuse_missing_image_ms: int,
        safe_zero_on_missing_images: bool,
        safe_zero_after_ms: int,
    ) -> None:
        super().__init__("lerobot_obs_action_bridge")
        self.include_image = include_image
        self.include_multi_image = include_multi_image
        self.instruction = instruction
        self.image_stale_ms = max(1, int(image_stale_ms))
        self.multi_image_sync_ms = max(1, int(multi_image_sync_ms))
        self.require_all_multi_images = require_all_multi_images
        self.reuse_missing_image_ms = max(0, int(reuse_missing_image_ms))
        self.safe_zero_on_missing_images = safe_zero_on_missing_images
        self.safe_zero_after_ms = max(1, int(safe_zero_after_ms))
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
        self.latest_image_stamp_ms: dict[str, int] = {}
        self.latest_image_recv_ms: dict[str, int] = {}
        self.last_complete_image_b64_map: dict[str, str] = {}
        self.last_complete_recv_ms: int = 0
        self.missing_multi_image_since_ms: int | None = None
        self.safe_zero_publish_count = 0
        self.req_count = 0
        self.skip_count = 0

        self.joint_sub = self.create_subscription(JointState, joint_topic, self._on_joint, qos_profile_sensor_data)
        # Base camera is used by both single-image and multi-image paths.
        self.image_sub = (
            self.create_subscription(Image, image_topic, self._on_image_base, qos_profile_sensor_data)
            if (include_image or include_multi_image)
            else None
        )
        self.image_sub_left = (
            self.create_subscription(Image, image_topic_left, self._on_image_left, qos_profile_sensor_data)
            if include_multi_image
            else None
        )
        self.image_sub_right = (
            self.create_subscription(Image, image_topic_right, self._on_image_right, qos_profile_sensor_data)
            if include_multi_image
            else None
        )
        self.action_pub = self.create_publisher(Float64MultiArray, action_topic, 10)
        self.timer = self.create_timer(max(1e-3, 1.0 / rate_hz), self._tick)

    def _on_joint(self, msg: JointState) -> None:
        self.latest_joint_state = [float(v) for v in msg.position]
        self.latest_joint_stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))

    def _on_image_base(self, msg: Image) -> None:
        recv_ms = int(time.time() * 1000)
        self.latest_image_b64 = base64.b64encode(bytes(msg.data)).decode("ascii")
        self.latest_image_b64_map["base_0_rgb"] = self.latest_image_b64
        self.latest_image_stamp_ms["base_0_rgb"] = self._stamp_to_ms(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.latest_image_recv_ms["base_0_rgb"] = recv_ms
        self.latest_image_meta = {
            "image_encoding": str(msg.encoding),
            "image_height": str(msg.height),
            "image_width": str(msg.width),
            "image_step": str(msg.step),
        }

    def _on_image_left(self, msg: Image) -> None:
        recv_ms = int(time.time() * 1000)
        self.latest_image_b64_map["left_wrist_0_rgb"] = base64.b64encode(bytes(msg.data)).decode("ascii")
        self.latest_image_stamp_ms["left_wrist_0_rgb"] = self._stamp_to_ms(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.latest_image_recv_ms["left_wrist_0_rgb"] = recv_ms
        if not self.latest_image_meta:
            self.latest_image_meta = {
                "image_encoding": str(msg.encoding),
                "image_height": str(msg.height),
                "image_width": str(msg.width),
                "image_step": str(msg.step),
            }

    def _on_image_right(self, msg: Image) -> None:
        recv_ms = int(time.time() * 1000)
        self.latest_image_b64_map["right_wrist_0_rgb"] = base64.b64encode(bytes(msg.data)).decode("ascii")
        self.latest_image_stamp_ms["right_wrist_0_rgb"] = self._stamp_to_ms(msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.latest_image_recv_ms["right_wrist_0_rgb"] = recv_ms
        if not self.latest_image_meta:
            self.latest_image_meta = {
                "image_encoding": str(msg.encoding),
                "image_height": str(msg.height),
                "image_width": str(msg.width),
                "image_step": str(msg.step),
            }

    @staticmethod
    def _stamp_to_ms(sec: int, nanosec: int) -> int:
        return int(sec) * 1000 + int(nanosec) // 1_000_000

    def _prune_stale_images(self, now_ms: int) -> None:
        stale_keys = [
            k for k, recv_ms in self.latest_image_recv_ms.items() if now_ms - recv_ms > self.image_stale_ms
        ]
        for key in stale_keys:
            self.latest_image_stamp_ms.pop(key, None)
            self.latest_image_recv_ms.pop(key, None)
            self.latest_image_b64_map.pop(key, None)
            if key == "base_0_rgb":
                self.latest_image_b64 = None

    def _is_multi_image_synced(self) -> tuple[bool, str]:
        required_keys = ["base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"]
        available = [k for k in required_keys if k in self.latest_image_stamp_ms]

        if self.require_all_multi_images and len(available) != len(required_keys):
            return False, "missing_multi_image"
        if len(available) < 2:
            return False, "insufficient_multi_image"

        stamps = [self.latest_image_stamp_ms[k] for k in available]
        skew_ms = max(stamps) - min(stamps)
        if skew_ms > self.multi_image_sync_ms:
            return False, f"multi_image_skew_ms={skew_ms}"
        return True, f"multi_image_skew_ms={skew_ms}"

    def _resolve_image_map(self, now_ms: int) -> tuple[dict[str, str] | None, str]:
        required_keys = ["base_0_rgb", "left_wrist_0_rgb", "right_wrist_0_rgb"]
        available = [k for k in required_keys if k in self.latest_image_b64_map]
        has_all = len(available) == len(required_keys)

        if has_all:
            synced, reason = self._is_multi_image_synced()
            if synced:
                resolved = {k: self.latest_image_b64_map[k] for k in required_keys}
                self.last_complete_image_b64_map = dict(resolved)
                self.last_complete_recv_ms = now_ms
                return resolved, "live_all"
            return None, reason

        if self.require_all_multi_images:
            if (
                self.reuse_missing_image_ms > 0
                and self.last_complete_image_b64_map
                and now_ms - self.last_complete_recv_ms <= self.reuse_missing_image_ms
            ):
                return dict(self.last_complete_image_b64_map), "reuse_last_complete"
            return None, "missing_multi_image"

        # Non-strict mode: use whatever cameras are currently available.
        return dict(self.latest_image_b64_map), "live_partial"

    def _tick(self) -> None:
        if not self.latest_joint_state:
            return
        now_ms = int(time.time() * 1000)
        self._prune_stale_images(now_ms)
        resolved_image_map: dict[str, str] | None = None
        image_mode = "none"

        if self.include_multi_image:
            resolved_image_map, image_mode = self._resolve_image_map(now_ms)
            if resolved_image_map is None:
                if self.missing_multi_image_since_ms is None:
                    self.missing_multi_image_since_ms = now_ms
                self.skip_count += 1
                missing_elapsed_ms = now_ms - self.missing_multi_image_since_ms
                if (
                    self.safe_zero_on_missing_images
                    and self.latest_joint_state
                    and missing_elapsed_ms >= self.safe_zero_after_ms
                ):
                    zero_msg = Float64MultiArray(data=[0.0] * len(self.latest_joint_state))
                    self.action_pub.publish(zero_msg)
                    self.safe_zero_publish_count += 1
                if self.skip_count % 20 == 0:
                    self.get_logger().warn(
                        f"skip request: {image_mode} keys={sorted(self.latest_image_b64_map.keys())} "
                        f"safe_zero_published={self.safe_zero_publish_count}"
                    )
                return
            self.missing_multi_image_since_ms = None

        metadata = {
            "source": "ros2_obs_action_bridge",
            "obs_ts_ms": str(now_ms),
            "action_dim": str(len(self.latest_joint_state)),
            "image_stale_ms": str(self.image_stale_ms),
            "multi_image_sync_ms": str(self.multi_image_sync_ms),
            "image_mode": image_mode,
        }
        if self.latest_joint_stamp is not None:
            metadata["joint_stamp_sec"] = str(self.latest_joint_stamp[0])
            metadata["joint_stamp_nanosec"] = str(self.latest_joint_stamp[1])
        metadata.update(self.latest_image_meta)

        obs = Observation(
            joint_state=self.latest_joint_state,
            image_b64=self.latest_image_b64 if self.include_image else None,
            image_b64_map=resolved_image_map if self.include_multi_image else None,
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
    parser.add_argument(
        "--image-stale-ms",
        type=int,
        default=250,
        help="Drop camera frames older than this threshold.",
    )
    parser.add_argument(
        "--multi-image-sync-ms",
        type=int,
        default=120,
        help="Require multi-camera timestamp skew <= this threshold.",
    )
    parser.add_argument(
        "--require-all-multi-images",
        action="store_true",
        help="When multi-image mode is enabled, require base/left/right all present.",
    )
    parser.add_argument(
        "--reuse-missing-image-ms",
        type=int,
        default=0,
        help="If strict multi-image is enabled, temporarily reuse last complete 3-camera set for this duration.",
    )
    parser.add_argument(
        "--safe-zero-on-missing-images",
        action="store_true",
        help="Publish zero action when multi-image missing persists, to reduce run-away drift.",
    )
    parser.add_argument(
        "--safe-zero-after-ms",
        type=int,
        default=500,
        help="Delay before publishing zero action in missing-image periods.",
    )
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
        image_stale_ms=args.image_stale_ms,
        multi_image_sync_ms=args.multi_image_sync_ms,
        require_all_multi_images=args.require_all_multi_images,
        reuse_missing_image_ms=args.reuse_missing_image_ms,
        safe_zero_on_missing_images=args.safe_zero_on_missing_images,
        safe_zero_after_ms=args.safe_zero_after_ms,
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
