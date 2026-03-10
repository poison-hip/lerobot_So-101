from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover - optional dependency in ROS setups
    CvBridge = None


_ENCODING_TO_DTYPE = {
    "mono8": np.uint8,
    "8UC1": np.uint8,
    "rgb8": np.uint8,
    "bgr8": np.uint8,
    "rgba8": np.uint8,
    "bgra8": np.uint8,
    "mono16": np.uint16,
    "16UC1": np.uint16,
}

_ENCODING_TO_CHANNELS = {
    "mono8": 1,
    "8UC1": 1,
    "mono16": 1,
    "16UC1": 1,
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
}


def safe_shutdown() -> None:
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


class Ros2ImageClient(Node):
    def __init__(self, topic: str, log_every: int) -> None:
        super().__init__("lerobot_image_client")
        self.bridge = CvBridge() if CvBridge is not None else None
        self.log_every = max(1, log_every)
        self._count = 0
        self._last_time = time.monotonic()
        self.subscription = self.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        self._count += 1
        image = self._to_numpy(msg)
        if self._count % self.log_every == 0:
            now = time.monotonic()
            elapsed = max(1e-6, now - self._last_time)
            fps = self.log_every / elapsed
            self._last_time = now
            self.get_logger().info(
                f"count={self._count} shape={image.shape} dtype={image.dtype} encoding={msg.encoding} approx_fps={fps:.2f}"
            )

    def _to_numpy(self, msg: Image) -> np.ndarray:
        if self.bridge is not None:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        dtype = _ENCODING_TO_DTYPE.get(msg.encoding, np.uint8)
        channels = _ENCODING_TO_CHANNELS.get(msg.encoding, 1)
        image = np.frombuffer(msg.data, dtype=dtype)
        itemsize = np.dtype(dtype).itemsize
        row_stride = int(msg.step // itemsize) if msg.step else msg.width * channels
        image = image.reshape((msg.height, row_stride))
        valid_cols = msg.width * channels
        image = image[:, :valid_cols]
        if channels == 1:
            return image.reshape((msg.height, msg.width))
        return image.reshape((msg.height, msg.width, channels))


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 image subscriber for Isaac Lab camera")
    parser.add_argument("--topic", default="/sim/camera/image_raw")
    parser.add_argument("--log-every", type=int, default=10)
    args = parser.parse_args()

    rclpy.init()
    node = Ros2ImageClient(topic=args.topic, log_every=args.log_every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        safe_shutdown()


if __name__ == "__main__":
    main()
