from __future__ import annotations

import argparse
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class ActionToJointCommandBridge(Node):
    def __init__(
        self,
        *,
        state_topic: str,
        action_topic: str,
        cmd_topic: str,
        mode: str,
        action_scale: float,
        clamp_abs: float,
    ) -> None:
        super().__init__("action_to_joint_command_bridge")
        self.mode = mode
        self.action_scale = action_scale
        self.clamp_abs = clamp_abs

        self.last_state: Optional[JointState] = None
        self.last_action: Optional[list[float]] = None
        self.publish_count = 0

        self.state_sub = self.create_subscription(JointState, state_topic, self._on_state, 10)
        self.action_sub = self.create_subscription(Float64MultiArray, action_topic, self._on_action, 10)
        self.cmd_pub = self.create_publisher(JointState, cmd_topic, 10)

        self.get_logger().info(
            f"started. state_topic={state_topic}, action_topic={action_topic}, cmd_topic={cmd_topic}, "
            f"mode={mode}, action_scale={action_scale}, clamp_abs={clamp_abs}"
        )

    def _on_state(self, msg: JointState) -> None:
        self.last_state = msg

    def _on_action(self, msg: Float64MultiArray) -> None:
        self.last_action = [float(v) for v in msg.data]
        self._publish_command()

    def _publish_command(self) -> None:
        if self.last_state is None or self.last_action is None:
            return
        if not self.last_state.position:
            return

        state = self.last_state
        num_joints = len(state.position)
        action = self.last_action
        if len(action) < num_joints:
            action = action + [0.0] * (num_joints - len(action))
        else:
            action = action[:num_joints]

        if self.mode == "delta":
            target_pos = [state.position[i] + self.action_scale * action[i] for i in range(num_joints)]
        else:
            target_pos = [self.action_scale * action[i] for i in range(num_joints)]

        if self.clamp_abs > 0:
            limit = abs(self.clamp_abs)
            target_pos = [max(-limit, min(limit, v)) for v in target_pos]

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(state.name)
        out.position = [float(v) for v in target_pos]
        out.velocity = []
        out.effort = []
        self.cmd_pub.publish(out)

        self.publish_count += 1
        if self.publish_count % 20 == 0:
            preview = [round(v, 4) for v in out.position[:3]]
            self.get_logger().info(
                f"published={self.publish_count} joints={len(out.position)} first3={preview}"
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Bridge /policy_action to /joint_command JointState")
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--action-topic", default="/policy_action")
    parser.add_argument("--cmd-topic", default="/joint_command")
    parser.add_argument("--mode", choices=["absolute", "delta"], default="absolute")
    parser.add_argument("--action-scale", type=float, default=1.0)
    parser.add_argument("--clamp-abs", type=float, default=3.14159)
    args = parser.parse_args()

    rclpy.init()
    node = ActionToJointCommandBridge(
        state_topic=args.state_topic,
        action_topic=args.action_topic,
        cmd_topic=args.cmd_topic,
        mode=args.mode,
        action_scale=args.action_scale,
        clamp_abs=args.clamp_abs,
    )
    try:
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

