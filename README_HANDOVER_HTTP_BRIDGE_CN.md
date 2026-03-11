# LeRobot x Isaac Sim/Isaac Lab 交接文档（可直接运行）

## 1. 目标与当前状态

目标：保持两个独立环境，不跨 conda import，仅通过本地 HTTP 通信。

- `env_lerobot`：策略服务、模型推理（PI0/VLA）
- `env_isaaclab` / 系统 ROS：仿真与 ROS topic 输入输出

当前链路：

`/joint_states + 3路图像 -> HTTP /act -> /policy_action -> /joint_command`

## 2. 关键文件（本次交接必须保留）

- `shared/schema.py`
- `shared/http_client.py`
- `lerobot_side/server.py`
- `isaaclab_side/ros2_obs_action_bridge.py`
- `isaaclab_side/ros2_action_to_joint_command.py`
- `isaaclab_side/run_env_loop.py`
- `IsaacLab_env.sh`
- `lerobot_env.sh`
- `README_HANDOVER_HTTP_BRIDGE_CN.md`（本文件）

## 3. 三终端标准启动命令

## 终端 A：LeRobot 服务（PI0）

```bash
cd /root/gpufree-data/lerobot
LEROBOT_POLICY_BACKEND=lerobot \
LEROBOT_MODEL_PATH=/root/gpufree-data/models/pi0_base \
LEROBOT_MODEL_DEVICE=cuda \
LEROBOT_STRICT_MODEL_LOAD=false \
./lerobot_env.sh -- python -m lerobot_side.server
```

健康检查：

```bash
curl -s http://127.0.0.1:8000/health
```

## 终端 B：ROS 观测桥（joint+3路图像 -> /act -> /policy_action，严格三路）

```bash
conda deactivate 2>/dev/null || true
unset PYTHONPATH
unset LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash

cd /root/gpufree-data/lerobot
/usr/bin/python3 -m isaaclab_side.ros2_obs_action_bridge \
  --server-url http://127.0.0.1:8000 \
  --joint-topic /joint_states \
  --image-topic /sim/camera/image_raw \
  --image-topic-left /sim/camera/image_raw_left \
  --image-topic-right /sim/camera/image_raw_right \
  --action-topic /policy_action \
  --include-image \
  --include-multi-image \
  --require-all-multi-images \
  --image-stale-ms 1200 \
  --multi-image-sync-ms 250 \
  --reuse-missing-image-ms 200 \
  --safe-zero-on-missing-images \
  --safe-zero-after-ms 500 \
  --instruction "pick the red block" \
  --rate-hz 4
```

预期日志（每 10 次）：

- `image_keys=['base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb']`

## 终端 C：动作桥（/policy_action -> /joint_command，抗抖动/抗漂移）

```bash
conda deactivate 2>/dev/null || true
unset PYTHONPATH
unset LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash

cd /root/gpufree-data/lerobot
/usr/bin/python3 -m isaaclab_side.ros2_action_to_joint_command \
  --state-topic /joint_states \
  --action-topic /policy_action \
  --cmd-topic /joint_command \
  --mode delta \
  --action-scale 0.015 \
  --clamp-abs 1.6 \
  --smoothing-alpha 0.15 \
  --max-delta 0.008 \
  --deadband 0.02
```

## 4. 验证标准（是否真在 PI0 推理）

执行：

```bash
curl -s http://127.0.0.1:8000/health
ros2 topic echo /policy_action --once
ros2 topic echo /joint_command --once
```

判定“已进入真实 PI0 路径”：

- `model_loaded = 1`
- `last_action_source = "pi0_select_action"`
- `inference_ok_count > 0` 且持续增长
- `fallback_count` 不持续增长

若出现 `All image features are missing`：

- 先确认终端 B 是否带了 `--include-image --include-multi-image`
- 先停掉旧进程再重启三终端，避免旧参数残留
- 确认 topic 名称是：
  - `/sim/camera/image_raw`
  - `/sim/camera/image_raw_left`
  - `/sim/camera/image_raw_right`

若出现 `skip request: missing_multi_image` 频繁：

- 优先检查相机源是否稳定：
  - `ros2 topic hz /sim/camera/image_raw`
  - `ros2 topic hz /sim/camera/image_raw_left`
  - `ros2 topic hz /sim/camera/image_raw_right`
- 已支持短时缺图保护：
  - `--reuse-missing-image-ms 200`：短时间复用最近一次完整三路图像包
  - `--safe-zero-on-missing-images --safe-zero-after-ms 500`：缺图持续时发布零动作，避免“找不到目标后跑飞”
- 若模型要求严格三路，不建议关闭 `--require-all-multi-images`。

## 5. 摄像头排布建议（避免“手臂乱窜”）

- `base_0_rgb`：固定在底座前上方，能同时看见夹爪工作区和目标物。
- `left_wrist_0_rgb`：夹爪左侧后方，朝夹爪前方略向下（确保目标在画面中央偏下）。
- `right_wrist_0_rgb`：与左腕对称。

原则：

- 可以拍到少量机械臂本体，但不要被大面积遮挡。
- 目标物必须在两个腕部视角中长期可见；动起来也不能丢。
- 三路相机方向不应完全重合，避免信息冗余。

## 6. 常见问题

- ROS + conda + Isaac 混用导致 `rclpy`/`numpy`/`GLIBCXX` 冲突：
  - ROS bridge 固定用系统 Python（`/usr/bin/python3`）。
- Isaac 端直接 `gym.make("Isaac-...")` 报未注册：
  - 用 `./isaaclab.sh -p ...` 运行环境脚本。
- GPU OOM：
  - 降低并行/频率，关闭其他占显存进程。
- 机械臂“跑着跑着单向漂移”：
  - 将动作桥保持 `delta` 模式，并使用较小 `action_scale` + `max_delta` + `deadband`。
  - 这通常是积分漂移，不是链路断开。

## 7. 本次新增参数（2026-03-11）

- `isaaclab_side/ros2_obs_action_bridge.py`
  - `--image-stale-ms`
  - `--multi-image-sync-ms`
  - `--require-all-multi-images`
  - `--reuse-missing-image-ms`
  - `--safe-zero-on-missing-images`
  - `--safe-zero-after-ms`
- `isaaclab_side/ros2_action_to_joint_command.py`
  - `--smoothing-alpha`
  - `--max-delta`
  - `--deadband`

## 8. 本地已清理的无用文件

本次已删除：

- `lerobot_isaaclab_bridge_bundle.tar.gz`
- `isaaclab_side/__pycache__/`
- `lerobot_side/__pycache__/`
- `shared/__pycache__/`
- `src/lerobot/__pycache__/`
