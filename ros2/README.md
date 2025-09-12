RK3588 ROS 2 (rclpy-only) Restructure

Overview
- This directory scaffolds a ROS 2 (rclpy-only) layout that mirrors `~/nagasware/src` while reusing existing algorithms and models from this repo.
- Initial focus is CAN bridging and message definitions; subsequent nodes will wrap model, controls, and supervisor logic.

Structure
- common/        Shared utilities and shims (params/time)
- op_core_msgs/  ROS 2 message package (CanData, PandaState, etc.)
- sensing/can/   SocketCAN bridge node (`op_can_bridge`)
- model/         Model node wrappers (to be added)
- control/       Controls node wrappers (to be added)
- system/        Supervisor/state machine (to be added)
- launch/        Launch files for bring-up

Notes
- Messages are defined in `op_core_msgs` and used by Python nodes via `rclpy`.
- Safety: for in-vehicle use, keep hardware safety (panda or equivalent). The SocketCAN bridge is suitable for bench/dev unless paired with a safety MCU.

