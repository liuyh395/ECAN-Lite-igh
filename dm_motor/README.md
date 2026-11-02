# 达妙 Classic CAN MIT 电机模块

本目录针对达妙电机的 8 字节 Classic CAN MIT 协议，不使用 CAN FD。协议依据当前项目中的达妙文档：

- 控制帧：位置 16 位、速度/KP/KD/力矩各 12 位；
- 反馈帧：`D[0] = ID[3:0] | ERR[3:0] << 4`，`D[6]` 为 MOS 温度，`D[7]` 为线圈温度；
- 控制字：`FC` 使能、`FD` 失能、`FE` 设置零点、`FB` 清除错误。

## 文件

- `dm_motor_protocol.h/.c`：纯协议编解码，不依赖 EtherCAT，便于单元测试；
- `dm_motor.h/.c`：绑定 `igh_mcan_port` 的 Classic CAN 单电机和 12 电机管理接口；
- `dm_motor_protocol_test.c`：协议层测试。

## 默认拓扑

`dm_motor_manager_default_config()` 生成 12 个电机配置：CAN0~CAN3 每路 3 个电机，电机 ID 依次为 1、2、3。实际项目应根据电机内部配置覆盖 `channel`、`motor_id`、`command_id`、`feedback_id` 和量程。

调用约束：在一个 `igh_mcan_port_cycle_begin_checked()` 和 `igh_mcan_port_cycle_end_checked()` 之间调用 `dm_motor_manager_receive()` 与 `dm_motor_manager_send()`。发送函数使用 `igh_mcan_port_send_can()`，固定 DLC 为 8，并拒绝 CAN FD 反馈帧。

`dm_motor_send_control()` 只负责追加控制帧，不自动开始或结束周期；首次上电应在机械安全、急停有效的条件下使用，并在收到正常反馈后再发送运动命令。
