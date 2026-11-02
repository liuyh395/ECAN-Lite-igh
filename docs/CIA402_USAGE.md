# CiA402 Usage

## 运动启动边界

单轴、单轴 SDO 和多轴运动示例都要求显式 `--enable-motor`。未指定时返回参数错误，
不申请 EtherCAT master、不配置驱动器、不发送使能或运动命令。先运行 `--help` 核对参数；
明确允许运动后再添加该选项。此软件确认不替代急停、机械限位和安全控制系统。
`--allow-missing-position`、跳过配置及诊断负载等选项属于专门验证功能，不能作为生产运动默认值。

`igh_cia402_protocol` 是 examples 侧的 CANopen/CiA402 电机控制层，负责 NMT、SDO、CSP PDO 配置、使能流程、目标位置发送和状态解析。它不属于核心 EtherCAT 转 CAN 数据通道，也不绑定 `igh_mcan_port`；demo 主循环优先使用 frame builder/parser，让 MCAN 收发在 demo 层显式出现。

单轴和 10 轴电机 demo 默认应用 `dc_1ms_cross_cycle` profile：1ms 周期、Sync0 shift 900us、PDO `N=3`（384B/方向），按 N 周期下发、N+1 周期读取组织闭环。需要更大动态 PDO 时使用 `--pdo-n N` 或 10 轴 demo 的 `--pdo-map a,b,c` 覆盖；覆盖后应在目标系统重新验证实时边界。

## 预约发送支持状态

当前电机 demo：

- `igh_cia402_single_axis_demo`
- `igh_cia402_10axis_demo`

默认不启用电机目标位置帧预约。普通模式的运动周期是：

1. 发送 RPDO1 目标帧，payload 为 `6040/6060/607A`，长度 7B；
2. 发送 CANopen SYNC；
3. 电机按 SYNC 锁存本周期目标。

也就是说，CiA402 demo 默认使用普通 MCAN 发送接口，目标帧在 PDO 到达后立即进入固件发送路径。

需要测试预约模式时，使用 demo 参数 `--scheduled-tx --schedule-target-offset-ns N --schedule-sync-offset-ns M`，或用 `--scheduled-target`、`--scheduled-sync` 分别只预约目标 RPDO 或只预约 SYNC。demo 会自动打开 DC Sync0；手动集成时也应调用 `ecan_igh_config_enable_dc_sync0()`。预约模式只影响运动阶段的目标 RPDO 和 SYNC，不影响 NMT/SDO 启动流程。RPDO 和 SYNC 都预约时，`sync_offset_ns` 必须晚于 `target_offset_ns`，例如 target=`100000ns`、SYNC=`200000ns`。

## 单轴初始化

```c
igh_cia402_axis_t axis;
igh_cia402_context_t cia;
igh_cia402_mcan_transport_t transport;

igh_cia402_axis_init(&axis,
                     0,      /* ECAN-Lite slave position */
                     0,      /* MCAN channel */
                     1,      /* CANopen Node ID */
                     500);   /* target step */

igh_cia402_mcan_transport_init(&transport, &mcan);
igh_cia402_context_init(&cia, &transport.transport, &axis, 1);
igh_cia402_set_scheduled_tx(&cia,
                            1,      /* target RPDO scheduled */
                            100000, /* target at SYNC0 + 100us */
                            1,      /* CANopen SYNC scheduled */
                            200000);/* SYNC at SYNC0 + 200us */
```

轴表只描述电机拓扑。CAN timing、PDO 大小和 MCAN 通道能力仍由 `igh_mcan_port_config_t` 配置；`examples/cia402/cia402_mcan_transport.h` 提供 MCAN/CiA402 帧转换和简单发送适配。

## 启动流程

推荐流程：

```c
igh_cia402_sdo_upload_u32_timeout(&cia, &axis, 0x2004, 0, &value, &size, 1000);
igh_cia402_configure_csp_rpdo_timeout(&cia, &axis, 1000);
igh_cia402_request_actual_position_timeout(&cia, &axis, 1000);
igh_cia402_enable_timeout(&cia, &axis, 1000);
```

demo 中的 CSP 配置会设置：

```text
RPDO1 = 6040:00 controlword
        6060:00 modes of operation
        607A:00 target position

TPDO1 = 6041:00 statusword
        6061:00 modes of operation display
        6064:00 position actual value
```

默认不主动发送 NMT reset，避免部分驱动复位后短时间内不响应 SDO。需要复位时显式使用 demo 参数或调用 `igh_cia402_send_nmt()`。

只有一个电机时，也可以用 10 轴 demo 的单轴模式复用多轴启动流程。例如电机接在从站 0、MCAN0、NodeID 1，classic CAN 1M。
下例会使能设备；即使 `--step 0` 也不等于无运动风险，必须先确认机械安全：

```sh
sudo ./build/igh_cia402_10axis_demo \
  --enable-motor \
  --slaves 1 \
  --pdo-n 3 \
  --only-axis \
  --axis-slave 0 \
  --axis-channel 0 \
  --axis-node 1 \
  --classic-can \
  --bitrate 1000000 \
  --cycles 20 \
  --step 0
```

`--only-axis` 会只构造一个运行轴，同时保留 10 轴 demo 的启动、guard、运动统计和诊断输出。`--classic-can/--canfd`、`--bitrate`、`--data-bitrate` 只覆盖这个单轴通道配置。

## 运动周期

10 轴 demo 的 `--motion-div N` 用于运动发送降频：每 `N` 个 EtherCAT 周期发送一次目标 RPDO 和 SYNC。默认 `N=1`，即每个 EtherCAT 周期都发送目标帧；如果设置 `--motion-div 2`，则 CAN 目标帧周期会变成 EtherCAT 周期的 2 倍，例如 `--cycle-us 2000 --motion-div 2` 时 CAN 侧会看到约 4ms 的目标帧周期。

```c
igh_mcan_rx_frame_t rx[64];
size_t rx_count = 0;

igh_mcan_port_cycle_begin(&mcan);

igh_mcan_port_receive(&mcan, rx, 64, &rx_count);
for (size_t i = 0; i < rx_count; i++) {
    igh_cia402_rx_frame_t cia_rx;

    igh_cia402_mcan_rx_to_can(&rx[i], &cia_rx);
    igh_cia402_on_can_frame(&cia, cia_rx.slave_pos, &cia_rx.frame);
}
if (axis.statusword_valid &&
    igh_cia402_statusword_is_operation_enabled(axis.statusword)) {
    igh_cia402_target_step_t step;
    igh_cia402_tx_frame_t target;
    igh_cia402_tx_frame_t sync;

    igh_cia402_prepare_target_step(&axis, &step);
    igh_cia402_build_prepared_target_frame(&cia, &axis, &step, &target);
    igh_cia402_mcan_send_frame(&mcan, &target);
    igh_cia402_set_target_position(&axis, step.next_position);

    igh_cia402_build_sync_frame(&cia, axis.slave_pos, axis.channel, &sync);
    igh_cia402_mcan_send_frame(&mcan, &sync);
}

igh_mcan_port_cycle_end(&mcan);
```

应用应在 EtherCAT domain 不完整、从站不在 OP、轴未 ready 或收到 EMCY 时停止推进目标位置。继续递增目标但没有稳定送达电机，会在通信恢复后形成目标跳变。

预约模式下仍必须保持同样的保护逻辑。预约只改变 CAN 释放相位，不改变目标位置递增、ready 判断和 EMCY 处理。

## 常用诊断

| 字段/接口 | 用途 |
| --- | --- |
| `axis.statusword` | CiA402 状态，`0x0027` 表示 Operation enabled |
| `axis.actual_position` | `0x6064` 实际位置 |
| `axis.sdo_abort_code` | 最近一次 SDO abort code |
| `axis.emcy_count` | 收到的 EMCY 数量 |
| `igh_cia402_statusword_state_name()` | statusword 转文本 |
| `igh_cia402_sdo_abort_name()` | SDO abort code 转文本 |

现场排查时优先查看 demo 输出中的 `slave/ch/node`、`status`、`state`、`last_sdo`、`abort` 和 EMCY 行。
