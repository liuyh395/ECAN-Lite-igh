# MCAN Port Usage

`igh_mcan_port` 构建在 `igh_adapter` 之上，负责 ECAN-Lite 的 MCAN/CAN FD 网关协议：通道配置、CAN timing SDO、动态 PDO 帧封装、接收解析、直接发送和 SYNC0 相位预约发送。

## 初始化流程

```c
ecan_igh_config_t igh_config;
ecan_igh_context_t igh;
igh_mcan_port_config_t mcan_config;
igh_mcan_port_t mcan;

ecan_igh_default_config(&igh_config);
ecan_igh_config_set_slave_count(&igh_config, 3);
ecan_igh_config_set_pdo_bytes(&igh_config, 384);
ecan_igh_config_set_cycle_us(&igh_config, 2000);
ecan_igh_config_enable_dc_sync0(&igh_config, 2000, 0);

igh_mcan_port_default_config(&mcan_config);
igh_mcan_port_set_channel_mask(&mcan_config, 0, 0x0f, 0x0f);
igh_mcan_port_set_channel_mask(&mcan_config, 1, 0x0f, 0x0f);
igh_mcan_port_set_channel_mask(&mcan_config, 2, 0x03, 0x03);
igh_mcan_port_apply_config(&igh_config, &mcan_config);

ecan_igh_init(&igh, &igh_config);
ecan_igh_wait_op_timeout(&igh, 10000);
igh_mcan_port_init(&mcan, &igh, &mcan_config);
```

`igh_mcan_port_apply_config()` 会把 MCAN 配置追加到 IgH 启动 SDO 队列，所以应在 `ecan_igh_init()` 之前调用。

默认 PDO 接口同时设置两个方向。如果主站下发命令较少、从站需要回传更多帧，可以独立设置两个方向：

```c
/* 主站输出 RxPDO=128 B，从站输入 TxPDO=256 B。 */
ecan_igh_config_set_direction_pdo_multiplier(&igh_config, 1, 2);
```

`ecan_igh_config_set_slave_direction_pdo_multiplier()` 可覆盖单个从站，传 `0` 表示该方向继续使用全局值。`ecan_igh_config_get_slave_rxpdo_bytes()` 和 `ecan_igh_config_get_slave_txpdo_bytes()` 返回最终生效的方向容量。原有 `ecan_igh_config_set_pdo_bytes()`、`ecan_igh_config_set_pdo_multiplier()` 和单从站对称接口保持原行为。

## 直接发送

普通业务代码直接使用 `igh_mcan_port_send_can()` 和 `igh_mcan_port_send_canfd()`。这两个接口会直接把 CAN_FRAME 写入当前周期 RxPDO，调用链短，数据内容只拷贝一次。

```c
uint8_t data[8] = {0, 1, 2, 3, 4, 5, 6, 7};

if (igh_mcan_port_cycle_try_begin(&mcan) == IGH_ECAN_OK) {
    igh_mcan_port_send_can(&mcan, 0, 0, 0x123, 0, data, sizeof(data));
    igh_mcan_port_cycle_end_checked(&mcan);
}
```

直接发送使用 `TSF_NONE`。固件收到有效 RxPDO 后尽快投递到 MCAN TX FIFO 或 TX buffer，CAN 发送时刻会跟随 EtherCAT PDO 到达和固件处理时刻抖动。

客户工程统一写成固定周期，避免额外的批量 exchange/预构造发送帧封装：

```text
cycle_begin -> receive -> send_can/send_canfd/schedule_can -> cycle_end
```

## CAN FD

```c
uint8_t fd_data[64] = {0};

igh_mcan_port_send_canfd(&mcan,
                         0,      /* slave_pos */
                         0,      /* channel */
                         0x123,  /* CAN ID */
                         0,      /* standard ID */
                         1,      /* BRS */
                         fd_data,
                         sizeof(fd_data));
```

Classic CAN 会自动限制到 8B。CAN FD 可发送 0..64B，DLC 由 `igh_mcan_len_to_dlc()` 转换。

## 预约发送

底层 MCAN port 支持 SYNC0 周期内相位预约：

```c
uint8_t data[8] = {0};

igh_mcan_port_schedule_can(&mcan,
                           0,      /* slave_pos */
                           0,      /* channel */
                           0x123,  /* CAN ID */
                           0,      /* standard ID */
                           data,
                           sizeof(data),
                           100000, /* next SYNC0 + 100us */
                           1);     /* tag */
```

预约发送使用动态 PDO 中的 `TSF_FULL64` 字段：

```text
format:u8 = 1
flags:u8 = 0
tag:u16
offset_ns:u32
```

`offset_ns` 表示“下一次 SYNC0 周期内的目标相位”。如果 PDO 到达时目标相位已经错过，固件应立即发送，不跨到下一个周期，以保证帧不会跨周期滞留。

使用预约发送前，主站应读取设备能力对象 `0xF000:16`，确认包含 `IGH_MCAN_PDO_FEATURE_RX_TSF_SCHEDULE_TX`。没有该能力时应退回直接发送。

示例程序 `igh_mcan_gateway_tx_probe --scheduled-tx` 会自动启用 DC Sync0；手写应用使用预约 API 时应显式配置 `ecan_igh_config_enable_dc_sync0()`。

## 接收与解析

```c
igh_mcan_rx_frame_t rx[64];
size_t rx_count = 0;

if (igh_mcan_port_cycle_begin_checked(&mcan) == IGH_ECAN_OK) {
    igh_mcan_port_receive(&mcan, rx, 64, &rx_count);
    igh_mcan_port_cycle_end_checked(&mcan);
}
```

每个 `igh_mcan_rx_frame_t` 包含来源从站、CAN/CAN FD 帧和本周期 PDO 头快照。解析失败时可读取最近一次错误：

```c
size_t slave = 0;
const igh_mcan_pdo_parse_diag_t *diag =
    igh_mcan_port_get_last_parse_error(&mcan, &slave);
```

`igh_mcan_port_cycle_begin/end()` 的 void 版本只为兼容旧 demo 保留。客户工程建议使用 checked 版本获取明确错误码；实时线程可使用 `igh_mcan_port_cycle_try_begin()`，锁忙或已有周期未结束时返回 `IGH_ECAN_NOT_READY`，不会阻塞等待。

`ecan_igh_wait_op()`、`ecan_igh_wait_op_timeout()` 和 `ecan_igh_reboot_slaves()` 是 blocking API，只应在初始化、恢复或维护流程中调用。周期线程若只想查询 OP 状态，可使用单次交换的 `ecan_igh_poll_op()`。

## 预约发送使用原则

预约发送 API 用于把 CAN 帧释放时刻约束到同步周期内的指定相位。普通功能验证优先使用直接发送；只有电机或执行器明确要求固定 CAN 发送相位时，才使用 `igh_mcan_port_schedule_can/canfd()`。

主站周期必须先稳定，否则预约时刻仍会受到 Linux 调度和 PDO 到达抖动影响。产品示例默认采用两套完整 profile：SM 使用 250us EtherCAT 子周期和四相轮转；DC 使用 1ms 周期、900us shift 和跨周期闭环。不要脱离 profile 单独复制相位数值。

预约发送会受 CAN 仲裁、帧长度、位率和控制器 TX 队列影响。多帧负载应逐级增加，并同时检查主站周期统计、WKC、发送失败、丢帧、重复帧和乱序；不能只依据平均周期判断是否稳定。需要自行调整 `offset_ns` 时，应使用 CAN 分析仪或接收端时间戳，在目标设备和实际总线负载下重新验收。

如果需要验证“MCAN 通道配置为 CAN FD-capable，但发送 classic CAN 帧”，使用同一个 probe：

```sh
sudo ./build/igh_mcan_gateway_tx_probe \
  --classic-can \
  --gateway-canfd \
  --len 8
```
