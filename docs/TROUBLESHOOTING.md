# IgH Troubleshooting

本文档记录 IgH 主站侧常见问题的定位入口，只描述当前模块和 demo 应如何使用已有诊断信息。

## EtherCAT 周期不完整

典型日志：

```text
domain_complete=0 wc=4 wc_state=1 slaves_operational=1
```

检查顺序：

1. `ecan_igh_domain_is_complete()` 是否连续失败。
2. `ecan_igh_slave_al_state()` 是否仍为 `0x08`，确认从站是否 OP。
3. 打开 `ECAN_IGH_PRINT_RUNTIME_DIAG=1`，在 demo 退出时查看 `receive_process_max_ns`、`master_send_max_ns`、`cycle_late_max_ns` 和最近的 `wkc-mismatch` 事件。
4. CPU/IRQ 绑核是否按 [CPU_AFFINITY.md](CPU_AFFINITY.md) 配置。
5. PDO 大小是否满足设备能力，domain/datagram 长度和期望 WKC 是否与实际映射一致。当前 1392 B 上限不要求超大单 FMMU 分段，见 [主站兼容性](MASTER_COMPATIBILITY.md)。

运动 demo 在 WC 不完整时应 hold 当前目标，不应继续递增目标位置。

## Adapter 运行诊断

`igh_adapter` 会记录周期阶段耗时、WKC 异常和周期超时事件。常用 demo 可通过环境变量打开退出时打印：

```sh
sudo ECAN_IGH_PRINT_RUNTIME_DIAG=1 ./build/igh_mcan_gateway_tx_probe --cycle-us 2000 --pdo-bytes 1024
./build/igh_cia402_10axis_demo --help
```

稳定性矩阵使用 `ECAN_IGH_FAIL_ON_DIAG_EVENT=1`，因此 demo 即使 CAN send API 全部成功，只要 adapter 记录到 WKC mismatch 或周期 overrun，也会返回失败。

CiA402 运动诊断不是只读操作。确认安全后必须显式添加 `--enable-motor`；该示例使用
`--pdo-n` 大小编码，例如 `--pdo-n 4` 表示 512 B，不支持 `--pdo-bytes`。

关键字段：

| 字段 | 含义 |
| --- | --- |
| `cycle_late_max_ns` | 应用周期开始相对上一周期的最大延后，排查 2ms 跳到 3ms 这类主站调度抖动 |
| `cycle_exec_max_ns` | 单个 IgH 周期从 begin 到 end 的最大耗时 |
| `receive_process_max_ns` | `ecrt_master_receive()` + `ecrt_domain_process()` 最大耗时 |
| `txpdo_copy_max_ns` / `rxpdo_copy_max_ns` | 非连续 PDO shadow 拷贝耗时，判断大 PDO 是否被拷贝放大 |
| `queue_max_ns` / `master_send_max_ns` | domain queue 和 master send 最大耗时 |
| `wkc` / `expected` / `wc_state` | 最近一次 domain 工作计数器和完整性 |
| `igh-event type=wkc-mismatch` | 某周期 WKC 不完整或计数不符合预期 |
| `igh-event type=cycle-overrun` | 周期延后超过 10% 周期时间 |

这些统计不会替代分析仪抓包。使用方法是先看主站自身是否有 `cycle-overrun` 或 `master_send_max_ns` 长尾；若主站周期稳定但 WKC 异常，再结合高精度抓包和从站诊断判断问题在链路、从站 ESC/PDI 窗口还是 PDO 内容。

## 启动等待提前返回配置失败

`EtherCAT startup configuration failed`表示标准IgH信息接口报告了已配置从站的
主站侧`error_flag`。日志中的`ring`为绝对环位置，`alias`和`position`为应用配置寻址。
检查主站日志中紧邻该错误的SDO拒绝、PDO映射或状态转换原因；不要将该标志解释为
CAN Bus-off、PDO压力或丢帧计数，也不要仅增加OP等待时间。

正常扫描、离线和暂未进入OP仍使用原等待期限。适配器不会因不完整WKC直接认定
配置失败，也不会为了恢复而自动复位、关闭错误检测或重复发送旧命令。
关闭失败上下文，消除实际故障后重新初始化。未将配置问题设置为`error_flag`的
主站实现仍可能返回等待超时，应继续核对其错误日志；此诊断不能替代主站配置正确性。

## 停机后出现同步错误

若业务阶段完整，而释放主站后出现`0x001A Synchronization error`，应分别核对
最后一次周期通信、close调用/返回、AL控制请求、AL状态读回及错误码的时间。
`ecan_igh_close()`释放资源后，状态转换仍可能由IgH后台线程继续执行；
调用返回和稍后回到PREOP均不能证明停机期间无错误。

在多从站拓扑中，后台线程处理其他从站可能延后目标从站的状态转换。
正常停止须在释放主站前完成状态切换和确认，并保留有界超时及错误反馈；
不能通过延长同步容差、关闭看门狗或清除告警将该问题判为通过。
业务帧完整性和停机状态分别验收，见[主站兼容性](MASTER_COMPATIBILITY.md)。

## 动态 PDO 解析异常

`igh_mcan_port`按所选V1/V2布局校验头部、payload长度、已启用的CRC、layout、通道掩码和帧封装长度，不应将所有布局固定解释为24B头部。解析失败时读取：

```c
size_t slave = 0;
const igh_mcan_pdo_parse_diag_t *diag =
    igh_mcan_port_get_last_parse_error(&mcan, &slave);
```

重点字段：

| 字段 | 含义 |
| --- | --- |
| `diag->code` | 解析错误类型 |
| `diag->offset` | 出错的 PDO 偏移 |
| `diag->ctrl` | 当前帧控制字 |
| `diag->payload_crc16` | PDO 头部记录的 CRC |
| `diag->layout_id` | 当前 layout/profile |

## SDO 超时

典型日志：

```text
CSP configure failed: slave=0 ch=0 node=0x09 rc=-5 index=0x1400:01
```

`rc=-5` 表示超时。排查顺序：

1. 先确认 `0x2004:00` 可读，设备处于 CANopen 通信模式。
2. 确认对应 MCAN 通道没有 bus off，`ch_status` 中 warning/bus-off 位没有持续出现。
3. 降低并发配置压力，单轴验证同一个 slave/channel/node。
4. 查看 `axis.last_sdo`、`axis.sdo_last_write_rc`、`axis.sdo_last_read_rc` 和 `axis.sdo_abort_code`。

## 预约发送没有改善

预约发送对直接使用 `igh_mcan_port_schedule_can/canfd()` 的应用生效。CiA402 电机 demo 通过 `--scheduled-tx` 或 `igh_cia402_set_scheduled_tx()` 同时启用预约目标帧和预约 SYNC，也可以用 `--scheduled-target`、`--scheduled-sync` 分别测试 RPDO-only 或 SYNC-only 预约；默认仍是普通发送。

如果使用了预约 API 但 CAN 抓包仍与直接发送接近，优先确认：

1. 固件版本支持 `IGH_MCAN_PDO_FEATURE_RX_TSF_SCHEDULE_TX`。
2. EtherCAT DC Sync0 已启用，主站周期与 `offset_ns` 在同一个时间基准下。
3. 主站应用周期本身足够稳定。`igh_mcan_gateway_tx_probe` 会输出 `master_period`；如果主站 1ms/2ms 周期已经有几百微秒长尾，CAN 侧预约也会被拖累。
4. Linux 测试时建议使用 `--rt-priority 80 --spin-us 100`，先把 demo 自身周期收敛后再评估预约效果。
5. `offset_ns` 没有设置得过早。PDO 到达时目标相位已错过时，固件会立即发送，表现会接近直接发送。
6. 多帧同相位发送时，MCAN 仲裁和 TX buffer 排队仍会带来帧间隔；可使用 `scheduled-spread` 展开相位。

不要把单次抓包中的平均值或 p99 当作通用保证。验收时应同时记录主站周期、从站数量、PDO 大小、CAN 位率和负载、预热区间、WKC、发送/接收数量、丢帧、重复、乱序和完整时延分布。

## 电机目标跳变

目标跳变通常不是单帧数据大小端错误，而是通信空洞后目标继续递增导致恢复时目标差过大。处理原则：

- domain 不完整时 hold；
- 从站不 OP 时 hold；
- 轴未 Operation enabled 时 hold；
- 轴收到 EMCY 后停止向该轴推进目标；
- 长测时同时保存 IgH 日志和 UCAN 抓包，确认空洞发生在 EtherCAT、网关还是 CAN 电机侧。
