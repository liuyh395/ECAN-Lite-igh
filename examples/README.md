# IgH Examples

本目录只保留对外集成和现场诊断仍有明确价值的示例。临时验证程序不再单独保留；能被通用 probe 覆盖的场景合并到通用 probe 参数中。

目录按功能拆分：

| 目录 | 内容 |
| --- | --- |
| `basic/` | 最小集成和 raw PDO 观察 |
| `mcan/` | ECAN-Lite MCAN/CAN FD 网关配置与收发示例 |
| `canopen/` | CANopen SDO、NMT、Heartbeat 现场 probe |
| `cia402/` | CiA402 应用协议库和电机 demo |
| `mit/` | MIT packed protocol CAN FD 电机 demo、编解码和 UCAN-Mini 模拟器 |

## 推荐入口

| 示例 | 构建目标 | 用途 |
| --- | --- | --- |
| `basic/simple_loop.c` | `igh_simple_loop` | 最小 MCAN Core classic CAN 收发循环，适合作为客户 API 入门 |
| `mcan/mcan_gateway_tx_probe.c` | `igh_mcan_gateway_tx_probe` | 通用 TX 测试：classic/CAN FD、通道 FD-capable、直接发送、预约发送 |
| `mcan/mcan_rx_dump.c` | `igh_mcan_rx_dump` | 接收侧抓帧和动态 PDO 解析 |
| `canopen/canopen_sdo_probe.c` | `igh_canopen_sdo_probe` | CANopen SDO/NMT/从站复位探测 |
| `cia402/cia402_single_axis_demo.c` | `igh_cia402_single_axis_demo` | 单轴 CiA402 CSP 启动和运动 |
| `cia402/cia402_10axis_demo.c` | `igh_cia402_10axis_demo` | 多轴 CiA402 CSP 启动、运动和 EtherCAT guard |
| `mit/mit_canfd_motor_demo.c` | `igh_mit_canfd_motor_demo` | 1 ms MIT CAN FD 电机闭环、回复率和循环延迟统计 |
| `pipeline/four_phase_250us_demo.c` | `igh_250us_four_phase_demo` | 独立250 μs四相SM API，MCAN0..3各1 ms |

## 分层示例

### 正常停止接入

`mcan_rx_dump`正常结束时先消费最后一个已回收PDO，再调用
`common/igh_example_stop.h`请求并确认PREOP，最后释放主站资源。
依赖可信安装的标准`ethercat`命令及主站设备权限；CLI缺失、失败或超时会导致示例失败退出。
该行为不代表电机安全停止，也不自动取消未来预约；其他发送和电机示例未自动迁移。
完整前提及错误反馈见[停机API](../API.md#111-有界停机事务)。

`mcan_rx_dump`的`truncated_batches`记录应用接收数组不足的调用次数，包含最后一次上行消费，
不是丢失帧数，也不是从站MCAN/PDO的溢出计数。首次发生即输出错误，不受`--diag-every`控制；
任意一次截断都会使最终退出码非零，后续接收或正常停机不会覆盖该结果。
`capture ... complete=1`仅表示本次示例未观察到接收数组不足且运行/停机返回成功，
不证明上游发送数量、物理CAN链路或整个业务窗口绝无缺帧；完整性仍须对齐发送端序号和抓包。

### 功能分层

| 层级 | 示例 | 保留原因 |
| --- | --- | --- |
| Raw IgH | `basic/raw_pdo_dump.c` | 验证 adapter 层 PDO buffer 和 working counter，不依赖 MCAN 协议 |
| MCAN Core | `basic/simple_loop.c` | 只展示配置、打开、周期收发和关闭，便于客户从零接入 |
| MCAN port | `mcan/mcan_multi_channel_demo.c` | 演示同一从站多通道、classic CAN 与 CAN FD 混合收发 |
| MCAN port | `mcan/mcan_gateway_tx_probe.c` | 统一覆盖原单帧 TX、classic-on-FD-channel、预约发送测试 |
| MCAN port | `mcan/mcan_rx_dump.c` | 接收解析和数据回放排查入口 |
| CANopen | `canopen/canopen_sdo_probe.c` | 读写对象字典、NMT start/pre-op/reset、ECAN 从站复位 |
| CANopen | `canopen/canopen_heartbeat_probe.c` | 心跳监听和批量节点在线状态排查 |
| CiA402 helper | `cia402/cia402_mcan_transport.h` | MCAN/CiA402 帧转换和发送适配，demo 私有使用 |
| CiA402 | `cia402/cia402_position_watch_demo.c` | 只读实际位置，适合运动前验证驱动响应 |
| CiA402 | `cia402/cia402_single_axis_demo.c` | 单轴启动流程参考实现 |
| CiA402 | `cia402/cia402_10axis_demo.c` | 多轴启动流程和稳定性测试入口 |
| MIT CAN FD | `mit/mit_canfd_motor_demo.c` | MIT 8-byte packed command/feedback；量程和 CAN ID 均可配置 |

## MIT CAN FD 电机与 UCAN-Mini 模拟器

`igh_mit_canfd_motor_demo` 使用 MIT packed 8-byte command，并通过 CAN FD+BRS
发送。默认量程是常见 AK 系列量程，仅用于示例；接真实电机前必须按电机手册修改
`--p-min/--p-max`、速度、增益和力矩量程。默认命令 ID 为 `0x001`，反馈 ID 为
`0x101`，也都可以通过命令行修改。

可选的协议功能对照可编译 Rust SocketCAN 模拟器；它不属于 CMake 默认构建，
也不是 UCAN Rust 直连 USB 驱动或固件内应答器，不能替代默认链路的性能验收：

```sh
cd examples/mit/ucan_mit_simulator
cargo build --release
```

只配置 UCAN-Mini 对应的 CAN 接口，不要修改 EtherCAT 网口或管理网口：

```sh
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.875 \
  dbitrate 5000000 dsample-point 0.875 fd on
sudo ./target/release/ucan-mit-motor-simulator \
  --interface can0 --node 1 --command-id 0x001 --feedback-id 0x101
```

直接 IgH demo 会独占 EtherCAT master，运行前应停止 `ecan_socketcan` 等其它 master
使用者，然后直接启动默认DC 1 ms/shift 900 μs错周期闭环：

```sh
sudo ECAN_IGH_PRINT_RUNTIME_DIAG=1 ./build/igh_mit_canfd_motor_demo \
  --slave 0 --channel 0 --node 1 \
  --command-id 0x001 --feedback-id 0x101 \
  --cycles 10000 \
  --position 1.0 --kp 5.0 --kd 0.1 --max-missing 16
```

结果中的 `latency_cycles_*` 是由未回复命令深度估算的EtherCAT循环数；默认
N→N+1表示估算的周期差，不能证明物理闭环不超过1 ms；一请求多应答时不能仅用队列深度关联请求。
模拟器无丢帧时应保持稳定。可给模拟器加 `--drop-every N`
验证 demo 的丢回复判定，或加 `--reply-delay-us N` 验证延迟统计。

MIT 示例默认不发送使能指令，但普通控制帧仍可能影响已使能电机。只有确认机械安全、急停、节点、量程和零位以后，才显式
添加 `--enable-motor`；`--zero-position` 同样必须显式指定。程序退出时默认发送
disable，只有明确指定 `--keep-enabled` 才不发送。

CiA402 单轴、单轴 SDO 和多轴运动示例另有启动保护：必须显式传入 `--enable-motor`，
否则在申请 master 前返回参数错误。所有示例支持 `--help`；未知参数或非法数值不会被静默截断为其他通道/节点。

四通道错相的1 ms闭环使用默认 `igh_250us_four_phase_demo`。调度库隐藏IgH/PDO和
实时周期细节，依次服务MCAN0、1、2、3。三从站建议先从
RxPDO=128 B、TxPDO=256 B开始验证。API、相位约束和运行方法见
[`../docs/FOUR_PHASE_250US_PIPELINE.md`](../docs/FOUR_PHASE_250US_PIPELINE.md)。

CiA402 demo 的周期主路径保持扁平：demo 调用 MCAN port 收帧，逐帧交给 `igh_cia402_on_can_frame()` 更新轴状态；发帧时先调用 `igh_cia402_build_*_frame()` 构造 CANopen/CiA402 帧，再显式调用 MCAN port 发送。`igh_cia402_process_rx()`、`igh_cia402_send_target_step()` 等 convenience API 保留给简单应用和配置流程，不作为 demo 主循环的首选展示路径。

## 已合并的示例

原单独的 classic TX probe 已合并。它只验证“网关通道配置为 CAN FD-capable，但发送 classic CAN 帧”这一场景，现在由通用 TX probe 覆盖：

```sh
sudo ./build/igh_mcan_gateway_tx_probe \
  --classic-can \
  --gateway-canfd \
  --len 8
```

通用 TX 示例的预约发送配置：

```sh
sudo ./build/igh_mcan_gateway_tx_probe \
  --scheduled-tx \
  --schedule-offset-ns 100000 \
  --cycle-us 1000 \
  --rt-priority 80 \
  --spin-us 100
```

`mcan_gateway_tx_probe` 使用绝对时间周期，并在结束时打印 `master_period`。评估实时性时
先确认 `max_abs_err_us` 已收敛，再用真实 CAN 对端或分析仪判断 CAN 侧周期。测试报告
同时记录参数、样本数、min/p99/max、丢帧和乱序，不能只记录一次平均值。

## CiA402 预约组合

单轴和 10 轴 demo 支持四种运动期发送模式：

| 参数 | 目标 RPDO | CANopen SYNC |
| --- | --- | --- |
| 默认或 `--direct-tx` | 直接发送 | 直接发送 |
| `--scheduled-target` | 预约发送 | 直接发送 |
| `--scheduled-sync` | 直接发送 | 预约发送 |
| `--scheduled-tx` | 预约发送 | 预约发送 |

只要启用任一预约参数，demo 会自动配置 DC Sync0。RPDO 和 SYNC 都预约时，`--schedule-sync-offset-ns` 必须大于 `--schedule-target-offset-ns`。

10 轴 demo 的 `--motion-div N` 是目标帧发送降频参数。默认 `N=1`，每个 EtherCAT 周期发送一次目标 RPDO 和 SYNC；显式设置 `--motion-div 2` 时，CAN 目标帧周期会变成 EtherCAT 周期的 2 倍。
