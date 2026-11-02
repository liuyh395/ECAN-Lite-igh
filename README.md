# ECAN IgH 主站库与示例

本目录提供 Linux IgH/EtherLab 主站适配层、ECAN MCAN/CAN FD 编解码和可直接运行的
示例。Windows 和 macOS 可以阅读源码、构建纯 C codec 测试；实时 EtherCAT 运行需要
Linux、IgH master、专用网卡和相应系统权限。

默认使用标准 IgH 和标准 generic 驱动。本示例包不包含主站补丁，不修改或自动安装
内核模块。DC 相位、并发 SDO 和启停行为的版本边界见 [主站兼容性](docs/MASTER_COMPATIBILITY.md)；
能够编译不代表全部运行场景已通过验证。

独立 IgH 包保留 `include/`、`src/`、`examples/`、`tests/`、`docs/` 和构建入口。
仅纳入明确白名单中的 API、应用示例及组件测试。编译及正常使用不依赖辅助脚本；
固件专用满载程序、时序注入、稳定性验收、SII 修复及内部台架脚本不随包提供。
固件源码包中的 IgH 副本进一步排除全部测试文件；其构建使用 `BUILD_TESTING=OFF`。
本文涉及组件测试的命令仅适用于独立 IgH 包。
请按本文直接构建并运行所选示例，接入电机前确认示例会发送的命令及安全条件。

完整阅读顺序与专题分类见 [文档导航](docs/README.md)。API 定义以 [API.md](API.md) 为入口；
本文保留快速上手步骤，专题指南不重复维护另一套接口定义。

## 获取版本包

自 2.23 起，独立 IgH 示例 ZIP 同时作为 GitHub Release 附件提供：

| 标签类型 | IgH 附件示例 | 发布范围 |
| --- | --- | --- |
| 测试 `t2.23` | `ECAN-Lite-t2.23-igh-demo.zip` | GitHub 预发布；不更新官网正式下载 |
| 正式 `v2.23` | `ECAN-Lite-2.23-igh-demo.zip` | GitHub Release 与官网同步提供同一归档 |

以上正式标签名称用于说明命名规则，不表示该正式版本已发布。
官网与同一正式 tag 的 Release 附件使用相同 ZIP，SHA256 一致。
请记录使用的 tag；示例包版本不等于固件在特定负载下已通过验收。

## 1. 十分钟开始

### 1.1 编译

先准备 IgH 的 `ecrt.h` 和 `libethercat.so`，然后在本目录执行：

```sh
cmake -S . -B build \
  -DECRT_INCLUDE_DIR=<ethercat-root>/include \
  -DECRT_LIBRARY=<ethercat-root>/lib/libethercat.so \
  -DECAN_IGH_WARNINGS_AS_ERRORS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

若只想验证 PDO codec 和 CMake，不连接 IgH：

```sh
cmake -S . -B build -DECAN_IGH_BUILD_DEMO=OFF
cmake --build build
ctest --test-dir build --output-on-failure
```

完整依赖、库加载和网卡检查见 [安装与环境检查](docs/INSTALLATION.md)。

### 1.2 确认主站和从站

```sh
sudo ethercat master
sudo ethercat slaves
```

必须确认：EtherCAT 使用的是专用接口；远程管理接口没有被修改；从站数量和产品身份
符合实际连接；开始实时测试前没有其他程序占用 master。

### 1.3 选择配置入口

| 应用 | EtherCAT 周期 | 默认参数 | 第一个命令 |
| --- | ---: | --- | --- |
| 四通道固定相位 | 250 us | MCAN0→1→2→3，每通道 1 ms | `sudo ./build/igh_250us_four_phase_demo` |
| DC 1 kHz 收发 | 1 ms | Sync0 shift 900 us；闭环周期需实测 | `./build/igh_simple_loop --help` |

两种配置的统一定义见 [实时 Profile](docs/REALTIME_PROFILES.md)。先确认示例默认拓扑和发帧内容，
按实际接线设置从站数、PDO、CAN 波特率和帧数，再在无运动风险的环境建立基线。

## 2. 推荐使用路径

SM 模式的在途报文保护、迟到补收、发送节拍及超时处理见
[SM 周期收包所有权与发送节拍](docs/SM_RECEIVE_OWNERSHIP.md)。应用应检查周期 API
返回值；数据完整性与周期实时性应分别验收。

实验性 Sync0/Sync1 双相位调度的 API、压力测试参数和时序定义见
[Sync0 发送、Sync1 发布的验证配置](docs/ADAPTER_USAGE.md#sync0-发送sync1-发布的验证配置)。
该配置需要对应的新调度固件，不改变现有 SM 默认值，也不代表已经完成发布资格验证。

### 2.1 只验证 EtherCAT

```sh
sudo ./build/igh_raw_pdo_dump
```

检查从站能否进入 OP、WKC 是否完整以及 PDO 映射是否与固件一致。此步骤不能证明 CAN
链路正常。

### 2.2 验证 SM 250 us 四相流水

```sh
sudo ./build/igh_250us_four_phase_demo \
  --sm --slaves 1 --cycles 40000 --closed-loop --frames-per-slot 1
```

物理回环时按接线设置 `--rx-channel-xor`；真实电机通常从同一 MCAN 通道回复，不需要
通道异或。详细说明见 [SM 250 us 四相指南](docs/FOUR_PHASE_250US_PIPELINE.md)。

### 2.3 验证 DC 1 ms 错周期闭环

```sh
sudo ./build/igh_simple_loop
```

请求在周期 N 下发，目标是在 N+1 取得对应回复，但该结果取决于固件调度、相位和负载。
N+1 不是物理延时不超过 1 ms 的证明。详细说明见
[DC 1 ms 指南](docs/DC_1MS_GUIDE.md)。

### 2.4 验证 CAN FD 网关

```sh
sudo ./build/igh_mcan_gateway_tx_probe \
  --canfd --brs --len 16 --cycle-us 1000 --count 10000
```

从一帧、一个通道开始，确认 ID、DLC、payload 和序号，再增加帧数与通道。常见组合和
结果判定见 [CAN/CAN FD 实用示例](docs/CANFD_GATEWAY_COOKBOOK.md)。

### 2.5 电机示例

```sh
sudo ./build/igh_mit_canfd_motor_demo --help
sudo ./build/igh_cia402_single_axis_demo --help
```

CiA402 单轴及多轴示例未传 `--enable-motor` 时在申请主站前退出；MIT 示例默认不发送使能和写零位命令，
但仍可发送普通控制帧，不能据此认为对已使能电机没有影响。运动前必须确认 Node ID、量程、
机械限位、急停和供电。CiA402 说明见 [CiA402 使用指南](docs/CIA402_USAGE.md)。

## 3. 代码分层

| 层 | 主要文件 | 职责 |
| --- | --- | --- |
| IgH Adapter | `include/igh_adapter.h`、`src/igh_adapter*.c` | master/domain、PDO、DC、启动 SDO、OP/WKC |
| 250 us Pipeline | `include/igh_250us_pipeline.h`、`src/igh_250us_pipeline.c` | SM 四相调度与每通道 1 ms 控制周期 |
| MCAN Port | `include/igh_mcan_port.h`、`src/igh_mcan_port.c` | 通道配置、CAN/CAN FD 收发 |
| PDO Codec | `include/igh_mcan_pdo_codec.h`、`src/igh_mcan_pdo_*.c` | 动态 PDO 写入、解析、CRC 和错误信息 |
| 应用协议 | `examples/mit/`、`examples/cia402/` | MIT CAN FD、CANopen/CiA402 示例 |

业务代码优先调用最高层 API。只有需要自定义 PDO 字节流时才直接使用 Adapter raw
buffer；不要让业务层依赖 `src/` 中的私有结构。

## 4. 文档导航

| 文档 | 何时阅读 |
| --- | --- |
| [安装与环境检查](docs/INSTALLATION.md) | 第一次在 Linux 构建和连接 IgH |
| [实时 Profile](docs/REALTIME_PROFILES.md) | 选择 SM 或 DC 默认模式 |
| [SM 250 us 四相指南](docs/FOUR_PHASE_250US_PIPELINE.md) | 四通道固定相位、每通道 1 ms |
| [DC 1 ms 指南](docs/DC_1MS_GUIDE.md) | 1 kHz N→N+1 闭环 |
| [DC 分相交换与同周期回收](docs/DC_PHASED_EXCHANGE.md) | 可选 LRW / LWR+LRD、准备余量和失败停止语义 |
| [CAN/CAN FD 实用示例](docs/CANFD_GATEWAY_COOKBOOK.md) | 从一帧扩展到多通道负载 |
| [Adapter 使用](docs/ADAPTER_USAGE.md) | raw PDO、OP/WKC 和生命周期 |
| [MCAN Port 使用](docs/MCAN_PORT_USAGE.md) | 通道、波特率、直接/预约发送 |
| [CiA402 使用](docs/CIA402_USAGE.md) | CANopen 电机配置和运动安全 |
| [API 参考](API.md) | 二次开发接口、参数和返回码 |
| [CPU 与 IRQ](docs/CPU_AFFINITY.md) | 周期抖动或长尾排查 |
| [测试与验收](docs/TEST_AND_ACCEPTANCE.md) | 统一测试顺序、实时定义和结果字段 |
| [故障排查](docs/TROUBLESHOOTING.md) | 不能 OP、WKC、PDO 或 CAN 异常 |
| [主站兼容性](docs/MASTER_COMPATIBILITY.md) | 标准驱动、PDO 容量及 DC/SDO/启停限制 |

## 5. 通过条件

功能和实时测试至少记录：从站数、PDO 大小、Profile、周期/shift、CAN 类型和波特率、
每通道帧数、payload、预热与统计时长、WKC、AL、丢失、重复、乱序、数据错误和延迟
分布。稳定期必须 WKC mismatch 为 0；负载恢复后设备应自动恢复 OP 和数据流。

空 PDO、未接线或固定伪回包不能替代真实 CAN/CAN FD 链路。测试结果模板见
[`docs/TEST_AND_ACCEPTANCE.md`](docs/TEST_AND_ACCEPTANCE.md)。

## 6. 集成原则

- 默认配置只使用 SM 250 us 四相流水和 DC 1 ms/900 us 两个 Profile；
- 在添加版本相关启动 SDO 前读取 `0xF000:05/:06/:16`：只有公开版本规范化后
  `>=0x0200` 才属于固件 V2，并继续按 bit13/bit7 分别确认 V2 头和 16B 细分能力；
- PDO 大小采用“低 5 bit 表示 128 B 基础块、高 3 bit 表示 16 B 尾块”的兼容编码，
  默认 `0x02=256 B`；高 3 bit 是固件 V2 才支持的扩展，必须同时匹配固件、ESI/FMMU 和主站配置；
- PDO codec和MCAN配置默认使用V1＋NO_CRC16（`OptionFlags=0x04`），5E/6E头长分别为24B/32B；
  需要CRC时主动选择V1 `0x00`或V2 `0x20`；
  只有确认固件2.x和bit13后，应用才通过API显式选择V2的16B/20B紧凑头；
- 周期线程使用绝对时间、`SCHED_FIFO`、内存锁定和明确 CPU/IRQ 规划；
- WKC 或 OP 异常时保持安全输出，不继续推进电机目标；
- CAN FD 5 Mbit/s 等高速数据段先验证采样点、TDC 和物理层；
- 过载允许受控丢业务帧，但不能导致进程死锁、设备故障或只能断电恢复。
