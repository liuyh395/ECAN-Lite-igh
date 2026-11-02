# IgH 使用文档导航

本目录是 ECAN IgH 独立代码包的使用文档。适用于随包提供的 Adapter、PDO codec、
MCAN Port 和示例；实际运行还取决于固件功能、IgH 版本、网卡和实时系统配置。
默认采用标准 IgH 与标准 generic 驱动，已知限制见主站兼容性说明。

## 1. 首次接入

| 顺序 | 文档 | 解决的问题 |
| --- | --- | --- |
| 1 | [代码包首页](../README.md) | 编译入口、设备检查、选择示例 |
| 2 | [安装与环境检查](INSTALLATION.md) | IgH 头文件与库、网卡、权限和运行环境 |
| 3 | [主站兼容性](MASTER_COMPATIBILITY.md) | 标准主站的适用范围、已知限制和验证要求 |
| 4 | [实时 Profile](REALTIME_PROFILES.md) | SM/DC 默认参数、周期和相位的区别 |
| 5 | [示例目录](../examples/README.md) | 按接线和业务选择程序，确认是否发送运动命令 |

CiA402 运动示例要求显式指定 `--enable-motor`。该选项不能替代机械安全措施、急停或
驱动器限位。其他 CAN 示例也可能向已经使能的设备发送控制帧，运行前须确认帧内容。

## 2. 应用集成与业务协议

| 文档 | 内容 |
| --- | --- |
| [API 参考](../API.md) | 接口分层、配置、返回值与周期调用 |
| [Adapter 使用](ADAPTER_USAGE.md) | 主站初始化、PDO 域与交换方式 |
| [MCAN Port 使用](MCAN_PORT_USAGE.md) | CAN 帧收发接口及处理规则 |
| [CAN FD 网关配置](CANFD_GATEWAY_COOKBOOK.md) | 波特率、帧格式与网关配置示例 |
| [CiA402 使用](CIA402_USAGE.md) | 电机示例、节点配置和运行条件 |
| [移植说明](PORTABILITY.md) | 平台依赖与代码移植边界 |

## 3. 实时调度与性能配置

| 文档 | 内容 |
| --- | --- |
| [SM 250 μs 四相流水](FOUR_PHASE_250US_PIPELINE.md) | 四通道分时轮转；与每周期全通道发送的区别 |
| [SM 收包所有权与节拍](SM_RECEIVE_OWNERSHIP.md) | 在途保护、迟到处理、超时和完整 WKC 检查 |
| [DC 1 ms 指南](DC_1MS_GUIDE.md) | 周期与相位选择、闭环批次的判定 |
| [DC 分相交换](DC_PHASED_EXCHANGE.md) | 合并/分离读写、输入更新时刻和固件前置条件 |
| [CPU 与 IRQ 配置](CPU_AFFINITY.md) | 调度与中断配置方法及验证要求 |

周期参数不是闭环延迟保证。N+k 应按请求和应答的批次统计；WKC 完整不能替代 CAN 帧的
完整性、内容与顺序检查。可选 Sync0/Sync1 调度须确认对应固件支持，不由 PDO V2 自动推导。

## 4. 验证与故障定位

- [测试与验收](TEST_AND_ACCEPTANCE.md)：功能、数据完整性、实时性及异常恢复的判定方法。
- [故障排查](TROUBLESHOOTING.md)：从构建、主站状态到 CAN 业务逐层定位。
- [主站兼容性](MASTER_COMPATIBILITY.md)：遇到相位、并发 SDO 或启停问题时先核对版本边界。

本文档提供配置方法和验证条件，不以历史台架结果代替当前设备的验收记录。
