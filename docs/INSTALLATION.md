# IgH 安装与环境检查

本文适用于第一次在 Linux 上构建并运行 ECAN IgH 示例。运行 EtherCAT 需要 IgH/EtherLab
master、实时或低延迟内核、专用以太网接口和管理员权限。

## 1. 前置条件

- C11 编译器、CMake 3.16+、Make 或 Ninja；
- 已编译安装的 IgH/EtherLab master；
- IgH 用户态头文件 `ecrt.h` 和库 `libethercat.so`；
- 标准 generic 驱动及其支持的 Linux 网卡驱动；
- 目标 ECAN 从站和正确的 ESI/EEPROM 配置。

先确认工具：

```sh
cc --version
cmake --version
ethercat version
```

任何命令不存在时先完成对应安装。不同 Linux 发行版的包名不同，本工程不假定固定安装
目录。

默认部署不要求修改 IgH 内核源码，也不使用定制 `direct_xmit` 驱动。
本包不提供补丁或内核模块；已有定制主站的设备不得在业务运行时直接替换驱动。
具体版本限制及验收要求见 [主站兼容性](MASTER_COMPATIBILITY.md)。

## 2. 定位 IgH 头文件和库

```sh
find <ethercat-root> -name ecrt.h -o -name libethercat.so
```

应得到一个头文件目录和一个实际库文件。若存在多个 IgH 版本，头文件、用户态库和已
加载的内核 master 必须来自同一套版本。

## 3. 构建

```sh
cmake -S . -B build \
  -DECRT_INCLUDE_DIR=<ethercat-root>/include \
  -DECRT_LIBRARY=<ethercat-root>/lib/libethercat.so \
  -DECAN_IGH_BUILD_DEMO=ON \
  -DECAN_IGH_WARNINGS_AS_ERRORS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

通过条件：CMake 和编译命令返回 0；测试全部通过；`build/` 下生成示例程序。若只验证
codec，可把 `ECAN_IGH_BUILD_DEMO` 设为 `OFF`，但该结果不能证明 IgH 可运行。

## 4. 运行时库

如果启动程序提示找不到 `libethercat.so`，临时验证可使用：

```sh
export LD_LIBRARY_PATH=<ethercat-root>/lib:${LD_LIBRARY_PATH}
```

长期部署应使用系统库配置或服务文件明确配置库路径，不依赖交互式 Shell 的临时环境。

## 5. 网卡和 master

先列出接口和 master 状态：

```sh
ip -brief link
sudo ethercat master
sudo ethercat slaves
```

确认哪个接口专用于 EtherCAT，哪个接口用于远程管理。后续所有绑定、IRQ 和驱动操作只
针对 EtherCAT 专用接口；不确定接口用途时停止操作。正常结果应显示 master Active、
链路 UP、从站数量正确且无持续增加的 lost frame。

## 6. 大 PDO 支持

当前 ECAN-Lite profile 的单方向 PDO 为 128～1392 B，按 16 B 调节；配置前应读取
设备能力并核对固件支持范围。该容量不要求增加超大单 FMMU 分段功能。
双向容量或多个从站的总量超过单个以太网帧时，主站可能生成多个 datagram/以太网帧，
不能将总量超限与单方向 FMMU 超长混同。容量合法不保证给定周期下满载稳定，
仍须核对完整 WKC、CAN 数量、字节、顺序和延迟。

## 7. 实时基础配置

开始性能测试前：

1. 使用性能调度器并避免 CPU 深度休眠造成大抖动；
2. 周期线程使用 `SCHED_FIFO` 和 `mlockall()`；
3. 周期线程、EtherCAT 线程和网卡 IRQ 规划到明确 CPU；
4. 停止会周期性抢占 CPU 或占用 EtherCAT master 的服务；
5. 先跑空载 60 秒确认 WKC 和周期，再接入真实 CAN 负载。

操作细节见 [CPU 与 IRQ 配置](CPU_AFFINITY.md)。优化前后都保存 `ethercat master`、
周期统计和 IRQ 分布，避免无法回退。

## 8. 常见失败

| 现象 | 优先检查 |
| --- | --- |
| 找不到 `ecrt.h` | `ECRT_INCLUDE_DIR` 是否直接包含头文件 |
| 链接失败 | `ECRT_LIBRARY` 是否指向匹配的实际库 |
| 找不到共享库 | 动态库搜索路径和架构是否正确 |
| `Failed to request master` | master 未加载、权限不足或已被占用 |
| 无从站 | 专用接口、驱动、线缆、电源和 EEPROM |
| PREOP 不能到 OP | ESI、PDO/SM/FMMU、启动 SDO 和 AL code |
| 周期长尾 | 调度优先级、CPU/IRQ、频率和后台服务 |

遇到硬件异常时继续阅读 [故障排查](TROUBLESHOOTING.md)。
