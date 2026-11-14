# 四路 CAN / 12 电机整机发送层

Docker 中的 ROS 2 Humble 如何订阅整机 MIT 命令、通过宿主机 IgH 下发并回传反馈，见 [ROS 2 Humble Docker 接入指南](ROS2_Humble_Docker_接入指南.md)。该文档给出新增桥接程序的实现方案、通信协议与逐级联调步骤。

新增的 `dm_robot.h/.c` 提供整机接口，原有 `dm_motor` 示例和底层文件保持不变。它只负责把已算好的 MIT 五元组编码为达妙 Classic CAN 8 字节帧，并通过 EtherCAT2CAN 的 `igh_mcan_port` 按周期发送；不负责步态计算或上层数据传输。

## 默认映射

| 电机索引 | CAN 通道 | 电机 ID | 控制帧 ID |        示例反馈帧 ID |
| -------: | -------: | ------: | --------: | -------------------: |
|     0–2 |     CAN0 |    1–3 |      1–3 | `0x101`–`0x103` |
|     3–5 |     CAN1 |    1–3 |      1–3 | `0x104`–`0x106` |
|     6–8 |     CAN2 |    1–3 |      1–3 | `0x107`–`0x109` |
|    9–11 |     CAN3 |    1–3 |      1–3 | `0x10A`–`0x10C` |

反馈 ID 只是示例，需要按实际电机反馈 CAN ID 修改。可修改每个 `dm_robot_motor_config_t` 的通道、ID、标准/扩展帧标志和 MIT 量程。CAN FD mask 固定为 `0`。

## 集成顺序

1. 用 `dm_robot_default_config()` 生成配置并逐个改成实际电机参数，然后调用 `dm_robot_init()`。
2. 在 MCAN 默认配置完成后调用 `dm_robot_apply_classic_can_config()`，设置该从站的四路 Classic CAN 和波特率；它不代替 IgH 主站本身的配置。
3. 上层将 12 个 `{position, velocity, kp, kd, torque}` 填进数组，调用 `dm_robot_submit_all()`；也可逐个调用 `dm_robot_submit()`。索引为 `通道 * 3 + 槽位(0..2)`。
4. EtherCAT 实时周期中调用 `dm_robot_cycle()`。函数在同一个 MCAN 周期内读取反馈、发送 12 条最新命令并结束周期。上层控制线程与周期线程并发时，应由集成方用双缓冲或短临界区交接命令；该实时路径没有普通互斥锁。

使能、失能、设零和清错使用 `dm_robot_send_control_all()`。反馈超时及故障状态机暂未实现。原有示例应用保持不变；整机库和 USB2CAN 联调程序分别作为独立 CMake 目标加入。本轮按要求不编译。

## 使用教程

以下示例展示调用顺序。上层步态或运动控制代码负责计算 `commands[12]`；本模块只缓存命令、编码 MIT 帧并通过 EtherCAT2CAN 发送。

### 1. 初始化电机配置和整机对象

```c
#include "dm_robot.h"

dm_robot_motor_config_t motor_configs[DM_ROBOT_MOTOR_COUNT];
dm_robot_t robot;
size_t slave_pos = 0U;
uint32_t can_bitrate = 1000000U;

dm_robot_default_config(motor_configs);

/* 按实际接线和电机文档修改每个 motor_configs[i]：
 * channel、motor_id、command_id、feedback_id、extended_id 和 limits。
 */
if (dm_robot_init(&robot, slave_pos, motor_configs) != 0) {
    /* 配置无效，处理错误 */
}
```

默认 12 个数组索引依次对应 CAN0 到 CAN3，每路三个电机。默认反馈 ID 只是占位示例；确认达妙电机的控制帧 ID、反馈帧 ID 和电机内部 ID 后再修改。`limits` 必须与电机型号及 MIT 协议量程一致。

### 2. 配置 EtherCAT2CAN 的 Classic CAN 通道

先按项目原有流程创建 IgH 和 MCAN 默认配置，再为所用从站应用四路 Classic CAN 配置。`dm_robot_apply_classic_can_config()` 会将 CAN0～CAN3 全部启用、CAN FD mask 设为 0，并应用波特率：

```c
ecan_igh_config_t igh_config;
igh_mcan_port_config_t mcan_config;

ecan_igh_default_config(&igh_config);
igh_mcan_port_default_config(&mcan_config);

/* 还应按项目要求设置从站数量、EtherCAT 周期等参数。 */
if (dm_robot_apply_classic_can_config(&igh_config, &mcan_config,
                                      slave_pos, can_bitrate) != 0) {
    /* MCAN 配置失败，处理错误 */
}
// 这里需要配置pdo大小，dc1ms模式，可参考dmmotordemo
/* 后续沿用项目现有流程：初始化 EtherCAT 主站，
 * 初始化 igh_mcan_port，并等待从站进入 OP 状态。
 */
```

### 3. 提交上层计算出的 MIT 指令

一个指令包含位置、速度、`kp`、`kd` 和力矩。批量接口要求 12 个命令都为有限数值；检查通过后才会整体更新缓存：

```c
dm_motor_command_t commands[DM_ROBOT_MOTOR_COUNT];

/* 在这里由已有的上层控制逻辑填充 commands[0..11]。 */
if (dm_robot_submit_all(&robot, commands) != 0) {
    /* 输入为空或包含 NaN / 无穷大，处理错误 */
}
```

也可以只更新某一个电机。索引公式为 `channel * 3 + 电机槽位`，槽位范围为 0～2：

```c
dm_motor_command_t one_command = {
    .position = 0.0F,
    .velocity = 0.0F,
    .kp = 20.0F,
    .kd = 1.0F,
    .torque = 0.0F
};

/* index=0 是 CAN0 的第一个电机；index=3 是 CAN1 的第一个电机。 */
if (dm_robot_submit(&robot, 0U, &one_command) != 0) {
    /* 参数错误，处理错误 */
}
```

提交接口只校验数值是否有限并保存 MIT 参数，不发送 CAN 帧，也不限制超出配置量程的有限数值；编码时由协议打包函数按量程裁剪到边界。

### 4. 在 EtherCAT 周期中收发

完成 EtherCAT 主站和 `igh_mcan_port` 初始化并进入 OP 后，在已有的实时循环中调用 `dm_robot_cycle()`。它会开始 MCAN 周期、读取反馈、发送当前缓存的 12 条 MIT 控制帧并结束周期：

```c
while (running) {
    size_t received_count = 0U;

    /* 如果控制器与本循环是同一线程，可在此先更新 commands，
     * 再执行周期收发。上层控制数据的来源由集成代码决定。
     */
    if (dm_robot_cycle(&robot, &port, &received_count) != IGH_ECAN_OK) {
        /* 周期收发失败，按整机策略处理 */
        break;
    }

    /* robot.motors[i].feedback / has_feedback 可用于读取已解析反馈。 */
}
```

周期时间由调用方的 EtherCAT 实时循环控制。若上层控制线程与 EtherCAT 周期线程不同，需由集成方通过双缓冲或短临界区交接整组命令，避免两个线程同时读写 `robot.motors[i].command`；本模块不加锁。

### 5. 发送使能或其他控制字

控制字接口会自行开始并结束一个 EtherCAT/MCAN 周期，因此不要在另一个活动周期中调用：

```c
if (dm_robot_send_control_all(&robot, &port,
                              DM_MOTOR_CONTROL_ENABLE) != IGH_ECAN_OK) {
    /* 使能帧发送失败 */
}
```

可传入 `DM_MOTOR_CONTROL_DISABLE`、`DM_MOTOR_CONTROL_ZERO_POSITION` 或 `DM_MOTOR_CONTROL_CLEAR_ERROR`。实际使能顺序、安全联锁和急停策略由整机控制程序负责。当前模块尚未实现反馈超时检测及故障状态机。

### 6. 将新模块加入工程

工程使用仓库 CMake 时，`dm_robot` 已作为独立库目标加入。调用它的应用链接 `dm_robot` 即可；该库会传递依赖达妙 MIT 协议库和 `igh_mcan_port`。若使用自有构建脚本，需将 `dm_motor/dm_robot.c` 加入编译，并确保包含 `dm_motor`、`include` 目录且链接 `dm_motor_protocol` 与 EtherCAT2CAN 端口库。

## USB2CAN 联调测试

新增的 `dm_robot_usb2can_test` 是无真实电机时的总线联调程序：树莓派上的 EtherCAT2CAN 使用仓库官方的 `ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE` profile，按 1 ms 周期向 CAN0～CAN3 持续发送固定 MIT 帧；电脑通过 USB2CAN 可读取这些帧，也可向总线发送模拟达妙反馈帧。程序启动后先执行 32 个 warm-up 周期，再进入打印和正式测试。程序打印收到帧的原始 ID/DLC/数据，并对匹配的反馈帧显示解析值。

测试固定 MIT 指令为 `position=1.0, velocity=2.0, kp=10.0, kd=0.5, torque=1.0`。程序启用四路通道，但只有连接 USB2CAN 的那路能在该 USB2CAN 上看到。默认通道映射如下：

| 通道 | 电机 ID | 控制帧 CAN ID | 模拟反馈 CAN ID |
|---|---|---|---|
| CAN0 | 1、2、3 | `0x001`、`0x002`、`0x003` | `0x101`、`0x102`、`0x103` |
| CAN1 | 1、2、3 | `0x001`、`0x002`、`0x003` | `0x104`、`0x105`、`0x106` |
| CAN2 | 1、2、3 | `0x001`、`0x002`、`0x003` | `0x107`、`0x108`、`0x109` |
| CAN3 | 1、2、3 | `0x001`、`0x002`、`0x003` | `0x10A`、`0x10B`、`0x10C` |

反馈 ID 是本测试程序里的示例值。测试前确认 USB2CAN 与目标 CAN 口波特率均为 1 Mbps、CAN_H/CAN_L 正确连接。CAN 总线两端应各有 120 Ω 终端电阻；如果只有一个，可能出现收发不稳定，建议补齐另一个终端。总线上不要接入真实电机，因为程序会持续发送非零 MIT 指令。

在仓库构建目录生成 `dm_robot_usb2can_test` 后，在树莓派 EtherCAT 主站环境运行：

```sh
sudo ./dm_robot_usb2can_test --slave 0 --bitrate 1000000
```

默认持续运行，按 `Ctrl-C` 退出。发送帧每 100 个周期打印一次（约每 100 ms），收到的帧则立即打印；如果想每周期打印 12 条 TX 帧，可用 `--log-every 1`，但高频终端输出会影响 1 ms 周期的调度。`--cycles N` 可限制运行周期数。该程序需要从站处于 OP 状态，并使用已有的 IgH EtherCAT 运行环境。

程序退出时会输出 `rx_stats` 分层统计：`raw_frames` 是 EtherCAT2CAN 实际交给应用的原始帧数，`accepted` 是成功解析并计入电机反馈的数量；`format_rejected`、`can_id_zero`、`can_id_unmatched`、`motor_id_mismatch` 和 `decode_errors` 分别表示格式、ID 或数据解析阶段被排除的数量。`no_space` 表示某个 EtherCAT 周期接收缓冲区不足，`receive_errors`、`cycle_begin_errors`、`cycle_end_errors` 表示周期 API 错误。判断丢帧时先比较 `raw_frames` 与 USB2CAN 的发送数，再看 `accepted` 与 `raw_frames` 的差值，不能只看某个 `motor[*].feedback_frames`。

要模拟 CAN0 上电机 1 的反馈，在电脑 CAN 上位机选择 **标准帧**、ID `0x101`、DLC `8`，发送数据：

```text
01 80 00 80 08 00 1E 28
```

这是按当前默认量程构造的位置/速度/力矩约为零的示例反馈，电机 ID 为 1，温度字节为 30 和 40。程序应显示原始帧，并解析出 motor ID、错误码、位置、速度、力矩和温度。测其他电机时，反馈 CAN ID 改成映射表对应值，数据首字节低四位改成对应电机 ID（1、2 或 3）；其他反馈数据按该协议格式填写。该测试程序不连接真实电机，也不调用电机使能控制字。

下表给出 12 个电机可直接发送的模拟反馈帧。上位机均选择**标准帧、DLC 8**；每行的数据都是零位置/速度/力矩附近的反馈值，温度字段示例为 30°C 和 40°C。根据 USB2CAN 当前接入的通道，发送该通道对应的三行即可：

| 通道 | 电机 ID | CAN ID | 数据 |
|---|---:|---:|---|
| CAN0 | 1 | `0x101` | `01 80 00 80 08 00 1E 28` |
| CAN0 | 2 | `0x102` | `02 80 00 80 08 00 1E 28` |
| CAN0 | 3 | `0x103` | `03 80 00 80 08 00 1E 28` |
| CAN1 | 1 | `0x104` | `01 80 00 80 08 00 1E 28` |
| CAN1 | 2 | `0x105` | `02 80 00 80 08 00 1E 28` |
| CAN1 | 3 | `0x106` | `03 80 00 80 08 00 1E 28` |
| CAN2 | 1 | `0x107` | `01 80 00 80 08 00 1E 28` |
| CAN2 | 2 | `0x108` | `02 80 00 80 08 00 1E 28` |
| CAN2 | 3 | `0x109` | `03 80 00 80 08 00 1E 28` |
| CAN3 | 1 | `0x10A` | `01 80 00 80 08 00 1E 28` |
| CAN3 | 2 | `0x10B` | `02 80 00 80 08 00 1E 28` |
| CAN3 | 3 | `0x10C` | `03 80 00 80 08 00 1E 28` |

例如 USB2CAN 接在 CAN2 时，发送 ID `0x108`、数据 `02 80 00 80 08 00 1E 28`，程序应将其识别为 CAN2 电机 2（数组索引 7）。

下面再给出两个**位置、速度、力矩均非零**的反馈例子，均以 CAN0 电机 1 为例：上位机选择标准帧、CAN ID `0x101`、DLC `8`。

| 示例 | 发送数据 | 对应的目标值 | 按当前协议量化后程序解析值 |
|---|---|---|---|
| A | `01 8A 3D 8E 39 55 1E 28` | 位置 `1.0 rad`，速度 `5.0 rad/s`，力矩 `3.0 Nm` | 位置约 `1.00004 rad`，速度 `5.00000 rad/s`，力矩约 `3.00220 Nm` |
| B | `01 66 66 63 95 55 1E 28` | 位置 `-2.5 rad`，速度 `-10.0 rad/s`，力矩 `-6.0 Nm` | 位置约 `-2.50000 rad`，速度约 `-9.98901 rad/s`，力矩约 `-6.00000 Nm` |

两帧的首字节 `0x01` 表示电机 ID 为 1、错误码为 0；末两字节 `0x1E 0x28` 表示示例温度 30°C 和 40°C。解析值与目标值可能有少量差别，这是 MIT 反馈字段只有有限位数、需要量化编码造成的。其他通道或电机可以使用同一组数据，只需按前表替换反馈 CAN ID，并把首字节低四位改为对应电机 ID。

## 真实达妙电机测试：dm_robot_motor_test

新增 `dm_robot_motor_test.c`，沿用 USB2CAN 测试已验证的 **Classic CAN、1 Mbit/s、DC 1 ms cross-cycle、Sync0 shift 900 us、PDO multiplier 3、32 周期 warmup**。原 `dm_robot_usb2can_test.c` 保留，用于无电机抓帧测试；其非零固定指令不直接用于本实机程序。

树莓派 EtherCAT 口连接模块 IN，电机接相应 CAN 口，供电按电机手册，CAN_H/CAN_L/信号地接对，总线两端按要求安装终端电阻。电机须已配置为 MIT 模式及 1 Mbit/s；程序不会自动更改电机内部的工作模式、ID 或量程。首次测试固定电机、卸载或架空腿部，并保留可直接切断电机动力电源的手段。

先确认实际电机型号的 MIT 量程，必要时修改本程序 `dm_robot_default_config(configs)` 后面的 `configs[i].limits`。当前默认位置 ±12.5 rad、速度 ±45 rad/s、力矩 ±18 Nm、Kp 0..500、Kd 0..5，必须与电机配置一致。程序对超出配置量程的输入直接报参数错误。

在树莓派 `~/ECAN-Lite-igh` 内新增目标并构建（本轮没有在 Windows 上编译）：

```sh
cmake -S . -B build \
  -DECRT_INCLUDE_DIR=/opt/etherlab/include \
  -DECRT_LIBRARY=/opt/etherlab/lib/libethercat.so \
  -DECAN_IGH_BUILD_DEMO=ON
cmake --build build --target dm_robot_motor_test -j"$(nproc)"
```

### 1. 单电机阻尼测试

例如 CAN0 电机 1，控制 ID `0x001`、反馈 ID `0x101`：

```sh
sudo ./build/dm_robot_motor_test --motor 0 --enable-motor \
  --command-id 0x001 --feedback-id 0x101 --motor-id 1 --cycles 5000
```

默认指令为 `position=0 velocity=0 kp=0 kd=0.2 torque=0`。Kp 为 0，不会以零位置作为位置闭环目标；Kd 提供小阻尼，用于先验证使能、指令发送和反馈。MIT 量化后零附近会有小偏差。阻尼测试不是位置保持，承重关节可能下落，因此应在卸载状态进行。

默认持续运行，可按 Ctrl-C 停止；传入 `--cycles N` 才运行固定数量的 MIT 周期。启动时显示所选电机的 CAN 通道、控制/反馈 ID 和固定 MIT 8 字节发送模板；每 1000 周期显示反馈计数及最新解析的位置、速度、力矩、状态 nibble 和温度。`--log-every 100` 可改为每约 100 ms 显示一次；本程序不逐帧打印，以减少日志对周期的影响。

不加 `--enable-motor` 时为**只接收模式**：不会发使能、MIT 指令或失能。部分达妙电机只在接收命令后回复，只接收模式中没有反馈不一定表示通信故障。

反馈严格按从站、CAN 通道、反馈 CAN ID、标准/扩展标志、8 字节长度及数据低四位 motor ID 匹配，**不沿用 USB2CAN 测试的 ID=0 诊断兜底**。若 `feedback_frames=0`，先核对实际反馈 ID、内部 ID、工作模式和接线。状态 nibble 原样显示为 `state`；某些型号的 `0x1` 表示使能，不能一律视为故障。

### 2. 指定实际 MIT 指令

五个命令字段都可由参数指定。例如在已确认当前位置接近 `0.1 rad`、方向与机械行程允许时，低增益测试该绝对位置：

```sh
sudo ./build/dm_robot_motor_test --motor 0 --enable-motor \
  --position 0.1 --velocity 0 --kp 1 --kd 0.2 --torque 0 --cycles 3000
```

`--position` 单位为 rad，`--velocity` 为 rad/s，`--torque` 为 Nm（需符合电机协议的力矩定义）。位置是电机协议的**绝对位置**，不是相对当前角度的偏移；启用非零 Kp 前先读取实际位置。本程序发送固定五元组，不生成步态或运动轨迹。

### 3. 选择其他电机或全部电机

`--motor` 是配置数组索引，不是 CAN ID。默认索引与布线对应如下：

| CAN 通道 | 电机 1 索引 | 电机 2 索引 | 电机 3 索引 |
|---|---:|---:|---:|
| CAN0 | 0 | 1 | 2 |
| CAN1 | 3 | 4 | 5 |
| CAN2 | 6 | 7 | 8 |
| CAN3 | 9 | 10 | 11 |

例如 CAN3 电机 3：

```sh
sudo ./build/dm_robot_motor_test --motor 11 --enable-motor \
  --command-id 0x003 --feedback-id 0x10C --motor-id 3
```

确认全部 12 台电机映射和量程后，可执行：

```sh
sudo ./build/dm_robot_motor_test --all --enable-motor --cycles 5000
```

`--all` 每个周期发送 12 条指令，每路 3 条，全部采用同一个命令五元组。它用于整体通信验证，不是四足站立或步态程序。全部模式不接受 `--command-id`、`--feedback-id`、`--motor-id` 覆盖，应在 `configs[]` 中逐台填写实际参数。单电机模式只对选定电机发送控制字和 MIT 指令。

### 4. 退出与统计边界

正常到期、Ctrl-C/SIGTERM 或运行中通信 API 报错时，默认对选中电机尝试连续 3 周期发送 `FF FF FF FF FF FF FF FD` 失能帧，然后继续交换 5 周期 PDO，给桥接模块处理队列的时间。使能帧为 `FF FF FF FF FF FF FF FC`。程序不会发送设零或清错控制字。

`--keep-enabled` 可显式跳过正常退出时的失能，仅供已确认有后续控制接管的场合；运行报错仍尝试失能。失能写入成功及 PDO drain 完成**不等于收到电机失能确认**；断线、进程被 SIGKILL 或主机掉电时，退出代码无法保证执行。电机内部命令超时保护应按设备手册配置。

退出显示各选中电机的 `queued`、`feedback_frames` 及主机周期 min/max/avg。`queued` 是发送 API 接受的 MIT 帧数，不是 CAN ACK 或电机执行次数；`cycles` 包含退出 drain 中的只接收周期，不能直接当成 MIT 发送次数。最新反馈值可能是之前收到的缓存值，应结合反馈计数判断是否仍在更新。本程序暂不实现反馈超时、故障自动停机状态机或使能状态确认，先用于台架联调；不能仅凭无通信 API 错误认定电机状态正常。

整机库新增 `dm_robot_cycle_mask()` 和 `dm_robot_send_control_mask()`：mask 第 i 位选择 `motors[i]`，mask=0 时只交换 PDO 并接收，不发电机指令；原全电机接口继续调用同一路径，保持原有调用方式。
