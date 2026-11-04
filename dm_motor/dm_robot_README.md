# 四路 CAN / 12 电机整机发送层

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
