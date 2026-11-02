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

使能、失能、设零和清错使用 `dm_robot_send_control_all()`。反馈超时及故障状态机暂未实现。示例应用没有改动，新增源文件尚未接入 CMake 目标；本轮按要求不编译。

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
