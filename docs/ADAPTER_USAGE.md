# IgH Adapter Usage

`igh_adapter` 是主站侧最底层的通用 EtherCAT 适配层。它只负责 IgH master/domain、从站寻址、PDO 映射、DC Sync0、启动 SDO 队列和 raw PDO byte stream，不包含 MCAN 或 CiA402 业务逻辑。

## 适用场景

- 只需要把 EtherCAT RxPDO/TxPDO 当作原始字节流使用。
- 集成非 ECAN-Lite 设备，或者在主站侧实现自己的 PDO 协议。
- 上层模块需要统一管理从站数量、PDO 大小、DC 周期和 OP 状态。

## 初始化流程

```c
ecan_igh_config_t config;
ecan_igh_context_t igh;

ecan_igh_default_config(&config);
ecan_igh_config_set_slave_count(&config, 3);
ecan_igh_config_set_cycle_us(&config, 2000);
ecan_igh_config_set_pdo_bytes(&config, 384);
ecan_igh_config_enable_dc_sync0(&config, 2000, 0);

if (ecan_igh_init(&igh, &config) != IGH_ECAN_OK) {
    return 1;
}
if (ecan_igh_wait_op_timeout(&igh, 10000) != IGH_ECAN_OK) {
    ecan_igh_close(&igh);
    return 1;
}
```

`ecan_igh_wait_op_timeout()` 同时检查 IgH `state.operational`、全部 PDO domain 的完整 WKC，
并要求连续 3 次就绪。它不会把上一应用尚未退出完毕时残留的 AL=OP 误判为新配置已经
生效。配置阶段以不高于1kHz的绝对节拍推进，不沿用10/20ms等较长业务周期。
分离DC的启动探测包含输入读取，因此对齐配置的TxPDO读取相位，而非固定相位0。
首次完整PDO或进入OP后，较长周期的检查等待至少一个完整发布周期，再对齐该读取相位；
合并收发仍对齐发送相位。不能在同一输入发布之间重复取数。业务周期、Sync0/Sync1参数不变，
完整WKC、在途所有权和连续3次就绪检查均保留。就绪后仍须使用空命令预热，再发送业务命令。

`[DC-PHASE] verified`表示ESC相位配置读回通过，不表示参考时钟偏差已经收敛。
`igh_mcan_port_warmup_cycle_checked()`的连续稳定计数检查OP、WKC及接收状态，当前不包含参考时钟偏差。
DC应用还需在业务准入前检查参考时钟与主站发包时基的偏差。使用标准
`ecrt_master_reference_clock_time()`时，应与同一次已回收同步报文的发包时基比较，
处理32位回绕及传播延时语义；不得将上一周期的读值直接与本周期当前时间相减。
误差门槛由实际Sync0准备余量决定，不以固定预热时长替代该检查。

适配器首次提交的应用时间用于建立DC相位原点；后续`application_time`提交当前单调时钟，
不再使用向下取整的业务周期起点。业务发送网格仍由周期调度器控制，Sync0/Sync1相位参数不变。
长周期启动时，持续提交旧周期起点会把量化误差带入ESC时间初始化；相位读回与参考误差仍须分别核对。

`ecan_igh_default_config()` 默认配置 3 个 ECAN-Lite 从站，SM 周期 250us、RxPDO 128B、TxPDO 256B。
四通道固定相位由 `igh_250us_pipeline` 实现；仅调用 Adapter 配置函数不会自动轮转 MCAN 通道。
应用应按实际拓扑覆盖从站数量。需要 DC 时应整体切换 Profile，再核对固件调度能力和实际相位。

初始化默认不打印 PDO offset 表，正常业务日志保持静默。需要排查 PDO 映射时，可在初始化前设置：

```c
config.init_flags |= ECAN_IGH_INIT_PRINT_PDO_OFFSETS;
```

也可以在初始化成功后显式调用 `ecan_igh_print_pdo_offsets(&igh)` 输出诊断。

## Sync0 发送、Sync1 发布的验证配置

此配置要求固件实现 **SM2 暂存请求、Sync0 释放 CAN 请求、Sync1 发布反馈**。
支持 PDO V2 并不等于支持此调度；不能仅根据固件主版本号判断。SM 默认配置保持不变。

```c
ecan_igh_default_config(&config);
ecan_igh_config_set_slave_count(&config, 1);
ecan_igh_config_set_cycle_us(&config, 1000);
ecan_igh_config_set_pdo_bytes(&config, 640);
/* 主站独立写入 / 读取 / 收集相位，单位 us。 */
ecan_igh_config_enable_dc_phased_cycle(&config, 100, 0, 200, 400);
/* 两个参数均为本周期绝对相位，必须在设置 phased_cycle 之后调用。 */
ecan_igh_config_enable_dc_sync01(&config, 1000, 100, 900);
```

应用必须检查每个配置函数及初始化函数的返回值。初始化自动配置
`AssignActivate=0x0700`、`0x1C32:01=2`、`0x1C33:01=3`；冲突的启动 SDO、
跳过启动 SDO、相同的Sync0/Sync1相位或超周期相位会被拒绝。
Sync1绝对相位小于Sync0时按跨周期处理。配置结构增加字段后，所有调用者需要重新编译。

| 配置项 | 本例值 | 含义 |
| --- | ---: | --- |
| Sync0 周期 | 1000 us | CAN 请求释放周期 |
| Sync0 绝对相位 | 100 us | 已完成暂存的请求在此释放 |
| Sync1 绝对相位 | 900 us | 在此开始构建、发布反馈 |
| `dc_sync1_offset_ns` | 800000 ns | Sync1 相对 Sync0 的间隔 |
| 主站 RxPDO 写入 | 0 us | 请求必须在 Sync0 前完成读取和校验 |
| 主站 TxPDO 读取 | 200 us | 读取上一周期 Sync1 已发布的反馈 |
| 主站接收收集 | 400 us | 收集 EtherCAT 返回帧并处理 domain |

IgH 调用使用第五个参数 `sync1_cycle=800000`，第六个参数为 0；不能将绝对相位
900 us 写入 `sync1_shift`，该参数在标准 IgH 接口中被忽略。Sync1 触发不是发布完成时刻。
本例先验证调度及数据稳定性，N+1 不表示端到端延时不超过 1 ms。

`ecan_igh_poll_op()` / `ecan_igh_wait_op_timeout()` 在 Sync01 模式下还会读取
ESC `0x0990..0x09A7`，核对实际 Sync0/Sync1 相位、周期和间隔，并打印 `[DC-PHASE]`。
应用必须等待检查通过再发送 CAN 请求。受影响的 IgH 启动实现以
`sync0_cycle + sync1_cycle` 对齐起点，已观测到请求 100/900 us 实际变为 500/300 us。
发生 `MISMATCH` 时会拒绝就绪；不可跳过检查或将该轮归为 100/900 us 测试通过。
此检查仅验证 ESC 编程相位，不测量 MCU 中断延迟。
适用的主站版本和验收边界见 [主站兼容性](MASTER_COMPATIBILITY.md)。本包不提供主站补丁；
受影响的主站不能直接用于此 Sync01 配置。应使用经验证的修正版本，或显式选择并独立验收
适合设备的其他同步模式，不自动回退、不跳过相位检查。

本例仅演示 API 调用顺序，不保证该相位与负载下数据完整或达到 N+1。应根据目标固件调度、
主站相位实测和 CAN 应答截止时间选择参数，按 [测试与验收](TEST_AND_ACCEPTANCE.md) 独立验证。

### Sync1 跨周期及输入读取窗口

绝对相位小于 Sync0 不等于 Sync1 在关联 Sync0 之前发生。周期 T 下应计算正向间隔
`offset=(phase1-phase0+T)%T`，再要求 `0<offset<T`。例如 T=1000us、200/100us 对应
900us 间隔，Sync1 在下一周期的 100us；900/100us 则仅有 200us 间隔。
不能将 `T+phase1` 写到 Sync1 cycle，也不能用被标准 IgH 忽略的 sync1_shift 表达它。

`ecan_igh_config_enable_dc_sync01()`支持上述跨周期配置，配置检查与寄存器读回均按
模T相位核验。必须检查函数返回值及实际ESC相位，不能直接绕过检查修改结构字段。
应用的 N+1 判定仍由实际发布后第一次有效读取和业务匹配决定，不由相位数字大小决定。

Sync0/Sync1触发是固件处理起点，并非TxPDO已经写入ESC的时刻。主站读取应避开输入
构建与发布窗口，并为固件执行时间、报文传输和调度抖动留出余量。例如周期1000us：

| 模式 | 固件反馈发布触发相位 | 主站写入/读取/收集示例，us | 读取对象 |
| --- | --- | --- | --- |
| Sync0=200、Sync1=950 | Sync1=950 | 0/200/400 | 上一周期已发布的反馈 |
| Sync0=200、Sync1=100 | 关联Sync0之后、下一周期100 | 0/500/750 | 本周期100us触发后发布的反馈 |
| 仅Sync0=200 | Sync0输入映射路径 | 0/500/750 | Sync0处理后已发布的反馈 |

以上为选取读取窗口的示例，不是所有负载的时序保证。不能把第一行的200us读取相位
直接用于后两行，也不能将WKC不完整改为忽略。启动就绪、空命令预运行和带载阶段
均须验证完整WKC、逐帧内容、数量与顺序。周期延迟N+1也不等于绝对往返时间小于1ms。

## 周期调用

标准模式保持兼容调用顺序：

```c
ecan_igh_cycle_begin(&igh);

uint8_t *rxpdo = NULL;
const uint8_t *txpdo = NULL;
size_t rx_size = 0;
size_t tx_size = 0;

ecan_igh_get_rxpdo_buffer(&igh, 0, &rxpdo, &rx_size);
ecan_igh_get_txpdo_buffer(&igh, 0, &txpdo, &tx_size);

/* 在 rxpdo 中写主站输出，从 txpdo 中读从站输入。 */

ecan_igh_cycle_end(&igh);
```

`rxpdo` 是主站输出给从站的数据，`txpdo` 是从站输入给主站的数据。周期中如果决定不发布本周期输出，可调用 `ecan_igh_cycle_abort()`；如果需要保持上一周期输出，可调用 `ecan_igh_cycle_repeat_output()`。

## 推荐实时 Profile

新应用从两个已收敛的 Profile 中选择，不需要自行组合周期和 DC 相位。

### SM 250 us / 每通道 1 ms

SM 四相流水由 `igh_250us_pipeline` 管理。四个 250 us 子周期依次服务 MCAN0、1、2、3，
每个通道每 1 ms 获得一次确定的业务时隙：

```c
igh_250us_pipeline_config_t pipeline_config;
igh_250us_pipeline_t pipeline = IGH_250US_PIPELINE_INITIALIZER;

igh_250us_pipeline_default_config(&pipeline_config);
if (igh_250us_pipeline_open(&pipeline, &pipeline_config) != IGH_ECAN_OK) {
    return 1;
}
```

直接运行：

```sh
sudo ./build/igh_250us_four_phase_demo
```

### DC 1 ms / 900 us / N 到 N+1

普通 1 kHz 控制应用使用 DC 错周期闭环：

```c
ecan_igh_default_config(&config);
if (ecan_igh_config_apply_realtime_profile(
        &config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) != IGH_ECAN_OK) {
    return 1;
}
```

请求在周期 N 的 RxPDO 中发送，实际 CAN 提交和反馈发布点由固件调度决定。
N+1 是需要实测的目标，不由本 Profile 保证；N+k 是周期差，不能直接换算为物理闭环时间。

直接运行：

```sh
sudo ./build/igh_simple_loop
```

更完整的参数和安全边界见 `REALTIME_PROFILES.md`。兼容保留的单周期分相接口不作为新
应用默认方案。

## PDO头版本

MCAN Port默认配置使用`IGH_MCAN_OPT_DEFAULT=0x04`，即V1并关闭payload CRC16。旧主站写`0x00`
仍兼容，只是主动启用CRC16。
固件2.x仅表示具备V2能力；确认`F000:16 bit13`后，应用必须调用
`igh_mcan_port_select_pdo_protocol()`显式选择V2。`disable_crc16=1`对应`0x24`，值为0对应
`0x20`；选择V1时分别对应`0x04/0x00`。codec始终读取`ver_ihl`，客户调用层无需计算固定offset。

## PDO 大小

动态 PDO 保留 128B 基础块，并可附加 0..7 个 16B 尾块：

| 配置 | 含义 |
| --- | --- |
| `ecan_igh_config_set_pdo_multiplier(config, n)` | 设置原始 5+3 bit 编码；低 5 bit 是 128B 块数，高 3 bit 是 16B 块数 |
| `ecan_igh_config_set_pdo_bytes(config, bytes)` | 按字节设置；范围 128..1392B，16B 对齐，推荐新代码使用 |
| `ecan_igh_config_set_slave_pdo_multiplier(config, slave, n)` | 为某个从站单独设置原始编码 |

基础块范围为 `1..10`，尾块范围为 `0..7`。默认值 `n=0x02` 仍为 256B，旧 IgH
配置的整数 `1..10` 无需修改。例如 144B 编码为 `0x21`，1392B 编码为 `0xEA`。
常用测试组合仍可使用前两个从站 `n=6`、第三个从站 `n=3`，因为第三个从站只启用
一半 MCAN 通道。

## 状态检查

运动或压力测试前建议检查：

```c
if (!ecan_igh_domain_is_complete(&igh)) {
    /* 本周期 working counter 不完整。 */
}
if (!ecan_igh_slaves_are_operational(&igh)) {
    /* 至少一个从站不在 OP。 */
}
```

常用诊断接口：

| 接口 | 说明 |
| --- | --- |
| `ecan_igh_domain_working_counter()` | 当前 domain working counter |
| `ecan_igh_domain_wc_state()` | IgH domain WC 状态 |
| `ecan_igh_slave_is_operational()` | 单个从站是否 OP |
| `ecan_igh_slave_al_state()` | 单个从站 AL 状态 |
| `ecan_igh_print_pdo_offsets()` | 用户显式调用时打印 PDO offset 表 |
| `ecan_igh_reset_runtime_diagnostics()` | 预热后清零调度、WKC 和事件统计 |

上层 demo 在 WC 不完整时应保持目标或跳过推进，避免通信恢复后目标位置突然跳变。
