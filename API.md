# ECAN IgH 稳定 API 参考

本文只描述业务应用应使用的稳定接口。安装和运行命令见 [README](README.md)，SM/DC
时序见 [实时 Profile](docs/REALTIME_PROFILES.md)，完整声明以 `include/` 头文件为准。

## 1. API 分层

| 层 | 头文件 | 业务用途 |
| --- | --- | --- |
| 通用定义 | `igh_ecan_common.h` | 返回码、通道、CAN/CAN FD 范围和默认值 |
| IgH Adapter | `igh_adapter.h` | master/domain、从站、PDO、DC、OP/WKC |
| SM Pipeline | `igh_250us_pipeline.h` | 250 us 四相调度，每通道 1 ms |
| MCAN 配置 | `igh_mcan_config.h` | 通道、CAN FD、波特率和启动 SDO |
| MCAN 运行 | `igh_mcan_port.h` | 周期、收帧、发帧和解析诊断 |
| PDO Codec | `igh_mcan_pdo_codec.h` | 自定义 raw PDO 时的写入和解析 |

优先使用最高层接口：SM 应用使用 Pipeline；普通 DC/CAN 应用使用 Adapter + MCAN
Port；只有自定义协议时才直接操作 raw PDO 和 Codec。

### 接收时间戳元数据

`igh_mcan_frame_t.rx_tsf`保留收到的TSF，`rx_timestamp_bytes[8]`保留小端原始字段，
有效长度依TSF为0/2/4/8B，其余字节清零。解析器完成整包校验后随CAN数据一起交付，
不再仅跳过时间字段；发送接口忽略这两个接收字段。

对已确认采用相对时间戳修订契约的固件，TSF=1是相对本PDO首帧的U16微秒差，
`0xFFFF`表示不可用。旧固件可能上传原始MCAN计数，API不自动根据大版本推定其语义。
这不是绝对DC时刻，不能跨PDO直接相减。接收结构体增加字段后，应用与静态库必须
整体重新编译；不承诺与旧二进制结构布局兼容。

## 2. 返回码

| 返回值 | 含义 | 应用处理 |
| ---: | --- | --- |
| `IGH_ECAN_OK` | 成功 | 继续当前状态 |
| `IGH_ECAN_FAIL` | 未细分失败 | 保存诊断并进入安全状态 |
| `IGH_ECAN_NO_SPACE` | PDO/队列空间不足 | 停止追加帧，记录过载 |
| `IGH_ECAN_INVALID_ARG` | 参数或组合非法 | 修正配置，不重试当前参数 |
| `IGH_ECAN_NOT_READY` | 未初始化、锁忙或状态不足 | 检查生命周期/OP |
| `IGH_ECAN_TIMEOUT` | 等待超时 | 检查链路和对端状态 |
| `IGH_ECAN_UNSUPPORTED` | Profile/功能不支持 | 选择受支持配置 |

实时循环使用带返回值的 `*_checked()`/`try_*()` 接口。无返回值函数只用于兼容旧应用，
新代码不应依赖其静默失败行为。

## 3. 两种实时 Profile

```c
ecan_igh_config_t config;

ecan_igh_default_config(&config); /* 默认 SM 250 us 配置。 */

/* 或切换为 DC 1 ms / shift 900 us / N→N+1。 */
if (ecan_igh_config_apply_realtime_profile(
        &config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) != IGH_ECAN_OK) {
    return IGH_ECAN_EXIT_FAIL;
}
```

| 枚举 | 说明 |
| --- | --- |
| `ECAN_IGH_PROFILE_SM_250US_1MS` | 250 us EtherCAT 子周期；配合 Pipeline 才实现 MCAN0~3 四相、每通道 1 ms |
| `ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE` | 1 ms DC，shift 900 us；N+1 回复是需验证的目标 |

Profile 只定义配置，不证明固件采用特定的 CAN 提交/反馈发布点，也不保证物理闭环延时。
应按 [主站兼容性](docs/MASTER_COMPATIBILITY.md) 核对模式，并独立统计请求与应答。

`ecan_igh_realtime_profile_name()` 返回可用于日志的稳定名称。新应用不要使用兼容保留的
DC 周期内分相函数组合默认模式。

## 4. Adapter 配置

### 4.1 从站和身份

```c
int ecan_igh_config_set_slave_count(ecan_igh_config_t *config, size_t count);
int ecan_igh_config_set_slave(ecan_igh_config_t *config,
                              size_t slot, int alias, int position);
int ecan_igh_config_set_identity(ecan_igh_config_t *config,
                                 size_t slot,
                                 uint32_t vendor_id,
                                 uint32_t product_id);
```

`slot` 是配置数组下标，不等于 EtherCAT position。`set_slave()` 决定实际寻址；身份必须
与目标从站一致，否则初始化失败。

### 4.2 固件 V1/V2 能力判断

主站应在添加版本相关启动 SDO 前读取 `0xF000:05`（公开版本高字）、`:06`（构建号）和
`:16`（PDO feature flags），再调用：

```c
igh_mcan_firmware_capabilities_t caps;

if (igh_mcan_firmware_capabilities_decode(
        f000_05, f000_06, f000_16, &caps) != IGH_ECAN_OK) {
    return IGH_ECAN_EXIT_FAIL;
}
```

`caps.protocol_v2` 只有在规范化版本 `>=0x0200` 时为 1；`caps.header_v2` 和
`caps.fine_size_16` 还会分别检查 feature bit13/bit7。不能只凭能力位把 1.x 固件当成 V2。
固件2.x只表示具备V2能力，不自动选择V2。`igh_mcan_port_default_config()`始终使用
`option_flags=0x04`，保持V1头并默认关闭payload CRC16。

### 4.3 PDO 大小

```c
int ecan_igh_config_set_pdo_multiplier(ecan_igh_config_t *config, uint8_t n);
int ecan_igh_config_set_pdo_bytes(ecan_igh_config_t *config, uint16_t bytes);
int ecan_igh_config_set_direction_pdo_multiplier(
    ecan_igh_config_t *config, uint8_t rx_n, uint8_t tx_n);
int ecan_igh_config_set_slave_direction_pdo_multiplier(
    ecan_igh_config_t *config, size_t slot, uint8_t rx_n, uint8_t tx_n);
```

PDO 大小编码的低 5 bit 是 128 B 基础块数，高 3 bit 是 16 B 尾块数：
`bytes = (n & 0x1f) * 128 + (n >> 5) * 16`。基础块范围为 `1..10`，尾块范围为
`0..7`，总范围为 128..1392 B。默认编码 `0x02` 仍表示 256 B，旧主站写入的
`1..10` 保持原义。新应用优先调用 `ecan_igh_config_set_pdo_bytes()`，避免手工编码。
`RxPDO` 表示主站到从站，`TxPDO` 表示从站到主站。字节数必须与固件、ESI/EEPROM
和 IgH FMMU 映射一致。对称接口同时设置两个方向；反馈较多时使用方向接口。

### 4.3.1 PDO头版本与CRC默认值

`igh_mcan_port_default_config()`设置`option_flags=IGH_MCAN_OPT_DEFAULT=0x04`，即默认V1＋NO_CRC16。
确认2.x且bit13后，应用调用`igh_mcan_port_select_pdo_protocol()`显式选择V2；参数
`disable_crc16=1`得到`0x24`，值为0得到`0x20`。选择V1时同一参数分别得到`0x04/0x00`。
关闭CRC时writer固定填0，固件接收主站RxPDO时忽略CRC字段；5E/6E的V2头为16B/20B，V1为24B/32B；
codec通过IHL计算帧区起点，应用不要硬编码offset。

### 4.4 启动 SDO

```c
int ecan_igh_config_add_sdo8(...);
int ecan_igh_config_add_sdo16(...);
int ecan_igh_config_add_sdo32(...);
```

启动 SDO 在 `ecan_igh_init()` 期间写入。仅添加设备明确支持的对象；错误的 index、
subindex、类型或顺序可能阻止进入 OP。

## 5. MCAN 配置

```c
igh_mcan_port_config_t mcan_config;

igh_mcan_port_default_config(&mcan_config);
igh_mcan_port_set_channel_mask(&mcan_config, slave, channel_mask, canfd_mask);
igh_mcan_port_set_bitrate(&mcan_config, slave, 1000000, 5000000);

if (igh_mcan_port_apply_config(&config, &mcan_config) != IGH_ECAN_OK) {
    return IGH_ECAN_EXIT_FAIL;
}
```

便捷接口：

| API | 用途 |
| --- | --- |
| `igh_mcan_port_set_classic_channel()` | 启用一个 Classic CAN 通道 |
| `igh_mcan_port_set_canfd_channel()` | 启用一个 CAN FD-capable 通道 |
| `igh_mcan_port_disable_channel()` | 关闭一个通道，不能关闭最后一路 |
| `igh_mcan_port_set_sample_point_range()` | 限制自动位时序采样点 |
| `igh_mcan_port_set_channel_policy()` | 设置已定义的通道策略字 |

RX drain/TX write budget 属于高级调优。未调用对应 setter 时保留固件默认值；不要仅为
提高瞬时帧数盲目增大预算。

## 6. Adapter 生命周期

```c
ecan_igh_context_t igh;

if (ecan_igh_init(&igh, &config) != IGH_ECAN_OK) {
    return IGH_ECAN_EXIT_FAIL;
}
if (ecan_igh_wait_op_timeout(&igh, 10000) != IGH_ECAN_OK) {
    ecan_igh_close(&igh);
    return IGH_ECAN_EXIT_FAIL;
}

/* 周期运行；正常停机还需按第11节完成状态切换，不能直接以close替代。 */

ecan_igh_close(&igh);
```

调用顺序固定为：默认配置→Profile→产品/拓扑覆盖→MCAN 配置→`init`→等待 OP→周期→
停止业务及状态确认→`close`。初始化失败后可以直接 `close` 已初始化的上下文资源。

这里的“等待 OP”不是只检查 AL 状态。`ecan_igh_wait_op_timeout()` 要求当前
`slave_config` 已真正应用、所有 PDO domain WKC 完整，并连续满足 3 个探测周期；SM
250 μs 模式的启动探测会限制为 1 kHz，避免从站配置/邮箱流量与大 PDO 探测互相挤占。
实时业务循环进入后仍按所选的 250 μs 或 1 ms 周期运行。

等待期间，适配器通过标准IgH从站信息接口检查已配置、已挂接且设备标识匹配的
从站。主站报告`error_flag`时返回`IGH_ECAN_FAIL`，不再等待OP超时；该标志为
主站配置状态，不是PDO压力标志或CAN错误计数。正常扫描、离线、未到OP及信息
查询失败不会直接认定为配置错误，仍遵守原等待期限。`timeout_ms=0`仅取消期限，
不忽略配置错误。

检查间隔不短于10ms，每次最多读取8个物理从站；大拓扑分批检查，发现延迟随
拓扑长度增加。支持绝对环位置及IgH的首次匹配别名加相对位置寻址。
此检查仅在启动等待接口中执行，不增加正常周期收发的查询或改变业务节拍。

`wait_op_timeout()`超时不等于所有从站都未启动：多站配置可能按顺序推进，返回超时时
部分设备已经处于OP。当前正常停机事务要求全部目标OP及完整空PDO回收，不能直接用于
这种部分启动状态。`mcan_rx_dump`已在该错误分支接入有界的启动撤销事务，随后仍返回
原始启动失败，不将撤销成功报告为业务成功。入口为`igh_mcan_startup_abort_prepare()`，
后续使用`igh_mcan_stop_poll()`；示例封装为`igh_example_startup_abort_preop()`。
该入口只允许已经开始启动探测、尚未运行任何业务周期、输出映像及实际域输出均为全零
的情况。它不重建或清零在途PDO，不发送CAN命令，不解除故障锁存；只有目标身份一致、
PREOP请求成功且全部目标实际处于无错误的PREOP，才返回成功。正常停机入口的前提不变。

启动撤销时，SAFEOP及OP目标均维持原全零启动输出，避免尚在进行的SAFEOP→OP请求因
缺少过程数据触发看门狗。在此全零、未开始业务的路径中，各域独立确认和推进：未
确认的输入域不被重排，也不阻断已经完整确认的输出域；输出域自身未确认时仍禁止重排。
正常业务停机仍要求同一从站两方向共同确认，不能把此启动策略用于运动命令或运行故障。

已知主站限制：部分IgH源码的DC配置等待过程不及时响应新的PREOP请求，可能使上述撤销
事务仍然超时。当前仅内部主站候选完成特定两站OP、一站PREOP，以及两站OP、一站SAFEOP
故障的限定验证；后者使用全零输出的逐域确认策略。不能将这些结论外推为标准IgH或全部
配置阶段均支持无告警撤销。撤销超时必须保留失败及逐站AL状态，
不能增加等待期限掩盖问题，也不能在等待失败后发送业务命令。运行期故障须使用第11.2节
独立退出入口及其前提，不能调用本启动撤销入口代替。库与应用须使用同一版头文件重新编译。
返回失败后，应查看主站配置错误记录并关闭上下文；修正配置或消除故障后重新初始化。
该接口不会自动复位设备、清除主站错误或重放CAN命令。

当前运行期失败边界：`mcan_rx_dump`在周期失败后尝试有界故障停止，再关闭上下文，
最终仍返回原始业务失败。无法满足接管前提或退出失败时，关闭操作仍可能产生DC同步
错误（AL 0x001A），不能视为已确认的PREOP退出。即使方向WKC完整，超过接收截止时间
仍保持失败；不得清除故障标志继续普通周期。故障退出的适用条件见第11.2节。

## 7. MCAN Port 周期

### 7.1 初始化

```c
igh_mcan_port_t port;

if (igh_mcan_port_init(&port, &igh, &mcan_config) != IGH_ECAN_OK) {
    ecan_igh_close(&igh);
    return IGH_ECAN_EXIT_FAIL;
}
```

### 7.2 收发循环

```c
igh_mcan_rx_frame_t frames[32];
size_t frame_count = 0;

if (igh_mcan_port_cycle_begin_checked(&port) != IGH_ECAN_OK) {
    /* 保持安全输出。 */
}

if (igh_mcan_port_receive(&port, frames, 32, &frame_count) != IGH_ECAN_OK) {
    igh_mcan_port_cycle_abort_checked(&port);
}

if (igh_mcan_port_send_canfd(&port, slave, channel, can_id,
                             is_extended, brs, payload, payload_size)
        != IGH_ECAN_OK) {
    /* 空间不足时不继续追加。 */
}

if (igh_mcan_port_cycle_end_checked(&port) != IGH_ECAN_OK) {
    /* 不推进电机目标。 */
}
```

一个成功的 `cycle_begin` 必须对应 `cycle_end` 或 `cycle_abort`。不得嵌套周期，也不得从
多个线程同时操作同一个 `igh_mcan_port_t`。

### 7.3 发送接口

```c
int igh_mcan_port_send_can(..., const uint8_t *data, size_t data_len);
int igh_mcan_port_send_canfd(..., int brs,
                             const uint8_t *data, size_t data_len);
```

Classic CAN 最大 8 B，CAN FD 最大 64 B。`is_extended=0` 使用 11 位 ID，非零使用 29 位
ID。`IGH_ECAN_NO_SPACE` 表示本周期 PDO 已满，不代表帧已经进入固件或 CAN 总线。

预约接口 `igh_mcan_port_schedule_can/canfd()` 只在明确启用 DC Sync0 且设备支持预约字段
时使用；`offset_ns` 是相对同步事件的偏移，`tag` 用于应用关联。

### 7.4 接收结构

`igh_mcan_rx_frame_t` 包含：

- `slave_pos`：来源从站；
- `frame`：通道、ID、CAN FD/BRS、DLC、长度和 payload；
- `header`：本次 TxPDO 固定头快照。

应用先检查返回码和 `rx_count`，再逐帧验证来源、通道、ID 和长度。需要解析错误时调用
`igh_mcan_port_get_last_parse_error()`；需要固定头时调用
`igh_mcan_port_get_tpdo_header()`。

## 8. SM 250 us Pipeline

```c
igh_250us_pipeline_config_t pipeline_config;
igh_250us_pipeline_t pipeline = IGH_250US_PIPELINE_INITIALIZER;

igh_250us_pipeline_default_config(&pipeline_config);
if (igh_250us_pipeline_open(&pipeline, &pipeline_config) != IGH_ECAN_OK) {
    return IGH_ECAN_EXIT_FAIL;
}
```

周期接口：

```c
igh_250us_pipeline_cycle_t cycle;
igh_mcan_rx_frame_t frames[64];

igh_250us_pipeline_cycle_begin(&pipeline, frames, 64, &cycle);
igh_250us_pipeline_send_canfd(&pipeline, slave, can_id,
                              extended, brs, payload, payload_size);
igh_250us_pipeline_cycle_end(&pipeline);
```

发送函数不接收通道参数，固定使用本周期的 `cycle.active_channel`，防止应用把四个通道
重新集中到同一子周期。异常时调用 `cycle_abort()`，退出时调用 `close()`。

`igh_250us_pipeline_get_stats()` 返回周期、WKC、收发和调度统计；`last_open_stage()` 与
`get_last_open_diag()` 用于 open 失败定位。

## 9. Raw PDO 和 Codec

只有完全自定义协议时使用：

```c
ecan_igh_get_rxpdo_buffer(&igh, slave, &rxpdo, &rx_size);
ecan_igh_get_txpdo_buffer(&igh, slave, &txpdo, &tx_size);
```

Codec 写入顺序：

```c
igh_mcan_pdo_writer_init(&writer, rxpdo, rx_size,
                         channel_mask, profile, options);
igh_mcan_pdo_writer_begin(&writer);
igh_mcan_pdo_writer_append(&writer, &frame);
igh_mcan_pdo_writer_finish(&writer);
```

解析使用 `igh_mcan_pdo_parse_frames_ex()`，回调只消费当前调用期间有效的数据。失败时
读取 `igh_mcan_pdo_parse_diag_t`，不要跳过 CRC、长度、padding、通道掩码或 Profile
校验。

## 10. 状态与诊断

| API | 说明 |
| --- | --- |
| `ecan_igh_domain_is_complete()` | 最近周期 WKC 是否完整 |
| `ecan_igh_domain_working_counter()` | 最近 WKC 数值 |
| `ecan_igh_domain_wc_state()` | IgH WC 状态 |
| `ecan_igh_slaves_are_operational()` | 全部从站是否 OP |
| `ecan_igh_slave_al_state()` | 单从站 AL 状态 |
| `ecan_igh_get_sched_stats()` | 周期各阶段最大耗时与 overrun |
| `ecan_igh_reset_runtime_diagnostics()` | 预热后清零正式统计 |
| `ecan_igh_get_diag_events()` | 获取WKC、overrun及DC收集超时事件快照 |
| `ecan_igh_print_pdo_offsets()` | 显式输出 PDO 映射 |

正式统计前先完成预热并调用 `reset_runtime_diagnostics()`。诊断打印不放在硬实时高频
路径中。

`wkc_mismatches`、`events`和`dropped`的统计起点为上下文初始化或最近一次诊断重置，
并非自动从全部从站进入OP开始。未重置时，启动配置期间的非完整WKC也包含在累计值中；
`mcan_rx_dump`当前保留该全过程统计。应保存启动阶段证据，再单独记录业务阶段的统计，
不得仅通过清零将全过程异常表述为不存在。

运行诊断中的`dropped`是事件环被覆盖的记录数，不是CAN丢帧数，也不是PDO包头的容量
标志或从站SDO丢弃计数。`expected=0`可能出现在启动尚未确定预期WKC的事件记录中，
不能解释为该映射不需要WKC确认。正常停机后`all_op=0`表示设备已离开OP；停机是否成功
应以停机事务结果及实际AL状态为准。数据完整性仍需逐帧核对数量、内容和顺序。

DC收集失败单独记录`type=dc-receive-timeout`，不以完整WKC代替按时完成。
例如三个写域实际WKC为3且完整，但收集超过截止时间时，事件可为`wkc=3 expected=3`，
同时周期接口仍返回超时并锁存故障。该事件不增加`wkc_mismatches`；若本次已采样且
WKC不完整，则同时计入WKC异常数量，不再重复生成一条混合方向的事件。

| 超时事件字段 | 定义 |
| --- | --- |
| `scope`，对应事件结构`flags` | 0：全部运行域；1：RxPDO写方向；2：TxPDO读方向。仅用于该事件类型 |
| `wkc`、`expected` | 同一收集范围的实际值、已知预期值；预期0表示尚未确定，不按从站数硬编码 |
| `receive_start_ns` | 本次有界收集开始时刻 |
| `receive_deadline_ns` | 实际期限，为原相位期限与从开始起100μs局部上限的较早者 |
| `receive_observed_ns` | 最后一次receive/process之后的时钟观察值；0表示时钟无效 |
| `receive_attempts` | 实际收集轮数；0表示进入时已经超期或时钟无效，尚未采样本代次WKC |
| `receive_wc_state` | 本次最后一次域采样的IgH状态；attempts=0时没有本代次证据 |
| `receive_stop_reason` | 1：到达截止；2：达到128次循环上限；3：时钟无效 |
| `recv_ns`、`late_ns` | 最后观察值减开始值、超出期限的时间；时钟无效时不得解释为有效耗时 |
| `domains`、`bad_domain` | 该收集范围的域缓存快照；-1只表示未发现缓存状态不完整，不表示按时完成。attempts=0时不是本代次回收证据 |

以上时间均属于主站`CLOCK_MONOTONIC`，不是CAN上线、从站IRQ或ESC发布时间，不能直接
与内核日志、设备时间戳跨时钟相减。总览行的`expected`仍为全部运行域预期；在输出方向
提前失败时，其`wkc`可能仅有输出范围，必须结合该条事件的`scope`和范围内预期判断，
不能将总览3/6推断成一半报文丢失。接口返回失败后不得重排旧代次或继续发布缓存输入。

## 11. 线程和所有权

`ecan_igh_close()`用于资源释放，不返回从站停机结果。其返回时，IgH后台状态机
可能尚未完成PREOP切换；最终能够恢复PREOP也不能抵扣期间出现的同步错误。
正常停止应先停止追加业务，在仍拥有主站时维持有效空PDO及必要的DC同步，
请求PREOP并确认实际AL状态及错误位后，再停止周期和释放资源。状态请求提交成功
不等于状态切换完成。新增`igh_mcan_stop.h`中的停机事务接口尚在分项验收，
不能将简单调用`close()`标记为无告警停机保证。详细边界见
[正常停止的判定](docs/MASTER_COMPATIBILITY.md#正常停止的判定)。

`close()`不得与周期线程并发调用；锁忙时该次调用保留上下文并报告错误。
空PDO和PREOP均不能替代电机安全停止、STO或急停。

### 11.1 有界停机事务

| 阶段 | 调用及前提 | 成功含义 |
| --- | --- | --- |
| 排空业务 | 应用停止追加命令，按业务协议收齐应答，并完成至少一个已回收的空PDO周期 | 不代表电机已安全停止，也不自动取消远期预约 |
| 准备 | 零初始化`igh_mcan_stop_t`，调用`igh_mcan_stop_prepare(&stop, &port, timeout_ms)`；超时必须非零 | 配置寻址已解析为`stop.positions[]`；空输出和完整回收前提通过；普通周期被禁止 |
| 请求 | 管理侧向指定主站、上述绝对环位置异步请求PREOP；可使用标准IgH CLI或系统已有管理接口 | 只表示请求提交，不表示状态已到达；库不启动外部进程、不直接写AL寄存器 |
| 轮询 | 周期线程持续调用`igh_mcan_stop_poll(&stop, request_result)`；示例管理轮询间隔为100 μs，部署时核验查询耗时和管理进程的调度余量 | 保持必要空PDO/DC服务；未确认域不重复排队 |
| 释放 | poll返回`IGH_ECAN_OK`后再调用`ecan_igh_close()` | 请求成功、所有目标实际AL状态为PREOP且无错误标志 |

`request_result`为0表示管理请求尚在进行，1表示成功，负数表示失败。
信息及域查询返回后，poll使用当前单调时间重新核对总期限与发送时刻，
不使用调用入口的旧时间判断；查询跨过总期限不得认定停机成功。
该检查不能中断阻塞的内核调用，也不构成硬实时响应上限保证。

poll返回`IGH_ECAN_NOT_READY`时应继续轮询；返回TIMEOUT或其他错误时是失败终态，
不会自动close、清错、重试业务或解除停机接管。应用须执行自身故障停止策略，
不能将失败后最终恢复PREOP改判为正常停止。成功或失败后重新运行业务须close并重新初始化。

`queued`统计停机阶段域排队次数，不是CAN帧数；`incomplete_polls`是未确认域的轮询次数，
不是丢帧数；`retired_after_op`仅记录确认对应从站已离开OP后的域退役次数。

管理查询周期与空PDO发送周期不同。示例将查询频率限制为10 kHz，避免短周期下大量
实时查询使同核普通优先级的管理进程无法运行。库仍按原业务周期检查空PDO发送期限，
迟到不追赶，未确认的域不重排；查询间隔不延长停止总超时。实际空PDO发送会受到查询
节拍及I/O耗时影响，必须结合从站看门狗进行验证。不得仅增加轮询频率来改善停止响应。
正常停机准备前必须完整回收最后空周期；接管后分离输入、输出域各自确认。
已确认的输出域可继续发送新序号空PDO，不等待同站输入域；未确认的输出域不得
改写或重排，输入域本身也不得重复排队。合并LRW仍按整个域确认。
停机状态统计独立于业务WKC，不清除原有错误。接口不分配动态内存、不增加内核补丁。
deadline用于本地判断，不构成Linux内核调用永不阻塞或硬实时停机上限的保证。
更新后应整体重编译应用与主站库，不混用旧头文件、对象文件或静态库。

Linux示例层提供`examples/common/igh_example_stop.h`，通过标准IgH CLI异步请求PREOP：

```c
/* 已停止追加业务、收齐应答，并完成一个已回收的空PDO周期。 */
igh_example_stop_result_t stop = {0};
int rc = igh_example_stop_preop(&port, config.master_index,
                               "ethercat", 2000U, &stop);
/* rc!=OK必须作为停机失败处理；close仅释放资源，不能把失败改判成功。 */
ecan_igh_close(&igh);
```

`ethercat`应来自可信安装且存在于进程PATH，也可传入可信绝对路径；需要对应主站设备权限。
示例不调用shell、不自动sudo，仅对prepare解析的绝对环位置请求PREOP。
CLI子进程不继承实时优先级及应用信号屏蔽。`cli_error`记录启动/等待错误，
`cli_status`是waitpid原始状态；`unreaped_child`非零表示超时后尚未回收的子进程，
不能报告正常退出。失败清理最多额外轮询20ms；内核阻塞仍不受用户态deadline硬保证。

目前`mcan_rx_dump`在正常结束时使用此接入，并消费最后一次周期结束后回收的上行映像。
其接收截止点是这次最终消费，之后外部CAN继续发送的帧不在录入范围；该示例不产生CAN命令。
其他发送及电机示例尚未统一迁移，必须先明确应答排空、预约取消和电机安全停止的业务条件。

### 11.2 运行期故障退出

`igh_mcan_fault_stop_prepare()`仅接管已经开始业务、当前应用周期已结束或abort的上下文。
目标身份必须一致，实际状态须为无AL错误的OP、SAFEOP，或管理侧已处理后到达的PREOP，
链路及拓扑检查正常。已在PREOP的目标不再接收旧PDO；确认输出状态结束后退役其在途域。
随后仍使用`igh_mcan_stop_poll()`并异步请求PREOP；示例入口为
`igh_example_fault_stop_preop()`。它不是CAN命令取消、业务恢复或电机安全停止接口。

接口不会确认AL错误。对于已锁存错误，应用应先保存故障原因，按自身明确的故障处理策略
请求状态转换，再调用故障停止接口核对完成；不得无条件清除未知错误或重新进入OP。
管理请求成功不等于实际状态转换成功，最终仍须满足全部目标为无错误PREOP的完成条件。
底层主站FSM自身的错误确认行为与本库调用策略不同，应用必须独立保存原始故障记录。

故障停机已成功接管后，若目标随后进入`SAFEOP+ERROR`，事务记录
`al_error_seen[n]`及`al_error_polls`，在原总期限内继续等待已请求的标准PREOP转换。
该期间不排该从站PDO、不改写或退役其在途输出，也不直接写AL错误确认。
`al_error_polls`统计逐从站观测次数，不是独立故障数；历史标志在退出成功后仍保留。
只有实际到达无错误PREOP且管理请求成功才返回OK。其他AL错误状态、身份/链路/拓扑变化、
管理失败及超时仍失败；接管前已有AL错误的处理规则不变。

| 边界 | 处理规则 |
| --- | --- |
| 准备接管 | 不清零或修改在途输出；保留原故障、WKC、接收超时与完整性统计 |
| 输出域未确认 | 不改写该域，不重新排队；不能重发原CAN命令维持watchdog |
| 输出域完整确认 | 用新序号空PDO替换；故障路径全长清零，覆盖尚未finish的残留正文 |
| 输入域未确认 | 分离模式不阻断已确认输出的空PDO服务；输入域自身仍不重排 |
| 接管后SAFEOP+ERROR | 保存错误观测，暂停该目标PDO；保留在途域，等待标准PREOP转换，不自行清错 |
| 域queue或send中途失败 | 锁存`pdo_queue_uncommitted`，禁止自动接管和继续发送；标准IgH没有通用公开接口撤销已入队PDO |
| 完成 | PREOP请求成功，且全部目标实际为无错误PREOP；只表示退出成功，原业务结果仍失败 |

分离DC发送先检查等待结果及发送截止，再复制、入队输出域，避免截止失败遗留未发送命令。
入队后send未成功返回时保持不确定状态，不通过重新send验证。该保护不证明已发CAN命令
已经取消，也不保证超过期限的Linux阻塞调用能够立即返回。

运行期故障退出仍在分项验收：内部主站候选的接收延迟故障与标准IgH、真实输出命令、
断链或发送失败是不同测试条件。接收示例的限定结果不能替代其他发送/电机示例的验证。
库、应用和头文件必须整体重新编译；不得混用旧版上下文结构或静态库。

接收示例对`IGH_ECAN_NO_SPACE`独立锁存不完整结果，正常停止不能清除该结果。
`truncated_batches`按接收调用计数而非按帧计数；告警不依赖周期诊断开关。
该情况表示传给port的主站应用数组不足，不能直接解释成从站硬件丢帧。

- 每个 context/port/pipeline 只有一个周期线程；
- 配置对象只在初始化前修改；
- 周期内获取的 raw PDO 指针不跨周期保存；
- 回调不阻塞、不分配大对象、不调用慢速 SDO；
- 管理线程通过快照读取统计，不直接改周期上下文；
- `try_begin()` 返回 `NOT_READY` 时不等待锁，应用保持上一安全输出。

## 12. 选择示例

| 目标 | 示例 |
| --- | --- |
| 最小 DC 循环 | `examples/basic/simple_loop.c` |
| SM 四相 | `examples/pipeline/four_phase_250us_demo.c` |
| 多通道 MCAN | `examples/mcan/mcan_multi_channel_demo.c` |
| CAN FD 电机 | `examples/mit/mit_canfd_motor_demo.c` |
| CiA402 单轴 | `examples/cia402/cia402_single_axis_demo.c` |
| CiA402 多轴 | `examples/cia402/cia402_10axis_demo.c` |

所有电机示例默认不自动使能、不自动写零位。运行前阅读对应安全说明。
