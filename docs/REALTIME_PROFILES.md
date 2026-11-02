# ECAN 实时主站双 Profile

主站库提供两种推荐实时数据面配置。周期、DC相位和默认PDO大小统一定义在
`include/ecan_realtime_profile.h`（独立 ZIP 内）。仓库构建使用共享头文件
`tools/include/ecan_realtime_profile.h`，打包时原样纳入 ZIP，避免复制维护两套默认值。

| Profile | EtherCAT周期 | 控制周期 | 调度方式 | 默认PDO |
| --- | ---: | ---: | --- | --- |
| `sm-250us-1ms` | 250 μs | 1 ms | MCAN0、1、2、3依次获得发送槽 | RxPDO 128 B / TxPDO 256 B |
| `dc-1ms-cross-cycle-shift900us` | 1 ms | 1 ms | Sync0 shift 900 μs，闭环落点需实测 | RxPDO 384 B / TxPDO 384 B |

## SM 250 μs / 1 ms

使用 `igh_250us_pipeline_default_config()` 后直接打开即可。默认按
`MCAN0 -> MCAN1 -> MCAN2 -> MCAN3` 服务；每个EtherCAT子周期只向当前通道提交
CAN帧，所以同一通道每四个子周期、即每1 ms获得一次确定发送机会。

该四相 Pipeline 服务 MCAN0～3；将通用编解码/适配层编译为 8 通道不会自动扩展为八相调度。
8 通道项目应明确选择和验证自己的调度策略，编译通过不代表八通道实时负载已通过。

```c
igh_250us_pipeline_config_t config;
igh_250us_pipeline_t pipeline = IGH_250US_PIPELINE_INITIALIZER;

igh_250us_pipeline_default_config(&config);
if (igh_250us_pipeline_open(&pipeline, &config) != IGH_ECAN_OK) {
    return 1;
}
```

零参数示例：

```bash
sudo ./build/igh_250us_four_phase_demo
```

## DC 1 ms / 900 μs / N→N+1

普通应用在默认配置上应用DC profile，不需要自行设置周期或相位：

```c
ecan_igh_config_t config;

ecan_igh_default_config(&config);
if (ecan_igh_config_apply_realtime_profile(
        &config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) != IGH_ECAN_OK) {
    return 1;
}
```

该 Profile 设置周期和 Sync0 相位，不保证请求对应的回复一定在 N+1 出现。
固件可能采用不同的 CAN 提交和反馈发布点；使用前须确认调度能力。
N+k 表示应用取到对应回复的周期差，不等于物理延时 k ms；物理闭环时间需独立测量。

零参数示例：

```bash
sudo ./build/igh_simple_loop
sudo ./build/igh_mit_canfd_motor_demo
./build/igh_cia402_single_axis_demo --help
```

CiA402 运动示例须显式添加 `--enable-motor`，否则不申请主站。MIT 示例不默认发送使能或写零位命令，
但普通控制帧仍可能影响已使能设备。执行前须确认节点 ID、量程、急停和机械状态。

## 参数覆盖原则

- 用户示例默认直接选择上述profile，不复制周期和相位常量。
- PDO大小、从站数、CAN波特率和CPU亲和性可以按拓扑覆盖。
- SDO、EEPROM、状态读取等管理工具可使用较慢周期；它们不属于实时闭环profile。
- 兼容保留的 DC 周期内三段分相 API 不作为应用默认配置；新应用使用上述两个 profile。
- 修改公共profile后，必须同时通过IgH编译测试、SocketCAN内核构建和对应硬件长测。
