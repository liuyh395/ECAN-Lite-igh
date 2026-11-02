# DC 1 ms 错周期闭环指南

## 1. 目标

该 Profile 面向 1 kHz 电机控制：主站在周期 N 写入命令，从站在 DC Sync0 相位服务
PDO 和 MCAN，主站在周期 N+1 读取回复。默认周期 1 ms，Sync0 shift 900 us。

```text
周期 N                       周期 N+1
应用写请求 -> EtherCAT -> CAN 请求/回复 -> TxPDO 发布 -> 应用读回复
```

N→N+1 是目标周期落点，不保证物理闭环小于等于 1 ms，也不由配置 Profile 自动保证。
必须确认固件的 CAN 提交、反馈发布点与主站读取相位，按请求标识计算实际 N+k 和物理延迟。

## 2. 最小配置

```c
ecan_igh_config_t config;

ecan_igh_default_config(&config);
if (ecan_igh_config_apply_realtime_profile(
        &config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) != IGH_ECAN_OK) {
    return 1;
}
```

Profile 会统一设置周期、DC Sync0、900 us shift 和默认 PDO。应用只覆盖从站数、实际
PDO、MCAN 通道与波特率，不复制周期和相位常量。

## 3. 最简运行

```sh
sudo ./build/igh_simple_loop
```

程序应完成初始化、等待 OP、执行固定周期并输出 Profile、WKC 和运行诊断。任何从站
不能 OP、WKC mismatch 或周期 overrun 都视为失败。

## 4. 应用循环

```c
if (igh_mcan_port_cycle_begin_checked(&mcan) != IGH_ECAN_OK) {
    return IGH_ECAN_EXIT_FAIL; /* 停止推进业务，不再使用本轮反馈。 */
}

if (igh_mcan_port_receive(&mcan, frames, capacity, &count) != IGH_ECAN_OK)
    return IGH_ECAN_EXIT_FAIL;
/* 解析上一周期回复，计算本周期控制量。 */
if (igh_mcan_port_send_canfd(&mcan, slave, channel, can_id,
                            extended, brs, payload, payload_size) != IGH_ECAN_OK)
    return IGH_ECAN_EXIT_FAIL;

if (igh_mcan_port_cycle_end_checked(&mcan) != IGH_ECAN_OK) {
    return IGH_ECAN_EXIT_FAIL; /* 不推进运动状态。 */
}
```

读取在发送前完成，因此业务自然形成“上一周期反馈→本周期控制命令”。

## 5. 从一帧扩展负载

1. 单从站、单通道、一帧，运行至少 60 秒；
2. 保持 payload 递增序号，增加到目标每通道帧数；
3. 逐个启用其他通道；
4. 最后增加从站和 PDO；
5. 每一步记录 WKC、丢失、重复、乱序、N+1/N+2 分布和 CAN 错误。

若 N+2 增多，先判断回复是否在 CAN 线上已经晚到，再判断固件是否未在下一次 TxPDO
发布前收集。主站周期抖动、CAN 总线利用率和应答器延时必须分别记录。

## 6. 通过条件

- 预热后全部从站保持 OP；
- WKC mismatch、receive error 和 overrun 为 0；
- 请求与回复 ID、DLC、payload、序号一致；
- 目标额定负载下回复位于 N+1；
- 不出现 N+2 长尾，或其数量符合产品明确允许的边界；
- 过载降回额定值后自动恢复。

## 7. 不建议的做法

- 不把 shift 改成未验证值后直接用于生产；
- 不在同一轮同时修改周期、PDO、CPU 亲和性和 CAN 负载；
- 不用空载 WKC 结果证明 CAN 闭环；
- 不在 WKC 不完整时继续更新电机目标；
- 不使用兼容保留的周期内分相 API 作为新应用默认方案。
