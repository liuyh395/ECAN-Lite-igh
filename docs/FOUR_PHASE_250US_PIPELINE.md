# SM 250 μs四相流水线

## 设计目标

主站每250 μs完成一次PDO交换，并按MCAN0、1、2、3依次服务。相同通道每四个
EtherCAT子周期再次获得发送槽，因此上层电机控制周期保持1 ms。

| 子周期 | 活动通道 | 同一通道下次槽位 |
| ---: | --- | ---: |
| N+0 | MCAN0 | N+4 |
| N+1 | MCAN1 | N+5 |
| N+2 | MCAN2 | N+6 |
| N+3 | MCAN3 | N+7 |

四相调度用于分散PDO打包和CAN发送压力，不增加CAN总线带宽，也不把单通道控制
周期变成250 μs。

## 用户API

`include/igh_250us_pipeline.h`隐藏IgH master/domain、PDO编解码、绝对周期时钟、
OP等待和运行诊断。应用循环只需要四步：

```c
igh_250us_pipeline_cycle_t cycle;
igh_mcan_rx_frame_t rx[128];

igh_250us_pipeline_cycle_begin(&pipeline, rx, 128, &cycle);
consume_feedback(rx, cycle.rx_count);
build_command(cycle.active_channel, payload);
igh_250us_pipeline_send_canfd(&pipeline, slave, can_id, 0, 1,
                              payload, sizeof(payload));
igh_250us_pipeline_cycle_end(&pipeline);
```

`send_canfd()`不接收channel参数，而是固定使用`cycle.active_channel`，从接口上避免
应用把多个通道重新集中到同一子周期。一个成功的`cycle_begin()`必须对应一次
`cycle_end()`；异常退出调用`cycle_abort()`，最终统一调用`close()`。

## 默认配置

- 模式：SM，DC关闭。
- EtherCAT子周期：250 μs。
- 控制周期：1 ms。
- 相位顺序：MCAN0、1、2、3。
- RxPDO：128 B；TxPDO：256 B。
- CAN FD：1 Mbit/s仲裁段、5 Mbit/s数据段、BRS开启。
- 实时优先级：80；最终自旋：20 μs。

独立包默认值来自 `include/ecan_realtime_profile.h`，由仓库共享头文件原样打包；不得在示例中复制另一套常量。

## 最简运行

```bash
sudo ./build/igh_250us_four_phase_demo
```

三从站和物理回环测试可显式增加拓扑参数：

```bash
sudo ./build/igh_250us_four_phase_demo \
  --slaves 3 --cycles 40000 --tx --tx-slave-mask 0x6 --rx-channel-xor 1
```

真实电机通常在同一MCAN通道回复，此时不要设置`--rx-channel-xor 1`。

## 已验证基线

三从站在线，从站1、2分别使用MCAN0↔MCAN1、MCAN2↔MCAN3硬件回环。每个
250 μs子周期在当前活动通道发送一帧CAN FD、11位ID、8-byte、BRS 1M+5M；
每从站RxPDO 128 B、TxPDO 256 B。

| 项目 | 结果 |
| --- | ---: |
| 正式子周期 | 120016 |
| CAN FD发送/接收 | 240000 / 240000 |
| 内容校验错误 | 0 |
| 回帧延迟 | 全部N+3，即750 μs；N+4及以后为0 |
| WKC mismatch / clock resync | 0 / 0 |
| receive/send/no-space | 0 / 0 / 0 |
| 周期范围 | 249.043～250.926 μs |

这组数据验证每个被选从站每通道1帧/ms。增加每槽帧数、PDO大小或从站数量后必须
重新执行带宽、WKC、帧内容和长尾测试，不能直接外推。

## 运行约束

1. 周期线程使用绝对时间推进，不以相对sleep累积漂移。
2. EtherCAT网卡IRQ与周期线程使用不同CPU；周期线程优先级不能压住网卡收包线程。
3. 测试必须检查WKC、OP、帧序号、CAN ID、通道和payload，不以“收到帧数相同”代替
   数据正确性。
4. SocketCAN的`sm_250us_1ms`使用相同的四相取队列规则；周期流若要求1 ms发送一次，
   `period_cycles`应设置为4。
5. DC不在本流水线中启用。需要DC时使用独立的
   `ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE`产品profile。
