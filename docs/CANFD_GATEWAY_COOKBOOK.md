# CAN/CAN FD 网关实用示例

本文给出从单帧到多通道的可复现测试顺序。所有示例都要求真实 CAN 接线、终端电阻和
能够返回带序号 payload 的对端。

## 1. 测试前记录

- 固件产品和版本；
- IgH 主站版本、内核和实时配置；
- 从站数、PDO、SM/DC Profile；
- 通道接线和终端；
- 11/29 位 ID、CAN/CAN FD、仲裁/数据波特率、BRS 和 payload；
- 每周期每通道帧数、预热和统计时长。

## 2. Classic CAN 单帧

```sh
sudo ./build/igh_mcan_gateway_tx_probe \
  --classic-can --len 8 --cycle-us 1000 --count 10000
```

用分析仪确认 ID、DLC、8 字节 payload 和周期。若网关通道配置为 FD-capable，但需要发
Classic CAN，可加 `--gateway-canfd`；线上的帧仍必须是 Classic CAN。

## 3. CAN FD+BRS 单帧

```sh
sudo ./build/igh_mcan_gateway_tx_probe \
  --canfd --brs --len 16 \
  --bitrate 1000000 --data-bitrate 5000000 \
  --cycle-us 1000 --count 10000
```

先用 8 B，再依次验证 16/32/64 B。高速数据段出现错误时检查采样点、TDC、收发器、
线长、终端和接地，不先扩大软件 FIFO 掩盖物理层问题。

## 4. 接收和闭环

```sh
sudo ./build/igh_mcan_rx_dump
```

对端回复应包含请求序号和固定标记。统计时区分：请求未发出、对端未回复、回复晚到、
TxPDO 未发布以及主站重复解析。

闭环应用优先从 `igh_simple_loop` 开始；需要四通道固定时隙时使用
`igh_250us_four_phase_demo`。

## 5. 多帧和多通道

按下列顺序增加压力：

1. 一个通道每周期 1 帧；
2. 一个通道每周期 2、4、6 帧；
3. 四个通道各 1 帧；
4. 四个通道逐步增加到目标负载；
5. 增加第二个、第三个从站。

每一步运行固定时长并保存每通道计数。若 MCAN0 与 MCAN1 物理互连，它们共享一条 CAN
总线，双向帧必须合并计算总线利用率，不能分别使用 100% 容量。

## 6. PDO 容量估算

动态 PDO 包含固定头和每帧控制字段，不能用 `PDO字节数 / payload字节数` 直接得到帧
数。使用 writer API 返回的 `IGH_ECAN_NO_SPACE` 作为容量边界，并结合产品四/八通道
固定头差异。需要扩大 PDO 时同步修改主站、固件配置和 ESI/FMMU。

## 7. 结果判定

通过至少满足：

- WKC mismatch 和 AL 状态变化为 0；
- 发送、接收数量符合测试模型；
- payload、DLC、ID、通道和序号全部正确；
- duplicate、out-of-order、parse error 为 0；
- 延迟桶满足所选 Profile；
- 降回额定负载后无需断电即可恢复。

出现丢帧时先保存 CAN 分析仪与 EtherCAT 两侧证据，再判断瓶颈属于 CAN 总线、对端、
PDO、固件队列还是主站调度。
