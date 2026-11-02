# IgH CPU Affinity Usage

这份文档用于 Linux 主机上检查和配置 IgH demo 的 CPU 亲和性。重点不是只把 demo 用
`taskset` 锁到某个核心，而是同时检查并处理：

- demo 应用跑在哪个核心；
- 网卡中断跑在哪个核心；
- PREEMPT_RT 下的 IRQ 线程跑在哪个核心；
- EtherCAT/IgH 相关线程跑在哪个核心。

如果只锁 demo，不管网卡和内核侧处理，可能会出现周期抖动，甚至 SDO 回复收不到。

## 1. 设置变量

在 Linux 主机上先设置：

```sh
export IFACE=your_ethercat_iface
export CORE=0
export DEMO=./build/igh_cia402_10axis_demo
export IGH_LIB_DIR='<IgH-library-directory>'
```

先确认专用EtherCAT接口、收发IRQ、软中断及主站线程位置，再划分周期线程与网络服务CPU。
核心编号必须属于当前机器，不能直接照搬其他台架。以下命令只适用于已停止业务的测试环境。

## 2. 查看 CPU 和网卡信息

查看 CPU：

```sh
lscpu
cat /sys/devices/system/cpu/online
nproc
```

查看网卡和驱动：

```sh
ip link show "$IFACE"
readlink -f "/sys/class/net/$IFACE/device/driver" || true
ethtool -i "$IFACE" 2>/dev/null || true
```

查网卡相关中断号：

```sh
cat /proc/interrupts | egrep -i "$IFACE|eth|stmmac|dwmac|eqos|ethercat"
```

查线程当前跑在哪个核心：

```sh
ps -eLo pid,tid,psr,cls,rtprio,comm | \
  egrep -i 'igh_cia|ethercat|ec_|irq/|ksoftirqd|stmmac|dwmac|eqos'
```

其中 `psr` 就是线程最近运行过的 CPU 编号。

## 3. 停止 irqbalance

如果系统开了 `irqbalance`，它可能会把中断重新挪走。测试期间建议先停掉：

```sh
IRQBALANCE_WAS_ACTIVE=$(systemctl is-active irqbalance || true)
printf '%s\n' "$IRQBALANCE_WAS_ACTIVE"
sudo systemctl stop irqbalance 2>/dev/null || true
```

测试结束后可以再恢复。

## 4. 绑定网卡中断

优先使用 `smp_affinity_list`，比手写十六进制 mask 更直观。

```sh
# 根据驱动信息及/proc/interrupts逐项核对，填入此专用接口的RX/TX IRQ编号。
# 留空时不执行写入；禁止把前面的模糊搜索结果直接作为修改目标。
IRQ_LIST=''
for IRQ in $IRQ_LIST; do
    case "$IRQ" in ''|*[!0-9]*) echo 'Invalid IRQ'; break;; esac
    cat "/proc/irq/$IRQ/smp_affinity_list"  # 保存原值用于恢复
    echo "$CORE" | sudo tee "/proc/irq/$IRQ/smp_affinity_list"
done
```

检查是否设置成功：

```sh
for IRQ in $IRQ_LIST; do
    printf 'IRQ %s affinity=' "$IRQ"
    cat "/proc/irq/$IRQ/smp_affinity_list"
done
```

观察中断计数是否在目标 CPU 上增长：

```sh
watch -n 1 "cat /proc/interrupts | egrep -i '$IFACE|eth|stmmac|dwmac|eqos|ethercat'"
```

## 5. 绑定 PREEMPT_RT 的 IRQ 线程

RT 内核里，中断可能会变成 `irq/<编号>-...` 这种线程。这里的 `...` 表示省略网卡或驱动相关的线程名后缀。也需要一起看。

先列出来：

```sh
ps -eLo pid,tid,psr,cls,rtprio,comm | \
  egrep -i 'irq/.*(eth|stmmac|dwmac|eqos|ethercat)'
```

绑定到目标核心：

```sh
for IRQ in $IRQ_LIST; do
    case "$IRQ" in ''|*[!0-9]*) echo 'Invalid IRQ'; break;; esac
    for TID in $(ps -eLo tid,comm | awk -v prefix="irq/$IRQ-" \
        'index($2,prefix)==1 {print $1}'); do
        sudo taskset -pc "$CORE" "$TID"
    done
done
```

再次检查：

```sh
ps -eLo pid,tid,psr,cls,rtprio,comm | \
  egrep -i 'irq/.*(eth|stmmac|dwmac|eqos|ethercat)'
```

## 6. 绑定 EtherCAT/IgH 相关线程

先只查看，不要盲目移动所有内核线程：

```sh
ps -eLo pid,tid,psr,cls,rtprio,comm | egrep -i 'ethercat|ec_|master'
```

如果能明确看到 EtherCAT/IgH 相关线程，再绑定：

```sh
# 核对完整线程名称、所属master及原亲和性后，手动指定单个TID。
MASTER_TID=''
if [ -n "$MASTER_TID" ]; then
    taskset -pc "$MASTER_TID"
    sudo taskset -pc "$CORE" "$MASTER_TID"
fi
```

检查结果：

```sh
ps -eLo pid,tid,psr,cls,rtprio,comm | egrep -i 'ethercat|ec_|master'
```

## 7. 运行 demo

先不要加实时优先级，只锁同一个核心：

```sh
sudo taskset -c "$CORE" env LD_LIBRARY_PATH="$IGH_LIB_DIR" \
  "$DEMO" \
  --cycle-us 1000 \
  --cycles 4294967295 \
  --step 1000 \
  --wait-op-ms 10000 \
  --sdo-timeout-ms 3000 \
  --diag-every 1000 \
  --nmt-reset \
  --stop-on-emcy
```

如果能正常配置 CSP 并进入电机运行，再尝试较低实时优先级：

```sh
sudo taskset -c "$CORE" chrt -f 30 env LD_LIBRARY_PATH="$IGH_LIB_DIR" \
  "$DEMO" \
  --cycle-us 1000 \
  --cycles 4294967295 \
  --step 1000 \
  --wait-op-ms 10000 \
  --sdo-timeout-ms 3000 \
  --diag-every 1000 \
  --nmt-reset \
  --stop-on-emcy
```

不要一开始用 `chrt -f 80`。优先级太高时，可能压住网卡或 EtherCAT 相关线程。

## 8. 运行时检查

demo 运行时，另开一个终端检查：

```sh
ps -eLo pid,tid,psr,cls,rtprio,comm | \
  egrep -i 'igh_cia|ethercat|ec_|irq/|ksoftirqd|stmmac|dwmac|eqos'
```

检查中断计数：

```sh
cat /proc/interrupts | egrep -i "$IFACE|eth|stmmac|dwmac|eqos|ethercat"
```

看 demo 输出时重点关注：

- PDO offset 是否都是 `contiguous=yes`；
- CSP 配置阶段 `payload_len` 是否从 0 变成非 0；
- 如果 `payload_len=0` 且 SDO 超时，说明电机回复没有通过网关回到主站；
- 如果通道状态是 `000A`，表示通道已启动且 CANFD-capable，不是 bus off。

## 9. 建议测试顺序

按下面顺序测，并记录结果：

```text
1. 不 taskset，不 chrt
2. taskset 到网卡 IRQ 当前核心，不 chrt
3. taskset 到 CPU0，不 chrt
4. taskset 到 CPU1，不 chrt
5. taskset 到 CPU2，不 chrt
6. taskset 到 CPU3，不 chrt
7. 最稳定的核心 + chrt -f 30
8. 最稳定的核心 + tcpdump
```

只有在“不加 chrt”能完成 CSP 配置并让电机运行后，再逐步提高实时优先级。

## 10. 恢复

如果前面停掉了 `irqbalance`，测试结束后恢复：

```sh
if [ "$IRQBALANCE_WAS_ACTIVE" = active ]; then
    sudo systemctl start irqbalance
fi
```

按修改前保存的值恢复IRQ亲和性、线程亲和性、调度类别与优先级；核对实际读回值。
仅当irqbalance原来运行时才重新启动该服务。重启主机不能替代恢复记录。

## 11. 发送排队与软中断

标准generic经过Linux网络发送路径。RX及时并不意味着TX及时；提高周期线程的FIFO
优先级也不保证网卡立即发送。检查周期线程所在CPU的`ksoftirqd`调度类别、网卡RX/TX
完成中断、qdisc排队和驱动统计，不能只检查其中一个IRQ。

```sh
tc -s qdisc show dev "$IFACE"
ethtool -c "$IFACE"
ethtool -S "$IFACE"
ps -eLo pid,tid,psr,cls,rtprio,comm | grep -E 'irq/|ksoftirqd|EtherCAT'
```

若主站已经判定超时，而对应报文随后才出现在网卡发送抓包中，应先定位主机发送路径，
不能直接归因为从站PDI耗时或CAN丢帧。需按datagram索引、命令、地址、长度和时间窗口
关联请求与应答；索引会回绕，单独匹配索引不足以确认同一事务。

软中断优先级调整只适用于明确的实时CPU规划，必须考虑该CPU上的其他网络与系统工作，
防止反向饿死周期线程。每次只改变一项设置，保存原值，复测完整启停、WKC及CAN内容、
数量和顺序。不能通过加大超时、忽略WKC、删除所有权保护或降低负载代替定位。

## 建议

根据实际收发和调度测量分配CPU及优先级，不要求所有线程放在同一核心，也不以核心编号
相邻作为实时性保证。不得用宽泛的`eth`或`master`匹配批量修改管理网卡、其他master或系统线程。

如果“不锁核能跑，锁到某个核心不能跑”，优先判断为绑核和中断位置问题，再继续排查 PDO 映射、从站状态和应用周期。
