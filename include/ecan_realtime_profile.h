/* SPDX-License-Identifier: MIT */
#ifndef ECAN_REALTIME_PROFILE_H
#define ECAN_REALTIME_PROFILE_H

/*
 * ECAN 实时数据面的统一产品配置。
 *
 * 该头文件只包含编译期常量，可同时被 IgH 用户态库和 SocketCAN 内核模块引用。
 * 用户示例只应从下面两个 profile 中选择，不再自行复制周期、相位和 PDO 大小。
 */

/* SM：250 us EtherCAT 子周期，四个 MCAN 相位组成 1 ms 控制周期。 */
#define ECAN_RT_SM_ECAT_CYCLE_US             250U
#define ECAN_RT_SM_CONTROL_CYCLE_US          1000U
#define ECAN_RT_SM_PHASE_COUNT               4U
#define ECAN_RT_SM_RXPDO_MULTIPLIER          1U
#define ECAN_RT_SM_TXPDO_MULTIPLIER          2U
/* SocketCAN 当前使用对称 PDO，取较大值以保留完整 TxPDO 容量。 */
#define ECAN_RT_SM_SYMMETRIC_PDO_MULTIPLIER  ECAN_RT_SM_TXPDO_MULTIPLIER

/* DC：1 ms EtherCAT 周期，Sync0 后移 900 us，采用 N -> N+1 错周期闭环。 */
#define ECAN_RT_DC_ECAT_CYCLE_US             1000U
#define ECAN_RT_DC_CONTROL_CYCLE_US          1000U
#define ECAN_RT_DC_SYNC0_SHIFT_US            900
#define ECAN_RT_DC_RXPDO_MULTIPLIER          3U
#define ECAN_RT_DC_TXPDO_MULTIPLIER          3U

#endif /* ECAN_REALTIME_PROFILE_H */
