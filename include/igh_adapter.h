#ifndef IGH_ADAPTER_H
#define IGH_ADAPTER_H

#include <stddef.h>
#include <stdint.h>
#include <pthread.h>
#include <ecrt.h>

#include "ecan_realtime_profile.h"
#include "igh_ecan_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ECAN-Lite4 EtherCAT Vendor/Product ID。 */
#define ECAN_IGH_VENDOR_ID                 0x0048504dU
#define ECAN_IGH_PRODUCT_ID                0x00001502U

/* 主站最多预留 8 个 EtherCAT 从站，应用可按需减少。 */
#define ECAN_IGH_MAX_SLAVES                8U

/* ECAN 动态 PDO 大小：低 5 bit 为 128B 基础块，高 3 bit 为 16B 尾块。 */
#define ECAN_IGH_PDO_CHUNK_BYTES           128U
#define ECAN_IGH_PDO_FINE_BYTES            16U
#define ECAN_IGH_PDO_BASE_MASK             0x1FU
#define ECAN_IGH_PDO_FINE_SHIFT            5U
#define ECAN_IGH_PDO_FINE_MASK             0x07U
#define ECAN_IGH_PDO_MULTIPLIER_MIN        1U
#define ECAN_IGH_PDO_MULTIPLIER_MAX        10U
#define ECAN_IGH_PDO_FINE_MAX              7U
#define ECAN_IGH_PDO_DEFAULT_MULTIPLIER    IGH_ECAN_DEFAULT_PDO_MULTIPLIER
#define ECAN_IGH_PDO_MAX_BYTES             \
    (ECAN_IGH_PDO_CHUNK_BYTES * ECAN_IGH_PDO_MULTIPLIER_MAX + \
     ECAN_IGH_PDO_FINE_BYTES * ECAN_IGH_PDO_FINE_MAX)
#define ECAN_IGH_PDO_DEFAULT_BYTES         (ECAN_IGH_PDO_CHUNK_BYTES * ECAN_IGH_PDO_DEFAULT_MULTIPLIER)
#define ECAN_IGH_UNITS_PER_CHUNK           8U
#define ECAN_IGH_UNIT_BYTES                16U
#define ECAN_IGH_MAX_UNITS                 \
    (ECAN_IGH_PDO_MULTIPLIER_MAX * ECAN_IGH_UNITS_PER_CHUNK + ECAN_IGH_PDO_FINE_MAX)
#define ECAN_IGH_MAX_PDO_MAPPINGS          (ECAN_IGH_PDO_MULTIPLIER_MAX + 1U)
#define ECAN_IGH_MAX_DOMAIN_REGS           (ECAN_IGH_MAX_SLAVES * ECAN_IGH_MAX_UNITS * 2U + 1U)
#define ECAN_IGH_MAX_SLAVE_DOMAIN_REGS     (ECAN_IGH_MAX_UNITS * 2U + 1U)
#define ECAN_IGH_DIAG_EVENT_RING_SIZE      128U
#define ECAN_IGH_DIAG_EVENT_SNAPSHOT_MAX   32U
#define ECAN_IGH_DIAG_DOMAIN_MAX           (ECAN_IGH_MAX_SLAVES * 2U)

static inline uint8_t ecan_igh_pdo_base_count(uint8_t encoded)
{
    return (uint8_t)(encoded & ECAN_IGH_PDO_BASE_MASK);
}

static inline uint8_t ecan_igh_pdo_fine_count(uint8_t encoded)
{
    return (uint8_t)((encoded >> ECAN_IGH_PDO_FINE_SHIFT) & ECAN_IGH_PDO_FINE_MASK);
}

static inline uint16_t ecan_igh_pdo_decode_bytes(uint8_t encoded)
{
    return (uint16_t)(ecan_igh_pdo_base_count(encoded) * ECAN_IGH_PDO_CHUNK_BYTES +
                      ecan_igh_pdo_fine_count(encoded) * ECAN_IGH_PDO_FINE_BYTES);
}

static inline uint8_t ecan_igh_pdo_mapping_count(uint8_t encoded)
{
    return (uint8_t)(ecan_igh_pdo_base_count(encoded) +
                     (ecan_igh_pdo_fine_count(encoded) != 0U ? 1U : 0U));
}

static inline int ecan_igh_pdo_size_encoding_is_valid(uint8_t encoded)
{
    uint8_t base = ecan_igh_pdo_base_count(encoded);
    uint8_t fine = ecan_igh_pdo_fine_count(encoded);
    return base >= ECAN_IGH_PDO_MULTIPLIER_MIN &&
           base <= ECAN_IGH_PDO_MULTIPLIER_MAX && fine <= ECAN_IGH_PDO_FINE_MAX;
}

static inline uint8_t ecan_igh_pdo_encode_bytes(uint16_t bytes)
{
    uint8_t base = (uint8_t)(bytes / ECAN_IGH_PDO_CHUNK_BYTES);
    uint8_t fine = (uint8_t)((bytes % ECAN_IGH_PDO_CHUNK_BYTES) /
                             ECAN_IGH_PDO_FINE_BYTES);
    return (uint8_t)(base | (uint8_t)(fine << ECAN_IGH_PDO_FINE_SHIFT));
}

/* 已在三从站、384B/方向配置下验证的 1ms DC 同周期相位。 */
#define ECAN_IGH_DC_SINGLE_CYCLE_US         1000U
#define ECAN_IGH_DC_SINGLE_RXPDO_PHASE_US   100U
#define ECAN_IGH_DC_SINGLE_TXPDO_PHASE_US   780U
#define ECAN_IGH_DC_SINGLE_COLLECT_PHASE_US 950U
#define ECAN_IGH_DC_SINGLE_SYNC0_SHIFT_US   600

/* 启动 SDO 队列容量。上层协议可把设备配置转换为这里的 SDO。 */
#define ECAN_IGH_MAX_STARTUP_SDOS          1536U

/* DC Sync0 模式来自 ESI: AssignActivate #x300。 */
#define ECAN_IGH_DC_ASSIGN_SYNC0           0x0300U
#define ECAN_IGH_DC_ASSIGN_SYNC01          0x0700U

/* EtherCAT PDO assignment 与 ECAN-Lite4 动态 PDO 映射对象。 */
#define ECAN_IGH_OD_RXPDO_ASSIGN           0x1C12U
#define ECAN_IGH_OD_TXPDO_ASSIGN           0x1C13U
#define ECAN_IGH_OD_RXPDO_MAP_BASE         0x1600U
#define ECAN_IGH_OD_TXPDO_MAP_BASE         0x1A00U
#define ECAN_IGH_OD_TX_ENTRY_BASE          0x6000U
#define ECAN_IGH_OD_RX_ENTRY_BASE          0x7000U
#define ECAN_IGH_OD_ENTRY_STRIDE           0x10U
#define ECAN_IGH_OD_ASSIGN_FIRST_ENTRY     0x01U
#define ECAN_IGH_SYNC_END                  0xFFU

/* ECAN-Lite4 device maintenance object used by adapter-level helpers. */
#define ECAN_IGH_OD_DEVICE_CONTROL         0x8000U
#define ECAN_IGH_OD_DEVICE_REBOOT          IGH_ECAN_OD_SUBINDEX_APPLY

/* 初始化标志：跳过 0x1C12/0x1C13 和用户启动 SDO 下载。 */
#define ECAN_IGH_INIT_SKIP_STARTUP_SDO     (1U << 0)
/* 初始化完成后打印 PDO offset 诊断。默认关闭，避免正常启动路径输出日志。 */
#define ECAN_IGH_INIT_PRINT_PDO_OFFSETS    (1U << 1)

typedef enum {
    ECAN_IGH_SDO_U8 = 1,
    ECAN_IGH_SDO_U16 = 2,
    ECAN_IGH_SDO_U32 = 4
} ecan_igh_sdo_size_t;

typedef enum {
    ECAN_IGH_DIAG_EVENT_NONE = 0,
    ECAN_IGH_DIAG_EVENT_WKC_MISMATCH = 1,
    ECAN_IGH_DIAG_EVENT_CYCLE_OVERRUN = 2,
    ECAN_IGH_DIAG_EVENT_DC_RECEIVE_TIMEOUT = 3
} ecan_igh_diag_event_type_t;

/* DC receive-timeout事件的flags表示本次实际收集范围；0为全部运行域。 */
enum { ECAN_IGH_DIAG_SCOPE_ALL = 0, ECAN_IGH_DIAG_SCOPE_RXPDO = 1,
       ECAN_IGH_DIAG_SCOPE_TXPDO = 2 };
enum { ECAN_IGH_DIAG_RECEIVE_DEADLINE = 1, ECAN_IGH_DIAG_RECEIVE_ATTEMPTS = 2,
       ECAN_IGH_DIAG_RECEIVE_CLOCK = 3 };

typedef enum {
    ECAN_IGH_DIAG_DOMAIN_NONE = 0,
    ECAN_IGH_DIAG_DOMAIN_COMBINED = 1,
    ECAN_IGH_DIAG_DOMAIN_RXPDO = 2,
    ECAN_IGH_DIAG_DOMAIN_TXPDO = 3
} ecan_igh_diag_domain_kind_t;

typedef enum {
    ECAN_IGH_CYCLE_MODE_STANDARD = 0,
    ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE = 1,
    ECAN_IGH_CYCLE_MODE_DC_COMBINED = 2 /* LRW, bounded collection before cycle_end returns. */
} ecan_igh_cycle_mode_t;

/* 面向用户的两种稳定实时 profile。 */
typedef enum {
    ECAN_IGH_PROFILE_SM_250US_1MS = 0,
    ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE = 1
} ecan_igh_realtime_profile_t;

/* 启动阶段 SDO。IgH 层只知道 index/subindex/value，不理解上层协议含义。 */
typedef struct {
    size_t slave_pos;
    uint16_t index;
    uint8_t subindex;
    ecan_igh_sdo_size_t size;
    uint32_t value;
} ecan_igh_startup_sdo_t;

/* 单个 EtherCAT 从站的寻址和对象 ID。 */
typedef struct {
    int alias;
    int position;
    uint32_t vendor_id;
    uint32_t product_id;
} ecan_igh_slave_config_t;

/* IgH 主站初始化配置，只描述 EtherCAT 和 raw PDO。 */
typedef struct {
    int master_index;
    size_t slave_count;
    uint8_t pdo_multiplier;
    uint8_t slave_pdo_multiplier[ECAN_IGH_MAX_SLAVES];
    uint8_t rxpdo_multiplier;
    uint8_t txpdo_multiplier;
    uint8_t slave_rxpdo_multiplier[ECAN_IGH_MAX_SLAVES];
    uint8_t slave_txpdo_multiplier[ECAN_IGH_MAX_SLAVES];
    uint8_t init_flags;
    uint32_t cycle_us;
    uint16_t dc_assign_activate;
    uint32_t dc_sync0_cycle_ns;
    int32_t dc_sync0_shift_ns;
    uint32_t dc_sync1_offset_ns; /* Sync1 相对 Sync0 的间隔；0 表示禁用。 */
    ecan_igh_cycle_mode_t cycle_mode;
    uint32_t dc_prepare_ahead_us; /* Optional early preparation; wire send stays on phase grid. */
    uint32_t dc_min_prepare_us; /* Host admission budget to Sync0; 0 disables this guard. */
    uint32_t rxpdo_phase_us;
    uint32_t txpdo_phase_us;
    uint32_t txpdo_collect_phase_us;
    ecan_igh_slave_config_t slaves[ECAN_IGH_MAX_SLAVES];
    ecan_igh_startup_sdo_t startup_sdos[ECAN_IGH_MAX_STARTUP_SDOS];
    size_t startup_sdo_count;
} ecan_igh_config_t;

typedef struct {
    uint32_t cycle_us;
    uint64_t ethercat_cycle;
    uint64_t cycle_exec_max_ns;
    uint64_t cycle_late_max_ns;
    uint64_t cycle_overrun_count;
    uint64_t receive_process_max_ns;
    uint64_t txpdo_copy_max_ns;
    uint64_t rxpdo_copy_max_ns;
    uint64_t queue_max_ns;
    uint64_t master_send_max_ns;
    uint64_t wkc_mismatch_count;
    uint64_t sm_late_receive_recovered;
    uint64_t sm_receive_timeouts;
    uint64_t sm_pacing_waits;
    uint64_t sm_pacing_wait_max_ns;
    uint64_t dc_late_receive_recovered;
    uint64_t dc_receive_timeouts;
    uint64_t dc_prepare_samples;
    int64_t dc_prepare_slack_min_ns; /* Host estimate before queue/send, NOT firmware READY. */
    uint64_t dc_send_deadline_misses;
    uint64_t sm_receive_poll_calls;
    uint64_t sm_receive_waits;
    uint64_t sm_receive_wait_max_ns;
    uint64_t sm_pacing_rebases; /* Delayed send/phase guard; never hidden in timing pass. */
} ecan_igh_sched_stats_t;

typedef struct {
    uint16_t slave_index;
    uint8_t kind;
    uint8_t wc_state;
    uint32_t working_counter;
} ecan_igh_diag_domain_sample_t;

typedef struct {
    uint64_t sequence;
    uint64_t ethercat_cycle;
    uint64_t app_time_ns;
    uint64_t record_time_ns;
    uint32_t event_type;
    uint32_t flags;
    uint32_t wkc_current;
    uint32_t wkc_expected;
    uint32_t domain_count;
    int32_t first_bad_domain;
    uint64_t late_ns;
    uint64_t receive_process_ns;
    uint64_t txpdo_copy_ns;
    uint64_t rxpdo_copy_ns;
    uint64_t queue_ns;
    uint64_t master_send_ns;
    uint64_t cycle_exec_ns;
    /* 仅DC receive-timeout有效，均为主站CLOCK_MONOTONIC；不是CAN/ESC时间。
     * deadline包含原绝对相位期限与100us局部上限。完整WKC仍可能超期失败。 */
    uint64_t receive_start_ns;
    uint64_t receive_deadline_ns;
    uint64_t receive_observed_ns;
    uint32_t receive_attempts;
    uint32_t receive_wc_state;
    uint32_t receive_stop_reason;
    ecan_igh_diag_domain_sample_t domains[ECAN_IGH_DIAG_DOMAIN_MAX];
} ecan_igh_diag_event_t;

typedef struct {
    uint64_t cursor;
    uint32_t max_events;
    uint32_t count;
    uint64_t head_sequence;
    uint64_t dropped;
    ecan_igh_diag_event_t events[ECAN_IGH_DIAG_EVENT_SNAPSHOT_MAX];
} ecan_igh_diag_event_snapshot_t;

/* 单个从站运行态：PDO offset、raw PDO 缓冲区和 IgH PDO 映射对象。 */
typedef struct {
    ecan_igh_slave_config_t config;
    ec_slave_config_t *sc;
    unsigned int rx_offsets[ECAN_IGH_MAX_UNITS];
    unsigned int tx_offsets[ECAN_IGH_MAX_UNITS];
    ec_pdo_entry_info_t rx_entries[ECAN_IGH_MAX_UNITS];
    ec_pdo_entry_info_t tx_entries[ECAN_IGH_MAX_UNITS];
    ec_pdo_info_t rx_pdos[ECAN_IGH_MAX_PDO_MAPPINGS];
    ec_pdo_info_t tx_pdos[ECAN_IGH_MAX_PDO_MAPPINGS];
    ec_sync_info_t syncs[5];
    uint8_t *domain_pd;
    uint8_t *rx_domain_pd;
    uint8_t *tx_domain_pd;
    uint8_t rx_domain_contiguous;
    uint8_t tx_domain_contiguous;
    uint8_t *rxpdo_view;
    uint8_t *txpdo_view;
    uint8_t rxpdo[ECAN_IGH_PDO_MAX_BYTES];
    uint8_t txpdo[ECAN_IGH_PDO_MAX_BYTES];
} ecan_igh_slave_t;

/* IgH 适配层上下文，负责 master/domain 和 raw PDO byte stream。 */
typedef struct {
    ec_master_t *master;
    ec_domain_t *domain;
    uint8_t *domain_pd;
    ec_domain_t *domains[ECAN_IGH_MAX_SLAVES];
    uint8_t *domain_pds[ECAN_IGH_MAX_SLAVES];
    ec_pdo_entry_reg_t slave_domain_regs[ECAN_IGH_MAX_SLAVES][ECAN_IGH_MAX_SLAVE_DOMAIN_REGS];
    ec_domain_t *rx_domains[ECAN_IGH_MAX_SLAVES];
    ec_domain_t *tx_domains[ECAN_IGH_MAX_SLAVES];
    uint8_t *rx_domain_pds[ECAN_IGH_MAX_SLAVES];
    uint8_t *tx_domain_pds[ECAN_IGH_MAX_SLAVES];
    ec_pdo_entry_reg_t rx_slave_domain_regs[ECAN_IGH_MAX_SLAVES][ECAN_IGH_MAX_UNITS + 1U];
    ec_pdo_entry_reg_t tx_slave_domain_regs[ECAN_IGH_MAX_SLAVES][ECAN_IGH_MAX_UNITS + 1U];
    size_t slave_count;
    uint8_t pdo_multiplier;
    uint8_t slave_pdo_multiplier[ECAN_IGH_MAX_SLAVES];
    uint8_t rxpdo_multiplier;
    uint8_t txpdo_multiplier;
    uint8_t slave_rxpdo_multiplier[ECAN_IGH_MAX_SLAVES];
    uint8_t slave_txpdo_multiplier[ECAN_IGH_MAX_SLAVES];
    uint8_t init_flags;
    uint32_t cycle_us;
    uint16_t dc_assign_activate;
    uint32_t dc_sync0_cycle_ns;
    int32_t dc_sync0_shift_ns;
    uint32_t dc_sync1_offset_ns;
    ec_reg_request_t *dc_phase_requests[ECAN_IGH_MAX_SLAVES];
    uint8_t dc_phase_check_started;
    uint8_t dc_phase_check_complete;
    ecan_igh_cycle_mode_t cycle_mode;
    uint32_t dc_prepare_ahead_us;
    uint32_t dc_min_prepare_us;
    uint32_t rxpdo_phase_us;
    uint32_t txpdo_phase_us;
    uint32_t txpdo_collect_phase_us;
    uint8_t domain_complete;
    uint8_t cycle_active;
    uint8_t stopping; /* 停机事务接管后禁止恢复普通PDO业务；close重新初始化后清零。 */
    uint8_t startup_poll_started; /* 已开始启动交换；统计重置不得清除。 */
    uint8_t business_started; /* 已进入业务周期/repeat入口；禁止再使用启动撤销。 */
    uint8_t pdo_queue_uncommitted; /* queue已开始但send未成功返回；禁止后续发送带出旧命令。 */
    unsigned int domain_working_counter;
    ec_wc_state_t domain_wc_state;
    unsigned int domain_expected_working_counter;
    unsigned int split_rx_working_counter;
    ec_wc_state_t split_rx_wc_state;
    uint8_t slaves_operational;
    uint8_t split_cycle_collected; /* A phased DC exchange (split or LRW) was retired. */
    /* Runtime SM ownership survives statistics resets. Timeout is fail-stop
     * until the adapter is closed/reinitialized; no stale PDO retransmission. */
    uint32_t sm_pending_mask;
    uint8_t sm_receive_fault;
    uint8_t dc_receive_fault; /* Latched: 1 output, 2 input, 3 interrupted phase, 4 send deadline. */
    uint64_t sm_last_send_ns;
    uint64_t sm_next_send_ns; /* Absolute SM send deadline; independent of stats reset. */
    uint8_t slave_operational[ECAN_IGH_MAX_SLAVES];
    unsigned int slave_al_state[ECAN_IGH_MAX_SLAVES];
    ecan_igh_slave_t slaves[ECAN_IGH_MAX_SLAVES];
    ec_pdo_entry_reg_t domain_regs[ECAN_IGH_MAX_DOMAIN_REGS];
    ecan_igh_sched_stats_t sched_stats;
    ecan_igh_diag_event_t diag_events[ECAN_IGH_DIAG_EVENT_RING_SIZE];
    uint64_t diag_event_head;
    uint64_t diag_event_dropped;
    uint64_t cycle_start_ns;
    uint64_t cycle_begin_ns;
    uint64_t last_receive_process_ns;
    uint64_t last_txpdo_copy_ns;
    uint64_t last_rxpdo_copy_ns;
    uint64_t last_queue_ns;
    uint64_t last_master_send_ns;
    uint8_t lock_initialized;
    pthread_mutex_t lock;
} ecan_igh_context_t;

/* 填充默认SM配置：3个从站、250us子周期、RxPDO 128B/TxPDO 256B。 */
void ecan_igh_default_config(ecan_igh_config_t *config);
int ecan_igh_config_apply_realtime_profile(ecan_igh_config_t *config,
                                           ecan_igh_realtime_profile_t profile);
const char *ecan_igh_realtime_profile_name(ecan_igh_realtime_profile_t profile);

/* 设置参与初始化的 EtherCAT 从站数量。 */
int ecan_igh_config_set_slave_count(ecan_igh_config_t *config, size_t slave_count);

/* 设置 IgH 应用周期，单位 us。 */
int ecan_igh_config_set_cycle_us(ecan_igh_config_t *config, uint32_t cycle_us);

/* 设置所有从站默认 PDO 倍数，实际 PDO 字节数为 multiplier * 128B。 */
int ecan_igh_config_set_pdo_multiplier(ecan_igh_config_t *config, uint8_t pdo_multiplier);

/* 设置单个从站 PDO 倍数；传 0 表示继续使用全局默认值。 */
int ecan_igh_config_set_slave_pdo_multiplier(ecan_igh_config_t *config,
                                             size_t slave_pos,
                                             uint8_t pdo_multiplier);

/*
 * 独立设置主站输出 RxPDO 和从站输入 TxPDO；原有对称接口仍同时设置两个方向，
 * 保持现有调用兼容。
 */
int ecan_igh_config_set_direction_pdo_multiplier(ecan_igh_config_t *config,
                                                 uint8_t rxpdo_multiplier,
                                                 uint8_t txpdo_multiplier);
int ecan_igh_config_set_slave_direction_pdo_multiplier(
    ecan_igh_config_t *config,
    size_t slave_pos,
    uint8_t rxpdo_multiplier,
    uint8_t txpdo_multiplier);

uint16_t ecan_igh_config_get_slave_rxpdo_bytes(const ecan_igh_config_t *config,
                                               size_t slave_pos);
uint16_t ecan_igh_config_get_slave_txpdo_bytes(const ecan_igh_config_t *config,
                                               size_t slave_pos);

/* 以字节数设置全局 PDO 大小，范围 128..1392B，按 16B 对齐。 */
int ecan_igh_config_set_pdo_bytes(ecan_igh_config_t *config, uint16_t pdo_bytes);

/* 获取全局 PDO 字节数。 */
uint16_t ecan_igh_config_get_pdo_bytes(const ecan_igh_config_t *config);

/* 获取单个从站最终生效的 PDO 字节数。 */
uint16_t ecan_igh_config_get_slave_pdo_bytes(const ecan_igh_config_t *config,
                                             size_t slave_pos);

/* 设置 EtherCAT 从站寻址，slave_pos 是配置数组下标。 */
int ecan_igh_config_set_slave(ecan_igh_config_t *config,
                              size_t slave_pos,
                              int alias,
                              int position);

/* 设置从站 Vendor/Product ID，便于同一个 IgH 层集成其他设备。 */
int ecan_igh_config_set_identity(ecan_igh_config_t *config,
                                 size_t slave_pos,
                                 uint32_t vendor_id,
                                 uint32_t product_id);

/* 追加一个 8bit 启动 SDO，随 ecan_igh_init() 下发。 */
int ecan_igh_config_add_sdo8(ecan_igh_config_t *config,
                             size_t slave_pos,
                             uint16_t index,
                             uint8_t subindex,
                             uint8_t value);

/* 追加一个 16bit 启动 SDO，随 ecan_igh_init() 下发。 */
int ecan_igh_config_add_sdo16(ecan_igh_config_t *config,
                              size_t slave_pos,
                              uint16_t index,
                              uint8_t subindex,
                              uint16_t value);

/* 追加一个 32bit 启动 SDO，随 ecan_igh_init() 下发。 */
int ecan_igh_config_add_sdo32(ecan_igh_config_t *config,
                              size_t slave_pos,
                              uint16_t index,
                              uint8_t subindex,
                              uint32_t value);

/* 开启 DC Sync0，cycle_us 为 0 时跟随 EtherCAT 应用周期。 */
/* Absolute phases: both in [0, cycle), unequal. If sync1 < sync0, publication
 * follows this Sync0 in the next master cycle. Offset is (sync1-sync0) modulo
 * cycle, strictly between zero and one cycle; equal phases are rejected.
 * Requires firmware supporting Sync0 output / Sync1 input synchronization. */
int ecan_igh_config_enable_dc_sync01(ecan_igh_config_t *config,
                                    uint32_t cycle_us,
                                    uint32_t sync0_phase_us,
                                    uint32_t sync1_phase_us);

int ecan_igh_config_enable_dc_sync0(ecan_igh_config_t *config,
                                    uint32_t cycle_us,
                                    int32_t shift_us);

/*
 * 开启已验证的 1ms DC 同周期档。该档按方向拆分 domain，并在一个应用周期内
 * 完成 RxPDO 发送、TxPDO 读取和接收处理。cycle_end() 返回后可立即解析本周期
 * TxPDO；应用必须在调用 cycle_end() 前准备好本周期 RxPDO。
 */
int ecan_igh_config_enable_dc_single_cycle(ecan_igh_config_t *config);

/* Enable one complete DC cycle with separate output/input Ethernet phases. */
int ecan_igh_config_enable_dc_phased_cycle(ecan_igh_config_t *config,
                                           int32_t sync0_shift_us,
                                           uint32_t rxpdo_phase_us,
                                           uint32_t txpdo_phase_us,
                                           uint32_t collect_phase_us);

/* 覆盖同周期档的三个应用相位；用于按 PDO 容量重新验证时序窗口。 */
/* rx < tx sends directions separately; rx == tx co-queues LWR/LRD in one
 * master send (Ethernet frame count depends on MTU). Both retire at collect.
 * Requires rx <= tx < collect < cycle; WKC is checked per direction. */
int ecan_igh_config_set_dc_single_cycle_phases(ecan_igh_config_t *config,
                                               uint32_t rxpdo_phase_us,
                                               uint32_t txpdo_phase_us,
                                               uint32_t collect_phase_us);

/* 返回当前上下文是否使用 DC 同周期档。 */
int ecan_igh_is_dc_single_cycle(const ecan_igh_context_t *ctx);
/* True when fresh input shadows are available after successful cycle_end. */
int ecan_igh_collects_at_cycle_end(const ecan_igh_context_t *ctx);
/* Preserve the configured DC clocks; send one combined domain per slave.
 * Requires send_phase < collect_phase < cycle. No extra EtherCAT read is sent. */
int ecan_igh_config_enable_dc_combined(ecan_igh_config_t *config,
                                      uint32_t send_phase_us, uint32_t collect_phase_us);

/* 通过维护对象重启指定数量的从站，并等待 settle_ms。 */
int ecan_igh_reboot_slaves(uint32_t master_index,
                           size_t slave_count,
                           uint32_t settle_ms);

/* 初始化 IgH master/domain、PDO 映射和启动 SDO。 */
int ecan_igh_init(ecan_igh_context_t *ctx, const ecan_igh_config_t *config);

/* 轮询一次 EtherCAT 状态，刷新 domain WC 和从站 OP 状态。 */
int ecan_igh_poll_op(ecan_igh_context_t *ctx);

/* 无期限等待所有从站进入 OP；遇到明确配置错误仍返回失败。 */
int ecan_igh_wait_op(ecan_igh_context_t *ctx);

/*
 * timeout_ms 为 0 表示不设置等待期限，不忽略明确的配置错误。
 * 启动探测以1kHz推进；长业务周期在首次完整PDO或OP后切换至业务周期。
 * 仅启动等待阶段通过标准IgH从站信息检查配置error_flag；错误返回IGH_ECAN_FAIL，
 * 未就绪仍受原超时限制。正常业务收发不执行该信息查询。
 */
int ecan_igh_wait_op_timeout(ecan_igh_context_t *ctx, uint32_t timeout_ms);

/* 开始一个 EtherCAT 周期，失败时返回错误码。 */
int ecan_igh_cycle_begin_checked(ecan_igh_context_t *ctx);

/* 尝试开始一个 EtherCAT 周期；如果已有周期未结束则返回错误。 */
int ecan_igh_cycle_try_begin(ecan_igh_context_t *ctx);

/*
 * 结束当前 EtherCAT 周期并发送 PDO 输出。DC 同周期档会阻塞到收集相位，
 * 在返回前完成本周期 TxPDO 接收；标准档保持原有的一次收发行为。
 */
int ecan_igh_cycle_end_checked(ecan_igh_context_t *ctx);

/* 放弃当前周期，不发送新的 PDO 输出。 */
int ecan_igh_cycle_abort_checked(ecan_igh_context_t *ctx);

/* 重发上一周期输出，适合需要保持输出的异常恢复路径。 */
int ecan_igh_cycle_repeat_output_checked(ecan_igh_context_t *ctx);

/* 兼容旧 demo 的无返回值周期开始接口。 */
void ecan_igh_cycle_begin(ecan_igh_context_t *ctx);

/* 兼容旧 demo 的无返回值周期结束接口。 */
void ecan_igh_cycle_end(ecan_igh_context_t *ctx);

/* 兼容旧 demo 的无返回值周期放弃接口。 */
void ecan_igh_cycle_abort(ecan_igh_context_t *ctx);

/* 兼容旧 demo 的无返回值重发输出接口。 */
void ecan_igh_cycle_repeat_output(ecan_igh_context_t *ctx);

/* 获取 raw RxPDO 视图。连续 PDO 映射时直接返回 IgH domain 指针，避免中间复制。 */
int ecan_igh_get_rxpdo_buffer(ecan_igh_context_t *ctx,
                              size_t slave_pos,
                              uint8_t **data,
                              size_t *size);

/* 获取 raw TxPDO 视图。连续 PDO 映射时直接返回 IgH domain 指针，避免中间复制。 */
int ecan_igh_get_txpdo_buffer(ecan_igh_context_t *ctx,
                              size_t slave_pos,
                              const uint8_t **data,
                              size_t *size);

/* 返回当前 PDO domain 是否完整，完整表示所有注册 entry 均已映射。 */
int ecan_igh_domain_is_complete(const ecan_igh_context_t *ctx);

/* 返回最近一次 domain working counter。 */
unsigned int ecan_igh_domain_working_counter(const ecan_igh_context_t *ctx);

/* 返回最近一次 domain WC 状态。 */
ec_wc_state_t ecan_igh_domain_wc_state(const ecan_igh_context_t *ctx);

/* 返回所有配置从站是否均处于 OP。 */
int ecan_igh_slaves_are_operational(const ecan_igh_context_t *ctx);

/* 返回单个从站是否处于 OP。 */
int ecan_igh_slave_is_operational(const ecan_igh_context_t *ctx, size_t slave_pos);

/* 返回单个从站最近一次 AL 状态。 */
unsigned int ecan_igh_slave_al_state(const ecan_igh_context_t *ctx, size_t slave_pos);

/* 获取 adapter 周期阶段耗时和 WKC 统计快照。 */
int ecan_igh_get_sched_stats(const ecan_igh_context_t *ctx,
                             ecan_igh_sched_stats_t *stats);

/* 清空运行期调度/WKC 统计；用于将启动和预热阶段排除在正式测量之外。 */
int ecan_igh_reset_runtime_diagnostics(ecan_igh_context_t *ctx);

/* 获取 WKC/overrun 诊断事件。cursor=0 表示读取最近事件。 */
int ecan_igh_get_diag_events(const ecan_igh_context_t *ctx,
                             uint64_t cursor,
                             uint32_t max_events,
                             ecan_igh_diag_event_snapshot_t *snapshot);

/* 打印 adapter 级周期统计和最近诊断事件。 */
void ecan_igh_print_runtime_diagnostics(const ecan_igh_context_t *ctx,
                                        uint32_t max_events);

/* 打印 PDO offset 诊断，供 bring-up 或排查映射问题时使用。 */
void ecan_igh_print_pdo_offsets(const ecan_igh_context_t *ctx);

/* 释放 IgH master 及上下文资源；非实时调用，不能与周期操作并发。
 * 此函数不提供从站到达PREOP的完成确认，也不是电机安全停止接口。
 * 正常DC停止应在释放前完成有界的状态切换和实际AL状态确认。
 * 锁被占用时会记录错误并保留上下文，不得据此假定资源已释放。 */
void ecan_igh_close(ecan_igh_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IGH_ADAPTER_H */
