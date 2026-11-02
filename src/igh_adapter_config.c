#define _DEFAULT_SOURCE

#include "igh_adapter_internal.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static int sleep_ms(uint32_t delay_ms)
{
    struct timespec delay;

    delay.tv_sec = (time_t)(delay_ms / 1000U);
    delay.tv_nsec = (long)(delay_ms % 1000U) * 1000000L;
    while (nanosleep(&delay, &delay) != 0) {
        if (errno != EINTR) {
            return IGH_ECAN_FAIL;
        }
    }
    return IGH_ECAN_OK;
}

/* Startup SDO helpers                                                         */
/* -------------------------------------------------------------------------- */

/* Queue a single configured startup SDO into IgH's slave config. */
static int apply_startup_sdo(ec_slave_config_t *sc, const ecan_igh_startup_sdo_t *sdo)
{
    switch (sdo->size) {
    case ECAN_IGH_SDO_U8:
        return ecrt_slave_config_sdo8(sc, sdo->index, sdo->subindex, (uint8_t)sdo->value);
    case ECAN_IGH_SDO_U16:
        return ecrt_slave_config_sdo16(sc, sdo->index, sdo->subindex, (uint16_t)sdo->value);
    case ECAN_IGH_SDO_U32:
        return ecrt_slave_config_sdo32(sc, sdo->index, sdo->subindex, sdo->value);
    default:
        return -1;
    }
}

/* Apply all user and MCAN startup SDOs to their target slave configs. */
int ecan_igh_adapter_apply_startup_sdos(ecan_igh_context_t *ctx, const ecan_igh_config_t *config)
{
    for (size_t i = 0U; i < config->startup_sdo_count; i++) {
        const ecan_igh_startup_sdo_t *sdo = &config->startup_sdos[i];
        if (sdo->slave_pos >= ctx->slave_count) {
            return IGH_ECAN_INVALID_ARG;
        }
        if (apply_startup_sdo(ctx->slaves[sdo->slave_pos].sc, sdo) != 0) {
            fprintf(stderr,
                    "startup SDO queue failed: slave=%zu index=0x%04X sub=0x%02X\n",
                    sdo->slave_pos,
                    sdo->index,
                    sdo->subindex);
            return IGH_ECAN_FAIL;
        }
    }
    return IGH_ECAN_OK;
}

/* Append one startup SDO record to the plain IgH configuration. */
static int add_startup_sdo(ecan_igh_config_t *config,
                           size_t slave_pos,
                           uint16_t index,
                           uint8_t subindex,
                           ecan_igh_sdo_size_t size,
                           uint32_t value)
{
    ecan_igh_startup_sdo_t *sdo;

    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(slave_pos < ECAN_IGH_MAX_SLAVES);
    ECAN_IGH_CHECK_ARG(config->startup_sdo_count < ECAN_IGH_MAX_STARTUP_SDOS);

    sdo = &config->startup_sdos[config->startup_sdo_count++];
    sdo->slave_pos = slave_pos;
    sdo->index = index;
    sdo->subindex = subindex;
    sdo->size = size;
    sdo->value = value;
    return IGH_ECAN_OK;
}

/* Validate the static IgH configuration before requesting the master. */
int ecan_igh_adapter_config_is_valid(const ecan_igh_config_t *config)
{
    if ((config == NULL) ||
        config->master_index < 0 ||
        config->slave_count == 0U ||
        config->slave_count > ECAN_IGH_MAX_SLAVES ||
        config->startup_sdo_count > ECAN_IGH_MAX_STARTUP_SDOS ||
        config->cycle_us == 0U ||
        config->cycle_us > (UINT32_MAX / 1000U) ||
        !ECAN_IGH_PDO_MULTIPLIER_IS_VALID(config->pdo_multiplier)) {
        return 0;
    }
    if (config->dc_assign_activate == ECAN_IGH_DC_ASSIGN_SYNC01) {
        if (config->dc_sync0_cycle_ns != cycle_us_to_ns(config->cycle_us) ||
            config->dc_sync0_shift_ns < 0 || config->dc_sync1_offset_ns == 0U ||
            (uint32_t)config->dc_sync0_shift_ns >= config->dc_sync0_cycle_ns ||
            config->dc_sync1_offset_ns >= config->dc_sync0_cycle_ns ||
            (config->init_flags & ECAN_IGH_INIT_SKIP_STARTUP_SDO) != 0U) return 0;
    } else if (config->dc_sync1_offset_ns != 0U) {
        return 0;
    }
    if (config->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
        if ((config->dc_assign_activate != ECAN_IGH_DC_ASSIGN_SYNC0 &&
             config->dc_assign_activate != ECAN_IGH_DC_ASSIGN_SYNC01) ||
            config->rxpdo_phase_us > config->txpdo_phase_us ||
            config->txpdo_phase_us >= config->txpdo_collect_phase_us ||
            config->txpdo_collect_phase_us >= config->cycle_us) {
            return 0;
        }
    } else if (config->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_COMBINED) {
        if ((config->dc_assign_activate != ECAN_IGH_DC_ASSIGN_SYNC0 &&
             config->dc_assign_activate != ECAN_IGH_DC_ASSIGN_SYNC01) ||
            config->rxpdo_phase_us >= config->txpdo_collect_phase_us ||
            config->txpdo_collect_phase_us >= config->cycle_us) return 0;
    } else if (config->cycle_mode != ECAN_IGH_CYCLE_MODE_STANDARD) {
        return 0;
    }
    if (config->dc_prepare_ahead_us != 0U &&
        ((config->cycle_mode != ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE &&
          config->cycle_mode != ECAN_IGH_CYCLE_MODE_DC_COMBINED) ||
         config->dc_prepare_ahead_us >= config->cycle_us - config->txpdo_collect_phase_us)) return 0;
    if (config->dc_min_prepare_us != 0U &&
        ((config->cycle_mode != ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE &&
          config->cycle_mode != ECAN_IGH_CYCLE_MODE_DC_COMBINED) ||
         config->dc_sync0_shift_ns <= 0 ||
         (uint64_t)config->rxpdo_phase_us * 1000ULL +
         (uint64_t)config->dc_min_prepare_us * 1000ULL > (uint32_t)config->dc_sync0_shift_ns)) return 0;
    for (size_t slave = 0U; slave < config->slave_count; slave++) {
        uint8_t multiplier = (config->slave_pdo_multiplier[slave] != 0U) ?
                             config->slave_pdo_multiplier[slave] :
                             config->pdo_multiplier;
        uint8_t rx_multiplier = (config->slave_rxpdo_multiplier[slave] != 0U) ?
                                config->slave_rxpdo_multiplier[slave] :
                                config->rxpdo_multiplier;
        uint8_t tx_multiplier = (config->slave_txpdo_multiplier[slave] != 0U) ?
                                config->slave_txpdo_multiplier[slave] :
                                config->txpdo_multiplier;
        const ecan_igh_slave_config_t *slave_config = &config->slaves[slave];

        if (!ECAN_IGH_PDO_MULTIPLIER_IS_VALID(multiplier) ||
            !ECAN_IGH_PDO_MULTIPLIER_IS_VALID(rx_multiplier) ||
            !ECAN_IGH_PDO_MULTIPLIER_IS_VALID(tx_multiplier) ||
            slave_config->alias < 0 || slave_config->alias > UINT16_MAX ||
            slave_config->position < 0 || slave_config->position > UINT16_MAX ||
            slave_config->vendor_id == 0U || slave_config->product_id == 0U) {
            return 0;
        }
        for (size_t previous = 0U; previous < slave; previous++) {
            if (config->slaves[previous].alias == slave_config->alias &&
                config->slaves[previous].position == slave_config->position) {
                return 0;
            }
        }
    }
    for (size_t i = 0U; i < config->startup_sdo_count; i++) {
        const ecan_igh_startup_sdo_t *sdo = &config->startup_sdos[i];

        if (config->dc_assign_activate == ECAN_IGH_DC_ASSIGN_SYNC01 &&
            sdo->subindex == 1U && (sdo->index == 0x1C32U || sdo->index == 0x1C33U) &&
            (sdo->size != ECAN_IGH_SDO_U16 ||
             sdo->value != (sdo->index == 0x1C32U ? 2U : 3U))) return 0;
        if (sdo->slave_pos >= config->slave_count ||
            (sdo->size != ECAN_IGH_SDO_U8 &&
             sdo->size != ECAN_IGH_SDO_U16 &&
             sdo->size != ECAN_IGH_SDO_U32)) {
            return 0;
        }
    }
    return 1;
}

/* Public configuration API                                                    */
/* -------------------------------------------------------------------------- */

static int apply_realtime_profile(ecan_igh_config_t *config,
                                  ecan_igh_realtime_profile_t profile)
{
    uint8_t rxpdo_multiplier;
    uint8_t txpdo_multiplier;

    ECAN_IGH_CHECK_ARG(config != NULL);
    switch (profile) {
    case ECAN_IGH_PROFILE_SM_250US_1MS:
        config->cycle_us = ECAN_RT_SM_ECAT_CYCLE_US;
        config->dc_assign_activate = 0U;
        config->dc_sync0_cycle_ns = 0U;
        config->dc_sync0_shift_ns = 0;
        rxpdo_multiplier = ECAN_RT_SM_RXPDO_MULTIPLIER;
        txpdo_multiplier = ECAN_RT_SM_TXPDO_MULTIPLIER;
        break;
    case ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE:
        config->cycle_us = ECAN_RT_DC_ECAT_CYCLE_US;
        config->dc_assign_activate = ECAN_IGH_DC_ASSIGN_SYNC0;
        config->dc_sync0_cycle_ns = cycle_us_to_ns(ECAN_RT_DC_ECAT_CYCLE_US);
        config->dc_sync0_shift_ns = ECAN_RT_DC_SYNC0_SHIFT_US * 1000;
        rxpdo_multiplier = ECAN_RT_DC_RXPDO_MULTIPLIER;
        txpdo_multiplier = ECAN_RT_DC_TXPDO_MULTIPLIER;
        break;
    default:
        return IGH_ECAN_INVALID_ARG;
    }

    config->dc_sync1_offset_ns = 0U;
    /* 两个产品 profile 都使用一次标准 PDO 交换；分相交换仅保留给诊断工具。 */
    config->cycle_mode = ECAN_IGH_CYCLE_MODE_STANDARD;
    config->dc_prepare_ahead_us = 0U;
    config->dc_min_prepare_us = 0U;
    config->rxpdo_phase_us = 0U;
    config->txpdo_phase_us = 0U;
    config->txpdo_collect_phase_us = 0U;
    config->pdo_multiplier =
        (ecan_igh_pdo_decode_bytes(rxpdo_multiplier) >
         ecan_igh_pdo_decode_bytes(txpdo_multiplier)) ?
        rxpdo_multiplier : txpdo_multiplier;
    ECAN_IGH_SET_ALL_SLAVE_PDO_MULTIPLIERS(config, config->pdo_multiplier);
    config->rxpdo_multiplier = rxpdo_multiplier;
    config->txpdo_multiplier = txpdo_multiplier;
    ECAN_IGH_SET_ALL_SLAVE_DIRECTION_PDO_MULTIPLIERS(
        config, rxpdo_multiplier, txpdo_multiplier);
    return IGH_ECAN_OK;
}

/* 默认采用 SM 250 us 子周期；四个子周期构成 1 ms 控制周期。 */
void ecan_igh_default_config(ecan_igh_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->master_index = 0;
    config->slave_count = 3U;
    config->pdo_multiplier = ECAN_IGH_PDO_DEFAULT_MULTIPLIER;
    ECAN_IGH_SET_ALL_SLAVE_PDO_MULTIPLIERS(config, config->pdo_multiplier);
    config->rxpdo_multiplier = config->pdo_multiplier;
    config->txpdo_multiplier = config->pdo_multiplier;
    ECAN_IGH_SET_ALL_SLAVE_DIRECTION_PDO_MULTIPLIERS(
        config, config->pdo_multiplier, config->pdo_multiplier);
    config->init_flags = 0U;
    config->cycle_us = IGH_ECAN_DEFAULT_CYCLE_US;
    config->dc_assign_activate = 0U;
    config->dc_sync0_cycle_ns = 0U;
    config->dc_sync0_shift_ns = 0;
    config->cycle_mode = ECAN_IGH_CYCLE_MODE_STANDARD;
    config->rxpdo_phase_us = 0U;
    config->txpdo_phase_us = 0U;
    config->txpdo_collect_phase_us = 0U;
    ECAN_IGH_FOR_EACH_CONFIG_SLAVE(slave) {
        config->slaves[slave].alias = 0;
        config->slaves[slave].position = (int)slave;
        config->slaves[slave].vendor_id = ECAN_IGH_VENDOR_ID;
        config->slaves[slave].product_id = ECAN_IGH_PRODUCT_ID;
    }
    (void)apply_realtime_profile(config, ECAN_IGH_PROFILE_SM_250US_1MS);
}

int ecan_igh_config_apply_realtime_profile(ecan_igh_config_t *config,
                                           ecan_igh_realtime_profile_t profile)
{
    return apply_realtime_profile(config, profile);
}

const char *ecan_igh_realtime_profile_name(ecan_igh_realtime_profile_t profile)
{
    switch (profile) {
    case ECAN_IGH_PROFILE_SM_250US_1MS:
        return "sm-250us-1ms";
    case ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE:
        return "dc-1ms-cross-cycle-shift900us";
    default:
        return "invalid";
    }
}

/* Set how many configured slaves are active in this run. */
int ecan_igh_config_set_slave_count(ecan_igh_config_t *config, size_t slave_count)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(slave_count > 0U && slave_count <= ECAN_IGH_MAX_SLAVES);
    config->slave_count = slave_count;
    return IGH_ECAN_OK;
}

/* Set the application cycle used by polling and optional DC sync. */
int ecan_igh_config_set_cycle_us(ecan_igh_config_t *config, uint32_t cycle_us)
{
    ECAN_IGH_CHECK_ARG(config != NULL && cycle_us != 0U &&
                       cycle_us <= (UINT32_MAX / 1000U));
    config->cycle_us = cycle_us;
    if (config->dc_assign_activate != 0U && config->dc_sync0_cycle_ns == 0U) {
        config->dc_sync0_cycle_ns = cycle_us_to_ns(cycle_us);
    }
    return IGH_ECAN_OK;
}

/* Set one PDO multiplier for every slave. */
int ecan_igh_config_set_pdo_multiplier(ecan_igh_config_t *config, uint8_t pdo_multiplier)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(ECAN_IGH_PDO_MULTIPLIER_IS_VALID(pdo_multiplier));
    config->pdo_multiplier = pdo_multiplier;
    ECAN_IGH_SET_ALL_SLAVE_PDO_MULTIPLIERS(config, pdo_multiplier);
    config->rxpdo_multiplier = pdo_multiplier;
    config->txpdo_multiplier = pdo_multiplier;
    ECAN_IGH_SET_ALL_SLAVE_DIRECTION_PDO_MULTIPLIERS(
        config, pdo_multiplier, pdo_multiplier);
    return IGH_ECAN_OK;
}

/* Override the PDO multiplier for one slave while preserving global defaults. */
int ecan_igh_config_set_slave_pdo_multiplier(ecan_igh_config_t *config,
                                             size_t slave_pos,
                                             uint8_t pdo_multiplier)
{
    ECAN_IGH_CHECK_ARG(config != NULL && slave_pos < ECAN_IGH_MAX_SLAVES);
    ECAN_IGH_CHECK_ARG(pdo_multiplier == 0U ||
                       ECAN_IGH_PDO_MULTIPLIER_IS_VALID(pdo_multiplier));
    config->slave_pdo_multiplier[slave_pos] = pdo_multiplier;
    config->slave_rxpdo_multiplier[slave_pos] = pdo_multiplier;
    config->slave_txpdo_multiplier[slave_pos] = pdo_multiplier;
    return IGH_ECAN_OK;
}

int ecan_igh_config_set_direction_pdo_multiplier(ecan_igh_config_t *config,
                                                 uint8_t rxpdo_multiplier,
                                                 uint8_t txpdo_multiplier)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(ECAN_IGH_PDO_MULTIPLIER_IS_VALID(rxpdo_multiplier) &&
                       ECAN_IGH_PDO_MULTIPLIER_IS_VALID(txpdo_multiplier));
    config->rxpdo_multiplier = rxpdo_multiplier;
    config->txpdo_multiplier = txpdo_multiplier;
    ECAN_IGH_SET_ALL_SLAVE_DIRECTION_PDO_MULTIPLIERS(
        config, rxpdo_multiplier, txpdo_multiplier);
    return IGH_ECAN_OK;
}

int ecan_igh_config_set_slave_direction_pdo_multiplier(
    ecan_igh_config_t *config,
    size_t slave_pos,
    uint8_t rxpdo_multiplier,
    uint8_t txpdo_multiplier)
{
    ECAN_IGH_CHECK_ARG(config != NULL && slave_pos < ECAN_IGH_MAX_SLAVES);
    ECAN_IGH_CHECK_ARG((rxpdo_multiplier == 0U ||
                        ECAN_IGH_PDO_MULTIPLIER_IS_VALID(rxpdo_multiplier)) &&
                       (txpdo_multiplier == 0U ||
                        ECAN_IGH_PDO_MULTIPLIER_IS_VALID(txpdo_multiplier)));
    config->slave_rxpdo_multiplier[slave_pos] = rxpdo_multiplier;
    config->slave_txpdo_multiplier[slave_pos] = txpdo_multiplier;
    return IGH_ECAN_OK;
}

/* Set PDO bytes by encoding 128B base chunks plus one optional 16B-granular tail. */
int ecan_igh_config_set_pdo_bytes(ecan_igh_config_t *config, uint16_t pdo_bytes)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(ECAN_IGH_PDO_BYTES_IS_VALID(pdo_bytes));
    config->pdo_multiplier = ecan_igh_pdo_encode_bytes(pdo_bytes);
    ECAN_IGH_SET_ALL_SLAVE_PDO_MULTIPLIERS(config, config->pdo_multiplier);
    config->rxpdo_multiplier = config->pdo_multiplier;
    config->txpdo_multiplier = config->pdo_multiplier;
    ECAN_IGH_SET_ALL_SLAVE_DIRECTION_PDO_MULTIPLIERS(
        config, config->pdo_multiplier, config->pdo_multiplier);
    return IGH_ECAN_OK;
}

/* Return the global PDO byte length configured for each direction. */
uint16_t ecan_igh_config_get_pdo_bytes(const ecan_igh_config_t *config)
{
    if (config == NULL) {
        return 0U;
    }
    return ecan_igh_pdo_decode_bytes(config->pdo_multiplier);
}

/* Return the effective per-slave PDO byte length. */
uint16_t ecan_igh_config_get_slave_pdo_bytes(const ecan_igh_config_t *config,
                                             size_t slave_pos)
{
    uint8_t multiplier;

    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES) {
        return 0U;
    }
    multiplier = (config->slave_pdo_multiplier[slave_pos] != 0U) ?
                 config->slave_pdo_multiplier[slave_pos] :
                 config->pdo_multiplier;
    return ecan_igh_pdo_decode_bytes(multiplier);
}

uint16_t ecan_igh_config_get_slave_rxpdo_bytes(const ecan_igh_config_t *config,
                                               size_t slave_pos)
{
    uint8_t multiplier;

    if (config == NULL || slave_pos >= ECAN_IGH_MAX_SLAVES) {
        return 0U;
    }
    multiplier = config->slave_rxpdo_multiplier[slave_pos] != 0U ?
                 config->slave_rxpdo_multiplier[slave_pos] :
                 config->rxpdo_multiplier;
    return ecan_igh_pdo_decode_bytes(multiplier);
}

uint16_t ecan_igh_config_get_slave_txpdo_bytes(const ecan_igh_config_t *config,
                                               size_t slave_pos)
{
    uint8_t multiplier;

    if (config == NULL || slave_pos >= ECAN_IGH_MAX_SLAVES) {
        return 0U;
    }
    multiplier = config->slave_txpdo_multiplier[slave_pos] != 0U ?
                 config->slave_txpdo_multiplier[slave_pos] :
                 config->txpdo_multiplier;
    return ecan_igh_pdo_decode_bytes(multiplier);
}

/* Set EtherCAT alias and ring position for one configured slave. */
int ecan_igh_config_set_slave(ecan_igh_config_t *config,
                              size_t slave_pos,
                              int alias,
                              int position)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(slave_pos < ECAN_IGH_MAX_SLAVES &&
                       alias >= 0 && alias <= UINT16_MAX &&
                       position >= 0 && position <= UINT16_MAX);
    config->slaves[slave_pos].alias = alias;
    config->slaves[slave_pos].position = position;
    return IGH_ECAN_OK;
}

/* Set expected EtherCAT identity for one configured slave. */
int ecan_igh_config_set_identity(ecan_igh_config_t *config,
                                 size_t slave_pos,
                                 uint32_t vendor_id,
                                 uint32_t product_id)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(slave_pos < ECAN_IGH_MAX_SLAVES &&
                       vendor_id != 0U &&
                       product_id != 0U);
    config->slaves[slave_pos].vendor_id = vendor_id;
    config->slaves[slave_pos].product_id = product_id;
    return IGH_ECAN_OK;
}

ECAN_IGH_DEFINE_ADD_SDO(ecan_igh_config_add_sdo8, ECAN_IGH_SDO_U8, uint8_t)
ECAN_IGH_DEFINE_ADD_SDO(ecan_igh_config_add_sdo16, ECAN_IGH_SDO_U16, uint16_t)
ECAN_IGH_DEFINE_ADD_SDO(ecan_igh_config_add_sdo32, ECAN_IGH_SDO_U32, uint32_t)

/* Enable DC Sync0 with cycle/shift converted from microseconds to nanoseconds. */
int ecan_igh_config_enable_dc_sync0(ecan_igh_config_t *config,
                                    uint32_t cycle_us,
                                    int32_t shift_us)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    if (cycle_us == 0U) {
        cycle_us = config->cycle_us;
    }
    ECAN_IGH_CHECK_ARG(cycle_us != 0U &&
                       cycle_us <= (UINT32_MAX / 1000U) &&
                       shift_us <= (INT32_MAX / 1000) &&
                       shift_us >= (INT32_MIN / 1000));
    config->dc_assign_activate = ECAN_IGH_DC_ASSIGN_SYNC0;
    config->dc_sync0_cycle_ns = cycle_us_to_ns(cycle_us);
    config->dc_sync0_shift_ns = shift_us * 1000;
    config->dc_sync1_offset_ns = 0U;
    return IGH_ECAN_OK;
}

int ecan_igh_config_enable_dc_sync01(ecan_igh_config_t *config,
                                    uint32_t cycle_us,
                                    uint32_t sync0_phase_us,
                                    uint32_t sync1_phase_us)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    if (cycle_us == 0U) cycle_us = config->cycle_us;
    ECAN_IGH_CHECK_ARG(cycle_us == config->cycle_us &&
                       cycle_us != 0U && cycle_us <= UINT32_MAX / 1000U &&
                       sync0_phase_us < cycle_us && sync1_phase_us < cycle_us &&
                       sync0_phase_us != sync1_phase_us &&
                       sync0_phase_us <= (uint32_t)(INT32_MAX / 1000));
    config->dc_assign_activate = ECAN_IGH_DC_ASSIGN_SYNC01;
    config->dc_sync0_cycle_ns = cycle_us_to_ns(cycle_us);
    config->dc_sync0_shift_ns = (int32_t)sync0_phase_us * 1000;
    /* 相位按主站周期定义，ESC 需要的是本次 Sync0 到后续 Sync1 的正间隔。
     * 较小的 Sync1 相位对应下一主站周期；不改变 Sync0 周期或增加子周期。 */
    config->dc_sync1_offset_ns = (sync1_phase_us > sync0_phase_us ?
        sync1_phase_us - sync0_phase_us :
        cycle_us - sync0_phase_us + sync1_phase_us) * 1000U;
    return IGH_ECAN_OK;
}

/* Enable the measured one-cycle DC exchange profile without changing defaults. */
int ecan_igh_config_enable_dc_single_cycle(ecan_igh_config_t *config)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(config->cycle_us == ECAN_IGH_DC_SINGLE_CYCLE_US);
    return ecan_igh_config_enable_dc_phased_cycle(
        config, ECAN_IGH_DC_SINGLE_SYNC0_SHIFT_US,
        ECAN_IGH_DC_SINGLE_RXPDO_PHASE_US,
        ECAN_IGH_DC_SINGLE_TXPDO_PHASE_US,
        ECAN_IGH_DC_SINGLE_COLLECT_PHASE_US);
}

int ecan_igh_config_enable_dc_phased_cycle(ecan_igh_config_t *config,
                                           int32_t sync0_shift_us,
                                           uint32_t rxpdo_phase_us,
                                           uint32_t txpdo_phase_us,
                                           uint32_t collect_phase_us)
{
    int rc;

    ECAN_IGH_CHECK_ARG(config != NULL && config->cycle_us != 0U);
    ECAN_IGH_CHECK_ARG(rxpdo_phase_us <= txpdo_phase_us &&
                       txpdo_phase_us < collect_phase_us &&
                       collect_phase_us < config->cycle_us);
    rc = ecan_igh_config_enable_dc_sync0(config, config->cycle_us,
                                         sync0_shift_us);
    if (rc != IGH_ECAN_OK) return rc;
    config->cycle_mode = ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE;
    config->rxpdo_phase_us = rxpdo_phase_us;
    config->txpdo_phase_us = txpdo_phase_us;
    config->txpdo_collect_phase_us = collect_phase_us;
    return IGH_ECAN_OK;
}

int ecan_igh_config_set_dc_single_cycle_phases(ecan_igh_config_t *config,
                                               uint32_t rxpdo_phase_us,
                                               uint32_t txpdo_phase_us,
                                               uint32_t collect_phase_us)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(config->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE);
    ECAN_IGH_CHECK_ARG(rxpdo_phase_us <= txpdo_phase_us &&
                       txpdo_phase_us < collect_phase_us &&
                       collect_phase_us < config->cycle_us);

    config->rxpdo_phase_us = rxpdo_phase_us;
    config->txpdo_phase_us = txpdo_phase_us;
    config->txpdo_collect_phase_us = collect_phase_us;
    return IGH_ECAN_OK;
}

int ecan_igh_config_enable_dc_combined(ecan_igh_config_t *config,
                                      uint32_t send_phase_us, uint32_t collect_phase_us)
{
    ECAN_IGH_CHECK_ARG(config != NULL);
    ECAN_IGH_CHECK_ARG(config->dc_assign_activate == ECAN_IGH_DC_ASSIGN_SYNC0 ||
                       config->dc_assign_activate == ECAN_IGH_DC_ASSIGN_SYNC01);
    ECAN_IGH_CHECK_ARG(send_phase_us < collect_phase_us && collect_phase_us < config->cycle_us);
    config->cycle_mode = ECAN_IGH_CYCLE_MODE_DC_COMBINED;
    config->rxpdo_phase_us = send_phase_us;
    config->txpdo_phase_us = send_phase_us;
    config->txpdo_collect_phase_us = collect_phase_us;
    return IGH_ECAN_OK;
}

/* Send the ECAN-Lite4 reboot command to slaves in reverse order and wait. */
int ecan_igh_reboot_slaves(uint32_t master_index,
                           size_t slave_count,
                           uint32_t settle_ms)
{
    ec_master_t *master;
    uint8_t reboot_command = 1U;
    int rc = IGH_ECAN_OK;

    if (slave_count == 0U || slave_count > ECAN_IGH_MAX_SLAVES) {
        return IGH_ECAN_INVALID_ARG;
    }

    master = ecrt_request_master(master_index);
    if (master == NULL) {
        perror("ecrt_request_master");
        return IGH_ECAN_NOT_READY;
    }

    for (size_t i = slave_count; i > 0U; i--) {
        uint32_t abort_code = 0U;
        uint16_t slave_pos = (uint16_t)(i - 1U);
        int sdo_rc = ecrt_master_sdo_download(master,
                                              slave_pos,
                                              ECAN_IGH_OD_DEVICE_CONTROL,
                                              ECAN_IGH_OD_DEVICE_REBOOT,
                                              &reboot_command,
                                              sizeof(reboot_command),
                                              &abort_code);
        if (sdo_rc != 0) {
            fprintf(stderr,
                    "slave reboot SDO failed: position=%u rc=%d abort=0x%08X\n",
                    slave_pos,
                    sdo_rc,
                    abort_code);
            rc = IGH_ECAN_FAIL;
        }
    }

    ecrt_release_master(master);
    if (settle_ms != 0U && sleep_ms(settle_ms) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }
    return rc;
}
