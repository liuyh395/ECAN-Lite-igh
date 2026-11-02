#ifndef IGH_ADAPTER_INTERNAL_H
#define IGH_ADAPTER_INTERNAL_H

#include "igh_adapter.h"

#define ECAN_IGH_RETURN_IF(condition, result) \
    do { \
        if (condition) { \
            return (result); \
        } \
    } while (0)
#define ECAN_IGH_CHECK_ARG(condition) \
    ECAN_IGH_RETURN_IF(!(condition), IGH_ECAN_INVALID_ARG)
#define ECAN_IGH_RETURN_VOID_IF(condition) \
    do { \
        if (condition) { \
            return; \
        } \
    } while (0)
#define ECAN_IGH_CHECK_VOID(condition) \
    ECAN_IGH_RETURN_VOID_IF(!(condition))
#define ECAN_IGH_FOR_EACH_SLAVE(ctx, slave) \
    for (size_t slave = 0U; slave < (ctx)->slave_count; slave++)
#define ECAN_IGH_FOR_EACH_CONFIG_SLAVE(slave) \
    for (size_t slave = 0U; slave < ECAN_IGH_MAX_SLAVES; slave++)
#define ECAN_IGH_PDO_MULTIPLIER_IS_VALID(multiplier) \
    ecan_igh_pdo_size_encoding_is_valid(multiplier)
#define ECAN_IGH_PDO_BYTES_IS_VALID(bytes) \
    ((bytes) >= (ECAN_IGH_PDO_CHUNK_BYTES * ECAN_IGH_PDO_MULTIPLIER_MIN) && \
     (bytes) <= ECAN_IGH_PDO_MAX_BYTES && \
     ((bytes) % ECAN_IGH_PDO_FINE_BYTES) == 0U)
#define ECAN_IGH_SLAVE_STATE_POLL_INTERVAL  16U
#define ECAN_IGH_SET_ALL_SLAVE_PDO_MULTIPLIERS(config, multiplier) \
    do { \
        ECAN_IGH_FOR_EACH_CONFIG_SLAVE(slave__) { \
            (config)->slave_pdo_multiplier[slave__] = (multiplier); \
        } \
    } while (0)
#define ECAN_IGH_SET_ALL_SLAVE_DIRECTION_PDO_MULTIPLIERS(config, rx_multiplier, tx_multiplier) \
    do { \
        ECAN_IGH_FOR_EACH_CONFIG_SLAVE(slave__) { \
            (config)->slave_rxpdo_multiplier[slave__] = (rx_multiplier); \
            (config)->slave_txpdo_multiplier[slave__] = (tx_multiplier); \
        } \
    } while (0)
#define ECAN_IGH_DEFINE_ADD_SDO(name, sdo_size, value_type) \
    int name(ecan_igh_config_t *config, \
             size_t slave_pos, \
             uint16_t index, \
             uint8_t subindex, \
             value_type value) \
    { \
        return add_startup_sdo(config, slave_pos, index, subindex, (sdo_size), value); \
    }

typedef enum {
    ECAN_IGH_PDO_RX = 0,
    ECAN_IGH_PDO_TX = 1
} ecan_igh_pdo_direction_t;

/* -------------------------------------------------------------------------- */
/* Shared adapter helpers                                                      */
/* -------------------------------------------------------------------------- */

static inline uint16_t rx_unit_index(size_t unit)
{
    return (uint16_t)(ECAN_IGH_OD_RX_ENTRY_BASE + unit * ECAN_IGH_OD_ENTRY_STRIDE);
}

static inline uint16_t tx_unit_index(size_t unit)
{
    return (uint16_t)(ECAN_IGH_OD_TX_ENTRY_BASE + unit * ECAN_IGH_OD_ENTRY_STRIDE);
}

static inline size_t multiplier_to_unit_count(uint8_t multiplier)
{
    return (size_t)ecan_igh_pdo_base_count(multiplier) * ECAN_IGH_UNITS_PER_CHUNK +
           ecan_igh_pdo_fine_count(multiplier);
}

static inline size_t multiplier_to_pdo_bytes(uint8_t multiplier)
{
    return ecan_igh_pdo_decode_bytes(multiplier);
}

static inline uint8_t multiplier_to_mapping_count(uint8_t multiplier)
{
    return ecan_igh_pdo_mapping_count(multiplier);
}

static inline uint8_t mapping_entry_count(uint8_t multiplier, uint8_t mapping)
{
    uint8_t base = ecan_igh_pdo_base_count(multiplier);
    return (mapping < base) ? ECAN_IGH_UNITS_PER_CHUNK :
           ecan_igh_pdo_fine_count(multiplier);
}

static inline uint8_t active_slave_rxpdo_multiplier(const ecan_igh_context_t *ctx,
                                                     size_t slave_pos)
{
    if (ctx == NULL || slave_pos >= ctx->slave_count) {
        return 0U;
    }
    return ctx->slave_rxpdo_multiplier[slave_pos] != 0U ?
           ctx->slave_rxpdo_multiplier[slave_pos] : ctx->rxpdo_multiplier;
}

static inline uint8_t active_slave_txpdo_multiplier(const ecan_igh_context_t *ctx,
                                                     size_t slave_pos)
{
    if (ctx == NULL || slave_pos >= ctx->slave_count) {
        return 0U;
    }
    return ctx->slave_txpdo_multiplier[slave_pos] != 0U ?
           ctx->slave_txpdo_multiplier[slave_pos] : ctx->txpdo_multiplier;
}

static inline size_t active_slave_direction_unit_count(const ecan_igh_context_t *ctx,
                                                        size_t slave_pos,
                                                        ecan_igh_pdo_direction_t direction)
{
    uint8_t multiplier = direction == ECAN_IGH_PDO_RX ?
                         active_slave_rxpdo_multiplier(ctx, slave_pos) :
                         active_slave_txpdo_multiplier(ctx, slave_pos);
    return multiplier_to_unit_count(multiplier);
}

static inline size_t active_slave_direction_pdo_bytes(const ecan_igh_context_t *ctx,
                                                       size_t slave_pos,
                                                       ecan_igh_pdo_direction_t direction)
{
    uint8_t multiplier = direction == ECAN_IGH_PDO_RX ?
                         active_slave_rxpdo_multiplier(ctx, slave_pos) :
                         active_slave_txpdo_multiplier(ctx, slave_pos);
    return multiplier_to_pdo_bytes(multiplier);
}

static inline int domain_units_are_contiguous(const unsigned int *offsets, size_t unit_count)
{
    if ((offsets == NULL) || unit_count == 0U) {
        return 0;
    }
    for (size_t unit = 1U; unit < unit_count; unit++) {
        if (offsets[unit] != offsets[0] + (unsigned int)(unit * ECAN_IGH_UNIT_BYTES)) {
            return 0;
        }
    }
    return 1;
}
static inline uint32_t cycle_us_to_ns(uint32_t cycle_us)
{
    if (cycle_us > (UINT32_MAX / 1000U)) {
        return UINT32_MAX;
    }
    return cycle_us * 1000U;
}

/* -------------------------------------------------------------------------- */
/* Cross-module adapter internals                                              */
/* -------------------------------------------------------------------------- */

int ecan_igh_adapter_config_is_valid(const ecan_igh_config_t *config);
int ecan_igh_adapter_apply_startup_sdos(ecan_igh_context_t *ctx,
                                        const ecan_igh_config_t *config);
void ecan_igh_adapter_build_pdo_infos(ecan_igh_slave_t *slave_ctx,
                                      uint8_t rx_multiplier,
                                      uint8_t tx_multiplier);
int ecan_igh_adapter_configure_pdo_assignment(ec_slave_config_t *sc,
                                               uint8_t rx_multiplier,
                                               uint8_t tx_multiplier);
int ecan_igh_adapter_build_slave_domain_regs(ecan_igh_context_t *ctx, size_t slave_pos);
int ecan_igh_adapter_build_slave_direction_domain_regs(
    ecan_igh_context_t *ctx,
    size_t slave_pos,
    ecan_igh_pdo_direction_t direction);
void ecan_igh_adapter_cache_slave_domain_views(ecan_igh_context_t *ctx);
void ecan_igh_adapter_copy_domain_txpdo_to_shadow(ecan_igh_context_t *ctx);
void ecan_igh_adapter_copy_shadow_rxpdo_to_domain(ecan_igh_context_t *ctx);
int ecan_igh_adapter_get_pdo_view(ecan_igh_context_t *ctx,
                                  size_t slave_pos,
                                  ecan_igh_pdo_direction_t direction,
                                  void **data,
                                  size_t *size);
uint64_t ecan_igh_adapter_monotonic_ns(void);
int ecan_igh_adapter_dc_phase_matches(const uint8_t *registers,
                                      uint32_t cycle_ns, uint32_t phase0_ns,
                                      uint32_t offset_ns);
void ecan_igh_adapter_update_max_u64(uint64_t *slot, uint64_t value);

#endif /* IGH_ADAPTER_INTERNAL_H */
