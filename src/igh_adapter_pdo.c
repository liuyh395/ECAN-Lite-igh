#include "igh_adapter_internal.h"

#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Process image copy helpers                                                  */
/* -------------------------------------------------------------------------- */

/* Copy scattered EtherCAT PDO units into a contiguous shadow buffer. */
static void copy_domain_units_to_linear_buffer(uint8_t *dst,
                                               const uint8_t *domain_pd,
                                               const unsigned int *offsets,
                                               size_t unit_count)
{
    if ((dst == NULL) || (domain_pd == NULL) || (offsets == NULL) || unit_count == 0U) {
        return;
    }
    if (domain_units_are_contiguous(offsets, unit_count)) {
        memcpy(dst, domain_pd + offsets[0], unit_count * ECAN_IGH_UNIT_BYTES);
        return;
    }
    for (size_t unit = 0U; unit < unit_count; unit++) {
        memcpy(&dst[unit * ECAN_IGH_UNIT_BYTES], domain_pd + offsets[unit],
               ECAN_IGH_UNIT_BYTES);
    }
}

/* Copy a contiguous shadow buffer back into scattered EtherCAT PDO units. */
static void copy_linear_buffer_to_domain_units(uint8_t *domain_pd,
                                               const uint8_t *src,
                                               const unsigned int *offsets,
                                               size_t unit_count)
{
    if ((domain_pd == NULL) || (src == NULL) || (offsets == NULL) || unit_count == 0U) {
        return;
    }
    if (domain_units_are_contiguous(offsets, unit_count)) {
        memcpy(domain_pd + offsets[0], src, unit_count * ECAN_IGH_UNIT_BYTES);
        return;
    }
    for (size_t unit = 0U; unit < unit_count; unit++) {
        memcpy(domain_pd + offsets[unit], &src[unit * ECAN_IGH_UNIT_BYTES],
               ECAN_IGH_UNIT_BYTES);
    }
}

/* Refresh non-contiguous TxPDO shadows after the master has received inputs. */
void ecan_igh_adapter_copy_domain_txpdo_to_shadow(ecan_igh_context_t *ctx)
{
    uint64_t t0;

    if (ctx == NULL) {
        return;
    }

    t0 = ecan_igh_adapter_monotonic_ns();
    ECAN_IGH_FOR_EACH_SLAVE(ctx, slave) {
        ecan_igh_slave_t *slave_ctx = &ctx->slaves[slave];

        if (slave_ctx->tx_domain_contiguous == 0U) {
            copy_domain_units_to_linear_buffer(slave_ctx->txpdo,
                                               slave_ctx->tx_domain_pd,
                                               slave_ctx->tx_offsets,
                                               active_slave_direction_unit_count(
                                                   ctx, slave, ECAN_IGH_PDO_TX));
        }
    }
    ctx->last_txpdo_copy_ns = ecan_igh_adapter_monotonic_ns() - t0;
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.txpdo_copy_max_ns,
                                    ctx->last_txpdo_copy_ns);
}

/* Publish non-contiguous RxPDO shadows before the master sends outputs. */
void ecan_igh_adapter_copy_shadow_rxpdo_to_domain(ecan_igh_context_t *ctx)
{
    uint64_t t0;

    if (ctx == NULL) {
        return;
    }

    t0 = ecan_igh_adapter_monotonic_ns();
    ECAN_IGH_FOR_EACH_SLAVE(ctx, slave) {
        ecan_igh_slave_t *slave_ctx = &ctx->slaves[slave];

        if (slave_ctx->rx_domain_contiguous == 0U) {
            copy_linear_buffer_to_domain_units(slave_ctx->rx_domain_pd,
                                               slave_ctx->rxpdo,
                                               slave_ctx->rx_offsets,
                                               active_slave_direction_unit_count(
                                                   ctx, slave, ECAN_IGH_PDO_RX));
        }
    }
    ctx->last_rxpdo_copy_ns = ecan_igh_adapter_monotonic_ns() - t0;
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.rxpdo_copy_max_ns,
                                    ctx->last_rxpdo_copy_ns);
}

/* Cache direct PDO views when IgH registered units are contiguous. */
void ecan_igh_adapter_cache_slave_domain_views(ecan_igh_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    for (size_t slave_pos = 0U; slave_pos < ctx->slave_count; slave_pos++) {
        ecan_igh_slave_t *slave = &ctx->slaves[slave_pos];
        size_t rx_unit_count = active_slave_direction_unit_count(
            ctx, slave_pos, ECAN_IGH_PDO_RX);
        size_t tx_unit_count = active_slave_direction_unit_count(
            ctx, slave_pos, ECAN_IGH_PDO_TX);

        slave->rx_domain_contiguous =
            domain_units_are_contiguous(slave->rx_offsets, rx_unit_count) ? 1U : 0U;
        slave->tx_domain_contiguous =
            domain_units_are_contiguous(slave->tx_offsets, tx_unit_count) ? 1U : 0U;
        slave->rxpdo_view = (slave->rx_domain_contiguous && slave->rx_domain_pd != NULL) ?
                            (slave->rx_domain_pd + slave->rx_offsets[0]) :
                            slave->rxpdo;
        slave->txpdo_view = (slave->tx_domain_contiguous && slave->tx_domain_pd != NULL) ?
                            (slave->tx_domain_pd + slave->tx_offsets[0]) :
                            slave->txpdo;
    }
}

/* -------------------------------------------------------------------------- */
/* PDO registration and diagnostics                                            */
/* -------------------------------------------------------------------------- */

/* Build IgH PDO entry registration records for one slave-local domain. */
int ecan_igh_adapter_build_slave_domain_regs(ecan_igh_context_t *ctx, size_t slave_pos)
{
    size_t n = 0U;
    ecan_igh_slave_t *slave_ctx;
    ec_pdo_entry_reg_t *regs;
    size_t unit_count;

    ECAN_IGH_CHECK_ARG(ctx != NULL && slave_pos < ctx->slave_count);

    slave_ctx = &ctx->slaves[slave_pos];
    regs = ctx->slave_domain_regs[slave_pos];
    unit_count = active_slave_direction_unit_count(ctx, slave_pos, ECAN_IGH_PDO_RX);
    memset(regs, 0, sizeof(ctx->slave_domain_regs[slave_pos]));
    for (size_t unit = 0U; unit < unit_count; unit++) {
        if (n + 1U >= ECAN_IGH_MAX_SLAVE_DOMAIN_REGS) {
            return IGH_ECAN_NO_SPACE;
        }
        regs[n++] = (ec_pdo_entry_reg_t){
            (uint16_t)slave_ctx->config.alias,
            (uint16_t)slave_ctx->config.position,
            slave_ctx->config.vendor_id,
            slave_ctx->config.product_id,
            rx_unit_index(unit),
            IGH_ECAN_OD_SUBINDEX_VALUE,
            &slave_ctx->rx_offsets[unit],
            NULL
        };
    }
    unit_count = active_slave_direction_unit_count(ctx, slave_pos, ECAN_IGH_PDO_TX);
    for (size_t unit = 0U; unit < unit_count; unit++) {
        if (n + 1U >= ECAN_IGH_MAX_SLAVE_DOMAIN_REGS) {
            return IGH_ECAN_NO_SPACE;
        }
        regs[n++] = (ec_pdo_entry_reg_t){
            (uint16_t)slave_ctx->config.alias,
            (uint16_t)slave_ctx->config.position,
            slave_ctx->config.vendor_id,
            slave_ctx->config.product_id,
            tx_unit_index(unit),
            IGH_ECAN_OD_SUBINDEX_VALUE,
            &slave_ctx->tx_offsets[unit],
            NULL
        };
    }
    return IGH_ECAN_OK;
}

/* Build one direction-only registration list for the DC single-cycle profile. */
int ecan_igh_adapter_build_slave_direction_domain_regs(
    ecan_igh_context_t *ctx,
    size_t slave_pos,
    ecan_igh_pdo_direction_t direction)
{
    ecan_igh_slave_t *slave_ctx;
    ec_pdo_entry_reg_t *regs;
    unsigned int *offsets;
    size_t unit_count;

    ECAN_IGH_CHECK_ARG(ctx != NULL && slave_pos < ctx->slave_count);
    ECAN_IGH_CHECK_ARG(direction == ECAN_IGH_PDO_RX || direction == ECAN_IGH_PDO_TX);

    slave_ctx = &ctx->slaves[slave_pos];
    unit_count = active_slave_direction_unit_count(ctx, slave_pos, direction);
    if (direction == ECAN_IGH_PDO_RX) {
        regs = ctx->rx_slave_domain_regs[slave_pos];
        offsets = slave_ctx->rx_offsets;
        memset(regs, 0, sizeof(ctx->rx_slave_domain_regs[slave_pos]));
    } else {
        regs = ctx->tx_slave_domain_regs[slave_pos];
        offsets = slave_ctx->tx_offsets;
        memset(regs, 0, sizeof(ctx->tx_slave_domain_regs[slave_pos]));
    }

    for (size_t unit = 0U; unit < unit_count; unit++) {
        regs[unit] = (ec_pdo_entry_reg_t){
            (uint16_t)slave_ctx->config.alias,
            (uint16_t)slave_ctx->config.position,
            slave_ctx->config.vendor_id,
            slave_ctx->config.product_id,
            (direction == ECAN_IGH_PDO_RX) ? rx_unit_index(unit) : tx_unit_index(unit),
            IGH_ECAN_OD_SUBINDEX_VALUE,
            &offsets[unit],
            NULL
        };
    }
    return IGH_ECAN_OK;
}

/* Check whether a sequence of unit offsets maps to one linear PDO buffer. */
static int pdo_offsets_are_contiguous(const unsigned int *offsets,
                                      size_t first_unit,
                                      size_t unit_count)
{
    unsigned int expected;

    if ((offsets == NULL) || unit_count == 0U) {
        return 0;
    }

    expected = offsets[first_unit];
    for (size_t i = 0U; i < unit_count; i++) {
        if (offsets[first_unit + i] != expected) {
            return 0;
        }
        expected += ECAN_IGH_UNIT_BYTES;
    }
    return 1;
}

/* Print one PDO chunk's entry indexes and process-image offsets. */
static void print_pdo_offset_chunk(size_t slave,
                                   const char *dir,
                                   uint16_t pdo_index,
                                   uint16_t first_entry_index,
                                   const unsigned int *offsets,
                                   size_t first_unit)
{
    int contiguous = pdo_offsets_are_contiguous(offsets,
                                                first_unit,
                                                ECAN_IGH_UNITS_PER_CHUNK);

    printf("[diag] pdo-offset slave=%zu dir=%s pdo=0x%04X entries=0x%04X..0x%04X"
           " offsets=%u,%u,%u,%u,%u,%u,%u,%u contiguous=%s\n",
           slave,
           dir,
           pdo_index,
           first_entry_index,
           (uint16_t)(first_entry_index +
                      (ECAN_IGH_UNITS_PER_CHUNK - 1U) * ECAN_IGH_OD_ENTRY_STRIDE),
           offsets[first_unit + 0U],
           offsets[first_unit + 1U],
           offsets[first_unit + 2U],
           offsets[first_unit + 3U],
           offsets[first_unit + 4U],
           offsets[first_unit + 5U],
           offsets[first_unit + 6U],
           offsets[first_unit + 7U],
           contiguous ? "yes" : "no");
}

/* Print a diagnostic map of every dynamic PDO chunk registered with IgH. */
void ecan_igh_print_pdo_offsets(const ecan_igh_context_t *ctx)
{
    if (ctx == NULL || ctx->slave_count == 0U) {
        return;
    }

    printf("[diag] pdo-offset-table slaves=%zu unit_bytes=%u"
           " note=rxpdo-master-output txpdo-slave-input\n",
           ctx->slave_count,
           (unsigned)ECAN_IGH_UNIT_BYTES);

    for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
        const ecan_igh_slave_t *slave_ctx = &ctx->slaves[slave];
        uint8_t rx_multiplier = active_slave_rxpdo_multiplier(ctx, slave);
        uint8_t tx_multiplier = active_slave_txpdo_multiplier(ctx, slave);
        uint8_t rx_mapping_count = multiplier_to_mapping_count(rx_multiplier);
        uint8_t tx_mapping_count = multiplier_to_mapping_count(tx_multiplier);
        size_t rx_unit_count = multiplier_to_unit_count(rx_multiplier);
        size_t tx_unit_count = multiplier_to_unit_count(tx_multiplier);
        int rx_contiguous;
        int tx_contiguous;

        if (rx_unit_count == 0U || rx_unit_count > ECAN_IGH_MAX_UNITS ||
            tx_unit_count == 0U || tx_unit_count > ECAN_IGH_MAX_UNITS) {
            printf("[diag] pdo-offset-summary slave=%zu invalid-units=%zu/%zu\n",
                   slave, rx_unit_count, tx_unit_count);
            continue;
        }
        rx_contiguous = pdo_offsets_are_contiguous(slave_ctx->rx_offsets, 0U, rx_unit_count);
        tx_contiguous = pdo_offsets_are_contiguous(slave_ctx->tx_offsets, 0U, tx_unit_count);

        printf("[diag] pdo-offset-summary slave=%zu rxpdo_bytes=%zu txpdo_bytes=%zu"
               " rx_base=%u rx_end=%u rx_contiguous=%s"
               " tx_base=%u tx_end=%u tx_contiguous=%s\n",
               slave,
               multiplier_to_pdo_bytes(rx_multiplier),
               multiplier_to_pdo_bytes(tx_multiplier),
               slave_ctx->rx_offsets[0],
               slave_ctx->rx_offsets[rx_unit_count - 1U] + ECAN_IGH_UNIT_BYTES - 1U,
               rx_contiguous ? "yes" : "no",
               slave_ctx->tx_offsets[0],
               slave_ctx->tx_offsets[tx_unit_count - 1U] + ECAN_IGH_UNIT_BYTES - 1U,
               tx_contiguous ? "yes" : "no");

        for (uint8_t chunk = 0U; chunk < rx_mapping_count; chunk++) {
            size_t first_unit = (size_t)chunk * ECAN_IGH_UNITS_PER_CHUNK;
            uint8_t entries = mapping_entry_count(rx_multiplier, chunk);
            if (entries == ECAN_IGH_UNITS_PER_CHUNK) {
                print_pdo_offset_chunk(slave, "rxpdo",
                                       (uint16_t)(ECAN_IGH_OD_RXPDO_MAP_BASE + chunk),
                                       rx_unit_index(first_unit), slave_ctx->rx_offsets,
                                       first_unit);
            } else {
                printf("[diag] pdo-offset-tail slave=%zu dir=rxpdo pdo=0x%04X"
                       " entries=%u base=%u end=%u\n",
                       slave, (uint16_t)(ECAN_IGH_OD_RXPDO_MAP_BASE + chunk), entries,
                       slave_ctx->rx_offsets[first_unit],
                       slave_ctx->rx_offsets[first_unit + entries - 1U] +
                           ECAN_IGH_UNIT_BYTES - 1U);
            }
        }
        for (uint8_t chunk = 0U; chunk < tx_mapping_count; chunk++) {
            size_t first_unit = (size_t)chunk * ECAN_IGH_UNITS_PER_CHUNK;
            uint8_t entries = mapping_entry_count(tx_multiplier, chunk);
            if (entries == ECAN_IGH_UNITS_PER_CHUNK) {
                print_pdo_offset_chunk(slave, "txpdo",
                                       (uint16_t)(ECAN_IGH_OD_TXPDO_MAP_BASE + chunk),
                                       tx_unit_index(first_unit), slave_ctx->tx_offsets,
                                       first_unit);
            } else {
                printf("[diag] pdo-offset-tail slave=%zu dir=txpdo pdo=0x%04X"
                       " entries=%u base=%u end=%u\n",
                       slave, (uint16_t)(ECAN_IGH_OD_TXPDO_MAP_BASE + chunk), entries,
                       slave_ctx->tx_offsets[first_unit],
                       slave_ctx->tx_offsets[first_unit + entries - 1U] +
                           ECAN_IGH_UNIT_BYTES - 1U);
            }
        }
    }
    fflush(stdout);
}

/* -------------------------------------------------------------------------- */
/* Slave PDO configuration                                                     */
/* -------------------------------------------------------------------------- */

/* Build IgH PDO entry/PDO/sync descriptors for one ECAN-Lite4 slave. */
void ecan_igh_adapter_build_pdo_infos(ecan_igh_slave_t *slave_ctx,
                                      uint8_t rx_multiplier,
                                      uint8_t tx_multiplier)
{
    size_t rx_unit_count;
    size_t tx_unit_count;
    uint8_t rx_mapping_count;
    uint8_t tx_mapping_count;

    if (slave_ctx == NULL) {
        return;
    }

    rx_unit_count = multiplier_to_unit_count(rx_multiplier);
    tx_unit_count = multiplier_to_unit_count(tx_multiplier);
    rx_mapping_count = multiplier_to_mapping_count(rx_multiplier);
    tx_mapping_count = multiplier_to_mapping_count(tx_multiplier);
    memset(slave_ctx->rx_entries, 0, sizeof(slave_ctx->rx_entries));
    memset(slave_ctx->tx_entries, 0, sizeof(slave_ctx->tx_entries));
    memset(slave_ctx->rx_pdos, 0, sizeof(slave_ctx->rx_pdos));
    memset(slave_ctx->tx_pdos, 0, sizeof(slave_ctx->tx_pdos));
    memset(slave_ctx->syncs, 0, sizeof(slave_ctx->syncs));

    for (size_t unit = 0U; unit < rx_unit_count; unit++) {
        slave_ctx->rx_entries[unit] = (ec_pdo_entry_info_t){
            rx_unit_index(unit),
            IGH_ECAN_OD_SUBINDEX_VALUE,
            (uint8_t)(ECAN_IGH_UNIT_BYTES * 8U)
        };
    }
    for (size_t unit = 0U; unit < tx_unit_count; unit++) {
        slave_ctx->tx_entries[unit] = (ec_pdo_entry_info_t){
            tx_unit_index(unit),
            IGH_ECAN_OD_SUBINDEX_VALUE,
            (uint8_t)(ECAN_IGH_UNIT_BYTES * 8U)
        };
    }

    for (uint8_t chunk = 0U; chunk < rx_mapping_count; chunk++) {
        slave_ctx->rx_pdos[chunk] = (ec_pdo_info_t){
            (uint16_t)(ECAN_IGH_OD_RXPDO_MAP_BASE + chunk),
            mapping_entry_count(rx_multiplier, chunk),
            &slave_ctx->rx_entries[(size_t)chunk * ECAN_IGH_UNITS_PER_CHUNK]
        };
    }
    for (uint8_t chunk = 0U; chunk < tx_mapping_count; chunk++) {
        slave_ctx->tx_pdos[chunk] = (ec_pdo_info_t){
            (uint16_t)(ECAN_IGH_OD_TXPDO_MAP_BASE + chunk),
            mapping_entry_count(tx_multiplier, chunk),
            &slave_ctx->tx_entries[(size_t)chunk * ECAN_IGH_UNITS_PER_CHUNK]
        };
    }

    slave_ctx->syncs[0] = (ec_sync_info_t){0, EC_DIR_OUTPUT, 0U, NULL, EC_WD_DISABLE};
    slave_ctx->syncs[1] = (ec_sync_info_t){1, EC_DIR_INPUT, 0U, NULL, EC_WD_DISABLE};
    slave_ctx->syncs[2] = (ec_sync_info_t){2, EC_DIR_OUTPUT, rx_mapping_count, slave_ctx->rx_pdos, EC_WD_ENABLE};
    slave_ctx->syncs[3] = (ec_sync_info_t){3, EC_DIR_INPUT, tx_mapping_count, slave_ctx->tx_pdos, EC_WD_DISABLE};
    slave_ctx->syncs[4] = (ec_sync_info_t){ECAN_IGH_SYNC_END, EC_DIR_INVALID, 0U, NULL, EC_WD_DEFAULT};
}

/* Queue 0x1C12/0x1C13 assignment SDOs to select the first N dynamic chunks. */
int ecan_igh_adapter_configure_pdo_assignment(ec_slave_config_t *sc,
                                               uint8_t rx_multiplier,
                                               uint8_t tx_multiplier)
{
    uint8_t rx_mapping_count = multiplier_to_mapping_count(rx_multiplier);
    uint8_t tx_mapping_count = multiplier_to_mapping_count(tx_multiplier);

    if (ecrt_slave_config_sdo8(sc, ECAN_IGH_OD_RXPDO_ASSIGN,
                               IGH_ECAN_OD_SUBINDEX_VALUE, 0) != 0 ||
        ecrt_slave_config_sdo8(sc, ECAN_IGH_OD_TXPDO_ASSIGN,
                               IGH_ECAN_OD_SUBINDEX_VALUE, 0) != 0) {
        return IGH_ECAN_FAIL;
    }
    for (uint8_t chunk = 0U; chunk < rx_mapping_count; chunk++) {
        if (ecrt_slave_config_sdo16(sc, ECAN_IGH_OD_RXPDO_ASSIGN,
                                    (uint8_t)(chunk + ECAN_IGH_OD_ASSIGN_FIRST_ENTRY),
                                    (uint16_t)(ECAN_IGH_OD_RXPDO_MAP_BASE + chunk)) != 0) {
            return IGH_ECAN_FAIL;
        }
    }
    for (uint8_t chunk = 0U; chunk < tx_mapping_count; chunk++) {
        if (ecrt_slave_config_sdo16(sc, ECAN_IGH_OD_TXPDO_ASSIGN,
                                    (uint8_t)(chunk + ECAN_IGH_OD_ASSIGN_FIRST_ENTRY),
                                    (uint16_t)(ECAN_IGH_OD_TXPDO_MAP_BASE + chunk)) != 0) {
            return IGH_ECAN_FAIL;
        }
    }
    if (ecrt_slave_config_sdo8(sc, ECAN_IGH_OD_RXPDO_ASSIGN,
                               IGH_ECAN_OD_SUBINDEX_VALUE, rx_mapping_count) != 0 ||
        ecrt_slave_config_sdo8(sc, ECAN_IGH_OD_TXPDO_ASSIGN,
                               IGH_ECAN_OD_SUBINDEX_VALUE, tx_mapping_count) != 0) {
        return IGH_ECAN_FAIL;
    }
    return IGH_ECAN_OK;
}

/* Return the active RxPDO or TxPDO buffer view for one slave. */
int ecan_igh_adapter_get_pdo_view(ecan_igh_context_t *ctx,
                        size_t slave_pos,
                        ecan_igh_pdo_direction_t direction,
                        void **data,
                        size_t *size)
{
    ecan_igh_slave_t *slave;
    uint8_t *view;

    ECAN_IGH_CHECK_ARG(ctx != NULL && data != NULL && slave_pos < ctx->slave_count);
    slave = &ctx->slaves[slave_pos];
    if (direction == ECAN_IGH_PDO_RX) {
        view = (slave->rxpdo_view != NULL) ? slave->rxpdo_view : slave->rxpdo;
    } else {
        view = (slave->txpdo_view != NULL) ? slave->txpdo_view : slave->txpdo;
    }
    *data = view;
    if (size != NULL) {
        *size = active_slave_direction_pdo_bytes(ctx, slave_pos, direction);
    }
    return IGH_ECAN_OK;
}

/* Return the master-output PDO buffer that the application writes. */
int ecan_igh_get_rxpdo_buffer(ecan_igh_context_t *ctx,
                              size_t slave_pos,
                              uint8_t **data,
                              size_t *size)
{
    void *view = NULL;
    int rc = ecan_igh_adapter_get_pdo_view(ctx, slave_pos, ECAN_IGH_PDO_RX, &view, size);

    if (rc == IGH_ECAN_OK) {
        *data = (uint8_t *)view;
    }
    return rc;
}

/* Return the slave-input PDO buffer that the application reads. */
int ecan_igh_get_txpdo_buffer(ecan_igh_context_t *ctx,
                              size_t slave_pos,
                              const uint8_t **data,
                              size_t *size)
{
    void *view = NULL;
    int rc = ecan_igh_adapter_get_pdo_view(ctx, slave_pos, ECAN_IGH_PDO_TX, &view, size);

    if (rc == IGH_ECAN_OK) {
        *data = (const uint8_t *)view;
    }
    return rc;
}
