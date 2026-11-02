#include "igh_mcan_port_internal.h"

#include <string.h>

static uint8_t channel_mask_all(void)
{
    return IGH_ECAN_MCAN_CHANNEL_MASK_ALL;
}

/* -------------------------------------------------------------------------- */
/* MCAN startup SDO composition                                                */
/* -------------------------------------------------------------------------- */

static void merge_result(int *rc, int result)
{
    if ((rc != NULL) && (*rc == IGH_ECAN_OK) && (result != IGH_ECAN_OK)) {
        *rc = result;
    }
}

/* Validate the EtherCAT side before appending MCAN startup SDOs. */
static int igh_config_is_valid_for_mcan(const ecan_igh_config_t *config)
{
    if ((config == NULL) ||
        config->slave_count == 0U ||
        config->slave_count > ECAN_IGH_MAX_SLAVES ||
        !ecan_igh_pdo_size_encoding_is_valid(config->pdo_multiplier)) {
        return 0;
    }
    for (size_t slave = 0U; slave < config->slave_count; slave++) {
        uint8_t multiplier = (config->slave_pdo_multiplier[slave] != 0U) ?
                             config->slave_pdo_multiplier[slave] :
                             config->pdo_multiplier;
        uint8_t rx_multiplier = config->slave_rxpdo_multiplier[slave] != 0U ?
                                config->slave_rxpdo_multiplier[slave] :
                                config->rxpdo_multiplier;
        uint8_t tx_multiplier = config->slave_txpdo_multiplier[slave] != 0U ?
                                config->slave_txpdo_multiplier[slave] :
                                config->txpdo_multiplier;
        if (!ecan_igh_pdo_size_encoding_is_valid(multiplier) ||
            !ecan_igh_pdo_size_encoding_is_valid(rx_multiplier) ||
            !ecan_igh_pdo_size_encoding_is_valid(tx_multiplier)) {
            return 0;
        }
    }
    return 1;
}

/* Keep MCAN profile options limited to features the current firmware supports. */
static int mcan_options_are_supported(uint8_t option_flags)
{
    const uint8_t supported_options = IGH_MCAN_OPT_ALIGN4 |
                                      IGH_MCAN_OPT_NO_CRC16 |
                                      IGH_MCAN_OPT_APP_BYPASS |
                                      IGH_MCAN_OPT_PAYLOAD_ONLY |
                                      IGH_MCAN_OPT_HEADER_V2;

    return (option_flags & (uint8_t)~supported_options) == 0U;
}

static int mcan_budget_policies_are_supported(const igh_mcan_port_config_t *config)
{
    uint8_t tx_fine;
    uint8_t rx_fine;

    if ((config == NULL) ||
        ((config->tx_budget_policy & (uint8_t)~IGH_MCAN_TX_BUDGET_ALLOWED_MASK) != 0U) ||
        ((config->rx_budget_policy & (uint8_t)~IGH_MCAN_RX_BUDGET_ALLOWED_MASK) != 0U)) {
        return 0;
    }

    tx_fine = (uint8_t)(config->tx_budget_policy &
                        (IGH_MCAN_TX_BYPASS_MCAN_READ |
                         IGH_MCAN_TX_BYPASS_BUILD |
                         IGH_MCAN_TX_BYPASS_COPY));
    rx_fine = (uint8_t)(config->rx_budget_policy &
                        (IGH_MCAN_RX_BYPASS_STAGE |
                         IGH_MCAN_RX_BYPASS_PARSE |
                         IGH_MCAN_RX_BYPASS_TX_RING |
                         IGH_MCAN_RX_BYPASS_MCAN_WRITE));

    if (((config->option_flags & IGH_MCAN_OPT_APP_BYPASS) != 0U) &&
        ((tx_fine != 0U) || (rx_fine != 0U))) {
        return 0;
    }
    if (((tx_fine & IGH_MCAN_TX_BYPASS_BUILD) != 0U) &&
        ((tx_fine & IGH_MCAN_TX_BYPASS_COPY) == 0U)) {
        return 0;
    }

    return (rx_fine == 0U) ||
           (rx_fine == IGH_MCAN_RX_BYPASS_MCAN_WRITE) ||
           (rx_fine == (IGH_MCAN_RX_BYPASS_TX_RING |
                        IGH_MCAN_RX_BYPASS_MCAN_WRITE)) ||
           (rx_fine == (IGH_MCAN_RX_BYPASS_PARSE |
                        IGH_MCAN_RX_BYPASS_TX_RING |
                        IGH_MCAN_RX_BYPASS_MCAN_WRITE)) ||
           (rx_fine == (IGH_MCAN_RX_BYPASS_STAGE |
                        IGH_MCAN_RX_BYPASS_PARSE |
                        IGH_MCAN_RX_BYPASS_TX_RING |
                        IGH_MCAN_RX_BYPASS_MCAN_WRITE));
}

/* Append one direction's 0x8010/0x8011 PDO profile SDOs and apply it. */
static int add_pdo_profile_sdos(ecan_igh_config_t *igh_config,
                                size_t slave,
                                uint16_t index,
                                uint8_t pdo_multiplier,
                                const igh_mcan_port_config_t *mcan_config,
                                uint8_t budget_policy,
                                uint8_t option_flags)
{
    int rc = IGH_ECAN_OK;
    const igh_mcan_slave_config_t *cfg = &mcan_config->slaves[slave];

    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_PROFILE_PDO_MULTIPLIER,
                                               pdo_multiplier));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_PROFILE_CHANNEL_MASK,
                                               cfg->channel_mask));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_PROFILE_ENCAPSULATION,
                                               mcan_config->encapsulation_profile));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_PROFILE_BUDGET,
                                               budget_policy));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_PROFILE_OPTION_FLAGS,
                                               option_flags));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_PROFILE_APPLY,
                                               IGH_ECAN_OD_APPLY_COMMAND));
    return rc;
}

/* Append one MCAN channel's minimal startup SDOs and apply them. */
static int add_channel_sdos(ecan_igh_config_t *igh_config,
                            size_t slave,
                            uint8_t channel,
                            const igh_mcan_slave_config_t *cfg)
{
    int rc = IGH_ECAN_OK;
    uint16_t index = (uint16_t)(IGH_MCAN_OD_CHANNEL_BASE + channel);
    uint8_t channel_bit = IGH_ECAN_BIT_U8(channel);
    int is_canfd = ((cfg->canfd_mask & channel_bit) != 0U);
    uint8_t rx_fifo0_elems = is_canfd ? IGH_ECAN_MCAN_CANFD_RXFIFO0_DEFAULT :
                                       IGH_ECAN_MCAN_CLASSIC_RXFIFO0_DEFAULT;
    uint8_t rx_fifo1_elems = is_canfd ? IGH_ECAN_MCAN_CANFD_RXFIFO1_DEFAULT :
                                       IGH_ECAN_MCAN_CLASSIC_RXFIFO1_DEFAULT;
    uint8_t rx_buf_elems = is_canfd ? IGH_ECAN_MCAN_CANFD_RXBUF_DEFAULT :
                                     IGH_ECAN_MCAN_CLASSIC_RXBUF_DEFAULT;
    uint8_t tx_dedicated_elems = is_canfd ? IGH_ECAN_MCAN_CANFD_TXBUF_DEFAULT :
                                           IGH_ECAN_MCAN_CLASSIC_TXBUF_DEFAULT;
    uint8_t tx_fifo_elems = is_canfd ? IGH_ECAN_MCAN_CANFD_TXFIFO_DEFAULT :
                                      IGH_ECAN_MCAN_CLASSIC_TXFIFO_DEFAULT;
    uint8_t tx_event_elems = is_canfd ? IGH_ECAN_MCAN_CANFD_TXEVT_DEFAULT :
                                       IGH_ECAN_MCAN_CLASSIC_TXEVT_DEFAULT;
    uint8_t data_size = is_canfd ? IGH_ECAN_MCAN_DATA_SIZE_64_BYTES :
                                  IGH_ECAN_MCAN_DATA_SIZE_8_BYTES;

    if ((cfg->channel_mask & channel_bit) == 0U) {
        return IGH_ECAN_OK;
    }

    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_TIMING_MODE,
                                               IGH_MCAN_BIT_TIMING_AUTO));
    merge_result(&rc, ecan_igh_config_add_sdo32(igh_config, slave, index,
                                                IGH_MCAN_OD_CHANNEL_DATA_POLICY,
                                                cfg->channel_policy[channel]));
    merge_result(&rc, ecan_igh_config_add_sdo32(igh_config, slave, index,
                                                IGH_MCAN_OD_CHANNEL_NOMINAL_BITRATE,
                                                cfg->nominal_bitrate));
    merge_result(&rc, ecan_igh_config_add_sdo16(igh_config, slave, index,
                                                IGH_MCAN_OD_CHANNEL_NOMINAL_SP_MIN_X10,
                                                cfg->nominal_sp_min_x10));
    merge_result(&rc, ecan_igh_config_add_sdo16(igh_config, slave, index,
                                                IGH_MCAN_OD_CHANNEL_NOMINAL_SP_MAX_X10,
                                                cfg->nominal_sp_max_x10));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_CANFD_ENABLE,
                                               is_canfd ? 1U : 0U));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_RXFIFO0_ELEMS,
                                               rx_fifo0_elems));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_RXFIFO1_ELEMS,
                                               rx_fifo1_elems));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_RXBUF_ELEMS,
                                               rx_buf_elems));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_RXFIFO0_DATA_SIZE,
                                               data_size));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_RXFIFO1_DATA_SIZE,
                                               data_size));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_RXBUF_DATA_SIZE,
                                               data_size));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_TX_FIFO_MODE,
                                               IGH_ECAN_MCAN_TX_FIFO_MODE));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_TX_DEDICATED_ELEMS,
                                               tx_dedicated_elems));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_TX_FIFO_ELEMS,
                                               tx_fifo_elems));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_TX_DATA_SIZE,
                                               data_size));
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_TX_EVENT_ELEMS,
                                               tx_event_elems));
    if (is_canfd) {
        merge_result(&rc, ecan_igh_config_add_sdo32(igh_config, slave, index,
                                                    IGH_MCAN_OD_CHANNEL_DATA_BITRATE,
                                                    cfg->data_bitrate));
        merge_result(&rc, ecan_igh_config_add_sdo16(igh_config, slave, index,
                                                    IGH_MCAN_OD_CHANNEL_DATA_SP_MIN_X10,
                                                    cfg->data_sp_min_x10));
        merge_result(&rc, ecan_igh_config_add_sdo16(igh_config, slave, index,
                                                    IGH_MCAN_OD_CHANNEL_DATA_SP_MAX_X10,
                                                    cfg->data_sp_max_x10));
        merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                                   IGH_MCAN_OD_CHANNEL_TDC_ENABLE,
                                                   IGH_ECAN_DEFAULT_CANFD_TDC_ENABLE));
        merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                                   IGH_MCAN_OD_CHANNEL_TDC_OFFSET,
                                                   IGH_ECAN_DEFAULT_CANFD_TDC_OFFSET));
        merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                                   IGH_MCAN_OD_CHANNEL_TDC_FILTER,
                                                   IGH_ECAN_DEFAULT_CANFD_TDC_FILTER));
    }

    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_APPLY,
                                               IGH_ECAN_OD_APPLY_COMMAND));
    return rc;
}

/* Append only explicitly requested 0x8021+channel overrides. Keeping these
 * SDOs out of the default path preserves compatibility with older firmware.
 */
static int add_channel_datapath_sdos(ecan_igh_config_t *igh_config,
                                     size_t slave,
                                     uint8_t channel,
                                     const igh_mcan_slave_config_t *cfg)
{
    int rc = IGH_ECAN_OK;
    uint8_t channel_bit = IGH_ECAN_BIT_U8(channel);
    uint16_t index = (uint16_t)(IGH_MCAN_OD_DATAPATH_CHANNEL_BASE + channel);
    int override_rx = (cfg->datapath_rx_budget_override_mask & channel_bit) != 0U;
    int override_tx = (cfg->datapath_tx_budget_override_mask & channel_bit) != 0U;

    if (((cfg->channel_mask & channel_bit) == 0U) || (!override_rx && !override_tx)) {
        return IGH_ECAN_OK;
    }

    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_DATAPATH_ENABLE,
                                               1U));
    if (override_rx) {
        merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                                   IGH_MCAN_OD_DATAPATH_RX_BUDGET_MAX,
                                                   cfg->datapath_rx_budget_max[channel]));
    }
    if (override_tx) {
        merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                                   IGH_MCAN_OD_DATAPATH_TX_BUDGET_MAX,
                                                   cfg->datapath_tx_budget_max[channel]));
    }
    merge_result(&rc, ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_DATAPATH_APPLY,
                                               IGH_ECAN_OD_APPLY_COMMAND));
    return rc;
}

/* -------------------------------------------------------------------------- */
/* Public configuration API                                                    */
/* -------------------------------------------------------------------------- */

int igh_mcan_firmware_supports_protocol_v2(uint16_t public_version_word)
{
    uint16_t normalized = (uint16_t)(public_version_word & 0x0FFFU);
    uint8_t major = (uint8_t)((normalized >> 8U) & 0x0FU);

    return major >= IGH_MCAN_FW_PROTOCOL_V2_MIN_MAJOR;
}

int igh_mcan_firmware_capabilities_decode(
    uint16_t public_version_word,
    uint16_t build_word,
    uint32_t pdo_feature_flags,
    igh_mcan_firmware_capabilities_t *capabilities)
{
    uint16_t normalized;

    if (capabilities == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }

    normalized = (uint16_t)(public_version_word & 0x0FFFU);
    memset(capabilities, 0, sizeof(*capabilities));
    capabilities->major = (uint8_t)((normalized >> 8U) & 0x0FU);
    capabilities->minor = (uint8_t)(normalized & 0x00FFU);
    capabilities->build = build_word;
    capabilities->public_version_word = normalized;
    capabilities->pdo_feature_flags = pdo_feature_flags;
    capabilities->protocol_v2 =
        (uint8_t)igh_mcan_firmware_supports_protocol_v2(normalized);
    capabilities->header_v2 =
        (uint8_t)(capabilities->protocol_v2 != 0U &&
                  (pdo_feature_flags & IGH_MCAN_PDO_FEATURE_HEADER_V2) != 0U);
    capabilities->fine_size_16 =
        (uint8_t)(capabilities->protocol_v2 != 0U &&
                  (pdo_feature_flags & IGH_MCAN_PDO_FEATURE_FINE_SIZE_16) != 0U);
    return IGH_ECAN_OK;
}

int igh_mcan_port_select_pdo_protocol(
    igh_mcan_port_config_t *config,
    const igh_mcan_firmware_capabilities_t *capabilities,
    uint8_t protocol_version,
    int disable_crc16)
{
    uint8_t option_flags;

    if (config == NULL ||
        (protocol_version != IGH_MCAN_HDR_VERSION_V1 &&
         protocol_version != IGH_MCAN_HDR_VERSION_V2)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (protocol_version == IGH_MCAN_HDR_VERSION_V2 &&
        (capabilities == NULL || capabilities->protocol_v2 == 0U ||
         capabilities->header_v2 == 0U)) {
        return IGH_ECAN_UNSUPPORTED;
    }

    option_flags = (uint8_t)(config->option_flags &
                             (uint8_t)~(IGH_MCAN_OPT_HEADER_V2 |
                                        IGH_MCAN_OPT_NO_CRC16));
    if (protocol_version == IGH_MCAN_HDR_VERSION_V2) {
        option_flags |= IGH_MCAN_OPT_HEADER_V2;
    }
    if (disable_crc16 != 0) {
        option_flags |= IGH_MCAN_OPT_NO_CRC16;
    }
    config->option_flags = option_flags;
    return IGH_ECAN_OK;
}

/* Initialize per-slave MCAN defaults used by simple bring-up demos. */
void igh_mcan_port_default_config(igh_mcan_port_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->encapsulation_profile = 0U;
    config->option_flags = IGH_MCAN_OPT_DEFAULT;
    config->tx_budget_policy = 0U;
    config->rx_budget_policy = 0U;
    for (size_t i = 0U; i < ECAN_IGH_MAX_SLAVES; i++) {
        config->slaves[i].channel_mask = channel_mask_all();
        config->slaves[i].canfd_mask = 0U;
        config->slaves[i].nominal_bitrate = IGH_ECAN_DEFAULT_CAN_BITRATE;
        config->slaves[i].data_bitrate = IGH_ECAN_DEFAULT_CANFD_BITRATE;
        config->slaves[i].nominal_sp_min_x10 = IGH_ECAN_DEFAULT_CAN20_SP_MIN_X10;
        config->slaves[i].nominal_sp_max_x10 = IGH_ECAN_DEFAULT_CAN20_SP_MAX_X10;
        config->slaves[i].data_sp_min_x10 = IGH_ECAN_DEFAULT_CANFD_SP_MIN_X10;
        config->slaves[i].data_sp_max_x10 = IGH_ECAN_DEFAULT_CANFD_SP_MAX_X10;
    }
}

/* Select enabled channels and CAN FD capability for one EtherCAT slave. */
int igh_mcan_port_set_channel_mask(igh_mcan_port_config_t *config,
                                   size_t slave_pos,
                                   uint8_t channel_mask,
                                   uint8_t canfd_mask)
{
    uint8_t valid_mask = channel_mask_all();

    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES ||
        channel_mask == 0U ||
        (channel_mask & (uint8_t)~valid_mask) != 0U ||
        (canfd_mask & (uint8_t)~valid_mask) != 0U ||
        (canfd_mask & (uint8_t)~channel_mask) != 0U) {
        return IGH_ECAN_INVALID_ARG;
    }
    config->slaves[slave_pos].channel_mask = channel_mask;
    config->slaves[slave_pos].canfd_mask = canfd_mask;
    return IGH_ECAN_OK;
}

/* Set nominal and data bitrates shared by all enabled channels on one slave. */
int igh_mcan_port_set_bitrate(igh_mcan_port_config_t *config,
                              size_t slave_pos,
                              uint32_t nominal_bitrate,
                              uint32_t data_bitrate)
{
    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES ||
        nominal_bitrate == 0U || data_bitrate == 0U) {
        return IGH_ECAN_INVALID_ARG;
    }
    config->slaves[slave_pos].nominal_bitrate = nominal_bitrate;
    config->slaves[slave_pos].data_bitrate = data_bitrate;
    return IGH_ECAN_OK;
}

int igh_mcan_port_set_sample_point_range(igh_mcan_port_config_t *config,
                                         size_t slave_pos,
                                         uint16_t nominal_min_x10,
                                         uint16_t nominal_max_x10,
                                         uint16_t data_min_x10,
                                         uint16_t data_max_x10)
{
    igh_mcan_slave_config_t *slave;

    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES ||
        nominal_min_x10 == 0U || nominal_max_x10 == 0U ||
        data_min_x10 == 0U || data_max_x10 == 0U ||
        nominal_min_x10 > nominal_max_x10 ||
        data_min_x10 > data_max_x10 ||
        nominal_max_x10 > 1000U ||
        data_max_x10 > 1000U) {
        return IGH_ECAN_INVALID_ARG;
    }

    slave = &config->slaves[slave_pos];
    slave->nominal_sp_min_x10 = nominal_min_x10;
    slave->nominal_sp_max_x10 = nominal_max_x10;
    slave->data_sp_min_x10 = data_min_x10;
    slave->data_sp_max_x10 = data_max_x10;
    return IGH_ECAN_OK;
}

/* Enable one channel as classic CAN and update the slave nominal bitrate. */
int igh_mcan_port_set_classic_channel(igh_mcan_port_config_t *config,
                                      size_t slave_pos,
                                      uint8_t channel,
                                      uint32_t nominal_bitrate)
{
    igh_mcan_slave_config_t *slave;

    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES ||
        channel >= IGH_MCAN_CHANNEL_COUNT || nominal_bitrate == 0U) {
        return IGH_ECAN_INVALID_ARG;
    }
    slave = &config->slaves[slave_pos];
    slave->channel_mask |= IGH_ECAN_BIT_U8(channel);
    slave->canfd_mask &= (uint8_t)~IGH_ECAN_BIT_U8(channel);
    slave->nominal_bitrate = nominal_bitrate;
    return IGH_ECAN_OK;
}

/* Enable one channel as CAN FD capable and update nominal/data bitrates. */
int igh_mcan_port_set_canfd_channel(igh_mcan_port_config_t *config,
                                    size_t slave_pos,
                                    uint8_t channel,
                                    uint32_t nominal_bitrate,
                                    uint32_t data_bitrate)
{
    igh_mcan_slave_config_t *slave;

    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES ||
        channel >= IGH_MCAN_CHANNEL_COUNT ||
        nominal_bitrate == 0U || data_bitrate == 0U) {
        return IGH_ECAN_INVALID_ARG;
    }
    slave = &config->slaves[slave_pos];
    slave->channel_mask |= IGH_ECAN_BIT_U8(channel);
    slave->canfd_mask |= IGH_ECAN_BIT_U8(channel);
    slave->nominal_bitrate = nominal_bitrate;
    slave->data_bitrate = data_bitrate;
    return IGH_ECAN_OK;
}

/* Disable one channel while keeping at least one channel enabled for PDO validity. */
int igh_mcan_port_disable_channel(igh_mcan_port_config_t *config,
                                  size_t slave_pos,
                                  uint8_t channel)
{
    igh_mcan_slave_config_t *slave;
    uint8_t bit;

    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES ||
        channel >= IGH_MCAN_CHANNEL_COUNT) {
        return IGH_ECAN_INVALID_ARG;
    }
    slave = &config->slaves[slave_pos];
    bit = IGH_ECAN_BIT_U8(channel);
    slave->channel_mask &= (uint8_t)~bit;
    slave->canfd_mask &= (uint8_t)~bit;
    if (slave->channel_mask == 0U) {
        slave->channel_mask = IGH_ECAN_BIT_U8(0U);
    }
    return IGH_ECAN_OK;
}

int igh_mcan_port_set_channel_policy(igh_mcan_port_config_t *config,
                                     size_t slave_pos,
                                     uint8_t channel,
                                     uint32_t policy_word)
{
    uint8_t budget_max;

    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES ||
        channel >= IGH_MCAN_CHANNEL_COUNT ||
        (policy_word & ~IGH_MCAN_CH_POLICY_ALLOWED_MASK) != 0U) {
        return IGH_ECAN_INVALID_ARG;
    }

    budget_max = (uint8_t)((policy_word & IGH_MCAN_CH_POLICY_RX_BUDGET_MAX_MASK) >>
                           IGH_MCAN_CH_POLICY_RX_BUDGET_MAX_SHIFT);
    if (budget_max > 8U) {
        return IGH_ECAN_INVALID_ARG;
    }

    config->slaves[slave_pos].channel_policy[channel] = policy_word;
    return IGH_ECAN_OK;
}

int igh_mcan_port_set_channel_rx_drain_budget(igh_mcan_port_config_t *config,
                                              size_t slave_pos,
                                              uint8_t channel,
                                              uint8_t budget_max)
{
    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES ||
        channel >= IGH_MCAN_CHANNEL_COUNT ||
        budget_max > IGH_MCAN_DATAPATH_BUDGET_MAX) {
        return IGH_ECAN_INVALID_ARG;
    }

    config->slaves[slave_pos].datapath_rx_budget_max[channel] = budget_max;
    config->slaves[slave_pos].datapath_rx_budget_override_mask |= IGH_ECAN_BIT_U8(channel);
    return IGH_ECAN_OK;
}

int igh_mcan_port_set_channel_tx_write_budget(igh_mcan_port_config_t *config,
                                              size_t slave_pos,
                                              uint8_t channel,
                                              uint8_t budget_max)
{
    if ((config == NULL) || slave_pos >= ECAN_IGH_MAX_SLAVES ||
        channel >= IGH_MCAN_CHANNEL_COUNT ||
        budget_max > IGH_MCAN_DATAPATH_BUDGET_MAX) {
        return IGH_ECAN_INVALID_ARG;
    }

    config->slaves[slave_pos].datapath_tx_budget_max[channel] = budget_max;
    config->slaves[slave_pos].datapath_tx_budget_override_mask |= IGH_ECAN_BIT_U8(channel);
    return IGH_ECAN_OK;
}

/* Append all PDO profile and MCAN channel startup SDOs to the IgH config. */
int igh_mcan_port_apply_config(ecan_igh_config_t *igh_config,
                               const igh_mcan_port_config_t *mcan_config)
{
    int rc = IGH_ECAN_OK;

    if (!igh_config_is_valid_for_mcan(igh_config) || (mcan_config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (!mcan_options_are_supported(mcan_config->option_flags) ||
        !mcan_budget_policies_are_supported(mcan_config)) {
        return IGH_ECAN_UNSUPPORTED;
    }

    for (size_t slave = 0U; slave < igh_config->slave_count; slave++) {
        const igh_mcan_slave_config_t *cfg = &mcan_config->slaves[slave];
        uint8_t rx_multiplier = igh_config->slave_rxpdo_multiplier[slave] != 0U ?
                                igh_config->slave_rxpdo_multiplier[slave] :
                                igh_config->rxpdo_multiplier;
        uint8_t tx_multiplier = igh_config->slave_txpdo_multiplier[slave] != 0U ?
                                igh_config->slave_txpdo_multiplier[slave] :
                                igh_config->txpdo_multiplier;

        if (cfg->channel_mask == 0U ||
            (cfg->channel_mask & (uint8_t)~channel_mask_all()) != 0U ||
            (cfg->canfd_mask & (uint8_t)~cfg->channel_mask) != 0U ||
            (cfg->datapath_rx_budget_override_mask & (uint8_t)~channel_mask_all()) != 0U ||
            (cfg->datapath_tx_budget_override_mask & (uint8_t)~channel_mask_all()) != 0U ||
            cfg->nominal_bitrate == 0U ||
            cfg->data_bitrate == 0U ||
            cfg->nominal_sp_min_x10 == 0U ||
            cfg->nominal_sp_max_x10 == 0U ||
            cfg->data_sp_min_x10 == 0U ||
            cfg->data_sp_max_x10 == 0U ||
            cfg->nominal_sp_min_x10 > cfg->nominal_sp_max_x10 ||
            cfg->data_sp_min_x10 > cfg->data_sp_max_x10) {
            return IGH_ECAN_INVALID_ARG;
        }
        for (uint8_t ch = 0U; ch < IGH_MCAN_CHANNEL_COUNT; ch++) {
            if ((cfg->channel_policy[ch] & ~IGH_MCAN_CH_POLICY_ALLOWED_MASK) != 0U ||
                cfg->datapath_rx_budget_max[ch] > IGH_MCAN_DATAPATH_BUDGET_MAX ||
                cfg->datapath_tx_budget_max[ch] > IGH_MCAN_DATAPATH_BUDGET_MAX) {
                return IGH_ECAN_INVALID_ARG;
            }
        }
        merge_result(&rc, add_pdo_profile_sdos(igh_config, slave,
                                               IGH_MCAN_OD_TXPDO_PROFILE,
                                               tx_multiplier,
                                               mcan_config,
                                               mcan_config->tx_budget_policy,
                                               mcan_config->option_flags));
        merge_result(&rc, add_pdo_profile_sdos(igh_config, slave,
                                               IGH_MCAN_OD_RXPDO_PROFILE,
                                               rx_multiplier,
                                               mcan_config,
                                               mcan_config->rx_budget_policy,
                                               mcan_config->option_flags &
                                               (IGH_MCAN_OPT_ALIGN4 |
                                                IGH_MCAN_OPT_NO_CRC16 |
                                                IGH_MCAN_OPT_APP_BYPASS |
                                                IGH_MCAN_OPT_PAYLOAD_ONLY |
                                                IGH_MCAN_OPT_HEADER_V2)));
        for (uint8_t ch = 0U; ch < IGH_MCAN_CHANNEL_COUNT; ch++) {
            merge_result(&rc, add_channel_sdos(igh_config, slave, ch, cfg));
            merge_result(&rc, add_channel_datapath_sdos(igh_config, slave, ch, cfg));
        }
    }
    return rc;
}
