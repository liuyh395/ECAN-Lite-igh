#include <stdint.h>
#include <stdio.h>

#include "igh_adapter.h"
#include "igh_mcan_config.h"
#include "../src/igh_adapter_internal.h"

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        return 1; \
    } \
} while (0)

int main(void)
{
    igh_mcan_firmware_capabilities_t fw_caps;
    igh_mcan_port_config_t mcan_config;

    CHECK(igh_mcan_firmware_supports_protocol_v2(0x01D2U) == 0);
    CHECK(igh_mcan_firmware_supports_protocol_v2(0x0200U) != 0);
    CHECK(igh_mcan_firmware_supports_protocol_v2(0x1200U) != 0);
    CHECK(igh_mcan_firmware_capabilities_decode(
              0x0200U, 596U,
              IGH_MCAN_PDO_FEATURE_HEADER_V2 |
                  IGH_MCAN_PDO_FEATURE_FINE_SIZE_16,
              &fw_caps) == IGH_ECAN_OK);
    CHECK(fw_caps.major == 2U);
    CHECK(fw_caps.minor == 0U);
    CHECK(fw_caps.build == 596U);
    CHECK(fw_caps.protocol_v2 != 0U);
    CHECK(fw_caps.header_v2 != 0U);
    CHECK(fw_caps.fine_size_16 != 0U);
    CHECK(igh_mcan_firmware_capabilities_decode(
              0x01D2U, 596U,
              IGH_MCAN_PDO_FEATURE_HEADER_V2 |
                  IGH_MCAN_PDO_FEATURE_FINE_SIZE_16,
              &fw_caps) == IGH_ECAN_OK);
    CHECK(fw_caps.protocol_v2 == 0U);
    CHECK(fw_caps.header_v2 == 0U);
    CHECK(fw_caps.fine_size_16 == 0U);

    igh_mcan_port_default_config(&mcan_config);
    CHECK(mcan_config.option_flags == IGH_MCAN_OPT_DEFAULT);
    CHECK(mcan_config.option_flags == IGH_MCAN_OPT_NO_CRC16);
    CHECK(igh_mcan_port_select_pdo_protocol(
              &mcan_config, &fw_caps, IGH_MCAN_HDR_VERSION_V2, 1) ==
          IGH_ECAN_UNSUPPORTED);
    CHECK(mcan_config.option_flags == IGH_MCAN_OPT_NO_CRC16);
    CHECK(igh_mcan_firmware_capabilities_decode(
              0x0200U, 596U,
              IGH_MCAN_PDO_FEATURE_HEADER_V2 |
                  IGH_MCAN_PDO_FEATURE_FINE_SIZE_16,
              &fw_caps) == IGH_ECAN_OK);
    CHECK(igh_mcan_port_select_pdo_protocol(
              &mcan_config, &fw_caps, IGH_MCAN_HDR_VERSION_V2, 1) ==
          IGH_ECAN_OK);
    CHECK(mcan_config.option_flags == IGH_MCAN_OPT_V2_NO_CRC16);
    CHECK(igh_mcan_port_select_pdo_protocol(
              &mcan_config, &fw_caps, IGH_MCAN_HDR_VERSION_V1, 0) ==
          IGH_ECAN_OK);
    CHECK(mcan_config.option_flags == 0U);
    CHECK(igh_mcan_port_select_pdo_protocol(
              &mcan_config, &fw_caps, IGH_MCAN_HDR_VERSION_V1, 1) ==
          IGH_ECAN_OK);
    CHECK(mcan_config.option_flags == IGH_MCAN_OPT_NO_CRC16);

    ecan_igh_config_t config;
    ecan_igh_context_t context;

    ecan_igh_default_config(&config);
    CHECK(config.cycle_us == ECAN_RT_SM_ECAT_CYCLE_US);
    CHECK(config.dc_assign_activate == 0U);
    CHECK(config.cycle_mode == ECAN_IGH_CYCLE_MODE_STANDARD);
    CHECK(ecan_igh_config_get_pdo_bytes(&config) == 256U);
    CHECK(ecan_igh_config_get_slave_rxpdo_bytes(&config, 0U) == 128U);
    CHECK(ecan_igh_config_get_slave_txpdo_bytes(&config, 0U) == 256U);
    CHECK(ecan_igh_config_apply_realtime_profile(
              &config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) == IGH_ECAN_OK);
    CHECK(config.cycle_us == ECAN_RT_DC_ECAT_CYCLE_US);
    CHECK(config.dc_assign_activate == ECAN_IGH_DC_ASSIGN_SYNC0);
    CHECK(config.dc_sync0_shift_ns == 900000);
    CHECK(config.cycle_mode == ECAN_IGH_CYCLE_MODE_STANDARD);
    CHECK(ecan_igh_config_get_pdo_bytes(&config) == 384U);
    CHECK(ecan_igh_config_get_slave_rxpdo_bytes(&config, 0U) == 384U);
    CHECK(ecan_igh_config_get_slave_txpdo_bytes(&config, 0U) == 384U);

    ecan_igh_default_config(&config);
    CHECK(ecan_igh_config_set_direction_pdo_multiplier(&config, 1U, 2U) == IGH_ECAN_OK);
    CHECK(ecan_igh_config_get_slave_rxpdo_bytes(&config, 0U) == 128U);
    CHECK(ecan_igh_config_get_slave_txpdo_bytes(&config, 0U) == 256U);
    CHECK(ecan_igh_config_set_slave_direction_pdo_multiplier(&config, 1U, 2U, 3U) ==
          IGH_ECAN_OK);
    CHECK(ecan_igh_config_get_slave_rxpdo_bytes(&config, 1U) == 256U);
    CHECK(ecan_igh_config_get_slave_txpdo_bytes(&config, 1U) == 384U);
    CHECK(ecan_igh_config_set_direction_pdo_multiplier(&config, 0U, 2U) ==
          IGH_ECAN_INVALID_ARG);
    CHECK(ecan_igh_config_set_direction_pdo_multiplier(&config, 0x21U, 0x22U) ==
          IGH_ECAN_OK);
    CHECK(ecan_igh_config_get_slave_rxpdo_bytes(&config, 0U) == 144U);
    CHECK(ecan_igh_config_get_slave_txpdo_bytes(&config, 0U) == 272U);
    CHECK(ecan_igh_config_set_slave_direction_pdo_multiplier(&config, 0U, 1U, 25U) ==
          IGH_ECAN_INVALID_ARG);

    CHECK(ecan_igh_config_set_pdo_bytes(&config, 1392U) == IGH_ECAN_OK);
    CHECK(config.pdo_multiplier == 0xEAU);
    CHECK(ecan_igh_config_get_pdo_bytes(&config) == 1392U);
    CHECK(ecan_igh_config_set_pdo_bytes(&config, 1408U) == IGH_ECAN_INVALID_ARG);

    ecan_igh_default_config(&config);
    CHECK(ecan_igh_config_set_cycle_us(&config, UINT32_MAX) == IGH_ECAN_INVALID_ARG);
    CHECK(ecan_igh_config_set_slave_pdo_multiplier(&config, 0U, 0U) == IGH_ECAN_OK);
    CHECK(ecan_igh_config_set_slave(&config, 0U, -1, 0) == IGH_ECAN_INVALID_ARG);
    CHECK(ecan_igh_config_set_slave(&config, 0U, 0, 70000) == IGH_ECAN_INVALID_ARG);

    ecan_igh_default_config(&config);
    CHECK(ecan_igh_config_set_slave_count(&config, 2U) == IGH_ECAN_OK);
    CHECK(ecan_igh_config_set_slave(&config, 0U, 0, 0) == IGH_ECAN_OK);
    CHECK(ecan_igh_config_set_slave(&config, 1U, 0, 0) == IGH_ECAN_OK);
    CHECK(ecan_igh_init(&context, &config) == IGH_ECAN_INVALID_ARG);

    ecan_igh_default_config(&config);
    CHECK(ecan_igh_config_set_slave_count(&config, 1U) == IGH_ECAN_OK);
    CHECK(ecan_igh_config_add_sdo8(&config, 2U, 0x8000U, 0U, 1U) == IGH_ECAN_OK);
    CHECK(ecan_igh_init(&context, &config) == IGH_ECAN_INVALID_ARG);

    ecan_igh_default_config(&config);
    CHECK(ecan_igh_config_set_cycle_us(&config, 500U) == IGH_ECAN_OK);
    CHECK(ecan_igh_config_enable_dc_phased_cycle(&config, 250, 50U, 350U, 470U) ==
          IGH_ECAN_OK);
    CHECK(config.cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE);
    CHECK(config.dc_sync0_cycle_ns == 500000U);
    CHECK(config.rxpdo_phase_us == 50U);
    CHECK(config.txpdo_phase_us == 350U);
    CHECK(config.txpdo_collect_phase_us == 470U);
    CHECK(ecan_igh_config_enable_dc_phased_cycle(&config, 250, 350U, 50U, 470U) ==
          IGH_ECAN_INVALID_ARG);
    CHECK(ecan_igh_config_set_dc_single_cycle_phases(&config, 0U, 0U, 400U) == IGH_ECAN_OK);
    CHECK(config.rxpdo_phase_us == config.txpdo_phase_us);
    CHECK(ecan_igh_config_set_dc_single_cycle_phases(&config, 1U, 0U, 400U) == IGH_ECAN_INVALID_ARG);

    ecan_igh_default_config(&config);
    CHECK(ecan_igh_config_set_cycle_us(&config, 1000U) == IGH_ECAN_OK);
    CHECK(ecan_igh_config_enable_dc_sync01(&config, 1000U, 100U, 900U) == IGH_ECAN_OK);
    CHECK(config.dc_assign_activate == ECAN_IGH_DC_ASSIGN_SYNC01);
    CHECK(config.dc_sync0_cycle_ns == 1000000U);
    CHECK(config.dc_sync0_shift_ns == 100000);
    CHECK(config.dc_sync1_offset_ns == 800000U);
    CHECK(ecan_igh_config_enable_dc_sync01(&config, 1000U, 900U, 100U) == IGH_ECAN_OK);
    CHECK(config.dc_sync0_shift_ns == 900000 && config.dc_sync1_offset_ns == 200000U);
    CHECK(ecan_igh_adapter_config_is_valid(&config));
    CHECK(ecan_igh_config_enable_dc_sync01(&config, 1000U, 600U, 0U) == IGH_ECAN_OK);
    CHECK(config.dc_sync1_offset_ns == 400000U);
    CHECK(ecan_igh_adapter_config_is_valid(&config));
    config.dc_sync1_offset_ns = 1000000U;
    CHECK(!ecan_igh_adapter_config_is_valid(&config));
    CHECK(ecan_igh_config_enable_dc_sync01(&config, 1000U, 100U, 900U) == IGH_ECAN_OK);
    CHECK(ecan_igh_config_enable_dc_sync01(&config, 1000U, 1000U, 100U) == IGH_ECAN_INVALID_ARG);
    CHECK(ecan_igh_config_enable_dc_sync01(&config, 1000U, 100U, 1000U) == IGH_ECAN_INVALID_ARG);
    CHECK(ecan_igh_config_enable_dc_sync01(&config, 1000U, 100U, 100U) == IGH_ECAN_INVALID_ARG);
    CHECK(ecan_igh_config_enable_dc_sync01(&config, 500U, 100U, 400U) == IGH_ECAN_INVALID_ARG);
    CHECK(config.dc_sync1_offset_ns == 800000U); /* Failed setters are atomic. */
    CHECK(ecan_igh_config_enable_dc_combined(&config, 0U, 400U) == IGH_ECAN_OK);
    CHECK(config.cycle_mode == ECAN_IGH_CYCLE_MODE_DC_COMBINED);
    CHECK(config.dc_sync1_offset_ns == 800000U);
    CHECK(config.rxpdo_phase_us == 0U && config.txpdo_collect_phase_us == 400U);
    CHECK(ecan_igh_config_enable_dc_combined(&config, 400U, 400U) == IGH_ECAN_INVALID_ARG);
    CHECK(ecan_igh_config_enable_dc_combined(&config, 0U, 1000U) == IGH_ECAN_INVALID_ARG);
    context.cycle_mode = config.cycle_mode;
    CHECK(ecan_igh_collects_at_cycle_end(&context) && !ecan_igh_is_dc_single_cycle(&context));
    CHECK(!ecan_igh_collects_at_cycle_end(NULL));
    config.dc_min_prepare_us = 101U; /* Sync0 is only 100 us: fail before opening master. */
    CHECK(ecan_igh_init(&context, &config) == IGH_ECAN_INVALID_ARG);
    config.dc_min_prepare_us = 0U;
    config.init_flags |= ECAN_IGH_INIT_SKIP_STARTUP_SDO;
    CHECK(ecan_igh_init(&context, &config) == IGH_ECAN_INVALID_ARG);
    config.init_flags = 0U;
    CHECK(ecan_igh_config_add_sdo16(&config, 0U, 0x1C33U, 1U, 2U) == IGH_ECAN_OK);
    CHECK(ecan_igh_init(&context, &config) == IGH_ECAN_INVALID_ARG);
    config.startup_sdo_count = 0U;
    CHECK(ecan_igh_config_enable_dc_sync0(&config, 1000U, 100) == IGH_ECAN_OK);
    CHECK(config.dc_sync1_offset_ns == 0U);
    CHECK(ecan_igh_config_enable_dc_sync01(&config, 0U, 100U, 900U) == IGH_ECAN_OK);
    CHECK(ecan_igh_config_apply_realtime_profile(&config, ECAN_IGH_PROFILE_SM_250US_1MS) == IGH_ECAN_OK);
    CHECK(config.dc_sync1_offset_ns == 0U);

    {
        uint8_t regs[24] = {0};
        EC_WRITE_U64(regs, 10000100000ULL);
        EC_WRITE_U64(regs + 8, 10000900000ULL);
        EC_WRITE_U32(regs + 16, 1000000U);
        EC_WRITE_U32(regs + 20, 800000U);
        CHECK(ecan_igh_adapter_dc_phase_matches(regs, 1000000U, 100000U, 800000U));
        /* Correct periods do not compensate for a shifted start grid. */
        EC_WRITE_U64(regs, 10000500000ULL);
        EC_WRITE_U64(regs + 8, 10001300000ULL);
        CHECK(!ecan_igh_adapter_dc_phase_matches(regs, 1000000U, 100000U, 800000U));
        CHECK(!ecan_igh_adapter_dc_phase_matches(regs, 0U, 100000U, 800000U));
        CHECK(!ecan_igh_adapter_dc_phase_matches(NULL, 1000000U, 100000U, 800000U));
        EC_WRITE_U64(regs, 10000900000ULL);
        EC_WRITE_U64(regs + 8, 10001100000ULL);
        EC_WRITE_U32(regs + 20, 200000U);
        CHECK(ecan_igh_adapter_dc_phase_matches(regs, 1000000U, 900000U, 200000U));
        EC_WRITE_U64(regs + 8, 10001200000ULL);
        CHECK(!ecan_igh_adapter_dc_phase_matches(regs, 1000000U, 900000U, 200000U));
        CHECK(!ecan_igh_adapter_dc_phase_matches(regs, 1000000U, 900000U, 1000000U));
    }
    puts("adapter_config_test: PASS");
    return 0;
}
