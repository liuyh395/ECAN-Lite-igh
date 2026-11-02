#define _DEFAULT_SOURCE

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "igh_adapter.h"

#define RAW_PDO_DUMP_BYTES      16U
#define RAW_PDO_DUMP_CYCLES     20U
#define RAW_PDO_DUMP_CYCLE_US   10000U

static void print_bytes(const char *name, const uint8_t *data, size_t size)
{
    size_t count = (size < RAW_PDO_DUMP_BYTES) ? size : RAW_PDO_DUMP_BYTES;

    printf("%s", name);
    for (size_t i = 0U; i < count; i++) {
        printf(" %02X", data[i]);
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    if (argc > 1) {
        printf("usage: %s [--help]\nClaims the master, configures one slave and exchanges PDOs.\n", argv[0]);
        return argc == 2 && (!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h")) ? 0 : 2;
    }
    ecan_igh_config_t config;
    ecan_igh_context_t igh;
    int rc;

    ecan_igh_default_config(&config);
    if (ecan_igh_config_set_slave_count(&config, 1U) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(&config, RAW_PDO_DUMP_CYCLE_US) != IGH_ECAN_OK ||
        ecan_igh_config_set_pdo_bytes(&config, IGH_ECAN_DEFAULT_PDO_BYTES) != IGH_ECAN_OK) {
        fprintf(stderr, "invalid raw PDO demo config\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    rc = ecan_igh_init(&igh, &config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "ecan_igh_init failed: %d\n", rc);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = ecan_igh_wait_op_timeout(&igh, 10000U);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "wait OP failed: %d\n", rc);
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    for (unsigned cycle = 0U; cycle < RAW_PDO_DUMP_CYCLES; cycle++) {
        uint8_t *rxpdo = NULL;
        const uint8_t *txpdo = NULL;
        size_t rxpdo_size = 0U;
        size_t txpdo_size = 0U;

        ecan_igh_cycle_begin(&igh);
        (void)ecan_igh_get_rxpdo_buffer(&igh, 0U, &rxpdo, &rxpdo_size);
        (void)ecan_igh_get_txpdo_buffer(&igh, 0U, &txpdo, &txpdo_size);
        if (rxpdo != NULL && rxpdo_size > 0U) {
            rxpdo[0] = (uint8_t)cycle;
        }
        printf("cycle=%u wc_complete=%d\n", cycle, ecan_igh_domain_is_complete(&igh));
        if (txpdo != NULL) {
            print_bytes("txpdo", txpdo, txpdo_size);
        }
        ecan_igh_cycle_end(&igh);
        usleep(config.cycle_us);
    }

    ecan_igh_close(&igh);
    return IGH_ECAN_EXIT_OK;
}
