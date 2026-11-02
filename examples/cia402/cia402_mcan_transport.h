#ifndef CIA402_MCAN_TRANSPORT_H
#define CIA402_MCAN_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include "igh_cia402_protocol.h"
#include "igh_mcan_port.h"

#define CIA402_MCAN_TRANSPORT_RX_CAPACITY 64U

typedef struct {
    igh_cia402_transport_t transport;
    igh_mcan_port_t *mcan;
} igh_cia402_mcan_transport_t;

static void igh_cia402_mcan_rx_to_can(const igh_mcan_rx_frame_t *src,
                                      igh_cia402_rx_frame_t *dst)
{
    if ((src == NULL) || (dst == NULL)) {
        return;
    }
    dst->slave_pos = src->slave_pos;
    dst->frame.channel = src->frame.channel;
    dst->frame.data_len = src->frame.data_len;
    dst->frame.is_extended = src->frame.is_extended;
    dst->frame.can_id = src->frame.can_id;
    for (size_t i = 0U; i < dst->frame.data_len && i < sizeof(dst->frame.data); i++) {
        dst->frame.data[i] = src->frame.data[i];
    }
}

static int igh_cia402_mcan_send_frame(igh_mcan_port_t *mcan,
                                      const igh_cia402_tx_frame_t *tx)
{
    if ((mcan == NULL) || (tx == NULL) ||
        (tx->frame.data_len > sizeof(tx->frame.data))) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (tx->scheduled != 0U) {
        return igh_mcan_port_schedule_can(mcan,
                                          tx->slave_pos,
                                          tx->frame.channel,
                                          tx->frame.can_id,
                                          tx->frame.is_extended,
                                          tx->frame.data,
                                          tx->frame.data_len,
                                          tx->offset_ns,
                                          tx->tag);
    }
    return igh_mcan_port_send_can(mcan,
                                  tx->slave_pos,
                                  tx->frame.channel,
                                  tx->frame.can_id,
                                  tx->frame.is_extended,
                                  tx->frame.data,
                                  tx->frame.data_len);
}

static int cia402_mcan_send_can(void *user,
                                int slave_pos,
                                uint8_t channel,
                                uint32_t can_id,
                                const uint8_t *data,
                                size_t data_len,
                                int scheduled,
                                uint32_t offset_ns,
                                uint16_t tag)
{
    igh_cia402_mcan_transport_t *transport = (igh_cia402_mcan_transport_t *)user;

    if (transport == NULL || transport->mcan == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (scheduled != 0) {
        return igh_mcan_port_schedule_can(transport->mcan,
                                          slave_pos,
                                          channel,
                                          can_id,
                                          0,
                                          data,
                                          data_len,
                                          offset_ns,
                                          tag);
    }
    return igh_mcan_port_send_can(transport->mcan,
                                  slave_pos,
                                  channel,
                                  can_id,
                                  0,
                                  data,
                                  data_len);
}

static int cia402_mcan_receive(void *user,
                               igh_cia402_rx_frame_t *frames,
                               size_t capacity,
                               size_t *count)
{
    igh_cia402_mcan_transport_t *transport = (igh_cia402_mcan_transport_t *)user;
    igh_mcan_rx_frame_t mcan_frames[CIA402_MCAN_TRANSPORT_RX_CAPACITY];
    size_t mcan_count = 0U;
    size_t copy_count;
    int rc;

    if (transport == NULL || transport->mcan == NULL ||
        (frames == NULL && capacity > 0U)) {
        return IGH_ECAN_INVALID_ARG;
    }

    rc = igh_mcan_port_receive(transport->mcan,
                               mcan_frames,
                               CIA402_MCAN_TRANSPORT_RX_CAPACITY,
                               &mcan_count);
    copy_count = (mcan_count < capacity) ? mcan_count : capacity;
    for (size_t i = 0U; i < copy_count; i++) {
        igh_cia402_mcan_rx_to_can(&mcan_frames[i], &frames[i]);
    }
    if (count != NULL) {
        *count = copy_count;
    }
    if (rc == IGH_ECAN_OK && mcan_count > capacity) {
        return IGH_ECAN_NO_SPACE;
    }
    return rc;
}

static void cia402_mcan_cycle_begin(void *user)
{
    igh_cia402_mcan_transport_t *transport = (igh_cia402_mcan_transport_t *)user;

    if (transport != NULL && transport->mcan != NULL) {
        igh_mcan_port_cycle_begin(transport->mcan);
    }
}

static void cia402_mcan_cycle_end(void *user)
{
    igh_cia402_mcan_transport_t *transport = (igh_cia402_mcan_transport_t *)user;

    if (transport != NULL && transport->mcan != NULL) {
        igh_mcan_port_cycle_end(transport->mcan);
    }
}

static void cia402_mcan_cycle_abort(void *user)
{
    igh_cia402_mcan_transport_t *transport = (igh_cia402_mcan_transport_t *)user;

    if (transport != NULL && transport->mcan != NULL) {
        igh_mcan_port_cycle_abort(transport->mcan);
    }
}

static void cia402_mcan_sleep_us(void *user, uint32_t sleep_us)
{
    (void)user;
    usleep(sleep_us);
}

static void igh_cia402_mcan_transport_init(igh_cia402_mcan_transport_t *transport,
                                           igh_mcan_port_t *mcan)
{
    if (transport == NULL) {
        return;
    }
    transport->mcan = mcan;
    transport->transport.user = transport;
    transport->transport.send_can = cia402_mcan_send_can;
    transport->transport.receive = cia402_mcan_receive;
    transport->transport.cycle_begin = cia402_mcan_cycle_begin;
    transport->transport.cycle_end = cia402_mcan_cycle_end;
    transport->transport.cycle_abort = cia402_mcan_cycle_abort;
    transport->transport.sleep_us = cia402_mcan_sleep_us;
    transport->transport.cycle_us = (mcan != NULL && mcan->igh != NULL) ?
                                    mcan->igh->cycle_us :
                                    IGH_CIA402_CSP_SYNC_PERIOD_US_DEFAULT;
}

#endif /* CIA402_MCAN_TRANSPORT_H */
