#include "igh_mcan_port_internal.h"

#include <string.h>

typedef enum {
    IGH_MCAN_TX_CLASSIC = 0,
    IGH_MCAN_TX_CANFD = 1U << 0,
    IGH_MCAN_TX_BRS = 1U << 1,
    IGH_MCAN_TX_SCHEDULED = 1U << 2
} igh_mcan_tx_options_t;

#define IGH_MCAN_OPT_IF(condition, option) \
    ((condition) ? (igh_mcan_tx_options_t)(option) : IGH_MCAN_TX_CLASSIC)
#define IGH_MCAN_TX_OPTIONS(canfd, brs, scheduled) \
    ((igh_mcan_tx_options_t)( \
        IGH_MCAN_OPT_IF((canfd), IGH_MCAN_TX_CANFD) | \
        IGH_MCAN_OPT_IF((brs), IGH_MCAN_TX_BRS) | \
        IGH_MCAN_OPT_IF((scheduled), IGH_MCAN_TX_SCHEDULED)))

typedef struct {
    int slave_pos;
    uint8_t channel;
    uint32_t can_id;
    int is_extended;
    igh_mcan_tx_options_t options;
    const uint8_t *data;
    size_t data_len;
    uint32_t offset_ns;
    uint16_t tag;
} mcan_tx_request_t;

#define IGH_MCAN_TX_REQ(slave, ch, id, ext, opts, payload, len, offset, sched_tag) \
    ((mcan_tx_request_t){ \
        (slave), \
        (ch), \
        (id), \
        (ext), \
        (opts), \
        (payload), \
        (len), \
        (offset), \
        (sched_tag) \
    })
#define IGH_MCAN_PORT_SEND_REQ(port, slave, ch, id, ext, opts, payload, len, offset, tag) \
    mcan_port_write_can_request( \
        (port), \
        &IGH_MCAN_TX_REQ((slave), (ch), (id), (ext), (opts), \
                         (payload), (len), (offset), (tag)))
#define MCAN_TX_IS_CANFD(req) (((req)->options & IGH_MCAN_TX_CANFD) != 0U)
#define MCAN_TX_IS_BRS(req) (((req)->options & IGH_MCAN_TX_BRS) != 0U)
#define MCAN_TX_IS_SCHEDULED(req) (((req)->options & IGH_MCAN_TX_SCHEDULED) != 0U)

/* -------------------------------------------------------------------------- */
/* Port runtime                                                                */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* Runtime cycle API                                                           */
/* -------------------------------------------------------------------------- */

/* Bind the MCAN port to an initialized IgH context and prepare PDO writers. */
int igh_mcan_port_init(igh_mcan_port_t *port,
                       ecan_igh_context_t *igh,
                       const igh_mcan_port_config_t *config)
{
    if ((port == NULL) || (igh == NULL) || (config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    memset(port, 0, sizeof(*port));
    port->igh = igh;
    port->config = *config;
    for (size_t slave = 0U; slave < igh->slave_count; slave++) {
        uint8_t *rxpdo = NULL;
        size_t pdo_size = 0U;
        int rc = ecan_igh_get_rxpdo_buffer(igh, slave, &rxpdo, &pdo_size);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        igh_mcan_pdo_writer_init(&port->writers[slave],
                                 rxpdo,
                                 pdo_size,
                                 port->config.slaves[slave].channel_mask,
                                 port->config.encapsulation_profile,
                                 port->config.option_flags &
                                 (IGH_MCAN_OPT_ALIGN4 |
                                  IGH_MCAN_OPT_NO_CRC16 |
                                  IGH_MCAN_OPT_APP_BYPASS |
                                  IGH_MCAN_OPT_PAYLOAD_ONLY |
                                  IGH_MCAN_OPT_HEADER_V2));
    }
    return IGH_ECAN_OK;
}

/* Reset every slave writer at the beginning of one EtherCAT cycle. */
static void begin_all_writers(igh_mcan_port_t *port)
{
    for (size_t slave = 0U; slave < port->igh->slave_count; slave++) {
        igh_mcan_pdo_writer_begin(&port->writers[slave]);
    }
}

/* Start a blocking IgH/MCAN cycle and initialize all outgoing PDO writers. */
int igh_mcan_port_cycle_begin_checked(igh_mcan_port_t *port)
{
    if ((port == NULL) || (port->igh == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (port->cycle_active != 0U) {
        return IGH_ECAN_NOT_READY;
    }
    if (port->igh->cycle_active != 0U) {
        return IGH_ECAN_NOT_READY;
    }
    {
        int rc = ecan_igh_cycle_begin_checked(port->igh);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
    port->cycle_active = 1U;
    begin_all_writers(port);
    return IGH_ECAN_OK;
}

/* Start a cycle only if the IgH master lock is immediately available. */
int igh_mcan_port_cycle_try_begin(igh_mcan_port_t *port)
{
    int rc;

    if ((port == NULL) || (port->igh == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (port->cycle_active != 0U || port->igh->cycle_active != 0U) {
        return IGH_ECAN_NOT_READY;
    }
    rc = ecan_igh_cycle_try_begin(port->igh);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    port->cycle_active = 1U;
    begin_all_writers(port);
    return IGH_ECAN_OK;
}

/* Finalize writers, publish RxPDO outputs and end the IgH cycle. */
int igh_mcan_port_cycle_end_checked(igh_mcan_port_t *port)
{
    int rc;

    if ((port == NULL) || (port->igh == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (port->cycle_active == 0U) {
        return IGH_ECAN_NOT_READY;
    }
    for (size_t slave = 0U; slave < port->igh->slave_count; slave++) {
        igh_mcan_pdo_writer_finish(&port->writers[slave]);
    }
    rc = ecan_igh_cycle_end_checked(port->igh);
    /* The adapter also releases the cycle on send/receive failure. Do not
     * leave the port active with an already released adapter mutex. */
    port->cycle_active = port->igh->cycle_active;
    return rc;
}

/* Abort an active cycle without publishing the current writer contents. */
int igh_mcan_port_cycle_abort_checked(igh_mcan_port_t *port)
{
    int rc;

    if ((port == NULL) || (port->igh == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (port->cycle_active == 0U) {
        return IGH_ECAN_NOT_READY;
    }
    rc = ecan_igh_cycle_abort_checked(port->igh);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    port->cycle_active = 0U;
    return IGH_ECAN_OK;
}

void igh_mcan_port_cycle_begin(igh_mcan_port_t *port)
{
    (void)igh_mcan_port_cycle_begin_checked(port);
}

void igh_mcan_port_cycle_end(igh_mcan_port_t *port)
{
    (void)igh_mcan_port_cycle_end_checked(port);
}

void igh_mcan_port_cycle_abort(igh_mcan_port_t *port)
{
    (void)igh_mcan_port_cycle_abort_checked(port);
}

/* -------------------------------------------------------------------------- */
/* TX request core                                                             */
/* -------------------------------------------------------------------------- */

/* Validate one flattened CAN/CAN FD transmit request before PDO packing. */
static int mcan_tx_request_validate(const mcan_tx_request_t *req)
{
    uint32_t max_id;
    int is_canfd;

    IGH_MCAN_CHECK_ARG(req != NULL);
    is_canfd = MCAN_TX_IS_CANFD(req);
    max_id = (req->is_extended != 0) ? IGH_ECAN_CAN_EXT_ID_MAX : IGH_ECAN_CAN_STD_ID_MAX;
    IGH_MCAN_CHECK_ARG((req->data != NULL) || (req->data_len == 0U));
    IGH_MCAN_CHECK_ARG(req->channel < IGH_MCAN_CHANNEL_COUNT);
    IGH_MCAN_CHECK_ARG(req->can_id <= max_id);
    IGH_MCAN_CHECK_ARG((is_canfd && req->data_len <= IGH_ECAN_CANFD_MAX_DLEN) ||
                       (!is_canfd && req->data_len <= IGH_ECAN_CLASSIC_CAN_MAX_DLEN));
    IGH_MCAN_CHECK_ARG(is_canfd || !MCAN_TX_IS_BRS(req));
    return IGH_ECAN_OK;
}

/* Pack a validated request into the current slave RxPDO frame stream. */
static inline int mcan_port_write_can_request(igh_mcan_port_t *port,
                                              const mcan_tx_request_t *req)
{
    int rc;
    int is_canfd;
    int brs;
    int scheduled;
    uint8_t dlc;

    IGH_MCAN_CHECK_ARG(port != NULL && port->igh != NULL);
    IGH_MCAN_CHECK_ARG(req != NULL);
    IGH_MCAN_CHECK_ARG(req->slave_pos >= 0 &&
                       (size_t)req->slave_pos < port->igh->slave_count);
    IGH_MCAN_CHECK_READY(port->cycle_active != 0U);
    rc = mcan_tx_request_validate(req);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    is_canfd = MCAN_TX_IS_CANFD(req);
    brs = MCAN_TX_IS_BRS(req);
    scheduled = MCAN_TX_IS_SCHEDULED(req);
    dlc = igh_mcan_len_to_dlc(req->data_len, is_canfd);
    return igh_mcan_pdo_writer_append_raw(&port->writers[req->slave_pos],
                                 req->channel,
                                 req->can_id,
                                 dlc,
                                 req->is_extended,
                                 is_canfd,
                                 brs,
                                 0,
                                 scheduled ? IGH_MCAN_TSF_FULL64 : IGH_MCAN_TSF_NONE,
                                 scheduled ? IGH_MCAN_SCHED_FORMAT_SYNC_PHASE_V1 : 0U,
                                 scheduled ? IGH_MCAN_SCHED_FLAGS_V1_SUPPORTED : 0U,
                                 req->tag,
                                 req->offset_ns,
                                 req->data,
                                 req->data_len);
}

/* -------------------------------------------------------------------------- */
/* Flat send API                                                               */
/* -------------------------------------------------------------------------- */

/* Append one immediate classic CAN frame to the current cycle. */
int igh_mcan_port_send_can(igh_mcan_port_t *port,
                           int slave_pos,
                           uint8_t channel,
                           uint32_t can_id,
                           int is_extended,
                           const uint8_t *data,
                           size_t data_len)
{
    return IGH_MCAN_PORT_SEND_REQ(port, slave_pos, channel, can_id, is_extended,
                                  IGH_MCAN_TX_CLASSIC, data, data_len, 0U, 0U);
}

/* Append one immediate CAN FD frame to the current cycle. */
int igh_mcan_port_send_canfd(igh_mcan_port_t *port,
                             int slave_pos,
                             uint8_t channel,
                             uint32_t can_id,
                             int is_extended,
                             int brs,
                             const uint8_t *data,
                             size_t data_len)
{
    return IGH_MCAN_PORT_SEND_REQ(port, slave_pos, channel, can_id, is_extended,
                                  IGH_MCAN_TX_OPTIONS(1, brs, 0),
                                  data, data_len, 0U, 0U);
}

/* Append one scheduled classic CAN frame using SYNC-phase timing metadata. */
int igh_mcan_port_schedule_can(igh_mcan_port_t *port,
                               int slave_pos,
                               uint8_t channel,
                               uint32_t can_id,
                               int is_extended,
                               const uint8_t *data,
                               size_t data_len,
                               uint32_t offset_ns,
                               uint16_t tag)
{
    return IGH_MCAN_PORT_SEND_REQ(port, slave_pos, channel, can_id, is_extended,
                                  IGH_MCAN_TX_SCHEDULED,
                                  data, data_len, offset_ns, tag);
}

/* Append one scheduled CAN FD frame using SYNC-phase timing metadata. */
int igh_mcan_port_schedule_canfd(igh_mcan_port_t *port,
                                 int slave_pos,
                                 uint8_t channel,
                                 uint32_t can_id,
                                 int is_extended,
                                 int brs,
                                 const uint8_t *data,
                                 size_t data_len,
                                 uint32_t offset_ns,
                                 uint16_t tag)
{
    return IGH_MCAN_PORT_SEND_REQ(port, slave_pos, channel, can_id, is_extended,
                                  IGH_MCAN_TX_OPTIONS(1, brs, 1),
                                  data, data_len, offset_ns, tag);
}

typedef struct {
    igh_mcan_rx_frame_t *frames;
    size_t capacity;
    size_t count;
    size_t dropped;
    int slave_pos;
} frame_collector_t;

/* Copy parsed frames into the caller-provided receive array. */
static int collect_frame(const igh_mcan_frame_t *frame,
                         const igh_mcan_pdo_header_t *header,
                         void *user)
{
    frame_collector_t *collector = (frame_collector_t *)user;
    igh_mcan_rx_frame_t *dst;

    if ((collector == NULL) || (frame == NULL) || (header == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (collector->count >= collector->capacity) {
        collector->dropped++;
        return IGH_ECAN_OK;
    }

    dst = &collector->frames[collector->count++];
    dst->slave_pos = collector->slave_pos;
    dst->frame = *frame;
    dst->header = *header;
    return IGH_ECAN_OK;
}

/* Remember the last PDO parse failure for diagnostics. */
static void save_parse_error(igh_mcan_port_t *port,
                             size_t slave,
                             const igh_mcan_pdo_parse_diag_t *diag)
{
    igh_mcan_pdo_parse_diag_t empty_diag;

    if (port == NULL) {
        return;
    }
    if (diag == NULL) {
        memset(&empty_diag, 0, sizeof(empty_diag));
        diag = &empty_diag;
    }
    port->last_parse_valid = 1U;
    port->last_parse_slave = slave;
    port->last_parse_diag = *diag;
}

/* Detect a new TxPDO image by comparing the fixed header snapshot. */
static int txpdo_header_changed(const igh_mcan_port_t *port,
                                size_t slave,
                                const igh_mcan_pdo_header_t *header)
{
    if ((port == NULL) || (header == NULL)) {
        return 0;
    }
    if (port->last_rx_header_valid[slave] == 0U) {
        return 1;
    }
    if (memcmp(&port->last_rx_header[slave], header, sizeof(*header)) != 0) {
        return 1;
    }
    return 0;
}

/* Cache a TxPDO header after its frame stream was parsed successfully. */
static void remember_txpdo_header(igh_mcan_port_t *port,
                                  size_t slave,
                                  const igh_mcan_pdo_header_t *header)
{
    if ((port == NULL) || (header == NULL)) {
        return;
    }
    port->last_rx_header_valid[slave] = 1U;
    port->last_rx_header[slave] = *header;
}

/* Parse one slave's TxPDO stream and collect newly received CAN frames. */
static int receive_one_slave(igh_mcan_port_t *port,
                             size_t slave,
                             frame_collector_t *collector)
{
    const uint8_t *txpdo = NULL;
    size_t pdo_size = 0U;
    igh_mcan_pdo_header_t header;
    igh_mcan_pdo_parse_diag_t diag;
    size_t count_before;
    size_t dropped_before;
    int rc;

    if ((port == NULL) || (collector == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    collector->slave_pos = (int)slave;
    rc = ecan_igh_get_txpdo_buffer(port->igh, slave, &txpdo, &pdo_size);
    if (rc != IGH_ECAN_OK) {
        memset(&diag, 0, sizeof(diag));
        diag.code = IGH_MCAN_PDO_PARSE_BAD_ARG;
        save_parse_error(port, slave, &diag);
        return IGH_ECAN_FAIL;
    }

    rc = igh_mcan_pdo_read_header(txpdo, pdo_size, &header);
    if (rc != IGH_ECAN_OK) {
        memset(&diag, 0, sizeof(diag));
        diag.code = IGH_MCAN_PDO_PARSE_BAD_HEADER;
        save_parse_error(port, slave, &diag);
        return IGH_ECAN_FAIL;
    }
    if (!txpdo_header_changed(port, slave, &header)) {
        return IGH_ECAN_OK;
    }

    count_before = collector->count;
    dropped_before = collector->dropped;
    rc = igh_mcan_pdo_parse_frames_ex(txpdo,
                                      pdo_size,
                                      port->config.slaves[slave].channel_mask,
                                      port->config.encapsulation_profile,
                                      port->config.option_flags,
                                      collect_frame,
                                      collector,
                                      NULL,
                                      &diag);
    if (rc != IGH_ECAN_OK) {
        collector->count = count_before;
        collector->dropped = dropped_before;
        save_parse_error(port, slave, &diag);
        return IGH_ECAN_FAIL;
    }

    remember_txpdo_header(port, slave, &header);
    return IGH_ECAN_OK;
}

/* -------------------------------------------------------------------------- */
/* Receive API                                                                 */
/* -------------------------------------------------------------------------- */

/* Parse new TxPDO images from every slave and return collected CAN frames. */
int igh_mcan_port_receive(igh_mcan_port_t *port,
                          igh_mcan_rx_frame_t *rx_frames,
                          size_t rx_capacity,
                          size_t *rx_count)
{
    int rc = IGH_ECAN_OK;
    frame_collector_t collector;

    if ((port == NULL) || (port->igh == NULL) ||
        ((rx_frames == NULL) && (rx_capacity > 0U))) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (rx_count != NULL) {
        *rx_count = 0U;
    }
    memset(&collector, 0, sizeof(collector));
    collector.frames = rx_frames;
    collector.capacity = rx_capacity;

    for (size_t slave = 0U; slave < port->igh->slave_count; slave++) {
        if (receive_one_slave(port, slave, &collector) != IGH_ECAN_OK) {
            rc = IGH_ECAN_FAIL;
        }
    }

    if (rx_count != NULL) {
        *rx_count = collector.count;
    }
    if ((rc == IGH_ECAN_OK) && (collector.dropped != 0U)) {
        rc = IGH_ECAN_NO_SPACE;
    }
    return rc;
}

int igh_mcan_port_warmup_cycle_checked(igh_mcan_port_t *port,
                                       igh_mcan_rx_frame_t *rx_frames,
                                       size_t rx_capacity,
                                       igh_mcan_port_warmup_result_t *result)
{
    size_t rx_count = 0U;
    int rc;
    int end_rc;

    if ((port == NULL) || (result == NULL) ||
        ((rx_frames == NULL) && (rx_capacity > 0U))) {
        return IGH_ECAN_INVALID_ARG;
    }

    rc = igh_mcan_port_cycle_begin_checked(port);
    if (rc != IGH_ECAN_OK) {
        result->begin_errors++;
        result->consecutive_stable_cycles = 0U;
        return rc;
    }

    rc = igh_mcan_port_receive(port, rx_frames, rx_capacity, &rx_count);
    if (rc == IGH_ECAN_NO_SPACE) {
        result->no_space_events++;
    } else if (rc != IGH_ECAN_OK) {
        result->receive_errors++;
    }

    end_rc = igh_mcan_port_cycle_end_checked(port);
    if (end_rc != IGH_ECAN_OK) {
        result->end_errors++;
        result->consecutive_stable_cycles = 0U;
        return end_rc;
    }

    if (rc != IGH_ECAN_OK ||
        !ecan_igh_domain_is_complete(port->igh) ||
        !ecan_igh_slaves_are_operational(port->igh)) {
        result->consecutive_stable_cycles = 0U;
    } else if (result->cycles_completed == 0U ||
               rx_count != result->last_rx_count) {
        result->consecutive_stable_cycles = 1U;
    } else {
        result->consecutive_stable_cycles++;
    }
    result->cycles_completed++;
    result->last_rx_count = rx_count;
    if (rx_count > result->max_rx_count) {
        result->max_rx_count = rx_count;
    }
    return rc;
}

/* Read the latest TxPDO fixed header for one slave without parsing frames. */
int igh_mcan_port_get_tpdo_header(igh_mcan_port_t *port,
                                  size_t slave_pos,
                                  igh_mcan_pdo_header_t *out_header)
{
    const uint8_t *txpdo = NULL;
    size_t pdo_size = 0U;

    if ((port == NULL) || (port->igh == NULL) || (out_header == NULL) ||
        slave_pos >= port->igh->slave_count) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (ecan_igh_get_txpdo_buffer(port->igh, slave_pos, &txpdo, &pdo_size) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    return igh_mcan_pdo_read_header(txpdo, pdo_size, out_header);
}

/* Return the last stored parse diagnostic and optional slave index. */
const igh_mcan_pdo_parse_diag_t *igh_mcan_port_get_last_parse_error(
    const igh_mcan_port_t *port,
    size_t *slave_index)
{
    if ((port == NULL) || (port->last_parse_valid == 0U)) {
        return NULL;
    }
    if (slave_index != NULL) {
        *slave_index = port->last_parse_slave;
    }
    return &port->last_parse_diag;
}
