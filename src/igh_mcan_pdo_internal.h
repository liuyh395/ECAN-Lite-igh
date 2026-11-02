#ifndef IGH_MCAN_PDO_INTERNAL_H
#define IGH_MCAN_PDO_INTERNAL_H

#include "igh_mcan_pdo_codec.h"

#include <stddef.h>
#include <stdint.h>

uint16_t igh_mcan_pdo_get_u16_le(const uint8_t *src);
uint32_t igh_mcan_pdo_get_u32_le(const uint8_t *src);
void igh_mcan_pdo_put_u16_le(uint8_t *dst, uint16_t value);
void igh_mcan_pdo_put_u32_le(uint8_t *dst, uint32_t value);

uint16_t igh_mcan_pdo_calc_payload_crc(uint8_t crc_alg, const uint8_t *data, size_t len);
void igh_mcan_pdo_write_barrier(void);

int igh_mcan_pdo_size_is_valid(size_t size);
int igh_mcan_pdo_profile_allows_tsf(uint8_t profile, uint8_t tsf);
int igh_mcan_pdo_profile_allows_d64(uint8_t profile);
size_t igh_mcan_pdo_align_packet_len(uint8_t option_flags, size_t len);
uint16_t igh_mcan_pdo_make_layout_id(size_t pdo_size,
                                     uint8_t channel_mask,
                                     uint8_t encapsulation_profile,
                                     uint8_t option_flags);
size_t igh_mcan_pdo_tsf_len(uint8_t tsf);

int igh_mcan_pdo_parse_fail(igh_mcan_pdo_parse_diag_t *diag,
                            igh_mcan_pdo_parse_error_t code,
                            size_t offset,
                            size_t packet_end,
                            uint16_t ctrl,
                            const igh_mcan_pdo_header_t *header,
                            uint8_t channel,
                            uint8_t dlc,
                            uint8_t tsf,
                            uint8_t kind);
int igh_mcan_pdo_header_is_empty(const igh_mcan_pdo_header_t *header);

int igh_mcan_pdo_writer_append_raw(igh_mcan_pdo_writer_t *writer,
                                   uint8_t channel,
                                   uint32_t can_id,
                                   uint8_t dlc,
                                   int is_extended,
                                   int is_canfd,
                                   int brs,
                                   int x_bit,
                                   uint8_t tx_tsf,
                                   uint8_t schedule_format,
                                   uint8_t schedule_flags,
                                   uint16_t schedule_tag,
                                   uint32_t schedule_offset_ns,
                                   const uint8_t *payload,
                                   size_t payload_len);

#endif /* IGH_MCAN_PDO_INTERNAL_H */
