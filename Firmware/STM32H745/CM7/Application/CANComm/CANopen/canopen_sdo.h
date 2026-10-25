#ifndef CANOPEN_SDO_H
#define CANOPEN_SDO_H

#include "../can_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CANOPEN_SDO_RESPONSE_INVALID = 0,
    CANOPEN_SDO_RESPONSE_READ,
    CANOPEN_SDO_RESPONSE_WRITE_OK,
    CANOPEN_SDO_RESPONSE_ABORT
} CanopenSdoResponseType;

typedef struct
{
    CanopenSdoResponseType type;

    uint16_t index;
    uint8_t subindex;

    /* Valid for expedited read responses: 1, 2, or 4 bytes. */
    uint8_t data_size;
    uint32_t value;

    /* Valid only when type == CANOPEN_SDO_RESPONSE_ABORT. */
    uint32_t abort_code;
} CanopenSdoResponse;

bool canopen_sdo_build_read(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanFrame *frame
);

bool canopen_sdo_build_write_u8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint8_t value,
    CanFrame *frame
);

bool canopen_sdo_build_write_u16(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint16_t value,
    CanFrame *frame
);

bool canopen_sdo_build_write_u32(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t value,
    CanFrame *frame
);

bool canopen_sdo_parse_response(
    uint8_t node_id,
    const CanFrame *frame,
    CanopenSdoResponse *response
);

#endif /* CANOPEN_SDO_H */
