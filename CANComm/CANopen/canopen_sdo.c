#include "canopen_sdo.h"

#include "canopen_ids.h"

#include <stddef.h>
#include <string.h>

#define CANOPEN_SDO_CS_READ_REQUEST      0x40U
#define CANOPEN_SDO_CS_WRITE_U8          0x2FU
#define CANOPEN_SDO_CS_WRITE_U16         0x2BU
#define CANOPEN_SDO_CS_WRITE_U32         0x23U

#define CANOPEN_SDO_CS_READ_U8_REPLY     0x4FU
#define CANOPEN_SDO_CS_READ_U16_REPLY    0x4BU
#define CANOPEN_SDO_CS_READ_U32_REPLY    0x43U
#define CANOPEN_SDO_CS_WRITE_OK_REPLY    0x60U
#define CANOPEN_SDO_CS_ABORT_REPLY       0x80U

static void write_u16_le(
    uint8_t *data,
    uint16_t value
)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void write_u32_le(
    uint8_t *data,
    uint32_t value
)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint16_t read_u16_le(
    const uint8_t *data
)
{
    return (uint16_t)(
        ((uint16_t)data[0]) |
        ((uint16_t)data[1] << 8)
    );
}

static uint32_t read_u32_le(
    const uint8_t *data
)
{
    return
        ((uint32_t)data[0]) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);
}

static bool build_request_header(
    uint8_t node_id,
    uint8_t command_specifier,
    uint16_t index,
    uint8_t subindex,
    CanFrame *frame
)
{
    if (
        frame == NULL ||
        !canopen_node_id_valid(node_id)
    )
    {
        return false;
    }

    memset(frame, 0, sizeof(*frame));

    frame->id = canopen_sdo_rx_id(node_id);
    frame->dlc = 8U;
    frame->data[0] = command_specifier;
    frame->data[1] = (uint8_t)(index & 0xFFU);
    frame->data[2] = (uint8_t)((index >> 8) & 0xFFU);
    frame->data[3] = subindex;

    return true;
}

bool canopen_sdo_build_read(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    CanFrame *frame
)
{
    return build_request_header(
        node_id,
        CANOPEN_SDO_CS_READ_REQUEST,
        index,
        subindex,
        frame
    );
}

bool canopen_sdo_build_write_u8(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint8_t value,
    CanFrame *frame
)
{
    if (!build_request_header(
            node_id,
            CANOPEN_SDO_CS_WRITE_U8,
            index,
            subindex,
            frame))
    {
        return false;
    }

    frame->data[4] = value;
    return true;
}

bool canopen_sdo_build_write_u16(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint16_t value,
    CanFrame *frame
)
{
    if (!build_request_header(
            node_id,
            CANOPEN_SDO_CS_WRITE_U16,
            index,
            subindex,
            frame))
    {
        return false;
    }

    write_u16_le(&frame->data[4], value);
    return true;
}

bool canopen_sdo_build_write_u32(
    uint8_t node_id,
    uint16_t index,
    uint8_t subindex,
    uint32_t value,
    CanFrame *frame
)
{
    if (!build_request_header(
            node_id,
            CANOPEN_SDO_CS_WRITE_U32,
            index,
            subindex,
            frame))
    {
        return false;
    }

    write_u32_le(&frame->data[4], value);
    return true;
}

bool canopen_sdo_parse_response(
    uint8_t node_id,
    const CanFrame *frame,
    CanopenSdoResponse *response
)
{
    if (
        frame == NULL ||
        response == NULL ||
        !canopen_node_id_valid(node_id) ||
        frame->id != canopen_sdo_tx_id(node_id) ||
        frame->dlc != 8U
    )
    {
        return false;
    }

    memset(response, 0, sizeof(*response));

    response->index = read_u16_le(&frame->data[1]);
    response->subindex = frame->data[3];

    switch (frame->data[0])
    {
        case CANOPEN_SDO_CS_READ_U8_REPLY:
            response->type = CANOPEN_SDO_RESPONSE_READ;
            response->data_size = 1U;
            response->value = frame->data[4];
            return true;

        case CANOPEN_SDO_CS_READ_U16_REPLY:
            response->type = CANOPEN_SDO_RESPONSE_READ;
            response->data_size = 2U;
            response->value = read_u16_le(&frame->data[4]);
            return true;

        case CANOPEN_SDO_CS_READ_U32_REPLY:
            response->type = CANOPEN_SDO_RESPONSE_READ;
            response->data_size = 4U;
            response->value = read_u32_le(&frame->data[4]);
            return true;

        case CANOPEN_SDO_CS_WRITE_OK_REPLY:
            response->type = CANOPEN_SDO_RESPONSE_WRITE_OK;
            return true;

        case CANOPEN_SDO_CS_ABORT_REPLY:
            response->type = CANOPEN_SDO_RESPONSE_ABORT;
            response->abort_code = read_u32_le(&frame->data[4]);
            return true;

        default:
            response->type = CANOPEN_SDO_RESPONSE_INVALID;
            return false;
    }
}
