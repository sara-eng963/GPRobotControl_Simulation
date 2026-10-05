#include "avatar_m_pdo.h"

#include "../../CANComm/CANopen/canopen_ids.h"

#include <stddef.h>
#include <string.h>


static void write_u32_le(
    uint8_t *data,
    uint32_t value
)
{
    data[0] = (uint8_t)(value);
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
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


static uint16_t read_u16_le(
    const uint8_t *data
)
{
    return
        (uint16_t)(
            ((uint16_t)data[0]) |
            ((uint16_t)data[1] << 8)
        );
}


bool avatar_m_build_rpdo4(
    uint8_t node_id,
    int32_t target_position,
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

    frame->id =
        canopen_rpdo4_id(node_id);

    frame->dlc = 4U;

    write_u32_le(
        frame->data,
        (uint32_t)target_position
    );

    return true;
}


bool avatar_m_parse_tpdo4(
    uint8_t node_id,
    const CanFrame *frame,
    AvatarMFeedback *feedback
)
{
    if (
        frame == NULL ||
        feedback == NULL ||
        !canopen_node_id_valid(node_id)
    )
    {
        return false;
    }

    if (
        frame->id != canopen_tpdo4_id(node_id) ||
        frame->dlc < 6U
    )
    {
        return false;
    }

    uint32_t raw_position =
        read_u32_le(frame->data);

    memcpy(
        &feedback->actual_position,
        &raw_position,
        sizeof(raw_position)
    );

    feedback->statusword =
        read_u16_le(&frame->data[4]);

    return true;
}


void avatar_m_build_sync(
    CanFrame *frame
)
{
    if (frame == NULL)
    {
        return;
    }

    memset(frame, 0, sizeof(*frame));

    frame->id = CANOPEN_COBID_SYNC;
    frame->dlc = 0U;
}