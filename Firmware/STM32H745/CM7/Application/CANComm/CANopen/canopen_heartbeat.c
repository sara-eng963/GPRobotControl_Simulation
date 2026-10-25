#include "canopen_heartbeat.h"

#include "canopen_ids.h"

#include <stddef.h>
#include <string.h>

bool canopen_heartbeat_build(
    uint8_t producer_node_id,
    CanopenHeartbeatState state,
    CanFrame *frame
)
{
    if (frame == NULL || !canopen_node_id_valid(producer_node_id))
    {
        return false;
    }

    if (state != CANOPEN_HEARTBEAT_OPERATIONAL &&
        state != CANOPEN_HEARTBEAT_PRE_OPERATIONAL &&
        state != CANOPEN_HEARTBEAT_STOPPED)
    {
        /* Boot-up (0x00) is a startup event, not a cyclic producer state. */
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->id = canopen_heartbeat_id(producer_node_id);
    frame->dlc = 1U;
    frame->data[0] = (uint8_t)state;
    return true;
}

bool canopen_heartbeat_parse(
    const CanFrame *frame,
    CanopenHeartbeat *heartbeat
)
{
    if (
        frame == NULL ||
        heartbeat == NULL ||
        frame->dlc != 1U ||
        frame->id <= CANOPEN_HEARTBEAT_BASE ||
        frame->id > (CANOPEN_HEARTBEAT_BASE + CANOPEN_NODE_ID_MAX)
    )
    {
        return false;
    }

    const uint16_t raw_node_id =
        (uint16_t)(frame->id - CANOPEN_HEARTBEAT_BASE);

    if (!canopen_node_id_valid((uint8_t)raw_node_id))
    {
        return false;
    }

    heartbeat->node_id = (uint8_t)raw_node_id;
    heartbeat->state = frame->data[0];

    return true;
}
