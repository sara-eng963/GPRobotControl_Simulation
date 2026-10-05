#include "canopen_heartbeat.h"

#include "canopen_ids.h"

#include <stddef.h>

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
