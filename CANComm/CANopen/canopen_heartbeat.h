#ifndef CANOPEN_HEARTBEAT_H
#define CANOPEN_HEARTBEAT_H

#include "../can_types.h"

#include <stdbool.h>
#include <stdint.h>

/* Standard CANopen NMT state values carried by heartbeat/boot-up messages. */
typedef enum
{
    CANOPEN_HEARTBEAT_BOOTUP = 0x00U,
    CANOPEN_HEARTBEAT_STOPPED = 0x04U,
    CANOPEN_HEARTBEAT_OPERATIONAL = 0x05U,
    CANOPEN_HEARTBEAT_PRE_OPERATIONAL = 0x7FU
} CanopenHeartbeatState;

typedef struct
{
    uint8_t node_id;

    /* Kept raw so device-specific deviations can be interpreted above here. */
    uint8_t state;
} CanopenHeartbeat;

/* Build one standard CANopen heartbeat frame (0x700 + producer node ID).
 * The controller uses OPERATIONAL (0x05) to announce a live NMT master.
 */
bool canopen_heartbeat_build(
    uint8_t producer_node_id,
    CanopenHeartbeatState state,
    CanFrame *frame
);

/* Parse one heartbeat/boot-up frame from 0x700 + node-id. */
bool canopen_heartbeat_parse(
    const CanFrame *frame,
    CanopenHeartbeat *heartbeat
);

#endif /* CANOPEN_HEARTBEAT_H */
