#ifndef CANOPEN_HEARTBEAT_H
#define CANOPEN_HEARTBEAT_H

#include "../can_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t node_id;
    uint8_t state;
} CanopenHeartbeat;

/*
 * Parse one heartbeat frame from 0x700 + node-id.
 *
 * The state byte is returned raw because the AVATAR vendor manual gives
 * vendor-specific meaning to 0x04 (alarm), so semantic interpretation belongs
 * in the AVATAR drive layer rather than generic CANopen transport code.
 */
bool canopen_heartbeat_parse(
    const CanFrame *frame,
    CanopenHeartbeat *heartbeat
);

#endif /* CANOPEN_HEARTBEAT_H */
