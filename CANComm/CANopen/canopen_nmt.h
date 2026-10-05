#ifndef CANOPEN_NMT_H
#define CANOPEN_NMT_H

#include "../can_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CANOPEN_NMT_START = 0x01U,
    CANOPEN_NMT_STOP = 0x02U,
    CANOPEN_NMT_PRE_OPERATIONAL = 0x80U,
    CANOPEN_NMT_RESET_APPLICATION = 0x81U,
    CANOPEN_NMT_RESET_COMMUNICATION = 0x82U
} CanopenNmtCommand;

/*
 * Build one CANopen NMT frame.
 *
 * node_id = 0 broadcasts to all nodes.
 * node_id = 1..127 addresses one node.
 */
bool canopen_nmt_build(
    CanopenNmtCommand command,
    uint8_t node_id,
    CanFrame *frame
);

#endif /* CANOPEN_NMT_H */
