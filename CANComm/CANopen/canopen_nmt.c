#include "canopen_nmt.h"

#include "canopen_ids.h"

#include <stddef.h>
#include <string.h>

static bool nmt_command_valid(
    CanopenNmtCommand command
)
{
    switch (command)
    {
        case CANOPEN_NMT_START:
        case CANOPEN_NMT_STOP:
        case CANOPEN_NMT_PRE_OPERATIONAL:
        case CANOPEN_NMT_RESET_APPLICATION:
        case CANOPEN_NMT_RESET_COMMUNICATION:
            return true;

        default:
            return false;
    }
}

bool canopen_nmt_build(
    CanopenNmtCommand command,
    uint8_t node_id,
    CanFrame *frame
)
{
    if (
        frame == NULL ||
        !nmt_command_valid(command) ||
        node_id > CANOPEN_NODE_ID_MAX
    )
    {
        return false;
    }

    memset(frame, 0, sizeof(*frame));

    frame->id = CANOPEN_COBID_NMT;
    frame->dlc = 2U;
    frame->data[0] = (uint8_t)command;
    frame->data[1] = node_id;

    return true;
}
