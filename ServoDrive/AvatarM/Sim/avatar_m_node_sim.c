#include "avatar_m_node_sim.h"

#include "../../../CANComm/CANopen/canopen_heartbeat.h"
#include "../../../CANComm/CANopen/canopen_ids.h"
#include "../../../CANComm/CANopen/canopen_nmt.h"
#include "../avatar_m_pdo.h"
#include "../avatar_m_registers.h"

#include <stddef.h>
#include <string.h>

static bool nmt_targets_node(
    const AvatarMNodeSim *node,
    const CanFrame *frame
)
{
    if (
        frame->id != CANOPEN_COBID_NMT ||
        frame->dlc != 2U
    )
    {
        return false;
    }

    /* Node-ID 0 is the NMT broadcast address. */
    return
        frame->data[1] == 0U ||
        frame->data[1] == node->node_id;
}

static void build_heartbeat(
    const AvatarMNodeSim *node,
    uint8_t state,
    CanFrame *frame
)
{
    memset(frame, 0, sizeof(*frame));

    frame->id = canopen_heartbeat_id(node->node_id);
    frame->dlc = 1U;
    frame->data[0] = state;
}

static AvatarMNodeResult handle_nmt(
    AvatarMNodeSim *node,
    const CanFrame *input,
    CanFrame *response
)
{
    if (!nmt_targets_node(node, input))
    {
        return AVATAR_M_NODE_NO_RESPONSE;
    }

    switch ((CanopenNmtCommand)input->data[0])
    {
        case CANOPEN_NMT_START:
            node->nmt_state = AVATAR_M_NMT_OPERATIONAL;
            node->heartbeat_elapsed_ms = 0U;
            return AVATAR_M_NODE_NO_RESPONSE;

        case CANOPEN_NMT_STOP:
            node->nmt_state = AVATAR_M_NMT_STOPPED;
            node->heartbeat_elapsed_ms = 0U;
            return AVATAR_M_NODE_NO_RESPONSE;

        case CANOPEN_NMT_PRE_OPERATIONAL:
            node->nmt_state = AVATAR_M_NMT_PRE_OPERATIONAL;
            node->heartbeat_elapsed_ms = 0U;
            return AVATAR_M_NODE_NO_RESPONSE;

        case CANOPEN_NMT_RESET_APPLICATION:
        case CANOPEN_NMT_RESET_COMMUNICATION:
            /*
             * AVATAR manual: reset emits boot-up 0 once, then 0x7F after
             * returning to pre-operational state.
             *
             * Saved-object restoration and CAN peripheral reinitialization are
             * not invented here because those details are not represented by
             * this protocol-only model yet.
             */
            node->nmt_state = AVATAR_M_NMT_PRE_OPERATIONAL;
            node->heartbeat_elapsed_ms = 0U;
            node->cached_target_valid = false;
            node->active_target_valid = false;

            build_heartbeat(
                node,
                CANOPEN_HEARTBEAT_BOOTUP,
                response
            );

            return AVATAR_M_NODE_RESPONSE;

        default:
            return AVATAR_M_NODE_INVALID_FRAME;
    }
}

static AvatarMNodeResult handle_rpdo4(
    AvatarMNodeSim *node,
    const CanFrame *input,
    CanFrame *response
)
{
    int32_t target_position = 0;

    /* PDOs are valid only in the operational NMT state. */
    if (node->nmt_state != AVATAR_M_NMT_OPERATIONAL)
    {
        return AVATAR_M_NODE_NO_RESPONSE;
    }

    /* This simulator only claims the documented interpolation-mode behavior. */
    if (node->work_mode != (uint8_t)AVATAR_M_MODE_INTERPOLATION)
    {
        return AVATAR_M_NODE_NO_RESPONSE;
    }

    if (!avatar_m_parse_rpdo4(
            node->node_id,
            input,
            &target_position))
    {
        return AVATAR_M_NODE_INVALID_FRAME;
    }

    /*
     * RPDO4 transport type is 1 (synchronous): cache 0x607A now, but do not
     * release the new target for execution until a SYNC frame is received.
     */
    node->cached_target_position = target_position;
    node->cached_target_valid = true;

    /*
     * TPDO4 is documented as asynchronous and is returned immediately after
     * RPDO4. It reports the CURRENT 0x6064 actual position and 0x6041
     * statusword, not the newly cached target.
     */
    if (!avatar_m_build_tpdo4(
            node->node_id,
            node->actual_position,
            node->statusword,
            response))
    {
        return AVATAR_M_NODE_INVALID_FRAME;
    }

    return AVATAR_M_NODE_RESPONSE;
}

static AvatarMNodeResult handle_sync(
    AvatarMNodeSim *node,
    const CanFrame *input
)
{
    if (
        input->id != CANOPEN_COBID_SYNC ||
        input->dlc != 0U
    )
    {
        return AVATAR_M_NODE_INVALID_FRAME;
    }

    if (
        node->nmt_state != AVATAR_M_NMT_OPERATIONAL ||
        node->work_mode != (uint8_t)AVATAR_M_MODE_INTERPOLATION ||
        !node->cached_target_valid
    )
    {
        return AVATAR_M_NODE_NO_RESPONSE;
    }

    /*
     * SYNC releases the cached target. It deliberately does NOT change
     * actual_position; no undocumented physical/servo dynamics are assumed.
     */
    node->active_target_position = node->cached_target_position;
    node->active_target_valid = true;
    node->cached_target_valid = false;

    return AVATAR_M_NODE_NO_RESPONSE;
}

bool avatar_m_node_init(
    AvatarMNodeSim *node,
    uint8_t node_id
)
{
    if (
        node == NULL ||
        !canopen_node_id_valid(node_id)
    )
    {
        return false;
    }

    memset(node, 0, sizeof(*node));

    node->node_id = node_id;
    node->nmt_state = AVATAR_M_NMT_PRE_OPERATIONAL;
    node->heartbeat_period_ms = AVATAR_M_DEFAULT_HEARTBEAT_PRODUCER_MS;

    return true;
}

AvatarMNodeResult avatar_m_node_process_frame(
    AvatarMNodeSim *node,
    const CanFrame *input,
    CanFrame *response
)
{
    if (
        node == NULL ||
        input == NULL ||
        response == NULL
    )
    {
        return AVATAR_M_NODE_INVALID_FRAME;
    }

    if (input->id == CANOPEN_COBID_NMT)
    {
        return handle_nmt(node, input, response);
    }

    if (input->id == CANOPEN_COBID_SYNC)
    {
        return handle_sync(node, input);
    }

    if (input->id == canopen_rpdo4_id(node->node_id))
    {
        return handle_rpdo4(node, input, response);
    }

    return AVATAR_M_NODE_NO_RESPONSE;
}

bool avatar_m_node_tick_ms(
    AvatarMNodeSim *node,
    uint32_t elapsed_ms,
    CanFrame *heartbeat
)
{
    if (
        node == NULL ||
        heartbeat == NULL
    )
    {
        return false;
    }

    /* AVATAR manual says heartbeat is invalid in the stopped state. */
    if (node->nmt_state == AVATAR_M_NMT_STOPPED)
    {
        return false;
    }

    /* Object 0x1017 value 0 disables heartbeat production. */
    if (node->heartbeat_period_ms == 0U)
    {
        return false;
    }

    node->heartbeat_elapsed_ms += elapsed_ms;

    if (node->heartbeat_elapsed_ms < node->heartbeat_period_ms)
    {
        return false;
    }

    node->heartbeat_elapsed_ms -= node->heartbeat_period_ms;

    if (node->nmt_state == AVATAR_M_NMT_OPERATIONAL)
    {
        build_heartbeat(
            node,
            CANOPEN_HEARTBEAT_OPERATIONAL,
            heartbeat
        );
    }
    else
    {
        build_heartbeat(
            node,
            CANOPEN_HEARTBEAT_PRE_OPERATIONAL,
            heartbeat
        );
    }

    return true;
}

void avatar_m_node_set_actual_position(
    AvatarMNodeSim *node,
    int32_t actual_position
)
{
    if (node != NULL)
    {
        node->actual_position = actual_position;
    }
}

void avatar_m_node_set_statusword(
    AvatarMNodeSim *node,
    uint16_t statusword
)
{
    if (node != NULL)
    {
        node->statusword = statusword;
    }
}

void avatar_m_node_set_work_mode(
    AvatarMNodeSim *node,
    uint8_t work_mode
)
{
    if (node != NULL)
    {
        node->work_mode = work_mode;
    }
}
