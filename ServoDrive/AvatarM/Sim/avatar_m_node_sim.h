#ifndef AVATAR_M_NODE_SIM_H
#define AVATAR_M_NODE_SIM_H

#include "../../../CANComm/can_types.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Protocol-level simulation of one AVATAR M-Series CANopen node.
 *
 * This module intentionally models only behavior documented by the AVATAR
 * CANopen manual. It does not invent motor dynamics, acceleration, torque,
 * friction, or closed-loop position response.
 */

typedef enum
{
    AVATAR_M_NMT_STOPPED = 0,
    AVATAR_M_NMT_PRE_OPERATIONAL,
    AVATAR_M_NMT_OPERATIONAL
} AvatarMNmtState;

typedef struct
{
    uint8_t node_id;
    AvatarMNmtState nmt_state;

    /* CANopen object 0x6060. Manual values include 1, 3, 6 and 7. */
    uint8_t work_mode;

    /* Values reported through TPDO4 (0x6064 + 0x6041). */
    int32_t actual_position;
    uint16_t statusword;

    /* RPDO4 writes 0x607A into a cache and waits for SYNC. */
    int32_t cached_target_position;
    bool cached_target_valid;

    /* Target released for execution after SYNC. No physical motion is implied. */
    int32_t active_target_position;
    bool active_target_valid;

    /* Object 0x1017. Vendor manual default: 1000 ms. */
    uint32_t heartbeat_period_ms;
    uint32_t heartbeat_elapsed_ms;
} AvatarMNodeSim;

typedef enum
{
    AVATAR_M_NODE_NO_RESPONSE = 0,
    AVATAR_M_NODE_RESPONSE,
    AVATAR_M_NODE_INVALID_FRAME
} AvatarMNodeResult;

bool avatar_m_node_init(
    AvatarMNodeSim *node,
    uint8_t node_id
);

AvatarMNodeResult avatar_m_node_process_frame(
    AvatarMNodeSim *node,
    const CanFrame *input,
    CanFrame *response
);

bool avatar_m_node_tick_ms(
    AvatarMNodeSim *node,
    uint32_t elapsed_ms,
    CanFrame *heartbeat
);

void avatar_m_node_set_actual_position(
    AvatarMNodeSim *node,
    int32_t actual_position
);

void avatar_m_node_set_statusword(
    AvatarMNodeSim *node,
    uint16_t statusword
);

void avatar_m_node_set_work_mode(
    AvatarMNodeSim *node,
    uint8_t work_mode
);

#endif /* AVATAR_M_NODE_SIM_H */
