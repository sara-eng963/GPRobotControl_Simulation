#ifndef AVATAR_M_SILKIT_NODE_H
#define AVATAR_M_SILKIT_NODE_H

#include "../../CANComm/SILKit/silkit_can_backend.h"
#include "../../ServoDrive/AvatarM/Sim/avatar_m_node_sim.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t node_id;

    const char *participant_name;
    const char *controller_name;
    const char *network_name;
    const char *registry_uri;

    uint32_t bitrate;

} AvatarMSilKitNodeConfig;

typedef struct
{
    CanBackend backend;
    AvatarMNodeSim motor;

} AvatarMSilKitNode;

typedef enum
{
    AVATAR_M_SILKIT_IDLE = 0,
    AVATAR_M_SILKIT_FRAME_PROCESSED,
    AVATAR_M_SILKIT_FRAME_SENT,
    AVATAR_M_SILKIT_ERROR

} AvatarMSilKitStepResult;

/*
 * Create one SIL Kit participant that represents one AVATAR M CANopen node.
 *
 * This function only creates the transport and initializes the protocol model.
 * It does not assume a work mode, actual position, statusword, or any physical
 * motor dynamics.
 */
bool avatar_m_silkit_node_create(
    AvatarMSilKitNode *node,
    const AvatarMSilKitNodeConfig *config
);

/*
 * Process at most one CAN frame currently waiting in the SIL Kit backend.
 *
 * If the AVATAR protocol model generates an immediate response (for example
 * TPDO4 after RPDO4), this function transmits that response on the same bus.
 */
AvatarMSilKitStepResult avatar_m_silkit_node_pump_once(
    AvatarMSilKitNode *node
);

/*
 * Advance the AVATAR heartbeat timer by elapsed_ms.
 *
 * If a heartbeat becomes due, it is transmitted through SIL Kit.
 */
AvatarMSilKitStepResult avatar_m_silkit_node_tick_ms(
    AvatarMSilKitNode *node,
    uint32_t elapsed_ms
);

void avatar_m_silkit_node_close(
    AvatarMSilKitNode *node
);

#ifdef __cplusplus
}
#endif

#endif /* AVATAR_M_SILKIT_NODE_H */
