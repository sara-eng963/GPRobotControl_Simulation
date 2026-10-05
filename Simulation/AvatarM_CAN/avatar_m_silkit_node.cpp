#include "avatar_m_silkit_node.h"

#include <cstring>

extern "C"
bool avatar_m_silkit_node_create(
    AvatarMSilKitNode *node,
    const AvatarMSilKitNodeConfig *config
)
{
    if (
        node == nullptr ||
        config == nullptr ||
        config->participant_name == nullptr ||
        config->controller_name == nullptr ||
        config->network_name == nullptr ||
        config->registry_uri == nullptr ||
        config->bitrate == 0U
    )
    {
        return false;
    }

    std::memset(node, 0, sizeof(*node));

    if (!avatar_m_node_init(
            &node->motor,
            config->node_id))
    {
        return false;
    }

    SilKitCanBackendConfig backend_config{};

    backend_config.participant_name =
        config->participant_name;

    backend_config.controller_name =
        config->controller_name;

    backend_config.network_name =
        config->network_name;

    backend_config.registry_uri =
        config->registry_uri;

    backend_config.bitrate =
        config->bitrate;

    if (!silkit_can_backend_create(
            &node->backend,
            &backend_config))
    {
        std::memset(node, 0, sizeof(*node));
        return false;
    }

    return true;
}


extern "C"
AvatarMSilKitStepResult avatar_m_silkit_node_pump_once(
    AvatarMSilKitNode *node
)
{
    if (node == nullptr)
    {
        return AVATAR_M_SILKIT_ERROR;
    }

    CanFrame incoming{};

    const CanBackendResult receive_result =
        can_backend_receive(
            &node->backend,
            &incoming
        );

    if (receive_result == CAN_BACKEND_WOULD_BLOCK)
    {
        return AVATAR_M_SILKIT_IDLE;
    }

    if (receive_result != CAN_BACKEND_OK)
    {
        return AVATAR_M_SILKIT_ERROR;
    }

    CanFrame response{};

    const AvatarMNodeResult motor_result =
        avatar_m_node_process_frame(
            &node->motor,
            &incoming,
            &response
        );

    if (motor_result == AVATAR_M_NODE_INVALID_FRAME)
    {
        return AVATAR_M_SILKIT_ERROR;
    }

    if (motor_result == AVATAR_M_NODE_NO_RESPONSE)
    {
        return AVATAR_M_SILKIT_FRAME_PROCESSED;
    }

    const CanBackendResult send_result =
        can_backend_send(
            &node->backend,
            &response
        );

    if (send_result != CAN_BACKEND_OK)
    {
        return AVATAR_M_SILKIT_ERROR;
    }

    return AVATAR_M_SILKIT_FRAME_SENT;
}


extern "C"
AvatarMSilKitStepResult avatar_m_silkit_node_tick_ms(
    AvatarMSilKitNode *node,
    uint32_t elapsed_ms
)
{
    if (node == nullptr)
    {
        return AVATAR_M_SILKIT_ERROR;
    }

    CanFrame heartbeat{};

    if (!avatar_m_node_tick_ms(
            &node->motor,
            elapsed_ms,
            &heartbeat))
    {
        return AVATAR_M_SILKIT_IDLE;
    }

    const CanBackendResult send_result =
        can_backend_send(
            &node->backend,
            &heartbeat
        );

    if (send_result != CAN_BACKEND_OK)
    {
        return AVATAR_M_SILKIT_ERROR;
    }

    return AVATAR_M_SILKIT_FRAME_SENT;
}


extern "C"
void avatar_m_silkit_node_close(
    AvatarMSilKitNode *node
)
{
    if (node == nullptr)
    {
        return;
    }

    can_backend_close(
        &node->backend
    );

    std::memset(node, 0, sizeof(*node));
}
