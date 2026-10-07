#include "sim_can_bus.h"

#include "../../CANComm/CANopen/canopen_ids.h"
#include "../../ServoDrive/AvatarM/avatar_m_registers.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static void log_frame(
    AvatarMSimBus *bus,
    AvatarSimLogDirection direction,
    const CanFrame *frame
)
{
    if (
        bus == NULL ||
        frame == NULL
    )
    {
        return;
    }

    AvatarSimCanLogEntry *entry =
        &bus->log[bus->log_head];

    entry->timestamp_ms =
        bus->now_ms;

    entry->direction =
        direction;

    entry->frame =
        *frame;

    bus->log_head =
        (bus->log_head + 1U) %
        AVATAR_SIM_LOG_CAPACITY;

    if (bus->log_count < AVATAR_SIM_LOG_CAPACITY)
    {
        bus->log_count++;
    }
}

static bool queue_rx(
    AvatarMSimBus *bus,
    const CanFrame *frame
)
{
    if (
        bus == NULL ||
        frame == NULL ||
        bus->rx_count >= AVATAR_SIM_RX_CAPACITY
    )
    {
        return false;
    }

    bus->rx_queue[bus->rx_tail] =
        *frame;

    bus->rx_tail =
        (bus->rx_tail + 1U) %
        AVATAR_SIM_RX_CAPACITY;

    bus->rx_count++;

    log_frame(
        bus,
        AVATAR_SIM_LOG_MOTOR_TX,
        frame
    );

    return true;
}

static CanBackendResult sim_send(
    void *context,
    const CanFrame *frame
)
{
    AvatarMSimBus *bus =
        (AvatarMSimBus *)context;

    if (
        bus == NULL ||
        frame == NULL ||
        frame->id > CAN_STANDARD_ID_MAX ||
        frame->dlc > CAN_CLASSIC_MAX_DATA_BYTES
    )
    {
        return CAN_BACKEND_ERROR;
    }

    log_frame(
        bus,
        AVATAR_SIM_LOG_CONTROLLER_TX,
        frame
    );

    for (size_t i = 0U; i < AVATAR_SIM_NODE_COUNT; ++i)
    {
        CanFrame response;

        memset(
            &response,
            0,
            sizeof(response)
        );

        const AvatarMNodeResult result =
            avatar_m_node_process_frame(
                &bus->nodes[i],
                frame,
                &response
            );

        if (result == AVATAR_M_NODE_INVALID_FRAME)
        {
            return CAN_BACKEND_ERROR;
        }

        if (
            result == AVATAR_M_NODE_RESPONSE &&
            !queue_rx(
                bus,
                &response)
        )
        {
            return CAN_BACKEND_ERROR;
        }
    }

    return CAN_BACKEND_OK;
}

static CanBackendResult sim_receive(
    void *context,
    CanFrame *frame
)
{
    AvatarMSimBus *bus =
        (AvatarMSimBus *)context;

    if (
        bus == NULL ||
        frame == NULL
    )
    {
        return CAN_BACKEND_ERROR;
    }

    if (bus->rx_count == 0U)
    {
        return CAN_BACKEND_WOULD_BLOCK;
    }

    *frame =
        bus->rx_queue[bus->rx_head];

    bus->rx_head =
        (bus->rx_head + 1U) %
        AVATAR_SIM_RX_CAPACITY;

    bus->rx_count--;

    return CAN_BACKEND_OK;
}

static void sim_close(
    void *context
)
{
    /*
     * The testbench owns AvatarMSimBus storage directly.
     * No heap or OS resource needs closing.
     */
    (void)context;
}

bool avatar_sim_bus_init(
    AvatarMSimBus *bus,
    CanBackend *controller_backend
)
{
    if (
        bus == NULL ||
        controller_backend == NULL
    )
    {
        return false;
    }

    memset(
        bus,
        0,
        sizeof(*bus)
    );

    for (size_t i = 0U; i < AVATAR_SIM_NODE_COUNT; ++i)
    {
        const uint8_t node_id =
            (uint8_t)(i + 1U);

        if (!avatar_m_node_init(
                &bus->nodes[i],
                node_id))
        {
            return false;
        }

        /*
         * Explicit test fixture, matching the existing six-node integration
         * test. These are NOT claimed to be power-on defaults.
         */
        avatar_m_node_set_work_mode(
            &bus->nodes[i],
            (uint8_t)AVATAR_M_MODE_INTERPOLATION
        );

        avatar_m_node_set_statusword(
            &bus->nodes[i],
            0x0437U
        );

        avatar_m_node_set_actual_position(
            &bus->nodes[i],
            0
        );

        bus->demo_position[i] =
            0.0;
    }

    bus->demo_motion_enabled =
        false;

    bus->demo_rate_units_per_second =
        40000.0;

    controller_backend->context =
        bus;

    controller_backend->send =
        sim_send;

    controller_backend->receive =
        sim_receive;

    controller_backend->close =
        sim_close;

    return true;
}

static void tick_demo_motion(
    AvatarMSimBus *bus,
    uint32_t elapsed_ms
)
{
    if (
        !bus->demo_motion_enabled ||
        elapsed_ms == 0U
    )
    {
        return;
    }

    const double dt =
        (double)elapsed_ms /
        1000.0;

    const double max_step =
        bus->demo_rate_units_per_second *
        dt;

    for (size_t i = 0U; i < AVATAR_SIM_NODE_COUNT; ++i)
    {
        AvatarMNodeSim *node =
            &bus->nodes[i];

        if (!node->active_target_valid)
        {
            continue;
        }

        const double target =
            (double)node->active_target_position;

        const double error =
            target -
            bus->demo_position[i];

        if (fabs(error) <= max_step)
        {
            bus->demo_position[i] =
                target;
        }
        else
        {
            bus->demo_position[i] +=
                error > 0.0
                    ? max_step
                    : -max_step;
        }

        avatar_m_node_set_actual_position(
            node,
            (int32_t)llround(
                bus->demo_position[i]
            )
        );
    }
}

void avatar_sim_bus_tick(
    AvatarMSimBus *bus,
    uint32_t elapsed_ms
)
{
    if (bus == NULL)
    {
        return;
    }

    bus->now_ms +=
        elapsed_ms;

    tick_demo_motion(
        bus,
        elapsed_ms
    );

    for (size_t i = 0U; i < AVATAR_SIM_NODE_COUNT; ++i)
    {
        CanFrame heartbeat;

        memset(
            &heartbeat,
            0,
            sizeof(heartbeat)
        );

        if (avatar_m_node_tick_ms(
                &bus->nodes[i],
                elapsed_ms,
                &heartbeat))
        {
            (void)queue_rx(
                bus,
                &heartbeat
            );
        }
    }
}

void avatar_sim_bus_set_demo_motion(
    AvatarMSimBus *bus,
    bool enabled
)
{
    if (bus == NULL)
    {
        return;
    }

    bus->demo_motion_enabled =
        enabled;

    /*
     * Re-seed continuous visualization state from the protocol model so
     * enabling/disabling never creates a numerical jump.
     */
    for (size_t i = 0U; i < AVATAR_SIM_NODE_COUNT; ++i)
    {
        bus->demo_position[i] =
            (double)bus->nodes[i].actual_position;
    }
}

void avatar_sim_bus_set_demo_rate(
    AvatarMSimBus *bus,
    double units_per_second
)
{
    if (
        bus == NULL ||
        !isfinite(units_per_second) ||
        units_per_second <= 0.0
    )
    {
        return;
    }

    bus->demo_rate_units_per_second =
        units_per_second;
}

void avatar_sim_bus_clear_log(
    AvatarMSimBus *bus
)
{
    if (bus == NULL)
    {
        return;
    }

    bus->log_head =
        0U;

    bus->log_count =
        0U;
}

size_t avatar_sim_bus_log_count(
    const AvatarMSimBus *bus
)
{
    return
        bus != NULL
            ? bus->log_count
            : 0U;
}

const AvatarSimCanLogEntry *avatar_sim_bus_log_newest(
    const AvatarMSimBus *bus,
    size_t index_from_newest
)
{
    if (
        bus == NULL ||
        index_from_newest >= bus->log_count
    )
    {
        return NULL;
    }

    const size_t newest =
        (
            bus->log_head +
            AVATAR_SIM_LOG_CAPACITY -
            1U
        ) %
        AVATAR_SIM_LOG_CAPACITY;

    const size_t index =
        (
            newest +
            AVATAR_SIM_LOG_CAPACITY -
            index_from_newest
        ) %
        AVATAR_SIM_LOG_CAPACITY;

    return
        &bus->log[index];
}
