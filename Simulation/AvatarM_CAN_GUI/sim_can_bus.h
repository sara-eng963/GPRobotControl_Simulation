#ifndef AVATAR_M_SIM_CAN_BUS_H
#define AVATAR_M_SIM_CAN_BUS_H

#include "../../CANComm/can_backend.h"
#include "../../ServoDrive/AvatarM/Sim/avatar_m_node_sim.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AVATAR_SIM_NODE_COUNT 6U
#define AVATAR_SIM_RX_CAPACITY 512U
#define AVATAR_SIM_LOG_CAPACITY 512U

typedef enum
{
    AVATAR_SIM_LOG_CONTROLLER_TX = 0,
    AVATAR_SIM_LOG_MOTOR_TX
} AvatarSimLogDirection;

typedef struct
{
    uint32_t timestamp_ms;
    AvatarSimLogDirection direction;
    CanFrame frame;
} AvatarSimCanLogEntry;

typedef struct
{
    AvatarMNodeSim nodes[AVATAR_SIM_NODE_COUNT];

    CanFrame rx_queue[AVATAR_SIM_RX_CAPACITY];
    size_t rx_head;
    size_t rx_tail;
    size_t rx_count;

    AvatarSimCanLogEntry log[AVATAR_SIM_LOG_CAPACITY];
    size_t log_head;
    size_t log_count;

    uint32_t now_ms;

    /*
     * Optional visualization-only plant.
     *
     * This is NOT an AVATAR motor-dynamics model. It simply moves the
     * simulated actual position toward the active CANopen target at a bounded
     * rate so that the GUI can visualize command/feedback evolution.
     */
    bool demo_motion_enabled;
    double demo_rate_units_per_second;
    double demo_position[AVATAR_SIM_NODE_COUNT];

} AvatarMSimBus;

/*
 * Initialize six AVATAR protocol models (node IDs 1..6) and expose the
 * controller side as the project's generic CanBackend.
 *
 * Test-fixture preconditions:
 *   - interpolation work mode = 7
 *   - statusword = 0x0437 (Operation Enabled example used by existing tests)
 *   - actual position = 0
 *
 * These values are simulation fixtures, not claimed AVATAR power-on defaults.
 */
bool avatar_sim_bus_init(
    AvatarMSimBus *bus,
    CanBackend *controller_backend
);

/*
 * Advance heartbeat timers and, when enabled, the visualization-only motion
 * model. elapsed_ms may be zero.
 */
void avatar_sim_bus_tick(
    AvatarMSimBus *bus,
    uint32_t elapsed_ms
);

void avatar_sim_bus_set_demo_motion(
    AvatarMSimBus *bus,
    bool enabled
);

void avatar_sim_bus_set_demo_rate(
    AvatarMSimBus *bus,
    double units_per_second
);

void avatar_sim_bus_clear_log(
    AvatarMSimBus *bus
);

size_t avatar_sim_bus_log_count(
    const AvatarMSimBus *bus
);

/*
 * index_from_newest = 0 returns the newest entry.
 * Returned pointer remains valid until the next log append.
 */
const AvatarSimCanLogEntry *avatar_sim_bus_log_newest(
    const AvatarMSimBus *bus,
    size_t index_from_newest
);

#ifdef __cplusplus
}
#endif

#endif /* AVATAR_M_SIM_CAN_BUS_H */
