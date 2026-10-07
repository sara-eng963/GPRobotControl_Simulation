#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "state_boot.h"
#include "state_homing.h"
#include "state_idle.h"

#include "robot_config.h"
#include "sim_can_bus.h"
#include "canopen_master.h"
#include "avatar_m_position.h"
#include "cia402.h"

#define TEST_NUM_AXES 6U
#define TEST_IDLE_CYCLES 200U
#define TEST_REDUCTION_RATIO 50.0

#define DEG2RAD(x) ((x) * ROBOT_PI / 180.0)

static const char *idle_phase_name(
    IdlePhase phase
)
{
    switch (phase)
    {
        case IDLE_PHASE_INIT: return "INIT";
        case IDLE_PHASE_HOLDING: return "HOLDING";
        case IDLE_PHASE_COMPLETE: return "COMPLETE";
        case IDLE_PHASE_FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

static const char *idle_error_name(
    IdleError error
)
{
    switch (error)
    {
        case IDLE_ERROR_NONE: return "NONE";
        case IDLE_ERROR_COMMUNICATION: return "COMMUNICATION";
        case IDLE_ERROR_POSITION_FEEDBACK: return "POSITION_FEEDBACK";
        case IDLE_ERROR_DRIVE_NOT_ENABLED: return "DRIVE_NOT_ENABLED";
        case IDLE_ERROR_CYCLIC_FEEDBACK: return "CYCLIC_FEEDBACK";
        case IDLE_ERROR_SAFETY: return "SAFETY";
        case IDLE_ERROR_INVALID_COMMAND: return "INVALID_COMMAND";
        default: return "UNKNOWN";
    }
}

static void configure_robot(
    RobotConfig *robot
)
{
    robot_config_init_ur5(robot);

    robot->configuration.homeDefined = true;
    robot->configuration.home[0] = DEG2RAD(0.0);
    robot->configuration.home[1] = DEG2RAD(-90.0);
    robot->configuration.home[2] = DEG2RAD(90.0);
    robot->configuration.home[3] = DEG2RAD(0.0);
    robot->configuration.home[4] = DEG2RAD(0.0);
    robot->configuration.home[5] = DEG2RAD(0.0);
}

static bool configure_scales(
    AvatarMPositionScale scales[TEST_NUM_AXES]
)
{
    for (size_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        if (!avatar_m_position_scale_default(
                &scales[i],
                TEST_REDUCTION_RATIO))
        {
            return false;
        }
    }

    return true;
}

static bool configure_simulated_start(
    AvatarMSimBus *bus,
    const RobotConfig *robot,
    const AvatarMPositionScale scales[TEST_NUM_AXES]
)
{
    for (size_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        int32_t initial_position = 0;

        if (!avatar_m_joint_rad_to_position_units(
                &scales[i],
                robot->configuration.home[i] + DEG2RAD(5.0),
                &initial_position))
        {
            return false;
        }

        avatar_m_node_set_work_mode(
            &bus->nodes[i],
            1U
        );

        avatar_m_node_set_statusword(
            &bus->nodes[i],
            CIA402_STATE_SWITCH_ON_DISABLED
        );

        avatar_m_node_set_actual_position(
            &bus->nodes[i],
            initial_position
        );
    }

    avatar_sim_bus_set_demo_rate(
        bus,
        1000000.0
    );

    avatar_sim_bus_set_demo_motion(
        bus,
        true
    );

    return true;
}

static bool run_boot(
    AvatarMSimBus *bus,
    CanopenMaster *master,
    uint32_t *now_ms
)
{
    BootState boot;
    state_boot_enter(&boot);

    StateStepResult result = STATE_STEP_RUNNING;

    while (
        result == STATE_STEP_RUNNING &&
        *now_ms < 5000U
    )
    {
        ++(*now_ms);
        avatar_sim_bus_tick(bus, 1U);

        result =
            state_boot_step(
                &boot,
                master,
                *now_ms
            );
    }

    return result == STATE_STEP_COMPLETE;
}

static bool run_homing(
    AvatarMSimBus *bus,
    CanopenMaster *master,
    const RobotConfig *robot,
    const AvatarMPositionScale scales[TEST_NUM_AXES],
    uint32_t *now_ms
)
{
    const HomingConfig config =
    {
        .duration = 1.0,
        .dt = 0.002,
        .positionTolerance = DEG2RAD(0.5),
        .requiredStableCycles = 20U,
        .maxVerificationCycles = 2000U
    };

    HomingState homing;
    state_homing_enter(&homing);

    StateStepResult result = STATE_STEP_RUNNING;

    while (
        result == STATE_STEP_RUNNING &&
        *now_ms < 12000U
    )
    {
        ++(*now_ms);
        avatar_sim_bus_tick(bus, 1U);

        result =
            state_homing_step(
                &homing,
                &config,
                robot,
                master,
                scales,
                *now_ms
            );
    }

    return result == STATE_STEP_COMPLETE;
}

static bool verify_hold_targets(
    const AvatarMSimBus *bus,
    const IdleState *idle
)
{
    for (size_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        if (
            !bus->nodes[i].active_target_valid ||
            bus->nodes[i].active_target_position !=
                idle->holdPositionUnits[i]
        )
        {
            printf(
                "Hold target mismatch on axis %zu\n",
                i + 1U
            );

            return false;
        }
    }

    return true;
}

int main(void)
{
    puts(
        "\n"
        "============================================================\n"
        " CANOPEN / AVATAR IDLE STATE TEST\n"
        "============================================================"
    );

    RobotConfig robot;
    configure_robot(&robot);

    AvatarMPositionScale scales[TEST_NUM_AXES];

    if (!configure_scales(scales))
    {
        puts("Could not configure AVATAR position scales.");
        return 1;
    }

    AvatarMSimBus bus = {0};
    CanBackend backend = {0};

    if (!avatar_sim_bus_init(&bus, &backend))
    {
        puts("Could not initialize AVATAR simulation bus.");
        return 1;
    }

    if (!configure_simulated_start(
            &bus,
            &robot,
            scales))
    {
        puts("Could not configure simulated start positions.");
        return 1;
    }

    CanopenMaster master = {0};

    const CanopenMasterConfig master_config =
    {
        .backend = &backend,
        .node_ids = {1U, 2U, 3U, 4U, 5U, 6U},
        .node_count = TEST_NUM_AXES,
        .heartbeat_timeout_ms = 300U,
        .sdo_timeout_ms = 50U
    };

    if (!canopen_master_init(
            &master,
            &master_config))
    {
        puts("Could not initialize CANopen master.");
        return 1;
    }

    uint32_t now_ms = 0U;

    if (!run_boot(
            &bus,
            &master,
            &now_ms))
    {
        puts("CAN BOOT failed before IDLE test.");
        canopen_master_close(&master);
        return 1;
    }

    puts("CAN BOOT COMPLETE");

    if (!run_homing(
            &bus,
            &master,
            &robot,
            scales,
            &now_ms))
    {
        puts("CAN HOMING failed before IDLE test.");
        canopen_master_close(&master);
        return 1;
    }

    puts("CAN HOMING COMPLETE -> starting CAN IDLE");

    IdleState idle;
    state_idle_enter(&idle);

    IdlePhase previous_phase = idle.phase;
    bool hold_verified = false;

    printf(
        "IDLE -> %s\n",
        idle_phase_name(idle.phase)
    );

    StateStepResult result = STATE_STEP_RUNNING;

    while (
        result == STATE_STEP_RUNNING &&
        now_ms < 15000U
    )
    {
        ++now_ms;
        avatar_sim_bus_tick(&bus, 1U);

        const IdleCommand command =
            idle.cyclesHeld >= TEST_IDLE_CYCLES
                ? IDLE_COMMAND_TEACH
                : IDLE_COMMAND_NONE;

        result =
            state_idle_step(
                &idle,
                command,
                &master,
                now_ms
            );

        if (idle.phase != previous_phase)
        {
            printf(
                "IDLE -> %s\n",
                idle_phase_name(idle.phase)
            );

            previous_phase = idle.phase;
        }

        if (
            !hold_verified &&
            idle.cyclesHeld >= 100U
        )
        {
            if (!verify_hold_targets(
                    &bus,
                    &idle))
            {
                puts("IDLE HOLD VERIFICATION FAILED");
                canopen_master_close(&master);
                return 1;
            }

            hold_verified = true;
            puts("IDLE hold targets verified.");
        }
    }

    if (result != STATE_STEP_COMPLETE)
    {
        printf(
            "\n"
            "============================================================\n"
            " IDLE TEST FAILED\n"
            "============================================================\n"
            "Error      : %s\n"
            "Failed axis: %d\n"
            "Cycles held: %u\n"
            "============================================================\n",
            idle_error_name(idle.error),
            idle.failedAxis,
            (unsigned)idle.cyclesHeld
        );

        canopen_master_close(&master);
        return 1;
    }

    if (
        idle.exitCommand != IDLE_COMMAND_TEACH ||
        !hold_verified
    )
    {
        puts("IDLE completed with invalid final state.");
        canopen_master_close(&master);
        return 1;
    }

    puts(
        "\n"
        "============================================================\n"
        " CANOPEN IDLE TEST PASSED\n"
        "============================================================"
    );
    printf(
        "Command period      : %u ms (500 Hz)\n"
        "Healthy hold cycles : %u\n"
        "Exit command        : TEACH\n",
        (unsigned)idle.commandPeriodMs,
        (unsigned)idle.cyclesHeld
    );
    puts(
        "Fresh TPDO4 feedback required after each hold command\n"
        "Exact captured AVATAR raw position held on all 6 axes\n"
        "============================================================"
    );

    canopen_master_close(&master);
    return 0;
}
