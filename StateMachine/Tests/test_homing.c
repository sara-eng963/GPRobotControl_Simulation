#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "state_boot.h"
#include "state_homing.h"
#include "robot_config.h"
#include "sim_can_bus.h"
#include "canopen_master.h"
#include "avatar_m_position.h"
#include "cia402.h"

#define TEST_NUM_AXES 6U
#define TEST_REDUCTION_RATIO 50.0
#define DEG2RAD(x) ((x) * ROBOT_PI / 180.0)

static const char *homing_phase_name(HomingPhase phase)
{
    switch (phase)
    {
        case HOMING_PHASE_INIT: return "INIT";
        case HOMING_PHASE_READ_POSITION: return "READ_POSITION";
        case HOMING_PHASE_PREPARE_TRAJECTORY: return "PREPARE_TRAJECTORY";
        case HOMING_PHASE_EXECUTE_TRAJECTORY: return "EXECUTE_TRAJECTORY";
        case HOMING_PHASE_VERIFY_HOME: return "VERIFY_HOME";
        case HOMING_PHASE_COMPLETE: return "COMPLETE";
        case HOMING_PHASE_FAILED: return "FAILED";
        default: return "UNKNOWN";
    }
}

static const char *homing_error_name(HomingError error)
{
    switch (error)
    {
        case HOMING_ERROR_NONE: return "NONE";
        case HOMING_ERROR_INVALID_CONFIG: return "INVALID_CONFIG";
        case HOMING_ERROR_HOME_NOT_DEFINED: return "HOME_NOT_DEFINED";
        case HOMING_ERROR_HOME_LIMIT: return "HOME_LIMIT";
        case HOMING_ERROR_POSITION_FEEDBACK: return "POSITION_FEEDBACK";
        case HOMING_ERROR_DRIVE_NOT_ENABLED: return "DRIVE_NOT_ENABLED";
        case HOMING_ERROR_TRAJECTORY: return "TRAJECTORY";
        case HOMING_ERROR_TRAJECTORY_LIMIT: return "TRAJECTORY_LIMIT";
        case HOMING_ERROR_COMMUNICATION: return "COMMUNICATION";
        case HOMING_ERROR_HOME_TIMEOUT: return "HOME_TIMEOUT";
        case HOMING_ERROR_POSITION_CONVERSION: return "POSITION_CONVERSION";
        case HOMING_ERROR_CYCLIC_FEEDBACK: return "CYCLIC_FEEDBACK";
        default: return "UNKNOWN";
    }
}

int main(void)
{
    puts("\n============================================================");
    puts(" CANOPEN / AVATAR HOMING STATE TEST");
    puts("============================================================");

    RobotConfig robot;
    robot_config_init_ur5(&robot);
    robot.configuration.homeDefined = true;
    robot.configuration.home[0] = DEG2RAD(0.0);
    robot.configuration.home[1] = DEG2RAD(-90.0);
    robot.configuration.home[2] = DEG2RAD(90.0);
    robot.configuration.home[3] = DEG2RAD(0.0);
    robot.configuration.home[4] = DEG2RAD(0.0);
    robot.configuration.home[5] = DEG2RAD(0.0);

    AvatarMPositionScale scales[TEST_NUM_AXES];

    for (size_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        if (!avatar_m_position_scale_default(&scales[i], TEST_REDUCTION_RATIO))
        {
            puts("Could not configure AVATAR position scale.");
            return 1;
        }
    }

    AvatarMSimBus bus = {0};
    CanBackend backend = {0};

    if (!avatar_sim_bus_init(&bus, &backend))
    {
        puts("Could not initialize AVATAR simulation bus.");
        return 1;
    }

    for (size_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        int32_t initial_position = 0;

        if (!avatar_m_joint_rad_to_position_units(
                &scales[i],
                robot.configuration.home[i] + DEG2RAD(5.0),
                &initial_position))
        {
            puts("Could not build simulated start position.");
            return 1;
        }

        avatar_m_node_set_work_mode(&bus.nodes[i], 1U);
        avatar_m_node_set_statusword(
            &bus.nodes[i],
            CIA402_STATE_SWITCH_ON_DISABLED
        );
        avatar_m_node_set_actual_position(
            &bus.nodes[i],
            initial_position
        );
    }

    avatar_sim_bus_set_demo_rate(&bus, 1000000.0);
    avatar_sim_bus_set_demo_motion(&bus, true);

    CanopenMaster master = {0};
    const CanopenMasterConfig master_config =
    {
        .backend = &backend,
        .node_ids = {1U, 2U, 3U, 4U, 5U, 6U},
        .node_count = TEST_NUM_AXES,
        .heartbeat_timeout_ms = 300U,
        .sdo_timeout_ms = 50U
    };

    if (!canopen_master_init(&master, &master_config))
    {
        puts("Could not initialize CANopen master.");
        return 1;
    }

    BootState boot;
    state_boot_enter(&boot);

    uint32_t now_ms = 0U;
    StateStepResult result = STATE_STEP_RUNNING;

    while (result == STATE_STEP_RUNNING && now_ms < 5000U)
    {
        ++now_ms;
        avatar_sim_bus_tick(&bus, 1U);
        result = state_boot_step(&boot, &master, now_ms);
    }

    if (result != STATE_STEP_COMPLETE)
    {
        printf(
            "CAN BOOT failed before HOMING. error=%u axis=%d\n",
            (unsigned)boot.error,
            boot.failedAxis
        );
        canopen_master_close(&master);
        return 1;
    }

    puts("CAN BOOT COMPLETE -> starting CAN HOMING");

    const HomingConfig homing_config =
    {
        .duration = 1.0,
        .dt = 0.002,
        .positionTolerance = DEG2RAD(0.5),
        .requiredStableCycles = 20U,
        .maxVerificationCycles = 2000U
    };

    HomingState homing;
    state_homing_enter(&homing);
    HomingPhase previous_phase = homing.phase;

    printf("HOMING -> %s\n", homing_phase_name(homing.phase));

    result = STATE_STEP_RUNNING;

    while (result == STATE_STEP_RUNNING && now_ms < 12000U)
    {
        ++now_ms;
        avatar_sim_bus_tick(&bus, 1U);

        result = state_homing_step(
            &homing,
            &homing_config,
            &robot,
            &master,
            scales,
            now_ms
        );

        if (homing.phase != previous_phase)
        {
            printf("HOMING -> %s\n", homing_phase_name(homing.phase));
            previous_phase = homing.phase;
        }
    }

    if (result != STATE_STEP_COMPLETE)
    {
        printf(
            "\nHOMING TEST FAILED\nError: %s\nFailed axis: %d\nSamples: %zu\n",
            homing_error_name(homing.error),
            homing.failedAxis,
            homing.samplesSent
        );
        canopen_master_close(&master);
        return 1;
    }

    for (size_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        const AvatarMDrive *drive = canopen_master_drive(&master, i);
        double q_actual = 0.0;

        if (
            drive == NULL ||
            !drive->feedback_valid ||
            !avatar_m_position_units_to_joint_rad(
                &scales[i],
                drive->feedback.actual_position,
                &q_actual) ||
            fabs(q_actual - robot.configuration.home[i]) >
                homing_config.positionTolerance
        )
        {
            printf("Final home verification failed on axis %zu.\n", i + 1U);
            canopen_master_close(&master);
            return 1;
        }
    }

    puts("\n============================================================");
    puts(" CANOPEN HOMING TEST PASSED");
    puts("============================================================");
    printf("Target period : %u ms (500 Hz)\n", (unsigned)homing.commandPeriodMs);
    printf("Samples sent  : %zu\n", homing.samplesSent);
    puts("Fresh TPDO4 feedback required after each command");
    puts("Final home tolerance verified on all 6 axes");
    puts("============================================================");

    canopen_master_close(&master);
    return 0;
}
