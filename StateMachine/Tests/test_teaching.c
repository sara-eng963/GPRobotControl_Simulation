#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "state_boot.h"
#include "state_homing.h"
#include "state_idle.h"
#include "state_teaching.h"

#include "robot_config.h"
#include "sim_can_bus.h"
#include "canopen_master.h"
#include "avatar_m_position.h"
#include "cia402.h"

#define TEST_NUM_AXES 6U
#define TEST_IDLE_CYCLES 20U
#define TEST_REDUCTION_RATIO 50.0
#define DEG2RAD(x) ((x) * ROBOT_PI / 180.0)

static void configure_robot(RobotConfig *robot)
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
        int32_t raw = 0;

        if (!avatar_m_joint_rad_to_position_units(
                &scales[i],
                robot->configuration.home[i] + DEG2RAD(5.0),
                &raw))
        {
            return false;
        }

        avatar_m_node_set_work_mode(&bus->nodes[i], 1U);
        avatar_m_node_set_statusword(
            &bus->nodes[i],
            CIA402_STATE_SWITCH_ON_DISABLED
        );
        avatar_m_node_set_actual_position(&bus->nodes[i], raw);
    }

    avatar_sim_bus_set_demo_rate(bus, 1000000.0);
    avatar_sim_bus_set_demo_motion(bus, true);
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

    while (result == STATE_STEP_RUNNING && *now_ms < 5000U)
    {
        ++(*now_ms);
        avatar_sim_bus_tick(bus, 1U);
        result = state_boot_step(&boot, master, *now_ms);
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

    while (result == STATE_STEP_RUNNING && *now_ms < 12000U)
    {
        ++(*now_ms);
        avatar_sim_bus_tick(bus, 1U);

        result = state_homing_step(
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

static bool run_idle_to_teach(
    AvatarMSimBus *bus,
    CanopenMaster *master,
    uint32_t *now_ms
)
{
    IdleState idle;
    state_idle_enter(&idle);

    StateStepResult result = STATE_STEP_RUNNING;

    while (result == STATE_STEP_RUNNING && *now_ms < 14000U)
    {
        ++(*now_ms);
        avatar_sim_bus_tick(bus, 1U);

        const IdleCommand command =
            idle.cyclesHeld >= TEST_IDLE_CYCLES
                ? IDLE_COMMAND_TEACH
                : IDLE_COMMAND_NONE;

        result = state_idle_step(
            &idle,
            command,
            master,
            *now_ms
        );
    }

    return
        result == STATE_STEP_COMPLETE &&
        idle.exitCommand == IDLE_COMMAND_TEACH;
}

/*
 * TEACHING itself never commands motion. This helper stands in for the
 * future manual-guidance/admittance controller. It changes J1 in the AVATAR
 * simulator, then publishes a normal RPDO4/SYNC/TPDO4 exchange so the master
 * receives the new measured pose before P2 is recorded.
 */
static bool simulate_guided_joint_move(
    AvatarMSimBus *bus,
    CanopenMaster *master,
    const AvatarMPositionScale scales[TEST_NUM_AXES],
    double joint1_offset_rad,
    uint32_t *now_ms
)
{
    int32_t targets[TEST_NUM_AXES];

    for (size_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        const AvatarMDrive *drive =
            canopen_master_drive(master, i);

        if (
            drive == NULL ||
            !drive->feedback_valid ||
            drive->cia402_state !=
                CIA402_STATE_OPERATION_ENABLED
        )
        {
            return false;
        }

        targets[i] = drive->feedback.actual_position;
    }

    double q1 = 0.0;

    if (!avatar_m_position_units_to_joint_rad(
            &scales[0],
            targets[0],
            &q1))
    {
        return false;
    }

    if (!avatar_m_joint_rad_to_position_units(
            &scales[0],
            q1 + joint1_offset_rad,
            &targets[0]))
    {
        return false;
    }

    for (size_t i = 0U; i < TEST_NUM_AXES; ++i)
    {
        avatar_m_node_set_actual_position(
            &bus->nodes[i],
            targets[i]
        );
    }

    if (!canopen_master_send_target_cycle(
            master,
            targets,
            TEST_NUM_AXES))
    {
        return false;
    }

    *now_ms += 2U;
    avatar_sim_bus_tick(bus, 2U);

    return canopen_master_poll(master, *now_ms);
}

static StateStepResult teaching_event(
    TeachingState *teaching,
    const RobotConfig *robot,
    TeachingRuntimeInputs *runtime,
    CanopenMaster *master,
    const AvatarMPositionScale scales[TEST_NUM_AXES],
    uint32_t now_ms,
    TeachingEvent event,
    TeachingOutputs *outputs
)
{
    runtime->timestamp_ms = now_ms;

    return state_teaching_step(
        teaching,
        robot,
        runtime,
        master,
        scales,
        now_ms,
        event,
        outputs
    );
}

int main(void)
{
    puts(
        "\n"
        "============================================================\n"
        " CANOPEN / AVATAR TEACHING INTEGRATION TEST\n"
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

    if (!configure_simulated_start(&bus, &robot, scales))
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

    if (!canopen_master_init(&master, &master_config))
    {
        puts("Could not initialize CANopen master.");
        return 1;
    }

    uint32_t now_ms = 0U;

    if (!run_boot(&bus, &master, &now_ms))
    {
        puts("CAN BOOT failed before TEACHING test.");
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
        puts("CAN HOMING failed before TEACHING test.");
        canopen_master_close(&master);
        return 1;
    }

    puts("CAN HOMING COMPLETE");

    if (!run_idle_to_teach(
            &bus,
            &master,
            &now_ms))
    {
        puts("CAN IDLE did not exit through TEACH.");
        canopen_master_close(&master);
        return 1;
    }

    puts("CAN IDLE COMPLETE -> TEACHING");

    const TeachingConfig teaching_config =
    {
        .default_speed_mps = 0.010F,
        .minimum_speed_mps = 0.001F,
        .maximum_speed_mps = 0.100F,
        .speed_step_mps = 0.001F,
        .minimum_point_separation_m = 0.002F,
        .collinearity_epsilon_m2 = 1.0e-10F
    };

    TeachingRuntimeInputs runtime =
    {
        .calibration_version = 1U,
        .active_frame_id = 1U,
        .active_tool_id = 1U,
        .robot_motion_settled = true,
        .manual_guidance_active = true,
        .motion_permitted = true,
        .estop_active = false,
        .protective_stop_active = false,
        .global_fault_active = false,
        .robot_homed = true
    };

    TeachingState teaching;
    TeachingOutputs outputs;

    state_teaching_enter(
        &teaching,
        &teaching_config,
        1U
    );

    if (!teaching.initialized)
    {
        puts("TEACHING initialization failed.");
        canopen_master_close(&master);
        return 1;
    }

    printf(
        "TEACHING -> %s\n",
        state_teaching_phase_name(teaching.phase)
    );

    (void)teaching_event(
        &teaching,
        &robot,
        &runtime,
        &master,
        scales,
        now_ms,
        TEACH_EVENT_SELECT_LINE,
        &outputs
    );

    if (!teaching.last_event_accepted)
    {
        printf(
            "SELECT_LINE rejected: %s\n",
            state_teaching_error_name(outputs.error)
        );
        canopen_master_close(&master);
        return 1;
    }

    puts("Teaching: LINE selected");

    (void)teaching_event(
        &teaching,
        &robot,
        &runtime,
        &master,
        scales,
        now_ms,
        TEACH_EVENT_RECORD_POINT,
        &outputs
    );

    if (!teaching.last_event_accepted)
    {
        printf(
            "P1 rejected: %s\n",
            state_teaching_error_name(outputs.error)
        );
        canopen_master_close(&master);
        return 1;
    }

    puts("Teaching: P1 recorded from AVATAR TPDO4 + FK");

    (void)teaching_event(
        &teaching,
        &robot,
        &runtime,
        &master,
        scales,
        now_ms,
        TEACH_EVENT_RECORD_POINT,
        &outputs
    );

    if (
        teaching.last_event_accepted ||
        outputs.error != TEACH_ERR_DUPLICATE_POINT
    )
    {
        puts("Duplicate-point behavior changed unexpectedly.");
        canopen_master_close(&master);
        return 1;
    }

    puts("Teaching: duplicate P2 correctly rejected");

    if (!simulate_guided_joint_move(
            &bus,
            &master,
            scales,
            DEG2RAD(10.0),
            &now_ms))
    {
        puts("Simulated CAN guidance move failed.");
        canopen_master_close(&master);
        return 1;
    }

    (void)teaching_event(
        &teaching,
        &robot,
        &runtime,
        &master,
        scales,
        now_ms,
        TEACH_EVENT_RECORD_POINT,
        &outputs
    );

    if (!teaching.last_event_accepted)
    {
        printf(
            "P2 rejected: %s\n",
            state_teaching_error_name(outputs.error)
        );
        canopen_master_close(&master);
        return 1;
    }

    if (teaching.draft.segment_count != 1U)
    {
        puts("LINE segment was not committed.");
        canopen_master_close(&master);
        return 1;
    }

    puts("Teaching: P2 recorded -> LINE segment complete");

    const StateStepResult teaching_result =
        teaching_event(
            &teaching,
            &robot,
            &runtime,
            &master,
            scales,
            now_ms,
            TEACH_EVENT_VALIDATE_PATH,
            &outputs
        );

    if (
        teaching_result != STATE_STEP_COMPLETE ||
        !teaching.last_event_accepted ||
        !outputs.validation_request
    )
    {
        printf(
            "Validation request failed: %s\n",
            state_teaching_error_name(outputs.error)
        );
        canopen_master_close(&master);
        return 1;
    }

    if (teaching.draft.draft_crc == 0U)
    {
        puts("Draft CRC was not generated.");
        canopen_master_close(&master);
        return 1;
    }

    printf(
        "TEACHING -> %s\n",
        state_teaching_phase_name(teaching.phase)
    );

    puts(
        "\n"
        "============================================================\n"
        " CANOPEN TEACHING INTEGRATION TEST PASSED\n"
        "============================================================"
    );

    printf(
        "Program ID    : %lu\n"
        "Segments      : %u\n"
        "Segment 1     : %s\n"
        "Draft revision: %lu\n"
        "Draft CRC     : 0x%08lX\n"
        "Next state    : PATH VALIDATION\n",
        (unsigned long)teaching.draft.program_id,
        (unsigned)teaching.draft.segment_count,
        state_teaching_segment_name(
            teaching.draft.segments[0].type
        ),
        (unsigned long)teaching.draft.draft_revision,
        (unsigned long)teaching.draft.draft_crc
    );

    canopen_master_close(&master);
    return 0;
}
