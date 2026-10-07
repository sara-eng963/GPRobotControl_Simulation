#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "state_boot.h"
#include "state_approach.h"

#include "robot_config.h"
#include "sim_can_bus.h"
#include "canopen_master.h"
#include "avatar_m_position.h"
#include "cia402.h"

#define TEST_NUM_AXES 6U
#define TEST_REDUCTION_RATIO 50.0

typedef struct
{
    PvExecutionSample sample;
} TestStorage;

static bool read_sample(
    uint32_t sample_index,
    PvExecutionSample *sample,
    void *context
)
{
    TestStorage *storage =
        (TestStorage *)context;

    if (
        storage == NULL ||
        sample == NULL ||
        sample_index != 0U
    )
    {
        return false;
    }

    *sample =
        storage->sample;

    return true;
}

static bool configure_scales(
    AvatarMPositionScale scales[TEST_NUM_AXES]
)
{
    for (size_t i = 0U;
         i < TEST_NUM_AXES;
         ++i)
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

static bool configure_initial_positions(
    AvatarMSimBus *bus,
    const AvatarMPositionScale scales[TEST_NUM_AXES],
    const double q_start[TEST_NUM_AXES]
)
{
    for (size_t i = 0U;
         i < TEST_NUM_AXES;
         ++i)
    {
        int32_t raw = 0;

        if (!avatar_m_joint_rad_to_position_units(
                &scales[i],
                q_start[i],
                &raw))
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
            raw
        );
    }

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

    StateStepResult result =
        STATE_STEP_RUNNING;

    while (
        result == STATE_STEP_RUNNING &&
        *now_ms < 5000U
    )
    {
        ++(*now_ms);

        avatar_sim_bus_tick(
            bus,
            1U
        );

        result =
            state_boot_step(
                &boot,
                master,
                *now_ms
            );
    }

    return
        result == STATE_STEP_COMPLETE;
}

int main(void)
{
    puts(
        "\n"
        "============================================================\n"
        " CANOPEN / AVATAR APPROACH TEST\n"
        "============================================================"
    );

    RobotConfig robot;
    robot_config_init_ur5(&robot);

    AvatarMPositionScale scales[TEST_NUM_AXES];

    if (!configure_scales(scales))
    {
        puts("Could not configure AVATAR position scales.");
        return 1;
    }

    const double q_start[TEST_NUM_AXES] =
    {
        0.00, -1.00, 1.00, 0.00, 0.00, 0.00
    };

    const double q_target[TEST_NUM_AXES] =
    {
        0.05, -1.00, 1.00, 0.00, 0.00, 0.00
    };

    AvatarMSimBus bus = {0};
    CanBackend backend = {0};

    if (!avatar_sim_bus_init(
            &bus,
            &backend))
    {
        puts("Could not initialize AVATAR simulation bus.");
        return 1;
    }

    if (!configure_initial_positions(
            &bus,
            scales,
            q_start))
    {
        puts("Could not configure initial joint positions.");
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
        puts("CAN BOOT failed before APPROACH.");
        canopen_master_close(&master);
        return 1;
    }

    puts("CAN BOOT COMPLETE");

    avatar_sim_bus_set_demo_rate(
        &bus,
        100000000.0
    );

    avatar_sim_bus_set_demo_motion(
        &bus,
        true
    );

    TestStorage storage;
    memset(
        &storage,
        0,
        sizeof(storage)
    );

    for (size_t i = 0U;
         i < TEST_NUM_AXES;
         ++i)
    {
        if (!avatar_m_joint_rad_to_position_units(
                &scales[i],
                q_target[i],
                &storage.sample.target_position_units[i]))
        {
            puts("Could not build validated target.");
            canopen_master_close(&master);
            return 1;
        }
    }

    ValidatedTrajectory trajectory;
    memset(
        &trajectory,
        0,
        sizeof(trajectory)
    );

    trajectory.program_id = 7U;
    trajectory.source_revision = 3U;
    trajectory.artifact_crc = 0x1234ABCDUL;
    trajectory.sample_count = 1U;
    trajectory.segment_count = 1U;
    trajectory.sample_period_us =
        PATH_VALIDATION_SAMPLE_PERIOD_US;

    const ApproachRequest request =
    {
        .operation = APPROACH_OPERATION_PREVIEW,
        .trajectory = &trajectory,
        .trajectory_ready = true,
        .expected_program_id = trajectory.program_id,
        .expected_source_revision = trajectory.source_revision,
        .expected_artifact_crc = trajectory.artifact_crc,
        .clearance_poses = NULL,
        .clearance_pose_count = 0U
    };

    const ApproachConfig config =
    {
        .duration_safety_factor = 1.10,
        .minimum_leg_duration_s = 0.05,
        .maximum_leg_duration_s = 5.0,
        .final_position_tolerance_rad = 0.002,
        .following_error_limit_rad = 0.20,
        .use_jerk_limits = false,
        .required_stable_cycles = 5U,
        .maximum_verification_cycles = 100U,
        .validation_samples_per_step = 64U,
        .require_collision_check = false
    };

    const ApproachServices services =
    {
        .read_validated_sample = read_sample,
        .storage_context = &storage,
        .collision_free = NULL,
        .collision_context = NULL
    };

    ApproachState approach;

    state_approach_enter(
        &approach,
        &robot,
        &master,
        scales,
        &request,
        &config,
        &services
    );

    ApproachControlInputs inputs;
    memset(
        &inputs,
        0,
        sizeof(inputs)
    );

    inputs.motion_permission =
        true;

    ApproachOutputs outputs;
    memset(
        &outputs,
        0,
        sizeof(outputs)
    );

    StateStepResult result =
        STATE_STEP_RUNNING;

    while (
        result == STATE_STEP_RUNNING &&
        now_ms < 10000U
    )
    {
        ++now_ms;

        avatar_sim_bus_tick(
            &bus,
            1U
        );

        result =
            state_approach_step(
                &approach,
                &inputs,
                now_ms,
                &outputs
            );
    }

    if (
        result != STATE_STEP_COMPLETE ||
        approach.result != APPROACH_RESULT_COMPLETE
    )
    {
        printf(
            "APPROACH failed: phase=%s error=%s axis=%u\n",
            state_approach_phase_name(approach.phase),
            state_approach_error_name(approach.error),
            (unsigned)approach.failed_joint
        );

        canopen_master_close(&master);
        return 1;
    }

    if (
        approach.command_period_ms != 2U ||
        approach.samples_sent == 0U
    )
    {
        puts("APPROACH command timing/sample count is invalid.");
        canopen_master_close(&master);
        return 1;
    }

    for (size_t axis = 0U;
         axis < TEST_NUM_AXES;
         ++axis)
    {
        const AvatarMDrive *drive =
            canopen_master_drive(
                &master,
                axis
            );

        double q_actual = 0.0;

        if (
            drive == NULL ||
            !drive->feedback_valid ||
            !avatar_m_position_units_to_joint_rad(
                &scales[axis],
                drive->feedback.actual_position,
                &q_actual) ||
            fabs(
                q_actual -
                q_target[axis]
            ) >
                config.final_position_tolerance_rad
        )
        {
            printf(
                "Final APPROACH target verification failed on axis %zu.\n",
                axis + 1U
            );

            canopen_master_close(&master);
            return 1;
        }
    }

    puts("APPROACH -> COMPLETE");

    printf(
        "Command period : %u ms (500 Hz)\n"
        "Samples sent   : %u\n"
        "Stable cycles  : %u\n",
        (unsigned)approach.command_period_ms,
        (unsigned)approach.samples_sent,
        (unsigned)approach.stable_cycles
    );

    puts(
        "Fresh TPDO4 feedback required after every RPDO4+SYNC command.\n"
        "Exact Path Validation sample-0 AVATAR units verified at completion.\n"
        "============================================================\n"
        " CANOPEN APPROACH TEST PASSED\n"
        "============================================================"
    );

    canopen_master_close(&master);
    return 0;
}
