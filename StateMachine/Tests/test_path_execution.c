#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "state_boot.h"
#include "state_path_execution.h"

#include "sim_can_bus.h"
#include "canopen_master.h"
#include "avatar_m_position.h"
#include "cia402.h"

#define TEST_NUM_AXES 6U
#define TEST_SAMPLE_COUNT 4U
#define TEST_REDUCTION_RATIO 50.0

typedef struct
{
    PvExecutionSample samples[TEST_SAMPLE_COUNT];
    bool wire_feed_enabled;
    bool wire_feed_was_enabled;
    unsigned retract_steps;
} TestContext;

static bool read_sample(
    uint32_t index,
    PvExecutionSample *sample,
    void *context
)
{
    TestContext *ctx = (TestContext *)context;

    if (
        ctx == NULL ||
        sample == NULL ||
        index >= TEST_SAMPLE_COUNT
    )
    {
        return false;
    }

    *sample = ctx->samples[index];
    return true;
}

static bool wire_feed_enable(
    bool enable,
    void *context
)
{
    TestContext *ctx = (TestContext *)context;

    if (ctx == NULL)
    {
        return false;
    }

    ctx->wire_feed_enabled = enable;

    if (enable)
    {
        ctx->wire_feed_was_enabled = true;
    }

    return true;
}

static bool prepare_retraction(void *context)
{
    TestContext *ctx = (TestContext *)context;

    if (ctx == NULL)
    {
        return false;
    }

    ctx->retract_steps = 0U;
    return true;
}

static StateStepResult step_retraction(void *context)
{
    TestContext *ctx = (TestContext *)context;

    if (ctx == NULL)
    {
        return STATE_STEP_FAILED;
    }

    ++ctx->retract_steps;

    return
        ctx->retract_steps >= 2U
        ? STATE_STEP_COMPLETE
        : STATE_STEP_RUNNING;
}

static bool clearance(void *context)
{
    (void)context;
    return true;
}

static bool controlled_stop(
    bool *stopped,
    void *context
)
{
    (void)context;

    if (stopped == NULL)
    {
        return false;
    }

    *stopped = true;
    return true;
}

static bool configure_scales(
    AvatarMPositionScale scales[TEST_NUM_AXES]
)
{
    for (size_t axis = 0U;
         axis < TEST_NUM_AXES;
         ++axis)
    {
        if (!avatar_m_position_scale_default(
                &scales[axis],
                TEST_REDUCTION_RATIO))
        {
            return false;
        }
    }

    return true;
}

static bool configure_nodes(
    AvatarMSimBus *bus,
    const AvatarMPositionScale scales[TEST_NUM_AXES]
)
{
    const double q_initial[TEST_NUM_AXES] =
    {
        0.0, -1.0, 1.0, 0.0, 0.0, 0.0
    };

    for (size_t axis = 0U;
         axis < TEST_NUM_AXES;
         ++axis)
    {
        int32_t raw = 0;

        if (!avatar_m_joint_rad_to_position_units(
                &scales[axis],
                q_initial[axis],
                &raw))
        {
            return false;
        }

        avatar_m_node_set_work_mode(
            &bus->nodes[axis],
            1U
        );

        avatar_m_node_set_statusword(
            &bus->nodes[axis],
            CIA402_STATE_SWITCH_ON_DISABLED
        );

        avatar_m_node_set_actual_position(
            &bus->nodes[axis],
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
        " CANOPEN / AVATAR PATH EXECUTION TEST\n"
        "============================================================"
    );

    AvatarMPositionScale scales[TEST_NUM_AXES];

    if (!configure_scales(scales))
    {
        puts("Could not configure AVATAR position scales.");
        return 1;
    }

    AvatarMSimBus bus = {0};
    CanBackend backend = {0};

    if (!avatar_sim_bus_init(
            &bus,
            &backend))
    {
        puts("Could not initialize AVATAR simulation bus.");
        return 1;
    }

    if (!configure_nodes(
            &bus,
            scales))
    {
        puts("Could not configure simulated nodes.");
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
        puts("CAN BOOT failed before PATH EXECUTION.");
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

    TestContext context;
    memset(
        &context,
        0,
        sizeof(context)
    );

    for (size_t sample = 0U;
         sample < TEST_SAMPLE_COUNT;
         ++sample)
    {
        const double q1 =
            0.01 * (double)sample;

        const double q[TEST_NUM_AXES] =
        {
            q1, -1.0, 1.0, 0.0, 0.0, 0.0
        };

        for (size_t axis = 0U;
             axis < TEST_NUM_AXES;
             ++axis)
        {
            if (!avatar_m_joint_rad_to_position_units(
                    &scales[axis],
                    q[axis],
                    &context.samples[sample]
                        .target_position_units[axis]))
            {
                puts("Could not build AVATAR trajectory sample.");
                canopen_master_close(&master);
                return 1;
            }
        }
    }

    ValidatedTrajectory trajectory;
    memset(
        &trajectory,
        0,
        sizeof(trajectory)
    );

    trajectory.program_id = 3U;
    trajectory.source_revision = 2U;
    trajectory.artifact_crc = 0xC0FFEE01UL;
    trajectory.sample_count = TEST_SAMPLE_COUNT;
    trajectory.segment_count = 1U;
    trajectory.sample_period_us =
        PATH_VALIDATION_SAMPLE_PERIOD_US;

    const PathExecutionRequest request =
    {
        .mode = ROBOT_EXECUTION_PRODUCTION,
        .trajectory = &trajectory,
        .trajectory_ready = true,
        .expected_program_id = trajectory.program_id,
        .expected_source_revision = trajectory.source_revision,
        .expected_artifact_crc = trajectory.artifact_crc
    };

    const PathExecutionConfig config =
    {
        .controlled_stop_timeout_ms = 1000U,
        .clearance_stable_cycles = 3U
    };

    const PathExecutionServices services =
    {
        .read_sample = read_sample,
        .set_wire_feed_enabled = wire_feed_enable,
        .prepare_retraction = prepare_retraction,
        .step_retraction = step_retraction,
        .clearance_verified = clearance,
        .controlled_stop = controlled_stop,
        .context = &context
    };

    PathExecutionState execution;

    state_path_execution_enter(
        &execution,
        &master,
        &request,
        &config,
        &services
    );

    PathExecutionInputs inputs;
    memset(
        &inputs,
        0,
        sizeof(inputs)
    );

    inputs.motion_permission = true;

    PathExecutionOutputs outputs;
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

        inputs.now_ms =
            now_ms;

        result =
            state_path_execution_step(
                &execution,
                &inputs,
                &outputs
            );
    }

    if (
        result != STATE_STEP_COMPLETE ||
        execution.result !=
            PATH_EXEC_RESULT_COMPLETE
    )
    {
        printf(
            "PATH EXECUTION failed: phase=%s error=%s\n",
            state_path_execution_phase_name(
                execution.phase
            ),
            state_path_execution_error_name(
                execution.error
            )
        );

        canopen_master_close(&master);
        return 1;
    }

    if (
        execution.sample_index !=
            TEST_SAMPLE_COUNT ||
        execution.command_period_ms != 2U ||
        !context.wire_feed_was_enabled ||
        context.wire_feed_enabled
    )
    {
        puts("PATH EXECUTION final state is invalid.");
        canopen_master_close(&master);
        return 1;
    }

    for (size_t axis = 0U;
         axis < TEST_NUM_AXES;
         ++axis)
    {
        if (
            !bus.nodes[axis].active_target_valid ||
            bus.nodes[axis].active_target_position !=
                context.samples[TEST_SAMPLE_COUNT - 1U]
                    .target_position_units[axis]
        )
        {
            printf(
                "Final RPDO4 target mismatch on axis %zu.\n",
                axis + 1U
            );

            canopen_master_close(&master);
            return 1;
        }
    }

    puts("PATH EXECUTION -> COMPLETE");

    printf(
        "Command period : %u ms (500 Hz)\n"
        "Samples sent   : %u\n"
        "Wire feed      : ON during production, OFF before retraction\n",
        (unsigned)execution.command_period_ms,
        (unsigned)execution.sample_index
    );

    puts(
        "Fresh TPDO4 feedback required after every RPDO4+SYNC sample.\n"
        "============================================================\n"
        " CANOPEN PATH EXECUTION TEST PASSED\n"
        "============================================================"
    );

    canopen_master_close(&master);
    return 0;
}
