#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "state_path_validation.h"

#include "control_fk.h"
#include "math3d.h"
#include "robot_config.h"
#include "avatar_m_position.h"

#define TEST_GEOMETRY_CAPACITY 512U
#define TEST_STORAGE_CAPACITY  10000U
#define TEST_REDUCTION_RATIO   50.0

typedef struct
{
    PvExecutionSample samples[TEST_STORAGE_CAPACITY];
    uint32_t sample_count;
    bool writing;
    bool committed;
    ValidatedTrajectory metadata;
} TestStorage;

static bool storage_begin(void *context)
{
    TestStorage *storage = (TestStorage *)context;

    if (storage == NULL)
    {
        return false;
    }

    memset(storage, 0, sizeof(*storage));
    storage->writing = true;
    return true;
}

static bool storage_write(
    uint32_t sample_index,
    const PvExecutionSample *sample,
    void *context
)
{
    TestStorage *storage = (TestStorage *)context;

    if (
        storage == NULL ||
        sample == NULL ||
        !storage->writing ||
        sample_index >= TEST_STORAGE_CAPACITY
    )
    {
        return false;
    }

    storage->samples[sample_index] = *sample;

    if (sample_index + 1U > storage->sample_count)
    {
        storage->sample_count = sample_index + 1U;
    }

    return true;
}

static bool storage_commit(
    const ValidatedTrajectory *metadata,
    void *context
)
{
    TestStorage *storage = (TestStorage *)context;

    if (
        storage == NULL ||
        metadata == NULL ||
        !storage->writing ||
        metadata->sample_count > storage->sample_count
    )
    {
        return false;
    }

    storage->metadata = *metadata;
    storage->writing = false;
    storage->committed = true;
    return true;
}

static void storage_abort(void *context)
{
    TestStorage *storage = (TestStorage *)context;

    if (storage != NULL)
    {
        storage->writing = false;
        storage->committed = false;
    }
}

static bool fill_taught_point(
    const RobotConfig *robot,
    const double q[ROBOT_DOF],
    uint32_t timestamp_ms,
    TaughtPoint *point
)
{
    double T[4][4];

    if (robot == NULL || q == NULL || point == NULL)
    {
        return false;
    }

    control_fk(robot, q, T);

    Mat3 rotation;

    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            rotation.m[row][column] = T[row][column];
        }
    }

    const Quat quat = rotm_to_quat(rotation);

    memset(point, 0, sizeof(*point));

    point->position_m[0] = (float)T[0][3];
    point->position_m[1] = (float)T[1][3];
    point->position_m[2] = (float)T[2][3];

    point->orientation_quat[0] = (float)quat.w;
    point->orientation_quat[1] = (float)quat.x;
    point->orientation_quat[2] = (float)quat.y;
    point->orientation_quat[3] = (float)quat.z;

    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        point->joint_position_rad[i] = (float)q[i];
    }

    point->record_timestamp_ms = timestamp_ms;
    point->calibration_version = 1U;
    point->frame_id = 1U;
    point->tool_id = 1U;
    point->point_valid = true;

    return true;
}

int main(void)
{
    puts(
        "\n"
        "============================================================\n"
        " AVATAR PATH VALIDATION TEST\n"
        "============================================================"
    );

    RobotConfig robot;
    robot_config_init_ur5(&robot);

    AvatarMPositionScale scales[ROBOT_DOF];

    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        if (!avatar_m_position_scale_default(
                &scales[i],
                TEST_REDUCTION_RATIO))
        {
            puts("Could not configure AVATAR position scales.");
            return 1;
        }
    }

    const double q_start[ROBOT_DOF] =
    {
        0.20, -1.00, 1.20, -1.00, 1.00, 0.50
    };

    const double q_end[ROBOT_DOF] =
    {
        0.25, -1.00, 1.20, -1.00, 1.00, 0.50
    };

    TaughtProgram program;
    memset(&program, 0, sizeof(program));

    program.program_id = 1U;
    program.draft_revision = 1U;
    program.segment_count = 1U;
    program.global_speed_scale = 1.0F;
    program.status = TEACH_DRAFT_SUBMITTED;

    TaughtSegment *segment = &program.segments[0];
    segment->segment_id = 1U;
    segment->type = TEACH_SEGMENT_LINE;
    segment->point_count = 2U;
    segment->speed_mps = 0.010F;
    segment->orientation_mode = TEACH_ORIENTATION_INTERPOLATED;
    segment->segment_valid = true;

    if (
        !fill_taught_point(
            &robot,
            q_start,
            1000U,
            &segment->points[0]
        ) ||
        !fill_taught_point(
            &robot,
            q_end,
            2000U,
            &segment->points[1]
        )
    )
    {
        puts("Could not build taught points.");
        return 1;
    }

    program.draft_crc =
        state_path_validation_calculate_draft_crc(
            &program
        );

    PathValidationConfig config;
    memset(&config, 0, sizeof(config));

    config.default_tcp_speed_mps = 0.010;
    config.max_tcp_speed_mps = 0.100;
    config.max_tcp_acceleration_mps2 = 0.250;
    config.max_tcp_jerk_mps3 = 1.000;
    config.minimum_segment_length_m = 0.002;
    config.maximum_fk_position_error_m = 0.002;
    config.maximum_fk_orientation_error_rad = 0.02;
    config.minimum_singularity_sigma = 0.0;
    config.maximum_joint_step_rad = 0.20;
    config.maximum_position_quantization_error_rad = 0.001;
    config.geometry_points_per_segment = 200U;
    config.arc_length_spacing_m = 0.005;
    config.check_joint_acceleration = false;
    config.require_collision_callback = false;

    adls_default_parameters(&config.ik_parameters);

    static Vec3 raw_geometry[TEST_GEOMETRY_CAPACITY];
    static Vec3 arc_geometry[TEST_GEOMETRY_CAPACITY];
    static real_t l_original[TEST_GEOMETRY_CAPACITY];
    static real_t l_arc[TEST_GEOMETRY_CAPACITY];
    static ADLSInfo ik_scratch;

    PathValidationWorkspace workspace =
    {
        .geometry_capacity = TEST_GEOMETRY_CAPACITY,
        .raw_geometry = raw_geometry,
        .arc_geometry = arc_geometry,
        .l_original = l_original,
        .l_arc = l_arc,
        .ik_scratch = &ik_scratch
    };

    TestStorage test_storage;
    memset(&test_storage, 0, sizeof(test_storage));

    PathValidationStorage storage =
    {
        .begin = storage_begin,
        .write_sample = storage_write,
        .commit = storage_commit,
        .abort = storage_abort,
        .capacity_samples = TEST_STORAGE_CAPACITY,
        .context = &test_storage
    };

    PathValidationServices services;
    memset(&services, 0, sizeof(services));

    ValidatedTrajectory artifact;
    memset(&artifact, 0, sizeof(artifact));

    PathValidationState state;
    state_path_validation_enter(
        &state,
        &robot,
        scales,
        &config,
        &services,
        &workspace,
        &storage,
        &artifact,
        &program,
        program.draft_revision,
        program.draft_crc
    );

    if (
        !state.initialized ||
        state.result != PV_RESULT_RUNNING
    )
    {
        printf(
            "PATH VALIDATION failed to initialize: %s\n",
            state_path_validation_error_name(state.error)
        );
        return 1;
    }

    PathValidationOutputs outputs;
    StateStepResult step = STATE_STEP_RUNNING;

    for (
        uint32_t iteration = 0U;
        iteration < 10000U &&
        step == STATE_STEP_RUNNING;
        ++iteration
    )
    {
        step =
            state_path_validation_step(
                &state,
                64U,
                &outputs
            );
    }

    if (
        step != STATE_STEP_COMPLETE ||
        outputs.report.result != PV_RESULT_VALID ||
        !outputs.trajectory_ready ||
        !test_storage.committed
    )
    {
        printf(
            "PATH VALIDATION failed: phase=%s error=%s\n",
            state_path_validation_phase_name(outputs.report.phase),
            state_path_validation_error_name(outputs.report.error)
        );
        return 1;
    }

    if (
        artifact.sample_period_us !=
            PATH_VALIDATION_SAMPLE_PERIOD_US ||
        artifact.sample_period_us != 2000U ||
        artifact.sample_count == 0U ||
        test_storage.sample_count != artifact.sample_count
    )
    {
        puts("Validated artifact timing/sample metadata is invalid.");
        return 1;
    }

    for (
        uint32_t sample_index = 0U;
        sample_index < artifact.sample_count;
        ++sample_index
    )
    {
        for (size_t axis = 0U; axis < ROBOT_DOF; ++axis)
        {
            double q = 0.0;

            if (!avatar_m_position_units_to_joint_rad(
                    &scales[axis],
                    test_storage.samples[sample_index]
                        .target_position_units[axis],
                    &q))
            {
                printf(
                    "AVATAR sample conversion failed at sample %u axis %zu.\n",
                    (unsigned)sample_index,
                    axis + 1U
                );
                return 1;
            }

            if (
                !isfinite(q) ||
                q < robot.limits.qMin[axis] ||
                q > robot.limits.qMax[axis]
            )
            {
                printf(
                    "Stored AVATAR target is invalid at sample %u axis %zu.\n",
                    (unsigned)sample_index,
                    axis + 1U
                );
                return 1;
            }
        }
    }

    puts(
        "PATH VALIDATION -> VALID\n"
        "AVATAR raw target conversion verified on all stored samples."
    );

    printf(
        "Sample period : %u us (500 Hz)\n"
        "Samples       : %u\n"
        "Duration      : %.3f s\n"
        "Artifact CRC  : 0x%08lX\n",
        (unsigned)artifact.sample_period_us,
        (unsigned)artifact.sample_count,
        (double)artifact.duration_s,
        (unsigned long)artifact.artifact_crc
    );

    puts(
        "============================================================\n"
        " AVATAR PATH VALIDATION TEST PASSED\n"
        "============================================================"
    );

    return 0;
}
