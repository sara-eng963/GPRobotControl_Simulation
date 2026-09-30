#include "../../StateMachine/States/state_path_validation.h"
#include "../../ControlCore/Config/robot_config.h"
#include "../../ControlCore/Pipeline/single_segment_circular.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_GEOMETRY_CAPACITY       1024U
#define TEST_STORAGE_CAPACITY        2000000U
#define FLASH_SECTOR_BYTES           UINT64_C(4096)
#define EXECUTION_SAMPLE_BYTES       ((uint64_t)sizeof(PvExecutionSample))

#define MIB(x_) ((uint64_t)(x_) * UINT64_C(1024) * UINT64_C(1024))

typedef struct
{
    uint32_t sample_count;
    bool writing;
    bool committed;
    ValidatedTrajectory metadata;

} CountingValidatedStorage;


typedef struct
{
    const char *name;
    uint8_t contour_repeats;
    float speed_mps;

} CapacityCase;


static Vec3 raw_geometry[TEST_GEOMETRY_CAPACITY];
static Vec3 arc_geometry[TEST_GEOMETRY_CAPACITY];
static real_t l_original[TEST_GEOMETRY_CAPACITY];
static real_t l_arc[TEST_GEOMETRY_CAPACITY];
static ADLSInfo ik_scratch;


static uint64_t round_up_u64(
    uint64_t value,
    uint64_t alignment
)
{
    return
        ((value + alignment - 1ULL) / alignment) * alignment;
}


static bool counting_begin(void *context)
{
    CountingValidatedStorage *storage =
        (CountingValidatedStorage *)context;

    if (storage == NULL)
    {
        return false;
    }

    memset(storage, 0, sizeof(*storage));
    storage->writing = true;

    return true;
}


static bool counting_write_sample(
    uint32_t sample_index,
    const PvExecutionSample *sample,
    void *context
)
{
    CountingValidatedStorage *storage =
        (CountingValidatedStorage *)context;

    if (
        storage == NULL ||
        sample == NULL ||
        !storage->writing ||
        sample_index != storage->sample_count
    )
    {
        return false;
    }

    storage->sample_count++;
    return true;
}


static bool counting_commit(
    const ValidatedTrajectory *metadata,
    void *context
)
{
    CountingValidatedStorage *storage =
        (CountingValidatedStorage *)context;

    if (
        storage == NULL ||
        metadata == NULL ||
        !storage->writing ||
        metadata->sample_count != storage->sample_count
    )
    {
        return false;
    }

    storage->metadata = *metadata;
    storage->writing = false;
    storage->committed = true;

    return true;
}


static void counting_abort(void *context)
{
    CountingValidatedStorage *storage =
        (CountingValidatedStorage *)context;

    if (storage == NULL)
    {
        return;
    }

    storage->writing = false;
    storage->committed = false;
    storage->sample_count = 0U;
}


static Mat4 array_to_mat4(const double input[4][4])
{
    Mat4 result;

    for (uint8_t row = 0U; row < 4U; ++row)
    {
        for (uint8_t column = 0U; column < 4U; ++column)
        {
            result.m[row][column] = input[row][column];
        }
    }

    return result;
}


static void fill_taught_point(
    TaughtPoint *point,
    Vec3 position,
    Quat orientation,
    const JointVector *joints,
    uint32_t timestamp_ms
)
{
    memset(point, 0, sizeof(*point));

    point->position_m[0] = (float)position.v[0];
    point->position_m[1] = (float)position.v[1];
    point->position_m[2] = (float)position.v[2];

    point->orientation_quat[0] = (float)orientation.w;
    point->orientation_quat[1] = (float)orientation.x;
    point->orientation_quat[2] = (float)orientation.y;
    point->orientation_quat[3] = (float)orientation.z;

    for (uint8_t joint = 0U; joint < PATH_VALIDATION_DOF; ++joint)
    {
        point->joint_position_rad[joint] =
            (float)joints->q[joint];
    }

    point->record_timestamp_ms = timestamp_ms;
    point->calibration_version = 1U;
    point->frame_id = 0U;
    point->tool_id = 0U;
    point->point_valid = true;
}


static void set_segment_common(
    TaughtSegment *segment,
    uint16_t segment_id,
    TeachingSegmentType type,
    uint8_t point_count,
    float speed_mps
)
{
    memset(segment, 0, sizeof(*segment));

    segment->segment_id = segment_id;
    segment->type = type;
    segment->point_count = point_count;
    segment->speed_mps = speed_mps;
    segment->circle_direction = TEACH_CIRCLE_CCW;
    segment->orientation_mode = TEACH_ORIENTATION_CONSTANT;
    segment->segment_valid = true;
}


static bool append_mixed_contour(
    TaughtProgram *program,
    const RobotConfig *robot,
    uint16_t *next_segment_id,
    uint32_t *timestamp_ms,
    float speed_mps,
    real_t x_offset
)
{
    if (
        program == NULL ||
        robot == NULL ||
        next_segment_id == NULL ||
        timestamp_ms == NULL ||
        program->segment_count + 6U > TEACHING_MAX_SEGMENTS
    )
    {
        return false;
    }

    const real_t q_start_deg[ROBOT_DOF] =
    {
         30.0,
        -45.0,
         60.0,
         20.0,
        -30.0,
         45.0
    };

    JointVector q_seed;

    for (uint8_t joint = 0U; joint < ROBOT_DOF; ++joint)
    {
        q_seed.q[joint] =
            q_start_deg[joint] * ROBOT_PI / 180.0;
    }

    double start_array[4][4];
    control_fk(robot, q_seed.q, start_array);

    const Mat4 start_pose = array_to_mat4(start_array);
    const Vec3 base = mat4_translation(start_pose);
    const Quat orientation = rotm_to_quat(mat4_rotation(start_pose));

    const Vec3 p0 = vec3_add(base, (Vec3){{x_offset, 0.00, 0.00}});
    const Vec3 p1 = vec3_add(base, (Vec3){{x_offset + 0.10, 0.00, 0.00}});
    const Vec3 p2 = vec3_add(base, (Vec3){{x_offset + 0.20, 0.10, 0.00}});
    const Vec3 p3 = vec3_add(base, (Vec3){{x_offset + 0.30, 0.00, 0.00}});
    const Vec3 p4 = vec3_add(base, (Vec3){{x_offset + 0.45, 0.15, 0.00}});
    const Vec3 p5 = vec3_add(base, (Vec3){{x_offset + 0.60, 0.00, 0.00}});
    const Vec3 p6 = vec3_add(base, (Vec3){{x_offset + 0.70, 0.00, 0.00}});
    const Vec3 p7 = vec3_add(base, (Vec3){{x_offset + 0.80, -0.10, 0.00}});
    const Vec3 p8 = vec3_add(base, (Vec3){{x_offset + 0.90, 0.00, 0.00}});
    const Vec3 p9 = vec3_add(base, (Vec3){{x_offset + 1.00, 0.00, 0.00}});

    TaughtSegment *segment =
        &program->segments[program->segment_count++];

    set_segment_common(
        segment,
        (*next_segment_id)++,
        TEACH_SEGMENT_LINE,
        2U,
        speed_mps
    );

    fill_taught_point(&segment->points[0], p0, orientation, &q_seed, (*timestamp_ms)++);
    fill_taught_point(&segment->points[1], p1, orientation, &q_seed, (*timestamp_ms)++);

    segment = &program->segments[program->segment_count++];

    set_segment_common(
        segment,
        (*next_segment_id)++,
        TEACH_SEGMENT_ARC,
        3U,
        speed_mps
    );

    fill_taught_point(&segment->points[0], p1, orientation, &q_seed, (*timestamp_ms)++);
    fill_taught_point(&segment->points[1], p2, orientation, &q_seed, (*timestamp_ms)++);
    fill_taught_point(&segment->points[2], p3, orientation, &q_seed, (*timestamp_ms)++);

    segment = &program->segments[program->segment_count++];

    set_segment_common(
        segment,
        (*next_segment_id)++,
        TEACH_SEGMENT_CIRCLE,
        3U,
        speed_mps
    );

    fill_taught_point(&segment->points[0], p3, orientation, &q_seed, (*timestamp_ms)++);
    fill_taught_point(&segment->points[1], p4, orientation, &q_seed, (*timestamp_ms)++);
    fill_taught_point(&segment->points[2], p5, orientation, &q_seed, (*timestamp_ms)++);

    segment = &program->segments[program->segment_count++];

    set_segment_common(
        segment,
        (*next_segment_id)++,
        TEACH_SEGMENT_LINE,
        2U,
        speed_mps
    );

    fill_taught_point(&segment->points[0], p3, orientation, &q_seed, (*timestamp_ms)++);
    fill_taught_point(&segment->points[1], p6, orientation, &q_seed, (*timestamp_ms)++);

    segment = &program->segments[program->segment_count++];

    set_segment_common(
        segment,
        (*next_segment_id)++,
        TEACH_SEGMENT_ARC,
        3U,
        speed_mps
    );

    fill_taught_point(&segment->points[0], p6, orientation, &q_seed, (*timestamp_ms)++);
    fill_taught_point(&segment->points[1], p7, orientation, &q_seed, (*timestamp_ms)++);
    fill_taught_point(&segment->points[2], p8, orientation, &q_seed, (*timestamp_ms)++);

    segment = &program->segments[program->segment_count++];

    set_segment_common(
        segment,
        (*next_segment_id)++,
        TEACH_SEGMENT_LINE,
        2U,
        speed_mps
    );

    fill_taught_point(&segment->points[0], p8, orientation, &q_seed, (*timestamp_ms)++);
    fill_taught_point(&segment->points[1], p9, orientation, &q_seed, (*timestamp_ms)++);

    return true;
}


static void configure_validation(
    RobotConfig *robot,
    PathValidationConfig *config,
    PathValidationWorkspace *workspace
)
{
    robot_config_init_ur5(robot);

    memset(config, 0, sizeof(*config));

    config->default_tcp_speed_mps = 0.010;
    config->max_tcp_speed_mps = 0.100;
    config->max_tcp_acceleration_mps2 = 0.250;
    config->max_tcp_jerk_mps3 = 1.000;
    config->minimum_segment_length_m = 0.002;
    config->maximum_fk_position_error_m = 0.001;
    config->maximum_fk_orientation_error_rad = 0.01;
    config->minimum_singularity_sigma = 0.0;
    config->maximum_joint_step_rad = 0.20;
    config->maximum_position_quantization_error_rad = 0.001;
    config->geometry_points_per_segment = 400U;
    config->arc_length_spacing_m = 0.005;
    config->check_joint_acceleration = false;
    config->require_collision_callback = false;

    adls_default_parameters(&config->ik_parameters);

    *workspace = (PathValidationWorkspace)
    {
        .geometry_capacity = TEST_GEOMETRY_CAPACITY,
        .raw_geometry = raw_geometry,
        .arc_geometry = arc_geometry,
        .l_original = l_original,
        .l_arc = l_arc,
        .ik_scratch = &ik_scratch
    };
}


static bool build_program(
    TaughtProgram *program,
    const RobotConfig *robot,
    uint32_t program_id,
    uint8_t contour_repeats,
    float speed_mps
)
{
    if (
        program == NULL ||
        robot == NULL ||
        contour_repeats == 0U
    )
    {
        return false;
    }

    memset(program, 0, sizeof(*program));

    program->program_id = program_id;
    program->draft_revision = 1U;
    program->global_speed_scale = 1.0F;
    program->status = TEACH_DRAFT_SUBMITTED;

    uint16_t segment_id = 1U;
    uint32_t timestamp_ms = 1000U;

    for (uint8_t contour = 0U; contour < contour_repeats; ++contour)
    {
        if (
            !append_mixed_contour(
                program,
                robot,
                &segment_id,
                &timestamp_ms,
                speed_mps,
                (real_t)contour * 0.05
            )
        )
        {
            return false;
        }
    }

    program->draft_crc =
        state_path_validation_calculate_draft_crc(program);

    return program->draft_crc != 0U;
}


static const char *segment_name(uint8_t segment_type)
{
    switch ((TeachingSegmentType)segment_type)
    {
        case TEACH_SEGMENT_LINE:
            return "LINE";

        case TEACH_SEGMENT_ARC:
            return "ARC";

        case TEACH_SEGMENT_CIRCLE:
            return "CIRCLE";

        default:
            return "UNKNOWN";
    }
}


static bool run_capacity_case(
    const CapacityCase *test_case,
    uint32_t program_id,
    const RobotConfig *robot,
    const PathValidationConfig *config,
    PathValidationWorkspace *workspace,
    uint64_t *allocated_bytes_out,
    uint64_t *raw_bytes_out,
    uint64_t *samples_out
)
{
    if (
        test_case == NULL ||
        robot == NULL ||
        config == NULL ||
        workspace == NULL ||
        allocated_bytes_out == NULL ||
        raw_bytes_out == NULL ||
        samples_out == NULL
    )
    {
        return false;
    }

    TaughtProgram program;
    ValidatedTrajectory artifact;
    PathValidationState state;
    PathValidationOutputs outputs;
    CountingValidatedStorage counting_storage;

    if (
        !build_program(
            &program,
            robot,
            program_id,
            test_case->contour_repeats,
            test_case->speed_mps
        )
    )
    {
        fprintf(stderr, "Failed to build test program: %s\n", test_case->name);
        return false;
    }

    const PathValidationStorage storage =
    {
        .begin = counting_begin,
        .write_sample = counting_write_sample,
        .commit = counting_commit,
        .abort = counting_abort,
        .capacity_samples = TEST_STORAGE_CAPACITY,
        .context = &counting_storage
    };

    memset(&artifact, 0, sizeof(artifact));
    memset(&state, 0, sizeof(state));
    memset(&outputs, 0, sizeof(outputs));
    memset(&counting_storage, 0, sizeof(counting_storage));

    state_path_validation_enter(
        &state,
        robot,
        config,
        NULL,
        workspace,
        &storage,
        &artifact,
        &program,
        program.draft_revision,
        program.draft_crc
    );

    while (state.result == PV_RESULT_RUNNING)
    {
        if (
            state_path_validation_step(
                &state,
                512U,
                &outputs
            ) == STATE_STEP_FAILED
        )
        {
            fprintf(stderr, "State step failed: %s\n", test_case->name);
            return false;
        }
    }

    if (
        outputs.report.result != PV_RESULT_VALID ||
        !outputs.trajectory_ready ||
        !counting_storage.committed
    )
    {
        fprintf(
            stderr,
            "Validation failed for %s: result=%d error=%s segment=%u sample=%u joint=%u\n",
            test_case->name,
            (int)outputs.report.result,
            state_path_validation_error_name(outputs.report.error),
            (unsigned)outputs.report.failed_segment,
            (unsigned)outputs.report.failed_sample,
            (unsigned)outputs.report.failed_joint
        );
        return false;
    }

    const uint64_t raw_sample_bytes =
        (uint64_t)artifact.sample_count *
        EXECUTION_SAMPLE_BYTES;

    const uint64_t allocated_sample_bytes =
        round_up_u64(
            raw_sample_bytes,
            FLASH_SECTOR_BYTES
        );

    const uint64_t allocated_total_bytes =
        FLASH_SECTOR_BYTES +
        allocated_sample_bytes;

    printf("\n============================================================\n");
    printf("TEST 7.6 TRAJECTORY: %s\n", test_case->name);
    printf("============================================================\n");
    printf("contours:              %u\n", (unsigned)test_case->contour_repeats);
    printf("segments:              %u\n", (unsigned)artifact.segment_count);
    printf("commanded TCP speed:   %.3f m/s\n", (double)test_case->speed_mps);
    printf("path length:           %.3f m\n", artifact.path_length_m);
    printf("duration:              %.3f s\n", artifact.duration_s);
    printf("samples:               %u\n", (unsigned)artifact.sample_count);
    printf("sample size:           %" PRIu64 " B\n", EXECUTION_SAMPLE_BYTES);
    printf("raw sample bytes:      %" PRIu64 " B (%.3f MiB)\n",
           raw_sample_bytes,
           (double)raw_sample_bytes / (1024.0 * 1024.0));
    printf("header reservation:    %" PRIu64 " B\n", FLASH_SECTOR_BYTES);
    printf("sector-rounded data:   %" PRIu64 " B\n", allocated_sample_bytes);
    printf("allocated trajectory:  %" PRIu64 " B (%.3f MiB)\n",
           allocated_total_bytes,
           (double)allocated_total_bytes / (1024.0 * 1024.0));

    printf("\nSegment breakdown:\n");

    for (uint16_t segment = 0U; segment < artifact.segment_count; ++segment)
    {
        const PvSegmentIndex *index =
            &artifact.segments[segment];

        const uint64_t segment_bytes =
            (uint64_t)index->sample_count *
            EXECUTION_SAMPLE_BYTES;

        printf(
            "  %2u %-6s samples=%7u  bytes=%9" PRIu64 "  approx=%.3f s\n",
            (unsigned)(segment + 1U),
            segment_name(index->segment_type),
            (unsigned)index->sample_count,
            segment_bytes,
            (double)index->sample_count * PATH_VALIDATION_SAMPLE_PERIOD_S
        );
    }

    *allocated_bytes_out = allocated_total_bytes;
    *raw_bytes_out = raw_sample_bytes;
    *samples_out = artifact.sample_count;

    return true;
}


static void print_flash_capacity_line(
    const char *label,
    uint64_t device_bytes,
    uint64_t required_bytes
)
{
    const double utilization =
        required_bytes > 0U
        ? (100.0 * (double)required_bytes / (double)device_bytes)
        : 0.0;

    const uint64_t remaining =
        required_bytes <= device_bytes
        ? device_bytes - required_bytes
        : 0U;

    printf(
        "%-8s capacity=%8.2f MiB  used=%6.2f%%  remaining=%8.3f MiB  %s\n",
        label,
        (double)device_bytes / (1024.0 * 1024.0),
        utilization,
        (double)remaining / (1024.0 * 1024.0),
        required_bytes <= device_bytes ? "FIT" : "DOES NOT FIT"
    );
}


int main(void)
{
    _Static_assert(
        sizeof(PvExecutionSample) == 24U,
        "Test 7.6 assumes the validated execution sample remains 24 bytes."
    );

    const CapacityCase cases[] =
    {
        {
            .name = "Mixed contour - welding speed",
            .contour_repeats = 1U,
            .speed_mps = 0.100F
        },
        {
            .name = "Mixed contour - medium speed",
            .contour_repeats = 1U,
            .speed_mps = 0.050F
        },
        {
            .name = "Mixed contour - current simulator default",
            .contour_repeats = 1U,
            .speed_mps = 0.010F
        },
        {
            .name = "Double mixed contour - welding speed",
            .contour_repeats = 2U,
            .speed_mps = 0.100F
        },
        {
            .name = "Double mixed contour - medium speed",
            .contour_repeats = 2U,
            .speed_mps = 0.050F
        }
    };

    RobotConfig robot;
    PathValidationConfig config;
    PathValidationWorkspace workspace;

    configure_validation(
        &robot,
        &config,
        &workspace
    );

    uint64_t total_allocated_bytes = 0U;
    uint64_t total_raw_bytes = 0U;
    uint64_t total_samples = 0U;

    printf("============================================================\n");
    printf("TEST 7.6 - MULTI-GEOMETRY TRAJECTORY STORAGE CAPACITY\n");
    printf("============================================================\n");
    printf("Each contour contains: LINE -> ARC -> CIRCLE -> LINE -> ARC -> LINE\n");
    printf("Sample period:         %lu us\n", (unsigned long)PATH_VALIDATION_SAMPLE_PERIOD_US);
    printf("PvExecutionSample:     %zu bytes\n", sizeof(PvExecutionSample));
    printf("Flash erase sector:    %" PRIu64 " bytes\n", FLASH_SECTOR_BYTES);
    printf("Allocation model:      4 KiB header + sector-rounded sample data\n");

    for (size_t i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        uint64_t allocated_bytes = 0U;
        uint64_t raw_bytes = 0U;
        uint64_t samples = 0U;

        if (
            !run_capacity_case(
                &cases[i],
                (uint32_t)(760U + i),
                &robot,
                &config,
                &workspace,
                &allocated_bytes,
                &raw_bytes,
                &samples
            )
        )
        {
            printf("\nTEST 7.6: FAIL\n");
            return 1;
        }

        total_allocated_bytes += allocated_bytes;
        total_raw_bytes += raw_bytes;
        total_samples += samples;
    }

    const uint64_t reserve_bytes =
        total_allocated_bytes / 4U;

    const uint64_t planning_bytes =
        total_allocated_bytes + reserve_bytes;

    printf("\n============================================================\n");
    printf("TEST 7.6 COMBINED STORAGE REPORT\n");
    printf("============================================================\n");
    printf("trajectories tested:   %zu\n", sizeof(cases) / sizeof(cases[0]));
    printf("total samples:         %" PRIu64 "\n", total_samples);
    printf("total raw bytes:       %" PRIu64 " (%.3f MiB)\n",
           total_raw_bytes,
           (double)total_raw_bytes / (1024.0 * 1024.0));
    printf("actual allocation:     %" PRIu64 " (%.3f MiB)\n",
           total_allocated_bytes,
           (double)total_allocated_bytes / (1024.0 * 1024.0));
    printf("planning reserve:      +25%% = %" PRIu64 " B\n", reserve_bytes);
    printf("capacity to plan for:  %" PRIu64 " B (%.3f MiB)\n",
           planning_bytes,
           (double)planning_bytes / (1024.0 * 1024.0));

    printf("\nCandidate flash sizes:\n");
    print_flash_capacity_line("8 MiB", MIB(8), planning_bytes);
    print_flash_capacity_line("16 MiB", MIB(16), planning_bytes);
    print_flash_capacity_line("32 MiB", MIB(32), planning_bytes);
    print_flash_capacity_line("64 MiB", MIB(64), planning_bytes);

    printf("\nTEST 7.6: PASS\n");
    return 0;
}
