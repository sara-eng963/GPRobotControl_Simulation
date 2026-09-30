/*
 * Test 7.6 - external-memory capacity characterization.
 *
 * This test answers one question only:
 *
 *     How much external flash do valid robot programs consume?
 *
 * Motion-feasibility sweeps belong in separate validation tests. For capacity
 * sizing, the dataset must be deterministic and valid. We therefore build one
 * already-proven compact mixed-geometry contour and create independent program
 * prefixes from it:
 *
 *     A: LINE
 *     B: LINE -> ARC
 *     C: LINE -> ARC -> CIRCLE
 *     D: LINE -> ARC -> CIRCLE -> LINE
 *     E: LINE -> ARC -> CIRCLE -> LINE -> ARC -> LINE
 *
 * Every program starts from the same known-good joint seed and runs through
 * the real ControlCore -> Path Validation pipeline before its samples are
 * counted. This keeps Test 7.6 focused on storage rather than searching for
 * arbitrary Cartesian geometries that happen to be IK-feasible.
 */
#define main test_trajectory_storage_capacity_original_main
#include "../../Simulation/Tests/test_trajectory_storage_capacity_realistic.c"
#undef main

typedef struct
{
    const char *name;
    float geometry_scale;
    float speed_mps;
    uint16_t segment_count;
} ScaledCapacityCase;

static void scale_program_geometry(
    TaughtProgram *program,
    float scale
)
{
    const float base_x = program->segments[0].points[0].position_m[0];
    const float base_y = program->segments[0].points[0].position_m[1];
    const float base_z = program->segments[0].points[0].position_m[2];

    for (uint16_t s = 0U; s < program->segment_count; ++s)
    {
        TaughtSegment *segment = &program->segments[s];

        for (uint8_t p = 0U; p < segment->point_count; ++p)
        {
            TaughtPoint *point = &segment->points[p];

            point->position_m[0] =
                base_x + scale * (point->position_m[0] - base_x);
            point->position_m[1] =
                base_y + scale * (point->position_m[1] - base_y);
            point->position_m[2] =
                base_z + scale * (point->position_m[2] - base_z);
        }
    }
}

static bool run_scaled_capacity_case(
    const ScaledCapacityCase *test_case,
    uint32_t program_id,
    const RobotConfig *robot,
    const PathValidationConfig *config,
    PathValidationWorkspace *workspace,
    uint64_t *allocated_bytes_out,
    uint64_t *raw_bytes_out,
    uint64_t *samples_out,
    double *duration_out
)
{
    TaughtProgram program;
    ValidatedTrajectory artifact;
    PathValidationState state;
    PathValidationOutputs outputs;
    CountingValidatedStorage counting_storage;

    if (
        test_case == NULL ||
        robot == NULL ||
        config == NULL ||
        workspace == NULL ||
        allocated_bytes_out == NULL ||
        raw_bytes_out == NULL ||
        samples_out == NULL ||
        duration_out == NULL ||
        !build_program(
            &program,
            robot,
            program_id,
            1U,
            test_case->speed_mps
        )
    )
    {
        return false;
    }

    scale_program_geometry(
        &program,
        test_case->geometry_scale
    );

    if (
        test_case->segment_count == 0U ||
        test_case->segment_count > program.segment_count
    )
    {
        return false;
    }

    /*
     * Create an independent program from a prefix of the known-valid contour.
     * The unused trailing segments are intentionally excluded from the draft.
     */
    program.segment_count =
        test_case->segment_count;

    program.draft_crc =
        state_path_validation_calculate_draft_crc(
            &program
        );

    if (program.draft_crc == 0U)
    {
        return false;
    }

    memset(&artifact, 0, sizeof(artifact));
    memset(&state, 0, sizeof(state));
    memset(&outputs, 0, sizeof(outputs));
    memset(&counting_storage, 0, sizeof(counting_storage));

    const PathValidationStorage storage =
    {
        .begin = counting_begin,
        .write_sample = counting_write_sample,
        .commit = counting_commit,
        .abort = counting_abort,
        .capacity_samples = TEST_STORAGE_CAPACITY,
        .context = &counting_storage
    };

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
            state_path_validation_step(&state, 512U, &outputs) ==
            STATE_STEP_FAILED
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
            "VALIDATION FAILURE in storage dataset: %s | error=%s segment=%u sample=%u joint=%u\n",
            test_case->name,
            state_path_validation_error_name(outputs.report.error),
            (unsigned)outputs.report.failed_segment,
            (unsigned)outputs.report.failed_sample,
            (unsigned)outputs.report.failed_joint
        );
        return false;
    }

    const uint64_t raw_sample_bytes =
        (uint64_t)artifact.sample_count * EXECUTION_SAMPLE_BYTES;

    const uint64_t allocated_sample_bytes =
        round_up_u64(raw_sample_bytes, FLASH_SECTOR_BYTES);

    const uint64_t allocated_total_bytes =
        FLASH_SECTOR_BYTES + allocated_sample_bytes;

    printf("\n============================================================\n");
    printf("TEST 7.6 VALID TRAJECTORY: %s\n", test_case->name);
    printf("============================================================\n");
    printf("geometry scale:         %.2f x reference contour\n", (double)test_case->geometry_scale);
    printf("segments:               %u\n", (unsigned)artifact.segment_count);
    printf("commanded TCP speed:    %.3f m/s\n", (double)test_case->speed_mps);
    printf("path length:            %.3f m\n", artifact.path_length_m);
    printf("duration:               %.3f s\n", artifact.duration_s);
    printf("samples:                %u\n", (unsigned)artifact.sample_count);
    printf("raw sample bytes:       %" PRIu64 " B (%.3f MiB)\n",
           raw_sample_bytes,
           (double)raw_sample_bytes / BYTES_PER_MIB);
    printf("allocated trajectory:   %" PRIu64 " B (%.3f MiB)\n",
           allocated_total_bytes,
           (double)allocated_total_bytes / BYTES_PER_MIB);

    printf("Segment breakdown:\n");
    for (uint16_t segment = 0U; segment < artifact.segment_count; ++segment)
    {
        const PvSegmentIndex *index = &artifact.segments[segment];
        const uint64_t segment_bytes =
            (uint64_t)index->sample_count * EXECUTION_SAMPLE_BYTES;

        printf(
            "  %2u %-6s samples=%7u bytes=%9" PRIu64 " approx=%.3f s\n",
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
    *duration_out = artifact.duration_s;

    return true;
}

int main(void)
{
    _Static_assert(
        sizeof(PvExecutionSample) == 24U,
        "Test 7.6 assumes PvExecutionSample remains 24 bytes."
    );

    /*
     * The 0.85-scale contour just passed all six segments in the previous
     * Test 7.6 run. Prefixes of that same sequential trajectory therefore give
     * us deterministic LINE / ARC / CIRCLE / mixed programs without changing
     * IK geometry between capacity cases.
     */
    const ScaledCapacityCase cases[] =
    {
        {"Program A - LINE",                              0.85F, 0.010F, 1U},
        {"Program B - LINE -> ARC",                      0.85F, 0.010F, 2U},
        {"Program C - LINE -> ARC -> CIRCLE",            0.85F, 0.010F, 3U},
        {"Program D - LINE -> ARC -> CIRCLE -> LINE",    0.85F, 0.010F, 4U},
        {"Program E - full mixed-geometry contour",      0.85F, 0.010F, 6U}
    };

    RobotConfig robot;
    PathValidationConfig config;
    PathValidationWorkspace workspace;

    configure_validation(&robot, &config, &workspace);

    uint64_t total_allocated_bytes = 0U;
    uint64_t total_raw_bytes = 0U;
    uint64_t total_samples = 0U;
    double total_duration_s = 0.0;

    printf("============================================================\n");
    printf("TEST 7.6A - VALID MULTI-PROGRAM STORAGE CAPACITY\n");
    printf("============================================================\n");
    printf("Question being tested:\n");
    printf("How much external flash do several VALID robot programs consume?\n\n");
    printf("Dataset uses progressive prefixes of one proven mixed-geometry contour.\n");
    printf("Programs are independent; each starts from a fresh known-good seed.\n");
    printf("TCP speed:             0.010 m/s\n");
    printf("Sample period:         %lu us\n", (unsigned long)PATH_VALIDATION_SAMPLE_PERIOD_US);
    printf("PvExecutionSample:     %zu bytes\n", sizeof(PvExecutionSample));
    printf("Storage rate @ 1 ms:   %.0f B/s = %.3f MiB/min raw\n",
           (double)sizeof(PvExecutionSample) * 1000.0,
           ((double)sizeof(PvExecutionSample) * 1000.0 * 60.0) / BYTES_PER_MIB);
    printf("Allocation model:      4 KiB header + sector-rounded sample data\n");

    for (size_t i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        uint64_t allocated_bytes = 0U;
        uint64_t raw_bytes = 0U;
        uint64_t samples = 0U;
        double duration_s = 0.0;

        if (
            !run_scaled_capacity_case(
                &cases[i],
                (uint32_t)(760U + i),
                &robot,
                &config,
                &workspace,
                &allocated_bytes,
                &raw_bytes,
                &samples,
                &duration_s
            )
        )
        {
            printf("\nTEST 7.6A: FAIL - storage dataset contains an invalid trajectory.\n");
            printf("This is a trajectory-generation/validation issue, not a flash-size result.\n");
            return 1;
        }

        total_allocated_bytes += allocated_bytes;
        total_raw_bytes += raw_bytes;
        total_samples += samples;
        total_duration_s += duration_s;
    }

    const uint64_t reserve_bytes = total_allocated_bytes / 4U;
    const uint64_t planning_bytes = total_allocated_bytes + reserve_bytes;

    printf("\n============================================================\n");
    printf("TEST 7.6A COMBINED STORAGE REPORT\n");
    printf("============================================================\n");
    printf("valid programs:         %zu / %zu\n",
           sizeof(cases) / sizeof(cases[0]),
           sizeof(cases) / sizeof(cases[0]));
    printf("combined motion time:   %.3f s (%.3f min)\n",
           total_duration_s,
           total_duration_s / 60.0);
    printf("total samples:          %" PRIu64 "\n", total_samples);
    printf("total raw bytes:        %" PRIu64 " (%.3f MiB)\n",
           total_raw_bytes,
           (double)total_raw_bytes / BYTES_PER_MIB);
    printf("actual allocation:      %" PRIu64 " (%.3f MiB)\n",
           total_allocated_bytes,
           (double)total_allocated_bytes / BYTES_PER_MIB);
    printf("planning reserve:       +25%% = %" PRIu64 " B\n", reserve_bytes);
    printf("capacity to plan for:   %" PRIu64 " B (%.3f MiB)\n",
           planning_bytes,
           (double)planning_bytes / BYTES_PER_MIB);

    printf("\nCandidate flash sizes:\n");
    print_flash_capacity_line("8 MiB",  MIB_U64(8),  planning_bytes);
    print_flash_capacity_line("16 MiB", MIB_U64(16), planning_bytes);
    print_flash_capacity_line("32 MiB", MIB_U64(32), planning_bytes);
    print_flash_capacity_line("64 MiB", MIB_U64(64), planning_bytes);

    printf("\nTEST 7.6A: PASS - five independent valid programs characterized.\n");
    return 0;
}
