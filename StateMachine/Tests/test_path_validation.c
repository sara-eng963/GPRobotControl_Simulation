/*
 * Path Validation integration/capacity characterization entry point.
 *
 * Test 7.6 intentionally keeps Path Validation active. A commanded case may
 * therefore be rejected for a real motion constraint (for example joint
 * velocity) without aborting the entire storage-characterization run.
 */
#define main test_trajectory_storage_capacity_original_main
#include "../../Simulation/Tests/test_trajectory_storage_capacity_realistic.c"
#undef main

int main(void)
{
    _Static_assert(
        sizeof(PvExecutionSample) == 24U,
        "Test 7.6 assumes PvExecutionSample remains 24 bytes."
    );

    const CapacityCase cases[] =
    {
        {"1 contour - welding speed",              1U, 0.100F},
        {"1 contour - medium speed",               1U, 0.050F},
        {"1 contour - current simulator default",  1U, 0.010F},
        {"3 contours - welding speed",             3U, 0.100F},
        {"3 contours - medium speed",              3U, 0.050F},
        {"3 contours - current simulator default", 3U, 0.010F}
    };

    RobotConfig robot;
    PathValidationConfig config;
    PathValidationWorkspace workspace;

    configure_validation(&robot, &config, &workspace);

    uint64_t total_allocated_bytes = 0U;
    uint64_t total_raw_bytes = 0U;
    uint64_t total_samples = 0U;
    double total_duration_s = 0.0;
    size_t accepted_cases = 0U;
    size_t rejected_cases = 0U;

    printf("============================================================\n");
    printf("TEST 7.6 - REALISTIC MULTI-GEOMETRY STORAGE CAPACITY\n");
    printf("============================================================\n");
    printf("Each contour: LINE -> ARC -> CIRCLE -> LINE -> ARC -> LINE\n");
    printf("Contour is closed, so repeats are continuous.\n");
    printf("Path Validation remains enabled: infeasible motion cases are rejected,\n");
    printf("reported, and excluded from flash-capacity totals.\n");
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
            !run_capacity_case(
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
            ++rejected_cases;
            printf("CASE RESULT: REJECTED / NOT COUNTED: %s\n", cases[i].name);
            continue;
        }

        ++accepted_cases;
        total_allocated_bytes += allocated_bytes;
        total_raw_bytes += raw_bytes;
        total_samples += samples;
        total_duration_s += duration_s;
    }

    if (accepted_cases == 0U)
    {
        printf("\nTEST 7.6: FAIL - no valid trajectory was available for storage sizing.\n");
        return 1;
    }

    const uint64_t reserve_bytes = total_allocated_bytes / 4U;
    const uint64_t planning_bytes = total_allocated_bytes + reserve_bytes;

    printf("\n============================================================\n");
    printf("TEST 7.6 COMBINED STORAGE REPORT - VALID TRAJECTORIES ONLY\n");
    printf("============================================================\n");
    printf("cases attempted:       %zu\n", sizeof(cases) / sizeof(cases[0]));
    printf("valid trajectories:    %zu\n", accepted_cases);
    printf("rejected trajectories: %zu\n", rejected_cases);
    printf("combined motion time:  %.3f s (%.3f min)\n",
           total_duration_s,
           total_duration_s / 60.0);
    printf("total samples:         %" PRIu64 "\n", total_samples);
    printf("total raw bytes:       %" PRIu64 " (%.3f MiB)\n",
           total_raw_bytes,
           (double)total_raw_bytes / BYTES_PER_MIB);
    printf("actual allocation:     %" PRIu64 " (%.3f MiB)\n",
           total_allocated_bytes,
           (double)total_allocated_bytes / BYTES_PER_MIB);
    printf("planning reserve:      +25%% = %" PRIu64 " B\n", reserve_bytes);
    printf("capacity to plan for:  %" PRIu64 " B (%.3f MiB)\n",
           planning_bytes,
           (double)planning_bytes / BYTES_PER_MIB);

    printf("\nCandidate flash sizes:\n");
    print_flash_capacity_line("8 MiB",  MIB_U64(8),  planning_bytes);
    print_flash_capacity_line("16 MiB", MIB_U64(16), planning_bytes);
    print_flash_capacity_line("32 MiB", MIB_U64(32), planning_bytes);
    print_flash_capacity_line("64 MiB", MIB_U64(64), planning_bytes);

    printf("\nTEST 7.6: PASS - storage characterized using valid Path Validation outputs.\n");
    return 0;
}
