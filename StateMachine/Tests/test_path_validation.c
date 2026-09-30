/*
 * Path Validation integration/capacity characterization entry point.
 *
 * Test 7.6 lives with the other simulator/external-memory tests, while the
 * root CMakeLists.txt already exposes a conditional `path_validation_test`
 * target at this path.
 */
#include "../../ControlCore/Kinematics/control_fk.h"
#include "../../ControlCore/Math/math3d.h"

/*
 * Pull the capacity test into this existing CMake target, but rename its main
 * so this wrapper can apply one test-only configuration correction before the
 * cases run.
 */
#define main test_trajectory_storage_capacity_original_main
#include "../../Simulation/Tests/test_trajectory_storage_capacity.c"
#undef main

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

    /*
     * Teaching stores speed as float. 0.100F converts to approximately
     * 0.10000000149 as double, which is microscopically above the exact
     * double literal 0.100 used by PathValidationConfig. Without a tolerance,
     * the normal maximum-speed setting is rejected as INVALID_PARAMETER.
     *
     * Keep the commanded test speed at exactly 0.100 m/s and allow only a
     * tiny representation margin in this characterization test. The production
     * validator should later clamp/tolerate the same float->double boundary.
     */
    config.max_tcp_speed_mps = 0.100001;

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
