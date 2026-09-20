#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "state_boot.h"
#include "state_homing.h"
#include "state_idle.h"
#include "state_teaching.h"

#include "robot_config.h"

#include "ethercat_master.h"

#include "a6ec_drive.h"
#include "cia402.h"


#define TEST_NUM_AXES       6
#define TEST_CYCLE_TIME_NS  1000000U

#define TEST_IDLE_CYCLES    20U


#define DEG2RAD(x) \
    ((x) * ROBOT_PI / 180.0)


/* ============================================================================
 * TEST HELPERS
 * ============================================================================
 */

static void sleep_1ms(void)
{
    const struct timespec delay =
    {
        .tv_sec = 0,
        .tv_nsec = 1000000L
    };


    nanosleep(
        &delay,
        NULL
    );
}


/* ============================================================================
 * RUN BOOT
 * ============================================================================
 */

static bool run_boot(
    const EtherCATMasterConfig *ethercatConfig
)
{
    BootState boot;


    state_boot_enter(
        &boot
    );


    for (;;)
    {
        const StateStepResult result =
            state_boot_step(
                &boot,
                ethercatConfig
            );


        if (
            result ==
            STATE_STEP_COMPLETE
        )
        {
            return true;
        }


        if (
            result ==
            STATE_STEP_FAILED
        )
        {
            printf(
                "BOOT failed before TEACHING test.\n"
            );


            return false;
        }


        sleep_1ms();
    }
}


/* ============================================================================
 * RUN HOMING
 * ============================================================================
 */

static bool run_homing(
    RobotConfig *robot
)
{
    robot_config_init_ur5(
        robot
    );


    /*
     * Same temporary simulation home used by the existing HOMING/IDLE tests.
     */
    robot->configuration.homeDefined =
        true;


    robot->configuration.home[0] =
        DEG2RAD(0.0);

    robot->configuration.home[1] =
        DEG2RAD(-90.0);

    robot->configuration.home[2] =
        DEG2RAD(90.0);

    robot->configuration.home[3] =
        DEG2RAD(0.0);

    robot->configuration.home[4] =
        DEG2RAD(0.0);

    robot->configuration.home[5] =
        DEG2RAD(0.0);


    const HomingConfig homingConfig =
    {
        .duration =
            5.0,

        .dt =
            0.001,

        .positionTolerance =
            DEG2RAD(0.5),

        .requiredStableCycles =
            20U,

        .maxVerificationCycles =
            2000U
    };


    HomingState homing;


    state_homing_enter(
        &homing
    );


    for (;;)
    {
        const StateStepResult result =
            state_homing_step(
                &homing,
                &homingConfig,
                robot
            );


        if (
            result ==
            STATE_STEP_COMPLETE
        )
        {
            return true;
        }


        if (
            result ==
            STATE_STEP_FAILED
        )
        {
            printf(
                "HOMING failed before TEACHING test.\n"
            );


            return false;
        }


        sleep_1ms();
    }
}


/* ============================================================================
 * RUN IDLE UNTIL TEACH COMMAND
 * ============================================================================
 */

static bool run_idle_to_teach(void)
{
    IdleState idle;


    state_idle_enter(
        &idle
    );


    for (;;)
    {
        IdleCommand command =
            IDLE_COMMAND_NONE;


        if (
            idle.cyclesHeld >=
            TEST_IDLE_CYCLES
        )
        {
            command =
                IDLE_COMMAND_TEACH;
        }


        const StateStepResult result =
            state_idle_step(
                &idle,
                command
            );


        if (
            result ==
            STATE_STEP_COMPLETE
        )
        {
            return
                idle.exitCommand ==
                IDLE_COMMAND_TEACH;
        }


        if (
            result ==
            STATE_STEP_FAILED
        )
        {
            printf(
                "IDLE failed before TEACHING test.\n"
            );


            return false;
        }


        sleep_1ms();
    }
}


/* ============================================================================
 * SIMULATE MANUAL GUIDANCE
 * ============================================================================
 *
 * TEACHING itself does not command motion.
 *
 * In the real robot the manual-guidance/admittance controller owns that job.
 *
 * For this PC/KickCAT test only, this helper changes J1 while holding the other
 * five joints at their measured positions. That gives Teaching a second real
 * PDO/FK pose to record.
 */

static bool simulate_guided_joint_move(
    double joint1OffsetRad
)
{
    int32_t targets[TEST_NUM_AXES];


    for (
        int slave = 1;
        slave <= TEST_NUM_AXES;
        ++slave
    )
    {
        A6ECPDOFeedback feedback;


        a6ec_read_feedback(
            slave,
            &feedback
        );


        targets[slave - 1] =
            feedback.actualPosition;
    }


    const double q1Current =
        a6ec_position_units_to_joint_rad(
            targets[0]
        );


    targets[0] =
        a6ec_joint_rad_to_position_units(
            q1Current +
            joint1OffsetRad
        );


    /*
     * Send the target for several cycles to represent an external guidance
     * controller moving the robot before the operator records P2.
     */
    for (
        int cycle = 0;
        cycle < 20;
        ++cycle
    )
    {
        for (
            int slave = 1;
            slave <= TEST_NUM_AXES;
            ++slave
        )
        {
            A6ECPDOFeedback feedback;


            a6ec_read_feedback(
                slave,
                &feedback
            );


            if (
                cia402_get_state(
                    feedback.statusword
                )
                !=
                CIA402_STATE_OPERATION_ENABLED
            )
            {
                return false;
            }


            const A6ECPDOCommand command =
            {
                .controlword =
                    CIA402_CONTROLWORD_ENABLE_OPERATION,

                .targetPosition =
                    targets[slave - 1]
            };


            a6ec_write_command(
                slave,
                &command
            );
        }


        if (
            ethercat_master_exchange()
            <
            ethercat_master_expected_wkc()
        )
        {
            return false;
        }


        sleep_1ms();
    }


    return true;
}


/* ============================================================================
 * CLEAN SHUTDOWN
 * ============================================================================
 */

static void disable_drives(void)
{
    for (
        int attempt = 0;
        attempt < 100;
        ++attempt
    )
    {
        bool allDisabled =
            true;


        for (
            int slave = 1;
            slave <= TEST_NUM_AXES;
            ++slave
        )
        {
            A6ECPDOFeedback feedback;


            a6ec_read_feedback(
                slave,
                &feedback
            );


            const uint16_t driveState =
                cia402_get_state(
                    feedback.statusword
                );


            if (
                driveState !=
                CIA402_STATE_SWITCH_ON_DISABLED
            )
            {
                allDisabled =
                    false;
            }


            const A6ECPDOCommand command =
            {
                .controlword =
                    cia402_get_disable_controlword(
                        driveState
                    ),

                .targetPosition =
                    feedback.actualPosition
            };


            a6ec_write_command(
                slave,
                &command
            );
        }


        ethercat_master_exchange();


        if (allDisabled)
        {
            return;
        }


        sleep_1ms();
    }
}


/* ============================================================================
 * TEACHING INTEGRATION TEST
 * ============================================================================
 */

int main(void)
{
    printf(
        "\n"
        "============================================================\n"
        " TEACHING STATE INTEGRATION TEST\n"
        "============================================================\n"
    );


    const EtherCATMasterConfig ethercatConfig =
    {
        .interfaceName =
            "ecatA",

        .expectedSlaveCount =
            TEST_NUM_AXES,

        .cycleTimeNs =
            TEST_CYCLE_TIME_NS
    };


    /* ------------------------------------------------------------------------
     * BOOT
     * ------------------------------------------------------------------------
     */

    if (
        !run_boot(
            &ethercatConfig
        )
    )
    {
        ethercat_master_close();

        return 1;
    }


    printf(
        "BOOT COMPLETE\n"
    );


    /* ------------------------------------------------------------------------
     * HOMING
     * ------------------------------------------------------------------------
     */

    RobotConfig robot;


    if (
        !run_homing(
            &robot
        )
    )
    {
        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "HOMING COMPLETE\n"
    );


    /* ------------------------------------------------------------------------
     * IDLE -> TEACH
     * ------------------------------------------------------------------------
     */

    if (!run_idle_to_teach())
    {
        printf(
            "IDLE did not exit through TEACH.\n"
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "IDLE COMPLETE -> TEACHING\n"
    );


    /* ------------------------------------------------------------------------
     * ENTER TEACHING
     * ------------------------------------------------------------------------
     */

    const TeachingConfig teachingConfig =
    {
        .default_speed_mps =
            0.010F,

        .minimum_speed_mps =
            0.001F,

        .maximum_speed_mps =
            0.100F,

        .speed_step_mps =
            0.001F,

        .minimum_point_separation_m =
            0.002F,

        .collinearity_epsilon_m2 =
            1.0e-10F
    };


    TeachingRuntimeInputs runtime =
    {
        .timestamp_ms =
            1000U,

        .calibration_version =
            1U,

        .active_frame_id =
            1U,

        .active_tool_id =
            1U,

        .robot_motion_settled =
            true,

        .manual_guidance_active =
            true,

        .motion_permitted =
            true,

        .estop_active =
            false,

        .protective_stop_active =
            false,

        .global_fault_active =
            false,

        .robot_homed =
            true
    };


    TeachingState teaching;
    TeachingOutputs outputs;


    state_teaching_enter(
        &teaching,
        &teachingConfig,
        1U
    );


    if (!teaching.initialized)
    {
        printf(
            "TEACHING initialization failed.\n"
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "TEACHING -> %s\n",
        state_teaching_phase_name(
            teaching.phase
        )
    );


    /* ------------------------------------------------------------------------
     * SELECT LINE
     * ------------------------------------------------------------------------
     */

    (void)state_teaching_step(
        &teaching,
        &robot,
        &runtime,
        TEACH_EVENT_SELECT_LINE,
        &outputs
    );


    if (!teaching.last_event_accepted)
    {
        printf(
            "SELECT_LINE rejected: %s\n",
            state_teaching_error_name(
                outputs.error
            )
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "Teaching: LINE selected\n"
    );


    /* ------------------------------------------------------------------------
     * RECORD P1 FROM REAL SIMULATED DRIVE FEEDBACK + FK
     * ------------------------------------------------------------------------
     */

    (void)state_teaching_step(
        &teaching,
        &robot,
        &runtime,
        TEACH_EVENT_RECORD_POINT,
        &outputs
    );


    if (!teaching.last_event_accepted)
    {
        printf(
            "P1 rejected: %s\n",
            state_teaching_error_name(
                outputs.error
            )
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "Teaching: P1 recorded\n"
    );


    /* ------------------------------------------------------------------------
     * VERIFY TEAMMATE DUPLICATE-POINT LOGIC IS STILL PRESENT
     * ------------------------------------------------------------------------
     */

    (void)state_teaching_step(
        &teaching,
        &robot,
        &runtime,
        TEACH_EVENT_RECORD_POINT,
        &outputs
    );


    if (
        teaching.last_event_accepted
        ||
        outputs.error !=
        TEACH_ERR_DUPLICATE_POINT
    )
    {
        printf(
            "Duplicate-point behavior changed unexpectedly.\n"
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "Teaching: duplicate P2 correctly rejected\n"
    );


    /* ------------------------------------------------------------------------
     * SIMULATE EXTERNAL MANUAL GUIDANCE
     * ------------------------------------------------------------------------
     */

    if (
        !simulate_guided_joint_move(
            DEG2RAD(10.0)
        )
    )
    {
        printf(
            "Simulated guidance move failed.\n"
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    runtime.timestamp_ms =
        2000U;


    /* ------------------------------------------------------------------------
     * RECORD P2 FROM UPDATED MOTOR FEEDBACK + FK
     * ------------------------------------------------------------------------
     */

    (void)state_teaching_step(
        &teaching,
        &robot,
        &runtime,
        TEACH_EVENT_RECORD_POINT,
        &outputs
    );


    if (!teaching.last_event_accepted)
    {
        printf(
            "P2 rejected: %s\n",
            state_teaching_error_name(
                outputs.error
            )
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    if (
        teaching.draft.segment_count !=
        1U
    )
    {
        printf(
            "LINE segment was not committed.\n"
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "Teaching: P2 recorded -> LINE segment complete\n"
    );


    /* ------------------------------------------------------------------------
     * VALIDATE / SUBMIT DRAFT
     * ------------------------------------------------------------------------
     */

    const StateStepResult teachingResult =
        state_teaching_step(
            &teaching,
            &robot,
            &runtime,
            TEACH_EVENT_VALIDATE_PATH,
            &outputs
        );


    if (
        teachingResult !=
        STATE_STEP_COMPLETE
        ||
        !teaching.last_event_accepted
        ||
        !outputs.validation_request
    )
    {
        printf(
            "Validation request failed: %s\n",
            state_teaching_error_name(
                outputs.error
            )
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    if (
        teaching.draft.draft_crc ==
        0U
    )
    {
        printf(
            "Draft CRC was not generated.\n"
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "TEACHING -> %s\n",
        state_teaching_phase_name(
            teaching.phase
        )
    );


    printf(
        "\n"
        "============================================================\n"
        " TEACHING INTEGRATION TEST PASSED\n"
        "============================================================\n"
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


    disable_drives();

    ethercat_master_close();


    printf(
        "Drives disabled and EtherCAT closed cleanly.\n"
    );


    return 0;
}
