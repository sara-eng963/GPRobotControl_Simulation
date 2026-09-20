#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "state_boot.h"
#include "state_homing.h"
#include "state_idle.h"

#include "robot_config.h"

#include "ethercat_master.h"

#include "a6ec_drive.h"
#include "cia402.h"


#define TEST_NUM_AXES       6
#define TEST_CYCLE_TIME_NS  1000000U

#define TEST_IDLE_CYCLES    200U


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


static const char *idle_phase_name(
    IdlePhase phase
)
{
    switch (phase)
    {
        case IDLE_PHASE_INIT:
            return "INIT";

        case IDLE_PHASE_HOLDING:
            return "HOLDING";

        case IDLE_PHASE_COMPLETE:
            return "COMPLETE";

        case IDLE_PHASE_FAILED:
            return "FAILED";

        default:
            return "UNKNOWN";
    }
}


static const char *idle_error_name(
    IdleError error
)
{
    switch (error)
    {
        case IDLE_ERROR_NONE:
            return "NONE";

        case IDLE_ERROR_PDO_UNAVAILABLE:
            return "PDO_UNAVAILABLE";

        case IDLE_ERROR_POSITION_FEEDBACK:
            return "POSITION_FEEDBACK";

        case IDLE_ERROR_DRIVE_NOT_ENABLED:
            return "DRIVE_NOT_ENABLED";

        case IDLE_ERROR_WKC:
            return "WKC";

        case IDLE_ERROR_SAFETY:
            return "SAFETY";

        case IDLE_ERROR_INVALID_COMMAND:
            return "INVALID_COMMAND";

        default:
            return "UNKNOWN";
    }
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
        StateStepResult result =
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
                "BOOT failed before IDLE test.\n"
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

static bool run_homing(void)
{
    RobotConfig robot;


    robot_config_init_ur5(
        &robot
    );


    /*
     * Same temporary simulation home used by the standalone HOMING test.
     */
    robot.configuration.homeDefined =
        true;


    robot.configuration.home[0] =
        DEG2RAD(0.0);

    robot.configuration.home[1] =
        DEG2RAD(-90.0);

    robot.configuration.home[2] =
        DEG2RAD(90.0);

    robot.configuration.home[3] =
        DEG2RAD(0.0);

    robot.configuration.home[4] =
        DEG2RAD(0.0);

    robot.configuration.home[5] =
        DEG2RAD(0.0);


    HomingConfig homingConfig =
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
        StateStepResult result =
            state_homing_step(
                &homing,
                &homingConfig,
                &robot
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
                "HOMING failed before IDLE test.\n"
            );


            return false;
        }


        sleep_1ms();
    }
}


/* ============================================================================
 * VERIFY HOLD COMMAND
 * ============================================================================
 */

static bool verify_hold_targets(
    const IdleState *idle
)
{
    for (
        int slave = 1;
        slave <= TEST_NUM_AXES;
        slave++
    )
    {
        A6ECPDOCommand command;


        a6ec_read_command(
            slave,
            &command
        );


        if (
            command.targetPosition !=
            idle->holdPositionUnits[
                slave - 1
            ]
        )
        {
            printf(
                "Hold target mismatch on axis %d\n",
                slave
            );


            return false;
        }
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
        attempt++
    )
    {
        bool allDisabled =
            true;


        for (
            int slave = 1;
            slave <= TEST_NUM_AXES;
            slave++
        )
        {
            A6ECPDOFeedback feedback;


            a6ec_read_feedback(
                slave,
                &feedback
            );


            uint16_t driveState =
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


            A6ECPDOCommand command =
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
 * IDLE TEST
 * ============================================================================
 */

int main(void)
{
    printf(
        "\n"
        "============================================================\n"
        " IDLE STATE TEST\n"
        "============================================================\n"
    );


    EtherCATMasterConfig ethercatConfig =
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

    if (!run_boot(&ethercatConfig))
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

    if (!run_homing())
    {
        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "HOMING COMPLETE -> starting IDLE\n"
    );


    /* ------------------------------------------------------------------------
     * IDLE
     * ------------------------------------------------------------------------
     */

    IdleState idle;


    state_idle_enter(
        &idle
    );


    IdlePhase previousPhase =
        idle.phase;


    printf(
        "IDLE -> %s\n",
        idle_phase_name(
            idle.phase
        )
    );


    for (;;)
    {
        /*
         * Stay idle for 200 healthy cycles.
         *
         * Afterward simulate the operator pressing TEACH.
         */
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


        StateStepResult result =
            state_idle_step(
                &idle,
                command
            );


        if (
            idle.phase !=
            previousPhase
        )
        {
            printf(
                "IDLE -> %s\n",
                idle_phase_name(
                    idle.phase
                )
            );


            previousPhase =
                idle.phase;
        }


        /*
         * Halfway through the test verify that every output PDO
         * is still commanding the captured IDLE hold position.
         */
        if (
            idle.phase ==
                IDLE_PHASE_HOLDING
            &&
            idle.cyclesHeld ==
                100U
        )
        {
            if (
                !verify_hold_targets(
                    &idle
                )
            )
            {
                printf(
                    "\nIDLE HOLD VERIFICATION FAILED\n"
                );


                disable_drives();

                ethercat_master_close();

                return 1;
            }


            printf(
                "IDLE hold targets verified.\n"
            );
        }


        if (
            result ==
            STATE_STEP_COMPLETE
        )
        {
            break;
        }


        if (
            result ==
            STATE_STEP_FAILED
        )
        {
            printf(
                "\n"
                "============================================================\n"
                " IDLE TEST FAILED\n"
                "============================================================\n"
                "Error      : %s\n"
                "Failed axis: %d\n"
                "============================================================\n",
                idle_error_name(
                    idle.error
                ),
                idle.failedAxis
            );


            disable_drives();

            ethercat_master_close();

            return 1;
        }


        sleep_1ms();
    }


    /* ------------------------------------------------------------------------
     * FINAL VALIDATION
     * ------------------------------------------------------------------------
     */

    if (
        idle.exitCommand !=
        IDLE_COMMAND_TEACH
    )
    {
        printf(
            "IDLE completed with wrong exit command.\n"
        );


        disable_drives();

        ethercat_master_close();

        return 1;
    }


    printf(
        "\n"
        "============================================================\n"
        " IDLE TEST PASSED\n"
        "============================================================\n"
        "Healthy hold cycles: %u\n"
        "Exit command       : TEACH\n",
        (unsigned)idle.cyclesHeld
    );


    disable_drives();

    ethercat_master_close();


    printf(
        "Drives disabled and EtherCAT closed cleanly.\n"
    );


    return 0;
}