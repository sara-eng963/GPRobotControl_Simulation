#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "state_boot.h"
#include "state_homing.h"

#include "robot_config.h"

#include "ethercat_master.h"

#include "a6ec_drive.h"
#include "cia402.h"


#define TEST_NUM_AXES       6
#define TEST_CYCLE_TIME_NS  1000000U

#define DEG2RAD(x) \
    ((x) * ROBOT_PI / 180.0)


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


static const char *homing_phase_name(
    HomingPhase phase
)
{
    switch (phase)
    {
        case HOMING_PHASE_INIT:
            return "INIT";

        case HOMING_PHASE_READ_POSITION:
            return "READ_POSITION";

        case HOMING_PHASE_PREPARE_TRAJECTORY:
            return "PREPARE_TRAJECTORY";

        case HOMING_PHASE_EXECUTE_TRAJECTORY:
            return "EXECUTE_TRAJECTORY";

        case HOMING_PHASE_VERIFY_HOME:
            return "VERIFY_HOME";

        case HOMING_PHASE_COMPLETE:
            return "COMPLETE";

        case HOMING_PHASE_FAILED:
            return "FAILED";

        default:
            return "UNKNOWN";
    }
}


static const char *homing_error_name(
    HomingError error
)
{
    switch (error)
    {
        case HOMING_ERROR_NONE:
            return "NONE";

        case HOMING_ERROR_INVALID_CONFIG:
            return "INVALID_CONFIG";

        case HOMING_ERROR_HOME_NOT_DEFINED:
            return "HOME_NOT_DEFINED";

        case HOMING_ERROR_HOME_LIMIT:
            return "HOME_LIMIT";

        case HOMING_ERROR_POSITION_FEEDBACK:
            return "POSITION_FEEDBACK";

        case HOMING_ERROR_DRIVE_NOT_ENABLED:
            return "DRIVE_NOT_ENABLED";

        case HOMING_ERROR_TRAJECTORY:
            return "TRAJECTORY";

        case HOMING_ERROR_TRAJECTORY_LIMIT:
            return "TRAJECTORY_LIMIT";

        case HOMING_ERROR_WKC:
            return "WKC";

        case HOMING_ERROR_HOME_TIMEOUT:
            return "HOME_TIMEOUT";

        default:
            return "UNKNOWN";
    }
}


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
                "BOOT failed before HOMING test.\n"
            );

            return false;
        }


        sleep_1ms();
    }
}


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


int main(void)
{
    printf(
        "\n"
        "============================================================\n"
        " HOMING STATE TEST\n"
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


    /*
     * HOMING assumes BOOT already succeeded.
     */
    if (!run_boot(&ethercatConfig))
    {
        ethercat_master_close();

        return 1;
    }


    printf(
        "BOOT COMPLETE -> starting HOMING\n"
    );


    RobotConfig robot;


    robot_config_init_ur5(
        &robot
    );


    /*
     * TEMPORARY SIMULATION HOME.
     *
     * Later move the approved real q_home values
     * into robot_config.c.
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


    HomingPhase previousPhase =
        homing.phase;


    printf(
        "HOMING -> %s\n",
        homing_phase_name(
            homing.phase
        )
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
            homing.phase !=
            previousPhase
        )
        {
            printf(
                "HOMING -> %s\n",
                homing_phase_name(
                    homing.phase
                )
            );


            previousPhase =
                homing.phase;
        }


        if (
            result ==
            STATE_STEP_COMPLETE
        )
        {
            printf(
                "\n"
                "============================================================\n"
                " HOMING TEST PASSED\n"
                "============================================================\n"
                "Samples sent: %zu\n",
                homing.samplesSent
            );

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
                " HOMING TEST FAILED\n"
                "============================================================\n"
                "Error      : %s\n"
                "Failed axis: %d\n"
                "============================================================\n",
                homing_error_name(
                    homing.error
                ),
                homing.failedAxis
            );


            disable_drives();

            ethercat_master_close();

            return 1;
        }


        sleep_1ms();
    }


    disable_drives();

    ethercat_master_close();


    printf(
        "Drives disabled and EtherCAT closed cleanly.\n"
    );


    return 0;
}