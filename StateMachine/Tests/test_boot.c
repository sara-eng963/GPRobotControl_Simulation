#include <stdio.h>
#include <stdbool.h>
#include <time.h>

#include "state_boot.h"

#include "ethercat_master.h"
#include "a6ec_drive.h"
#include "cia402.h"

#include "soem_backend.h"


#define TEST_NUM_AXES       6
#define TEST_CYCLE_TIME_NS  1000000U


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


static const char *boot_phase_name(
    BootPhase phase
)
{
    switch (phase)
    {
        case BOOT_PHASE_INIT:
            return "INIT";

        case BOOT_PHASE_OPEN_BUS:
            return "OPEN_BUS";

        case BOOT_PHASE_DISCOVER_SLAVES:
            return "DISCOVER_SLAVES";

        case BOOT_PHASE_VERIFY_SLAVE_IDENTITIES:
            return "VERIFY_SLAVE_IDENTITIES";

        case BOOT_PHASE_CONFIGURE_DRIVES:
            return "CONFIGURE_DRIVES";

        case BOOT_PHASE_MAP_PDOS:
            return "MAP_PDOS";

        case BOOT_PHASE_CONFIGURE_DC:
            return "CONFIGURE_DC";

        case BOOT_PHASE_SAFE_OP:
            return "SAFE_OP";

        case BOOT_PHASE_INITIAL_PDO_EXCHANGE:
            return "INITIAL_PDO_EXCHANGE";

        case BOOT_PHASE_OPERATIONAL:
            return "OPERATIONAL";

        case BOOT_PHASE_VERIFY_WKC:
            return "VERIFY_WKC";

        case BOOT_PHASE_VERIFY_CYCLIC_COMMUNICATION:
            return "VERIFY_CYCLIC_COMMUNICATION";

        case BOOT_PHASE_ENABLE_DRIVES:
            return "ENABLE_DRIVES";

        case BOOT_PHASE_VERIFY_POSITION_FEEDBACK:
            return "VERIFY_POSITION_FEEDBACK";

        case BOOT_PHASE_VERIFY_SAFETY:
            return "VERIFY_SAFETY";

        case BOOT_PHASE_COMPLETE:
            return "COMPLETE";

        case BOOT_PHASE_FAILED:
            return "FAILED";

        default:
            return "UNKNOWN";
    }
}


static const char *boot_error_name(
    BootError error
)
{
    switch (error)
    {
        case BOOT_ERROR_NONE:
            return "NONE";

        case BOOT_ERROR_MASTER_INIT:
            return "MASTER_INIT";

        case BOOT_ERROR_OPEN_BUS:
            return "OPEN_BUS";

        case BOOT_ERROR_SLAVE_COUNT:
            return "SLAVE_COUNT";

        case BOOT_ERROR_SLAVE_IDENTITY:
            return "SLAVE_IDENTITY";

        case BOOT_ERROR_DRIVE_CONFIGURATION:
            return "DRIVE_CONFIGURATION";

        case BOOT_ERROR_PDO_MAPPING:
            return "PDO_MAPPING";

        case BOOT_ERROR_DC_CONFIGURATION:
            return "DC_CONFIGURATION";

        case BOOT_ERROR_SAFE_OP:
            return "SAFE_OP";

        case BOOT_ERROR_OPERATIONAL:
            return "OPERATIONAL";

        case BOOT_ERROR_WKC:
            return "WKC";

        case BOOT_ERROR_CYCLIC_COMMUNICATION:
            return "CYCLIC_COMMUNICATION";

        case BOOT_ERROR_DRIVE_FAULT:
            return "DRIVE_FAULT";

        case BOOT_ERROR_DRIVE_QUICK_STOP:
            return "DRIVE_QUICK_STOP";

        case BOOT_ERROR_DRIVE_STATE:
            return "DRIVE_STATE";

        case BOOT_ERROR_DRIVE_ENABLE:
            return "DRIVE_ENABLE";

        case BOOT_ERROR_POSITION_FEEDBACK:
            return "POSITION_FEEDBACK";

        case BOOT_ERROR_SAFETY:
            return "SAFETY";

        default:
            return "UNKNOWN";
    }
}


/* ============================================================================
 * CLEAN TEST SHUTDOWN
 * ============================================================================
 */

static void disable_drives(void)
{
    printf(
        "\nDisabling drives...\n"
    );


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
            printf(
                "All drives disabled.\n"
            );

            return;
        }


        sleep_1ms();
    }


    printf(
        "WARNING: Drive disable timeout.\n"
    );
}


/* ============================================================================
 * BOOT TEST
 * ============================================================================
 */

int main(void)
{
    printf(
        "\n"
        "============================================================\n"
        " BOOT STATE TEST\n"
        "============================================================\n"
    );


    EtherCATMasterConfig config =
    {
        .interfaceName =
            "ecatA",

        .expectedSlaveCount =
            TEST_NUM_AXES,

        .cycleTimeNs =
            TEST_CYCLE_TIME_NS
    };


    BootState boot;


    state_boot_enter(
        &boot
    );


    BootPhase previousPhase =
        boot.phase;


    /*
     * Pass 5B test flag.
     *
     * Inject SAFE-OP + ERROR only once.
     */
    bool safeOpErrorInjected =
        false;


    printf(
        "BOOT -> %s\n",
        boot_phase_name(
            boot.phase
        )
    );


    for (;;)
    {
        /*
         * PASS 5B:
         *
         * After several healthy cyclic communication cycles,
         * make slave 3 report:
         *
         *      SAFE-OP + ERROR
         *
         * This is a deterministic test-only injection used to
         * exercise the EtherCAT recovery decision path.
         */
        if (
            boot.phase ==
                BOOT_PHASE_VERIFY_CYCLIC_COMMUNICATION
            &&
            boot.stableCycles >=
                5U
            &&
            !safeOpErrorInjected
        )
        {
            printf(
                "\n"
                "TEST: Injecting SAFE-OP + ERROR "
                "on EtherCAT slave 3\n"
            );


            if (
                !soem_backend_test_force_safe_op_error(
                    3
                )
            )
            {
                printf(
                    "TEST ERROR: Could not inject "
                    "SAFE-OP + ERROR on slave 3\n"
                );


                ethercat_master_close();

                return 1;
            }


            safeOpErrorInjected =
                true;


            printf(
                "TEST: Slave 3 now reports "
                "SAFE-OP + ERROR\n\n"
            );
        }


        /*
         * Execute one BOOT state-machine step.
         */
        StateStepResult result =
            state_boot_step(
                &boot,
                &config
            );


        /*
         * Print only when BOOT moves to another phase.
         */
        if (
            boot.phase !=
            previousPhase
        )
        {
            printf(
                "BOOT -> %s\n",
                boot_phase_name(
                    boot.phase
                )
            );


            previousPhase =
                boot.phase;
        }


        /*
         * BOOT completed successfully.
         */
        if (
            result ==
            STATE_STEP_COMPLETE
        )
        {
            printf(
                "\n"
                "============================================================\n"
                " BOOT TEST PASSED\n"
                "============================================================\n"
            );

            break;
        }


        /*
         * BOOT failed.
         */
        if (
            result ==
            STATE_STEP_FAILED
        )
        {
            printf(
                "\n"
                "============================================================\n"
                " BOOT TEST FAILED\n"
                "============================================================\n"
                "Phase      : %s\n"
                "Error      : %s\n"
                "Failed axis: %d\n"
                "============================================================\n",
                boot_phase_name(
                    boot.phase
                ),
                boot_error_name(
                    boot.error
                ),
                boot.failedAxis
            );


            ethercat_master_close();

            return 1;
        }


        /*
         * Nominal BOOT cycle:
         *
         *      1 ms
         */
        sleep_1ms();
    }


    /*
     * Clean test shutdown.
     */
    disable_drives();


    ethercat_master_close();


    printf(
        "EtherCAT closed cleanly.\n"
    );


    return 0;
}