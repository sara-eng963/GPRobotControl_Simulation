#include "state_boot.h"

#include "../../EtherCATComm/ethercat_master.h"

#include "../../ServoDrive/A6EC/a6ec_drive.h"
#include "../../ServoDrive/A6EC/a6ec_registers.h"
#include "../../ServoDrive/A6EC/a6ec_identity.h"

#include "../../ServoDrive/CiA402/cia402.h"


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


/* ============================================================================
 * BOOT CONFIGURATION
 * ============================================================================
 */

/*
 * Number of consecutive healthy EtherCAT PDO cycles required before
 * communication is considered stable.
 *
 * Current control period:
 *
 *      1 cycle = 1 ms
 *
 * Therefore:
 *
 *      20 cycles = 20 ms
 */
#define BOOT_STABILITY_REQUIRED_CYCLES     20U
#define BOOT_ETHERCAT_RECOVERY_MAX_ATTEMPTS    5U

/*
 * Maximum number of cyclic attempts allowed while enabling all drives.
 *
 * At a 1 ms EtherCAT period:
 *
 *      5000 cycles = 5 seconds
 */
#define BOOT_DRIVE_ENABLE_TIMEOUT_CYCLES   5000U


/* ============================================================================
 * INTERNAL HELPERS
 * ============================================================================
 */

static void boot_advance(
    BootState *boot,
    BootPhase nextPhase
)
{
    boot->phase =
        nextPhase;

    boot->stableCycles =
        0U;

    boot->recoveryAttempts =
        0U;
}


static StateStepResult boot_fail(
    BootState *boot,
    BootError error,
    int failedAxis
)
{
    boot->error =
        error;

    boot->failedAxis =
        failedAxis;

    boot->phase =
        BOOT_PHASE_FAILED;

    return
        STATE_STEP_FAILED;
}

static bool boot_attempt_ethercat_recovery(
    BootState *boot,
    int slaveCount
)
{
    /*
     * Do not retry forever.
     */
    if (
        boot->recoveryAttempts >=
        BOOT_ETHERCAT_RECOVERY_MAX_ATTEMPTS
    )
    {
        printf(
            "BOOT: EtherCAT recovery limit reached (%u attempts)\n",
            BOOT_ETHERCAT_RECOVERY_MAX_ATTEMPTS
        );

        return false;
    }


    /*
     * Count this recovery attempt.
     */
    boot->recoveryAttempts++;


    printf(
        "\nBOOT: EtherCAT recovery attempt %u/%u\n",
        boot->recoveryAttempts,
        BOOT_ETHERCAT_RECOVERY_MAX_ATTEMPTS
    );


    /*
     * Show us what state every slave is currently in.
     */
    ethercat_master_print_slave_diagnostics();


    /*
     * Give every slave one recovery decision.
     */
    for (
        int slave = 1;
        slave <= slaveCount;
        slave++
    )
    {
        EtherCATRecoveryAction action =
            ethercat_master_recovery_step(
                slave
            );


        if (
            action !=
            ETHERCAT_RECOVERY_ACTION_NONE
        )
        {
            printf(
                "BOOT: Slave %d recovery action = %d\n",
                slave,
                action
            );
        }
    }


    /*
     * true means:
     *
     * "A recovery attempt was allowed/performed.
     *  Let BOOT run another cycle and check again."
     */
    return true;
}


/* ============================================================================
 * ENTER BOOT
 * ============================================================================
 */

void state_boot_enter(
    BootState *boot
)
{
    if (boot == NULL)
    {
        return;
    }


    boot->phase =
        BOOT_PHASE_INIT;

    boot->error =
        BOOT_ERROR_NONE;

    boot->failedAxis =
        0;

    boot->stableCycles =
        0U;

    boot->recoveryAttempts =
        0U;
}


/* ============================================================================
 * BOOT STATE
 * ============================================================================
 */

StateStepResult state_boot_step(
    BootState *boot,
    const EtherCATMasterConfig *ethercatConfig
)
{
    if (
        boot == NULL ||
        ethercatConfig == NULL
    )
    {
        if (boot != NULL)
        {
            return boot_fail(
                boot,
                BOOT_ERROR_MASTER_INIT,
                0
            );
        }

        return
            STATE_STEP_FAILED;
    }


    switch (boot->phase)
    {
        /* ====================================================================
         * INITIALIZE ETHERCAT MASTER CONFIGURATION
         * ====================================================================
         */

        case BOOT_PHASE_INIT:
        {
            if (
                !ethercat_master_init(
                    ethercatConfig
                )
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_MASTER_INIT,
                    0
                );
            }


            boot_advance(
                boot,
                BOOT_PHASE_OPEN_BUS
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * OPEN ETHERCAT INTERFACE
         * ====================================================================
         */

        case BOOT_PHASE_OPEN_BUS:
        {
            if (
                !ethercat_master_open()
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_OPEN_BUS,
                    0
                );
            }


            boot_advance(
                boot,
                BOOT_PHASE_DISCOVER_SLAVES
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * DISCOVER EXPECTED SLAVES
         * ====================================================================
         */

        case BOOT_PHASE_DISCOVER_SLAVES:
        {
            int slaveCount =
                ethercat_master_scan();


            if (
                slaveCount <= 0 ||
                slaveCount !=
                    ethercatConfig->expectedSlaveCount
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_SLAVE_COUNT,
                    0
                );
            }


            boot_advance(
                boot,
                BOOT_PHASE_VERIFY_SLAVE_IDENTITIES
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * VERIFY SLAVE IDENTITIES
         * ====================================================================
         */

        case BOOT_PHASE_VERIFY_SLAVE_IDENTITIES:
{
    for (
        int slave = 1;
        slave <= ethercatConfig->expectedSlaveCount;
        slave++
    )
    {
        EtherCATSlaveIdentity identity;


        if (
            !ethercat_master_slave_identity(
                slave,
                &identity
            )
        )
        {
            printf(
                "BOOT: Could not read identity of slave %d\n",
                slave
            );


            return boot_fail(
                boot,
                BOOT_ERROR_SLAVE_IDENTITY,
                slave
            );
        }


        /* ---------------------------------------------------------------
         * VERIFY VENDOR ID
         * ---------------------------------------------------------------
         */

        if (
            identity.vendorId !=
            A6EC_EXPECTED_VENDOR_ID
        )
        {
            printf(
                "BOOT: Slave %d Vendor ID mismatch\n"
                "      Expected: 0x%08lX\n"
                "      Actual  : 0x%08lX\n",
                slave,
                (unsigned long)A6EC_EXPECTED_VENDOR_ID,
                (unsigned long)identity.vendorId
            );


            return boot_fail(
                boot,
                BOOT_ERROR_SLAVE_IDENTITY,
                slave
            );
        }


        /* ---------------------------------------------------------------
         * VERIFY PRODUCT CODE
         * ---------------------------------------------------------------
         */

        if (
            identity.productCode !=
            A6EC_EXPECTED_PRODUCT_CODE
        )
        {
            printf(
                "BOOT: Slave %d Product Code mismatch\n"
                "      Expected: 0x%08lX\n"
                "      Actual  : 0x%08lX\n",
                slave,
                (unsigned long)A6EC_EXPECTED_PRODUCT_CODE,
                (unsigned long)identity.productCode
            );


            return boot_fail(
                boot,
                BOOT_ERROR_SLAVE_IDENTITY,
                slave
            );
        }


        /*
         * Revision and serial number are recorded for diagnostics,
         * but are not hard-fail conditions yet.
         */
        printf(
            "BOOT: Slave %d identity OK "
            "[Vendor=0x%08lX Product=0x%08lX "
            "Revision=0x%08lX Serial=0x%08lX]\n",
            slave,
            (unsigned long)identity.vendorId,
            (unsigned long)identity.productCode,
            (unsigned long)identity.revision,
            (unsigned long)identity.serialNumber
        );
    }


    boot_advance(
        boot,
        BOOT_PHASE_MAP_PDOS
    );


    return
        STATE_STEP_RUNNING;
}


        /* ====================================================================
         * CONFIGURE ALL A6-EC DRIVES
         * ====================================================================
         */

        case BOOT_PHASE_CONFIGURE_DRIVES:
{
    for (
        int slave = 1;
        slave <= ethercatConfig->expectedSlaveCount;
        slave++
    )
    {
        int8_t modeReadback =
            0;


        if (
            !a6ec_set_csp_mode(
                slave,
                &modeReadback
            )
        )
        {
            printf(
                "BOOT: Slave %d CSP SDO configuration FAILED\n",
                slave
            );

            while (
                ethercat_master_has_error()
            )
            {
                printf(
                    "SOEM: %s\n",
                    ethercat_master_pop_error_string()
                );
            }

            return boot_fail(
                boot,
                BOOT_ERROR_DRIVE_CONFIGURATION,
                slave
            );
        }


        printf(
            "BOOT: Slave %d CSP readback = %d\n",
            slave,
            modeReadback
        );


        if (
            modeReadback !=
            A6EC_MODE_CSP
        )
        {
            printf(
                "BOOT: Slave %d expected CSP=%d but read back %d\n",
                slave,
                A6EC_MODE_CSP,
                modeReadback
            );

            return boot_fail(
                boot,
                BOOT_ERROR_DRIVE_CONFIGURATION,
                slave
            );
        }
    }


    boot_advance(
        boot,
        BOOT_PHASE_CONFIGURE_DC
    );

    return
        STATE_STEP_RUNNING;
}


        /* ====================================================================
         * MAP PDO PROCESS DATA
         * ====================================================================
         */

        case BOOT_PHASE_MAP_PDOS:
        {
            int mappedBytes =
                ethercat_master_map_pdos();


            if (mappedBytes <= 0)
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_PDO_MAPPING,
                    0
                );
            }


            boot_advance(
                boot,
                BOOT_PHASE_CONFIGURE_DRIVES
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * CONFIGURE DISTRIBUTED CLOCKS
         * ====================================================================
         */

        case BOOT_PHASE_CONFIGURE_DC:
        {
            if (
                !ethercat_master_configure_distributed_clocks()
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_DC_CONFIGURATION,
                    0
                );
            }


            boot_advance(
                boot,
                BOOT_PHASE_SAFE_OP
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * WAIT FOR ETHERCAT SAFE-OP
         * ====================================================================
         */

        case BOOT_PHASE_SAFE_OP:
        {
            if (
                !ethercat_master_wait_for_safe_op()
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_SAFE_OP,
                    0
                );
            }


            boot_advance(
                boot,
                BOOT_PHASE_INITIAL_PDO_EXCHANGE
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * INITIAL PDO EXCHANGE
         * ====================================================================
         */

        case BOOT_PHASE_INITIAL_PDO_EXCHANGE:
        {
            /*
             * Same initial process-data exchange used by the currently
             * tested main.c before requesting OPERATIONAL.
             *
             * Expected WKC has not yet been established, so this phase does
             * not validate WKC yet.
             */
            ethercat_master_exchange();


            boot_advance(
                boot,
                BOOT_PHASE_OPERATIONAL
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * REQUEST ETHERCAT OPERATIONAL
         * ====================================================================
         */

        case BOOT_PHASE_OPERATIONAL:
        {
            if (
                !ethercat_master_request_operational()
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_OPERATIONAL,
                    0
                );
            }


            boot_advance(
                boot,
                BOOT_PHASE_VERIFY_WKC
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * VERIFY EXPECTED WKC
         * ====================================================================
         */

        case BOOT_PHASE_VERIFY_WKC:
        {
            int expectedWkc =
                ethercat_master_expected_wkc();


            if (expectedWkc <= 0)
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_WKC,
                    0
                );
            }


            int actualWkc =
                ethercat_master_exchange();


if (
    actualWkc <
    expectedWkc
)
{
    printf(
        "BOOT: WKC mismatch. Expected=%d Actual=%d\n",
        expectedWkc,
        actualWkc
    );


    if (
        !boot_attempt_ethercat_recovery(
            boot,
            ethercatConfig->expectedSlaveCount
        )
    )
    {
        return boot_fail(
            boot,
            BOOT_ERROR_WKC,
            0
        );
    }


    return
        STATE_STEP_RUNNING;
}

int failedSlave =
    0;


if (
    !ethercat_master_all_slaves_operational(
        ethercatConfig->expectedSlaveCount,
        &failedSlave
    )
)
{
    printf(
        "BOOT: EtherCAT slave %d is not OPERATIONAL\n",
        failedSlave
    );


    if (
        !boot_attempt_ethercat_recovery(
            boot,
            ethercatConfig->expectedSlaveCount
        )
    )
    {
        return boot_fail(
            boot,
            BOOT_ERROR_OPERATIONAL,
            failedSlave
        );
    }


    return
        STATE_STEP_RUNNING;
}
            boot_advance(
                boot,
                BOOT_PHASE_VERIFY_CYCLIC_COMMUNICATION
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * VERIFY CYCLIC PDO COMMUNICATION
         * ====================================================================
         */

      case BOOT_PHASE_VERIFY_CYCLIC_COMMUNICATION:
{
    int expectedWkc =
        ethercat_master_expected_wkc();


    int actualWkc =
        ethercat_master_exchange();


    /*
     * First requirement:
     * PDO communication must have the expected WKC.
     */
    if (
        actualWkc <
        expectedWkc
    )
    {
        printf(
            "BOOT: Cyclic WKC mismatch. Expected=%d Actual=%d\n",
            expectedWkc,
            actualWkc
        );


        /*
         * Stability requires consecutive healthy cycles.
         */
        boot->stableCycles =
            0U;


        if (
            !boot_attempt_ethercat_recovery(
                boot,
                ethercatConfig->expectedSlaveCount
            )
        )
        {
            return boot_fail(
                boot,
                BOOT_ERROR_CYCLIC_COMMUNICATION,
                0
            );
        }


        return
            STATE_STEP_RUNNING;
    }


    /*
     * Second requirement:
     * Every EtherCAT slave must still be OPERATIONAL.
     *
     * WKC alone is not sufficient because a slave may
     * leave OP while communication still appears healthy.
     */
    int failedSlave =
        0;


    if (
        !ethercat_master_all_slaves_operational(
            ethercatConfig->expectedSlaveCount,
            &failedSlave
        )
    )
    {
        printf(
            "BOOT: EtherCAT slave %d left OPERATIONAL\n",
            failedSlave
        );


        boot->stableCycles =
            0U;


        if (
            !boot_attempt_ethercat_recovery(
                boot,
                ethercatConfig->expectedSlaveCount
            )
        )
        {
            return boot_fail(
                boot,
                BOOT_ERROR_OPERATIONAL,
                failedSlave
            );
        }


        return
            STATE_STEP_RUNNING;
    }


    /*
     * This cycle was completely healthy:
     *
     *      WKC valid
     *      AND
     *      all slaves OPERATIONAL
     */
    boot->stableCycles++;


    if (
        boot->stableCycles >=
        BOOT_STABILITY_REQUIRED_CYCLES
    )
    {
        boot_advance(
            boot,
            BOOT_PHASE_ENABLE_DRIVES
        );
    }


    return
        STATE_STEP_RUNNING;
}

        /* ====================================================================
         * ENABLE ALL SIX CiA-402 DRIVES
         * ====================================================================
         */
case BOOT_PHASE_ENABLE_DRIVES:
{

bool allOperationEnabled =
        true;


    int failedSlave =
        0;


    if (
        !ethercat_master_all_slaves_operational(
            ethercatConfig->expectedSlaveCount,
            &failedSlave
        )
    )
    {
        printf(
            "BOOT: EtherCAT slave %d left OPERATIONAL while enabling drives\n",
            failedSlave
        );


        if (
            !boot_attempt_ethercat_recovery(
                boot,
                ethercatConfig->expectedSlaveCount
            )
        )
        {
            return boot_fail(
                boot,
                BOOT_ERROR_OPERATIONAL,
                failedSlave
            );
        }


        return
            STATE_STEP_RUNNING;
    }


    /*
     * Read the current state of every drive...
     */
    for (
        int slave = 1;
        slave <= ethercatConfig->expectedSlaveCount;
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
    driveState ==
    CIA402_STATE_FAULT
)
{
    uint16_t errorCode =
        0U;


    bool errorCodeReadable =
        a6ec_read_error_code(
            slave,
            &errorCode
        );


    printf(
        "BOOT: Slave %d drive FAULT\n"
        "      CiA-402 state : %s\n"
        "      Statusword    : 0x%04X\n",
        slave,
        cia402_state_name(
            driveState
        ),
        feedback.statusword
    );


    if (errorCodeReadable)
    {
        printf(
            "      Error Code    : 0x%04X\n",
            errorCode
        );
    }
    else
    {
        printf(
            "      Error Code    : unreadable\n"
        );
    }


    return boot_fail(
        boot,
        BOOT_ERROR_DRIVE_FAULT,
        slave
    );
}

if (
    driveState ==
    CIA402_STATE_FAULT_REACTION_ACTIVE
)
{
    printf(
        "BOOT: Slave %d is in FAULT_REACTION_ACTIVE\n"
        "      CiA-402 state : %s\n"
        "      Statusword    : 0x%04X\n",
        slave,
        cia402_state_name(
            driveState
        ),
        feedback.statusword
    );


    return boot_fail(
        boot,
        BOOT_ERROR_DRIVE_FAULT,
        slave
    );
}

if (
    driveState ==
    CIA402_STATE_QUICK_STOP_ACTIVE
)
{
    printf(
        "BOOT: Slave %d is in QUICK_STOP_ACTIVE\n"
        "      CiA-402 state : %s\n"
        "      Statusword    : 0x%04X\n",
        slave,
        cia402_state_name(
            driveState
        ),
        feedback.statusword
    );


    return boot_fail(
        boot,
        BOOT_ERROR_DRIVE_QUICK_STOP,
        slave
    );
}

if (
    driveState ==
    CIA402_STATE_UNKNOWN
)
{
    printf(
        "BOOT: Slave %d reported an UNKNOWN CiA-402 state\n"
        "      Statusword : 0x%04X\n",
        slave,
        feedback.statusword
    );


    return boot_fail(
        boot,
        BOOT_ERROR_DRIVE_STATE,
        slave
    );
}


                if (
                    driveState !=
                    CIA402_STATE_OPERATION_ENABLED
                )
                {
                    allOperationEnabled =
                        false;
                }


                /*
                 * Existing CiA-402 module decides the appropriate next
                 * Controlword:
                 *
                 * Switch On Disabled -> Shutdown
                 * Ready To Switch On -> Switch On
                 * Switched On -> Enable Operation
                 */
uint16_t controlword =
    CIA402_CONTROLWORD_DISABLE_VOLTAGE;


if (
    !cia402_get_enable_controlword(
        driveState,
        &controlword
    )
)
{
    printf(
        "BOOT: Slave %d cannot be automatically enabled\n"
        "      CiA-402 state: %s\n"
        "      Statusword   : 0x%04X\n",
        slave,
        cia402_state_name(
            driveState
        ),
        feedback.statusword
    );


    return boot_fail(
    boot,
    BOOT_ERROR_DRIVE_STATE,
    slave
);
}

                /*
                 * While enabling CSP, command the drive to HOLD its current
                 * measured position.
                 *
                 * This prevents BOOT from intentionally commanding motion.
                 */
                A6ECPDOCommand command =
                {
                    .controlword =
                        controlword,

                    .targetPosition =
                        feedback.actualPosition
                };


                a6ec_write_command(
                    slave,
                    &command
                );
            }


            /*
             * Send the Controlwords and receive the resulting Statuswords.
             */
            int actualWkc =
                ethercat_master_exchange();


            int expectedWkc =
                ethercat_master_expected_wkc();

if (
    actualWkc <
    expectedWkc
)
{
    printf(
        "BOOT: WKC mismatch while enabling drives. "
        "Expected=%d Actual=%d\n",
        expectedWkc,
        actualWkc
    );


    if (
        !boot_attempt_ethercat_recovery(
            boot,
            ethercatConfig->expectedSlaveCount
        )
    )
    {
        return boot_fail(
            boot,
            BOOT_ERROR_CYCLIC_COMMUNICATION,
            0
        );
    }


    return
        STATE_STEP_RUNNING;
}

            /*
             * If all six were already Operation Enabled when this phase
             * started, the condition is satisfied.
             */
            if (allOperationEnabled)
            {
                boot_advance(
                    boot,
                    BOOT_PHASE_VERIFY_POSITION_FEEDBACK
                );

                return
                    STATE_STEP_RUNNING;
            }

/*
 * Timeout protection for drives that remain in a valid startup state
 * but fail to reach Operation Enabled within the allowed time.
 */
            boot->stableCycles++;


            if (
                boot->stableCycles >=
                BOOT_DRIVE_ENABLE_TIMEOUT_CYCLES
            )
            {
                return boot_fail(
                    boot,
                    BOOT_ERROR_DRIVE_ENABLE,
                    0
                );
            }


            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * VERIFY POSITION FEEDBACK
         * ====================================================================
         */

        case BOOT_PHASE_VERIFY_POSITION_FEEDBACK:
        {
            for (
                int slave = 1;
                slave <= ethercatConfig->expectedSlaveCount;
                slave++
            )
            {
                /*
                 * At the moment a6ec_read_feedback() does not return a status.
                 *
                 * Therefore first confirm that this slave actually owns a
                 * mapped input PDO area.
                 */
                if (
                    ethercat_master_slave_inputs(
                        slave
                    ) == NULL
                )
                {
                    return boot_fail(
                        boot,
                        BOOT_ERROR_POSITION_FEEDBACK,
                        slave
                    );
                }


                A6ECPDOFeedback feedback;


                a6ec_read_feedback(
                    slave,
                    &feedback
                );


                /*
                 * Simply reading actualPosition here is intentional.
                 *
                 * A value of zero is perfectly valid and must NOT be treated
                 * as communication failure.
                 *
                 * Communication validity is established by the PDO mapping
                 * and WKC checks performed above.
                 */
                (void)feedback.actualPosition;
            }


            boot_advance(
                boot,
                BOOT_PHASE_VERIFY_SAFETY
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * VERIFY SAFETY FEEDBACK
         * ====================================================================
         */

        case BOOT_PHASE_VERIFY_SAFETY:
        {
            /*
             * TODO:
             *
             * Replace this phase when the electrical safety-feedback
             * interface exists.
             *
             * Eventually:
             *
             *      E-Stop released
             *      AND
             *      safety relay healthy
             *      AND
             *      safety chain closed
             *
             * For the current PC/KickCAT simulation there is no physical
             * safety feedback, so this condition temporarily passes.
             */

            boot_advance(
                boot,
                BOOT_PHASE_COMPLETE
            );

            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * BOOT COMPLETE
         * ====================================================================
         */

        case BOOT_PHASE_COMPLETE:
        {
            return
                STATE_STEP_COMPLETE;
        }


        /* ====================================================================
         * BOOT FAILED
         * ====================================================================
         */

        case BOOT_PHASE_FAILED:
        {
            return
                STATE_STEP_FAILED;
        }


        /* ====================================================================
         * INVALID STATE
         * ====================================================================
         */

        default:
        {
            return boot_fail(
                boot,
                BOOT_ERROR_MASTER_INIT,
                0
            );
        }
    }
}