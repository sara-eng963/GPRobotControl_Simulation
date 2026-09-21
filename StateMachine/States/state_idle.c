#include "state_idle.h"

#include "../../EtherCATComm/ethercat_master.h"

#include "../../ServoDrive/A6EC/a6ec_drive.h"
#include "../../ServoDrive/CiA402/cia402.h"

#include <stddef.h>


/* ============================================================================
 * INTERNAL HELPERS
 * ============================================================================
 */

static void idle_advance(
    IdleState *idle,
    IdlePhase nextPhase
)
{
    idle->phase =
        nextPhase;
}


static StateStepResult idle_fail(
    IdleState *idle,
    IdleError error,
    int failedAxis
)
{
    idle->error =
        error;

    idle->failedAxis =
        failedAxis;

    idle->phase =
        IDLE_PHASE_FAILED;

    return
        STATE_STEP_FAILED;
}


/* ============================================================================
 * ENTER IDLE
 * ============================================================================
 */

void state_idle_enter(
    IdleState *idle
)
{
    if (idle == NULL)
    {
        return;
    }


    idle->phase =
        IDLE_PHASE_INIT;

    idle->error =
        IDLE_ERROR_NONE;

    idle->failedAxis =
        0;

    idle->exitCommand =
        IDLE_COMMAND_NONE;

    idle->cyclesHeld =
        0U;


    for (
        int joint = 0;
        joint < ROBOT_DOF;
        joint++
    )
    {
        idle->holdPositionUnits[joint] =
            0;
    }
}


/* ============================================================================
 * IDLE STATE
 * ============================================================================
 */

StateStepResult state_idle_step(
    IdleState *idle,
    IdleCommand command
)
{
    if (idle == NULL)
    {
        return
            STATE_STEP_FAILED;
    }


    switch (idle->phase)
    {
        /* ====================================================================
         * INITIALIZE IDLE
         * ====================================================================
         *
         * Capture the exact current encoder positions.
         *
         * These become the stationary CSP targets used while IDLE is active.
         */

        case IDLE_PHASE_INIT:
        {
            for (
                int slave = 1;
                slave <= ROBOT_DOF;
                slave++
            )
            {
                /*
                 * IDLE requires both mapped input and output PDO areas.
                 */
                if (
                    ethercat_master_slave_inputs(
                        slave
                    ) == NULL
                    ||
                    ethercat_master_slave_outputs(
                        slave
                    ) == NULL
                )
                {
                    return idle_fail(
                        idle,
                        IDLE_ERROR_PDO_UNAVAILABLE,
                        slave
                    );
                }


                A6ECPDOFeedback feedback;


                a6ec_read_feedback(
                    slave,
                    &feedback
                );


                uint16_t driveState =
                    cia402_get_state(
                        feedback.statusword
                    );


                /*
                 * BOOT and HOMING should already have left every drive
                 * Operation Enabled.
                 */
                if (
                    driveState !=
                    CIA402_STATE_OPERATION_ENABLED
                )
                {
                    return idle_fail(
                        idle,
                        IDLE_ERROR_DRIVE_NOT_ENABLED,
                        slave
                    );
                }


                /*
                 * Capture exact measured position as the IDLE hold target.
                 */
                idle->holdPositionUnits[
                    slave - 1
                ] =
                    feedback.actualPosition;
            }


            /*
             * BOOT should already have established a valid expected WKC.
             */
            if (
                ethercat_master_expected_wkc()
                <=
                0
            )
            {
                return idle_fail(
                    idle,
                    IDLE_ERROR_WKC,
                    0
                );
            }


            idle_advance(
                idle,
                IDLE_PHASE_HOLDING
            );


            return
                STATE_STEP_RUNNING;
        }


        /* ====================================================================
         * HOLD POSITION
         * ====================================================================
         *
         * This is where IDLE normally remains indefinitely.
         *
         * Every cycle:
         *
         *      read drive state
         *      command q_hold
         *      exchange PDOs
         *      verify WKC
         *      check safety
         *      check operator command
         */

        case IDLE_PHASE_HOLDING:
        {
            for (
                int slave = 1;
                slave <= ROBOT_DOF;
                slave++
            )
            {
                if (
                    ethercat_master_slave_inputs(
                        slave
                    ) == NULL
                    ||
                    ethercat_master_slave_outputs(
                        slave
                    ) == NULL
                )
                {
                    return idle_fail(
                        idle,
                        IDLE_ERROR_PDO_UNAVAILABLE,
                        slave
                    );
                }


                A6ECPDOFeedback feedback;


                a6ec_read_feedback(
                    slave,
                    &feedback
                );


                uint16_t driveState =
                    cia402_get_state(
                        feedback.statusword
                    );


                /*
                 * A drive leaving Operation Enabled while IDLE is
                 * considered an IDLE failure.
                 */
                if (
                    driveState !=
                    CIA402_STATE_OPERATION_ENABLED
                )
                {
                    return idle_fail(
                        idle,
                        IDLE_ERROR_DRIVE_NOT_ENABLED,
                        slave
                    );
                }


                /*
                 * Keep each A6-EC servo in CSP and continuously command
                 * the exact position captured when IDLE began.
                 */
                A6ECPDOCommand holdCommand =
                {
                    .controlword =
                        CIA402_CONTROLWORD_ENABLE_OPERATION,

                    .targetPosition =
                        idle->holdPositionUnits[
                            slave - 1
                        ]
                };


                a6ec_write_command(
                    slave,
                    &holdCommand
                );
            }


            /* ---------------------------------------------------------------
             * SEND / RECEIVE ONE ETHERCAT CYCLE
             * ---------------------------------------------------------------
             */

            int actualWkc =
                ethercat_master_exchange();


            int expectedWkc =
                ethercat_master_expected_wkc();


            if (
                expectedWkc <= 0 ||
                actualWkc < expectedWkc
            )
            {
                return idle_fail(
                    idle,
                    IDLE_ERROR_WKC,
                    0
                );
            }


            idle->cyclesHeld++;


            /* ---------------------------------------------------------------
             * SAFETY CHECK
             * ---------------------------------------------------------------
             *
             * TODO:
             *
             * Replace this temporary pass when the real electrical safety
             * feedback interface exists.
             *
             * Eventually IDLE should monitor:
             *
             *      E-Stop
             *      safety relay
             *      safety chain
             *      zone / protective-stop feedback
             *
             * For the current KickCAT simulation there is no physical
             * safety input.
             */


            /* ---------------------------------------------------------------
             * OPERATOR COMMAND
             * ---------------------------------------------------------------
             */

            switch (command)
            {
                case IDLE_COMMAND_NONE:
                {
                    /*
                     * No command.
                     *
                     * Stay in IDLE and continue holding.
                     */
                    return
                        STATE_STEP_RUNNING;
                }


                case IDLE_COMMAND_TEACH:
                case IDLE_COMMAND_REPLAY:
                {
                    idle->exitCommand =
                        command;


                    idle_advance(
                        idle,
                        IDLE_PHASE_COMPLETE
                    );


                    return
                        STATE_STEP_RUNNING;
                }


                default:
                {
                    return idle_fail(
                        idle,
                        IDLE_ERROR_INVALID_COMMAND,
                        0
                    );
                }
            }
        }


        /* ====================================================================
         * IDLE COMPLETE
         * ====================================================================
         *
         * A valid operator command was received.
         *
         * The outer robot FSM will later inspect:
         *
         *      idle->exitCommand
         *
         * and decide which state comes next.
         */

        case IDLE_PHASE_COMPLETE:
        {
            return
                STATE_STEP_COMPLETE;
        }


        /* ====================================================================
         * IDLE FAILED
         * ====================================================================
         */

        case IDLE_PHASE_FAILED:
        {
            return
                STATE_STEP_FAILED;
        }


        /* ====================================================================
         * INVALID PHASE
         * ====================================================================
         */

        default:
        {
            return idle_fail(
                idle,
                IDLE_ERROR_INVALID_COMMAND,
                0
            );
        }
    }
}