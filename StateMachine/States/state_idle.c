#include "state_idle.h"

#include "../../ServoDrive/CiA402/cia402.h"

#include <stddef.h>
#include <string.h>

#define IDLE_COMMAND_PERIOD_MS          2U
#define IDLE_CYCLIC_FEEDBACK_TIMEOUT_MS 100U

static StateStepResult idle_fail(
    IdleState *idle,
    IdleError error,
    int failed_axis
)
{
    idle->error = error;
    idle->failedAxis = failed_axis;
    idle->phase = IDLE_PHASE_FAILED;
    return STATE_STEP_FAILED;
}

static bool master_layout_valid(
    const CanopenMaster *master
)
{
    return
        master != NULL &&
        master->initialized &&
        master->node_count == ROBOT_DOF;
}

static int first_disabled_axis(
    const CanopenMaster *master
)
{
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        const AvatarMDrive *drive =
            canopen_master_drive(master, i);

        if (
            drive == NULL ||
            drive->cia402_state !=
                CIA402_STATE_OPERATION_ENABLED
        )
        {
            return (int)i + 1;
        }
    }

    return 0;
}

static IdleError idle_network_status(
    const CanopenMaster *master,
    uint32_t now_ms,
    int *failed_axis
)
{
    *failed_axis = 0;

    if (
        !master_layout_valid(master) ||
        !canopen_master_healthy(master, now_ms)
    )
    {
        return IDLE_ERROR_COMMUNICATION;
    }

    if (!canopen_master_all_feedback_valid(master))
    {
        return IDLE_ERROR_POSITION_FEEDBACK;
    }

    const int disabled_axis =
        first_disabled_axis(master);

    if (disabled_axis != 0)
    {
        *failed_axis = disabled_axis;
        return IDLE_ERROR_DRIVE_NOT_ENABLED;
    }

    return IDLE_ERROR_NONE;
}

static void capture_tpdo_counts(
    IdleState *idle,
    const CanopenMaster *master
)
{
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        idle->commandTpdoCount[i] =
            canopen_master_tpdo_rx_count(master, i);
    }
}

static bool all_command_feedback_arrived(
    const IdleState *idle,
    const CanopenMaster *master
)
{
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        if (
            canopen_master_tpdo_rx_count(master, i) <=
            idle->commandTpdoCount[i]
        )
        {
            return false;
        }
    }

    return true;
}

static bool command_due(
    const IdleState *idle,
    uint32_t now_ms
)
{
    return
        !idle->commandClockStarted ||
        (uint32_t)(now_ms - idle->lastCommandMs) >=
            idle->commandPeriodMs;
}

static StateStepResult send_hold_cycle(
    IdleState *idle,
    CanopenMaster *master,
    uint32_t now_ms
)
{
    capture_tpdo_counts(idle, master);

    if (!canopen_master_send_target_cycle(
            master,
            idle->holdPositionUnits,
            ROBOT_DOF))
    {
        return idle_fail(
            idle,
            IDLE_ERROR_COMMUNICATION,
            0
        );
    }

    idle->lastCommandMs = now_ms;
    idle->commandClockStarted = true;
    idle->awaitingFeedback = true;

    return STATE_STEP_RUNNING;
}

void state_idle_enter(
    IdleState *idle
)
{
    if (idle == NULL)
    {
        return;
    }

    memset(idle, 0, sizeof(*idle));

    idle->phase = IDLE_PHASE_INIT;
    idle->error = IDLE_ERROR_NONE;
    idle->exitCommand = IDLE_COMMAND_NONE;
    idle->commandPeriodMs = IDLE_COMMAND_PERIOD_MS;
}

StateStepResult state_idle_step(
    IdleState *idle,
    IdleCommand command,
    CanopenMaster *master,
    uint32_t now_ms
)
{
    if (
        idle == NULL ||
        master == NULL
    )
    {
        if (idle != NULL)
        {
            return idle_fail(
                idle,
                IDLE_ERROR_COMMUNICATION,
                0
            );
        }

        return STATE_STEP_FAILED;
    }

    if (!canopen_master_poll(master, now_ms))
    {
        return idle_fail(
            idle,
            IDLE_ERROR_COMMUNICATION,
            0
        );
    }

    switch (idle->phase)
    {
        case IDLE_PHASE_INIT:
        {
            int failed_axis = 0;

            const IdleError status =
                idle_network_status(
                    master,
                    now_ms,
                    &failed_axis
                );

            if (status != IDLE_ERROR_NONE)
            {
                return idle_fail(
                    idle,
                    status,
                    failed_axis
                );
            }

            for (size_t i = 0U; i < ROBOT_DOF; ++i)
            {
                const AvatarMDrive *drive =
                    canopen_master_drive(master, i);

                if (
                    drive == NULL ||
                    !drive->feedback_valid
                )
                {
                    return idle_fail(
                        idle,
                        IDLE_ERROR_POSITION_FEEDBACK,
                        (int)i + 1
                    );
                }

                idle->holdPositionUnits[i] =
                    drive->feedback.actual_position;
            }

            idle->phase = IDLE_PHASE_HOLDING;

            return STATE_STEP_RUNNING;
        }

        case IDLE_PHASE_HOLDING:
        {
            int failed_axis = 0;

            const IdleError status =
                idle_network_status(
                    master,
                    now_ms,
                    &failed_axis
                );

            if (status != IDLE_ERROR_NONE)
            {
                return idle_fail(
                    idle,
                    status,
                    failed_axis
                );
            }

            if (idle->awaitingFeedback)
            {
                if (all_command_feedback_arrived(
                        idle,
                        master))
                {
                    idle->awaitingFeedback = false;
                    idle->cyclesHeld++;
                }
                else
                {
                    if (
                        (uint32_t)(
                            now_ms -
                            idle->lastCommandMs
                        ) >=
                        IDLE_CYCLIC_FEEDBACK_TIMEOUT_MS
                    )
                    {
                        return idle_fail(
                            idle,
                            IDLE_ERROR_CYCLIC_FEEDBACK,
                            0
                        );
                    }

                    return STATE_STEP_RUNNING;
                }
            }

            switch (command)
            {
                case IDLE_COMMAND_NONE:
                    break;

                case IDLE_COMMAND_TEACH:
                case IDLE_COMMAND_REPLAY:
                    idle->exitCommand = command;
                    idle->phase = IDLE_PHASE_COMPLETE;
                    return STATE_STEP_RUNNING;

                default:
                    return idle_fail(
                        idle,
                        IDLE_ERROR_INVALID_COMMAND,
                        0
                    );
            }

            if (!command_due(idle, now_ms))
            {
                return STATE_STEP_RUNNING;
            }

            return send_hold_cycle(
                idle,
                master,
                now_ms
            );
        }

        case IDLE_PHASE_COMPLETE:
            return STATE_STEP_COMPLETE;

        case IDLE_PHASE_FAILED:
            return STATE_STEP_FAILED;

        default:
            return idle_fail(
                idle,
                IDLE_ERROR_INVALID_COMMAND,
                0
            );
    }
}
