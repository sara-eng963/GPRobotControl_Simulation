#include "state_homing.h"
#include "../../ServoDrive/CiA402/cia402.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define HOMING_CYCLIC_FEEDBACK_TIMEOUT_MS 100U

static StateStepResult homing_fail(
    HomingState *homing,
    HomingError error,
    int failed_axis
)
{
    homing->error = error;
    homing->failedAxis = failed_axis;
    homing->phase = HOMING_PHASE_FAILED;
    return STATE_STEP_FAILED;
}

static bool homing_config_valid(
    const HomingConfig *config,
    uint32_t *period_ms
)
{
    if (
        config == NULL ||
        period_ms == NULL ||
        !isfinite(config->duration) ||
        !isfinite(config->dt) ||
        !isfinite(config->positionTolerance) ||
        config->duration <= 0.0 ||
        config->dt <= 0.0 ||
        config->positionTolerance <= 0.0 ||
        config->requiredStableCycles == 0U ||
        config->maxVerificationCycles == 0U
    )
    {
        return false;
    }

    const double requested_ms = (double)config->dt * 1000.0;

    if (
        !isfinite(requested_ms) ||
        requested_ms < 1.0 ||
        requested_ms > (double)UINT32_MAX
    )
    {
        return false;
    }

    *period_ms = (uint32_t)llround(requested_ms);
    return *period_ms > 0U;
}

static bool position_scales_valid(
    const AvatarMPositionScale position_scales[ROBOT_DOF]
)
{
    if (position_scales == NULL)
    {
        return false;
    }

    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        if (!avatar_m_position_scale_valid(&position_scales[i]))
        {
            return false;
        }
    }

    return true;
}

static bool master_layout_valid(const CanopenMaster *master)
{
    return
        master != NULL &&
        master->initialized &&
        master->node_count == ROBOT_DOF;
}

static int first_disabled_axis(const CanopenMaster *master)
{
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        const AvatarMDrive *drive = canopen_master_drive(master, i);

        if (
            drive == NULL ||
            drive->cia402_state != CIA402_STATE_OPERATION_ENABLED
        )
        {
            return (int)i + 1;
        }
    }

    return 0;
}

static HomingError network_status_error(
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
        return HOMING_ERROR_COMMUNICATION;
    }

    if (!canopen_master_all_feedback_valid(master))
    {
        return HOMING_ERROR_POSITION_FEEDBACK;
    }

    const int disabled_axis = first_disabled_axis(master);

    if (disabled_axis != 0)
    {
        *failed_axis = disabled_axis;
        return HOMING_ERROR_DRIVE_NOT_ENABLED;
    }

    return HOMING_ERROR_NONE;
}

static void capture_tpdo_counts(
    HomingState *homing,
    const CanopenMaster *master
)
{
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        homing->commandTpdoCount[i] =
            canopen_master_tpdo_rx_count(master, i);
    }
}

static bool all_command_feedback_arrived(
    const HomingState *homing,
    const CanopenMaster *master
)
{
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        if (
            canopen_master_tpdo_rx_count(master, i) <=
            homing->commandTpdoCount[i]
        )
        {
            return false;
        }
    }

    return true;
}

static StateStepResult wait_for_command_feedback(
    HomingState *homing,
    const CanopenMaster *master,
    uint32_t now_ms,
    bool *ready
)
{
    *ready = false;

    if (!homing->awaitingFeedback)
    {
        *ready = true;
        return STATE_STEP_RUNNING;
    }

    if (all_command_feedback_arrived(homing, master))
    {
        homing->awaitingFeedback = false;
        *ready = true;
        return STATE_STEP_RUNNING;
    }

    if (
        (uint32_t)(now_ms - homing->lastCommandMs) >=
        HOMING_CYCLIC_FEEDBACK_TIMEOUT_MS
    )
    {
        return homing_fail(
            homing,
            HOMING_ERROR_CYCLIC_FEEDBACK,
            0
        );
    }

    return STATE_STEP_RUNNING;
}

static bool command_due(
    const HomingState *homing,
    uint32_t now_ms
)
{
    return
        !homing->commandClockStarted ||
        (uint32_t)(now_ms - homing->lastCommandMs) >=
            homing->commandPeriodMs;
}

static bool build_target_positions(
    const JointVector *q,
    const AvatarMPositionScale position_scales[ROBOT_DOF],
    int32_t targets[ROBOT_DOF],
    int *failed_axis
)
{
    for (size_t i = 0U; i < ROBOT_DOF; ++i)
    {
        if (!avatar_m_joint_rad_to_position_units(
                &position_scales[i],
                q->q[i],
                &targets[i]))
        {
            *failed_axis = (int)i + 1;
            return false;
        }
    }

    return true;
}

static StateStepResult send_joint_cycle(
    HomingState *homing,
    CanopenMaster *master,
    const JointVector *q,
    const AvatarMPositionScale position_scales[ROBOT_DOF],
    uint32_t now_ms,
    bool count_trajectory_sample
)
{
    int32_t targets[ROBOT_DOF];
    int failed_axis = 0;

    if (!build_target_positions(
            q,
            position_scales,
            targets,
            &failed_axis))
    {
        return homing_fail(
            homing,
            HOMING_ERROR_POSITION_CONVERSION,
            failed_axis
        );
    }

    capture_tpdo_counts(homing, master);

    if (!canopen_master_send_target_cycle(
            master,
            targets,
            ROBOT_DOF))
    {
        return homing_fail(
            homing,
            HOMING_ERROR_COMMUNICATION,
            0
        );
    }

    homing->lastCommandMs = now_ms;
    homing->commandClockStarted = true;
    homing->awaitingFeedback = true;

    if (count_trajectory_sample)
    {
        homing->samplesSent++;
    }

    return STATE_STEP_RUNNING;
}

void state_homing_enter(HomingState *homing)
{
    if (homing == NULL)
    {
        return;
    }

    memset(homing, 0, sizeof(*homing));
    homing->phase = HOMING_PHASE_INIT;
    homing->error = HOMING_ERROR_NONE;
}

StateStepResult state_homing_step(
    HomingState *homing,
    const HomingConfig *config,
    const RobotConfig *robot,
    CanopenMaster *master,
    const AvatarMPositionScale position_scales[ROBOT_DOF],
    uint32_t now_ms
)
{
    if (
        homing == NULL ||
        robot == NULL ||
        master == NULL ||
        position_scales == NULL
    )
    {
        if (homing != NULL)
        {
            return homing_fail(
                homing,
                HOMING_ERROR_INVALID_CONFIG,
                0
            );
        }

        return STATE_STEP_FAILED;
    }

    if (!canopen_master_poll(master, now_ms))
    {
        return homing_fail(
            homing,
            HOMING_ERROR_COMMUNICATION,
            0
        );
    }

    switch (homing->phase)
    {
        case HOMING_PHASE_INIT:
        {
            if (
                !homing_config_valid(config, &homing->commandPeriodMs) ||
                !position_scales_valid(position_scales) ||
                !master_layout_valid(master)
            )
            {
                return homing_fail(
                    homing,
                    HOMING_ERROR_INVALID_CONFIG,
                    0
                );
            }

            if (!robot->configuration.homeDefined)
            {
                return homing_fail(
                    homing,
                    HOMING_ERROR_HOME_NOT_DEFINED,
                    0
                );
            }

            for (int joint = 0; joint < ROBOT_DOF; ++joint)
            {
                const real_t q_home = robot->configuration.home[joint];

                if (
                    !isfinite(q_home) ||
                    q_home < robot->limits.qMin[joint] ||
                    q_home > robot->limits.qMax[joint]
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_HOME_LIMIT,
                        joint + 1
                    );
                }

                homing->qHome.q[joint] = q_home;
            }

            homing->phase = HOMING_PHASE_READ_POSITION;
            return STATE_STEP_RUNNING;
        }

        case HOMING_PHASE_READ_POSITION:
        {
            int failed_axis = 0;
            const HomingError status =
                network_status_error(master, now_ms, &failed_axis);

            if (status != HOMING_ERROR_NONE)
            {
                return homing_fail(homing, status, failed_axis);
            }

            for (size_t i = 0U; i < ROBOT_DOF; ++i)
            {
                const AvatarMDrive *drive = canopen_master_drive(master, i);

                if (
                    drive == NULL ||
                    !drive->feedback_valid
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_POSITION_FEEDBACK,
                        (int)i + 1
                    );
                }

                double q_actual = 0.0;

                if (!avatar_m_position_units_to_joint_rad(
                        &position_scales[i],
                        drive->feedback.actual_position,
                        &q_actual))
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_POSITION_CONVERSION,
                        (int)i + 1
                    );
                }

                homing->qStart.q[i] = (real_t)q_actual;
            }

            homing->phase = HOMING_PHASE_PREPARE_TRAJECTORY;
            return STATE_STEP_RUNNING;
        }

        case HOMING_PHASE_PREPARE_TRAJECTORY:
        {
            if (!joint_trajectory_init(
                    &homing->trajectory,
                    &homing->qStart,
                    &homing->qHome,
                    config->duration,
                    config->dt))
            {
                return homing_fail(
                    homing,
                    HOMING_ERROR_TRAJECTORY,
                    0
                );
            }

            for (int joint = 0; joint < ROBOT_DOF; ++joint)
            {
                if (
                    homing->trajectory.info.peakJointVelocity.q[joint] >
                    robot->limits.qdMax[joint]
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_TRAJECTORY_LIMIT,
                        joint + 1
                    );
                }

                if (
                    robot->limits.qddMaxDefined &&
                    homing->trajectory.info.peakJointAcceleration.q[joint] >
                    robot->limits.qddMax[joint]
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_TRAJECTORY_LIMIT,
                        joint + 1
                    );
                }
            }

            homing->commandClockStarted = false;
            homing->awaitingFeedback = false;
            homing->phase = HOMING_PHASE_EXECUTE_TRAJECTORY;
            return STATE_STEP_RUNNING;
        }

        case HOMING_PHASE_EXECUTE_TRAJECTORY:
        {
            int failed_axis = 0;
            const HomingError status =
                network_status_error(master, now_ms, &failed_axis);

            if (status != HOMING_ERROR_NONE)
            {
                return homing_fail(homing, status, failed_axis);
            }

            bool feedback_ready = false;
            const StateStepResult feedback_result =
                wait_for_command_feedback(
                    homing,
                    master,
                    now_ms,
                    &feedback_ready
                );

            if (feedback_result == STATE_STEP_FAILED)
            {
                return feedback_result;
            }

            if (!feedback_ready || !command_due(homing, now_ms))
            {
                return STATE_STEP_RUNNING;
            }

            JointTrajectorySample sample;

            if (!joint_trajectory_next(&homing->trajectory, &sample))
            {
                if (joint_trajectory_is_finished(&homing->trajectory))
                {
                    homing->commandClockStarted = false;
                    homing->awaitingFeedback = false;
                    homing->phase = HOMING_PHASE_VERIFY_HOME;
                    return STATE_STEP_RUNNING;
                }

                return homing_fail(
                    homing,
                    HOMING_ERROR_TRAJECTORY,
                    0
                );
            }

            return send_joint_cycle(
                homing,
                master,
                &sample.q,
                position_scales,
                now_ms,
                true
            );
        }

        case HOMING_PHASE_VERIFY_HOME:
        {
            int failed_axis = 0;
            const HomingError status =
                network_status_error(master, now_ms, &failed_axis);

            if (status != HOMING_ERROR_NONE)
            {
                return homing_fail(homing, status, failed_axis);
            }

            if (homing->awaitingFeedback)
            {
                bool feedback_ready = false;
                const StateStepResult feedback_result =
                    wait_for_command_feedback(
                        homing,
                        master,
                        now_ms,
                        &feedback_ready
                    );

                if (feedback_result == STATE_STEP_FAILED)
                {
                    return feedback_result;
                }

                if (!feedback_ready)
                {
                    return STATE_STEP_RUNNING;
                }

                bool all_within_tolerance = true;

                for (size_t i = 0U; i < ROBOT_DOF; ++i)
                {
                    const AvatarMDrive *drive =
                        canopen_master_drive(master, i);

                    double q_actual = 0.0;

                    if (
                        drive == NULL ||
                        !drive->feedback_valid
                    )
                    {
                        return homing_fail(
                            homing,
                            HOMING_ERROR_POSITION_FEEDBACK,
                            (int)i + 1
                        );
                    }

                    if (!avatar_m_position_units_to_joint_rad(
                            &position_scales[i],
                            drive->feedback.actual_position,
                            &q_actual))
                    {
                        return homing_fail(
                            homing,
                            HOMING_ERROR_POSITION_CONVERSION,
                            (int)i + 1
                        );
                    }

                    if (
                        fabs(q_actual - (double)homing->qHome.q[i]) >
                        (double)config->positionTolerance
                    )
                    {
                        all_within_tolerance = false;
                    }
                }

                homing->verificationCycles++;

                if (all_within_tolerance)
                {
                    homing->stableCycles++;
                }
                else
                {
                    homing->stableCycles = 0U;
                }

                if (
                    homing->stableCycles >=
                    config->requiredStableCycles
                )
                {
                    homing->phase = HOMING_PHASE_COMPLETE;
                    return STATE_STEP_RUNNING;
                }

                if (
                    homing->verificationCycles >=
                    config->maxVerificationCycles
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_HOME_TIMEOUT,
                        0
                    );
                }
            }

            if (!command_due(homing, now_ms))
            {
                return STATE_STEP_RUNNING;
            }

            return send_joint_cycle(
                homing,
                master,
                &homing->qHome,
                position_scales,
                now_ms,
                false
            );
        }

        case HOMING_PHASE_COMPLETE:
            return STATE_STEP_COMPLETE;

        case HOMING_PHASE_FAILED:
            return STATE_STEP_FAILED;

        default:
            return homing_fail(
                homing,
                HOMING_ERROR_INVALID_CONFIG,
                0
            );
    }
}
