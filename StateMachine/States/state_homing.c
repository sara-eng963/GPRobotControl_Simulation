#include "state_homing.h"

#include "../../EtherCATComm/ethercat_master.h"

#include "../../ServoDrive/A6EC/a6ec_drive.h"
#include "../../ServoDrive/CiA402/cia402.h"

#include <math.h>
#include <stddef.h>


static void homing_advance(
    HomingState *homing,
    HomingPhase nextPhase
)
{
    homing->phase =
        nextPhase;
}


static StateStepResult homing_fail(
    HomingState *homing,
    HomingError error,
    int failedAxis
)
{
    homing->error =
        error;

    homing->failedAxis =
        failedAxis;

    homing->phase =
        HOMING_PHASE_FAILED;

    return
        STATE_STEP_FAILED;
}


static bool homing_config_valid(
    const HomingConfig *config
)
{
    return
        config != NULL &&
        config->duration > 0.0 &&
        config->dt > 0.0 &&
        config->positionTolerance > 0.0 &&
        config->requiredStableCycles > 0U &&
        config->maxVerificationCycles > 0U;
}


static bool homing_wkc_valid(void)
{
    int expectedWkc =
        ethercat_master_expected_wkc();

    if (expectedWkc <= 0)
    {
        return false;
    }

    int actualWkc =
        ethercat_master_exchange();

    return
        actualWkc >= expectedWkc;
}


void state_homing_enter(
    HomingState *homing
)
{
    if (homing == NULL)
    {
        return;
    }

    homing->phase =
        HOMING_PHASE_INIT;

    homing->error =
        HOMING_ERROR_NONE;

    homing->failedAxis =
        0;

    homing->samplesSent =
        0U;

    homing->stableCycles =
        0U;

    homing->verificationCycles =
        0U;
}


StateStepResult state_homing_step(
    HomingState *homing,
    const HomingConfig *config,
    const RobotConfig *robot
)
{
    if (
        homing == NULL ||
        robot == NULL
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

        return
            STATE_STEP_FAILED;
    }


    switch (homing->phase)
    {
        case HOMING_PHASE_INIT:
        {
            if (!homing_config_valid(config))
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


            for (
                int joint = 0;
                joint < ROBOT_DOF;
                joint++
            )
            {
                real_t qHome =
                    robot->configuration.home[joint];


                if (
                    !isfinite(qHome) ||
                    qHome < robot->limits.qMin[joint] ||
                    qHome > robot->limits.qMax[joint]
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_HOME_LIMIT,
                        joint + 1
                    );
                }


                homing->qHome.q[joint] =
                    qHome;
            }


            homing_advance(
                homing,
                HOMING_PHASE_READ_POSITION
            );

            return
                STATE_STEP_RUNNING;
        }


        case HOMING_PHASE_READ_POSITION:
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
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_POSITION_FEEDBACK,
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


                if (
                    driveState !=
                    CIA402_STATE_OPERATION_ENABLED
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_DRIVE_NOT_ENABLED,
                        slave
                    );
                }


                homing->qStart.q[slave - 1] =
                    a6ec_position_units_to_joint_rad(
                        feedback.actualPosition
                    );
            }


            homing_advance(
                homing,
                HOMING_PHASE_PREPARE_TRAJECTORY
            );

            return
                STATE_STEP_RUNNING;
        }


        case HOMING_PHASE_PREPARE_TRAJECTORY:
        {
            if (
                !joint_trajectory_init(
                    &homing->trajectory,
                    &homing->qStart,
                    &homing->qHome,
                    config->duration,
                    config->dt
                )
            )
            {
                return homing_fail(
                    homing,
                    HOMING_ERROR_TRAJECTORY,
                    0
                );
            }


            for (
                int joint = 0;
                joint < ROBOT_DOF;
                joint++
            )
            {
                if (
                    homing->trajectory
                        .info
                        .peakJointVelocity
                        .q[joint]
                    >
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
                    homing->trajectory
                        .info
                        .peakJointAcceleration
                        .q[joint]
                    >
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


            homing_advance(
                homing,
                HOMING_PHASE_EXECUTE_TRAJECTORY
            );

            return
                STATE_STEP_RUNNING;
        }


        case HOMING_PHASE_EXECUTE_TRAJECTORY:
        {
            JointTrajectorySample sample;


            if (
                !joint_trajectory_next(
                    &homing->trajectory,
                    &sample
                )
            )
            {
                if (
                    joint_trajectory_is_finished(
                        &homing->trajectory
                    )
                )
                {
                    homing_advance(
                        homing,
                        HOMING_PHASE_VERIFY_HOME
                    );

                    return
                        STATE_STEP_RUNNING;
                }


                return homing_fail(
                    homing,
                    HOMING_ERROR_TRAJECTORY,
                    0
                );
            }


            for (
                int slave = 1;
                slave <= ROBOT_DOF;
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
                    CIA402_STATE_OPERATION_ENABLED
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_DRIVE_NOT_ENABLED,
                        slave
                    );
                }


                A6ECPDOCommand command =
                {
                    .controlword =
                        CIA402_CONTROLWORD_ENABLE_OPERATION,

                    .targetPosition =
                        a6ec_joint_rad_to_position_units(
                            sample.q.q[slave - 1]
                        )
                };


                a6ec_write_command(
                    slave,
                    &command
                );
            }


            if (!homing_wkc_valid())
            {
                return homing_fail(
                    homing,
                    HOMING_ERROR_WKC,
                    0
                );
            }


            homing->samplesSent++;


            if (
                joint_trajectory_is_finished(
                    &homing->trajectory
                )
            )
            {
                homing_advance(
                    homing,
                    HOMING_PHASE_VERIFY_HOME
                );
            }


            return
                STATE_STEP_RUNNING;
        }


        case HOMING_PHASE_VERIFY_HOME:
        {
            bool allWithinTolerance =
                true;


            for (
                int slave = 1;
                slave <= ROBOT_DOF;
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
                    CIA402_STATE_OPERATION_ENABLED
                )
                {
                    return homing_fail(
                        homing,
                        HOMING_ERROR_DRIVE_NOT_ENABLED,
                        slave
                    );
                }


                real_t qActual =
                    a6ec_position_units_to_joint_rad(
                        feedback.actualPosition
                    );


                real_t error =
                    fabs(
                        qActual -
                        homing->qHome.q[slave - 1]
                    );


                if (
                    error >
                    config->positionTolerance
                )
                {
                    allWithinTolerance =
                        false;
                }


                A6ECPDOCommand command =
                {
                    .controlword =
                        CIA402_CONTROLWORD_ENABLE_OPERATION,

                    .targetPosition =
                        a6ec_joint_rad_to_position_units(
                            homing->qHome.q[slave - 1]
                        )
                };


                a6ec_write_command(
                    slave,
                    &command
                );
            }


            if (!homing_wkc_valid())
            {
                return homing_fail(
                    homing,
                    HOMING_ERROR_WKC,
                    0
                );
            }


            homing->verificationCycles++;


            if (allWithinTolerance)
            {
                homing->stableCycles++;
            }
            else
            {
                homing->stableCycles =
                    0U;
            }


            if (
                homing->stableCycles >=
                config->requiredStableCycles
            )
            {
                homing_advance(
                    homing,
                    HOMING_PHASE_COMPLETE
                );

                return
                    STATE_STEP_RUNNING;
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


            return
                STATE_STEP_RUNNING;
        }


        case HOMING_PHASE_COMPLETE:
        {
            return
                STATE_STEP_COMPLETE;
        }


        case HOMING_PHASE_FAILED:
        {
            return
                STATE_STEP_FAILED;
        }


        default:
        {
            return homing_fail(
                homing,
                HOMING_ERROR_INVALID_CONFIG,
                0
            );
        }
    }
}