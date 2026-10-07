#ifndef STATE_HOMING_H
#define STATE_HOMING_H

#include "../state_machine_types.h"
#include "../../CANComm/CANopen/canopen_master.h"
#include "../../ControlCore/Config/robot_config.h"
#include "../../ControlCore/Math/control_types.h"
#include "../../ControlCore/Trajectory/joint_trajectory.h"
#include "../../ServoDrive/AvatarM/avatar_m_position.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    HOMING_PHASE_INIT = 0,
    HOMING_PHASE_READ_POSITION,
    HOMING_PHASE_PREPARE_TRAJECTORY,
    HOMING_PHASE_EXECUTE_TRAJECTORY,
    HOMING_PHASE_VERIFY_HOME,
    HOMING_PHASE_COMPLETE,
    HOMING_PHASE_FAILED
} HomingPhase;

typedef enum
{
    HOMING_ERROR_NONE = 0,
    HOMING_ERROR_INVALID_CONFIG,
    HOMING_ERROR_HOME_NOT_DEFINED,
    HOMING_ERROR_HOME_LIMIT,
    HOMING_ERROR_POSITION_FEEDBACK,
    HOMING_ERROR_DRIVE_NOT_ENABLED,
    HOMING_ERROR_TRAJECTORY,
    HOMING_ERROR_TRAJECTORY_LIMIT,
    HOMING_ERROR_COMMUNICATION,
    HOMING_ERROR_HOME_TIMEOUT,
    HOMING_ERROR_POSITION_CONVERSION,
    HOMING_ERROR_CYCLIC_FEEDBACK
} HomingError;

typedef struct
{
    real_t duration;
    real_t dt;
    real_t positionTolerance;
    uint32_t requiredStableCycles;
    uint32_t maxVerificationCycles;
} HomingConfig;

typedef struct
{
    HomingPhase phase;
    HomingError error;
    int failedAxis;

    JointVector qStart;
    JointVector qHome;
    JointTrajectory trajectory;

    size_t samplesSent;
    uint32_t stableCycles;
    uint32_t verificationCycles;

    uint32_t commandPeriodMs;
    uint32_t lastCommandMs;
    bool commandClockStarted;
    bool awaitingFeedback;
    uint32_t commandTpdoCount[ROBOT_DOF];
} HomingState;

void state_homing_enter(HomingState *homing);

StateStepResult state_homing_step(
    HomingState *homing,
    const HomingConfig *config,
    const RobotConfig *robot,
    CanopenMaster *master,
    const AvatarMPositionScale position_scales[ROBOT_DOF],
    uint32_t now_ms
);

#endif /* STATE_HOMING_H */
