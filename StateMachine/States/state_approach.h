#ifndef STATE_APPROACH_H
#define STATE_APPROACH_H


#include "../state_machine_types.h"
#include "state_path_validation.h"

#include "../../ControlCore/Config/robot_config.h"
#include "../../ControlCore/Math/control_types.h"
#include "../../ControlCore/Trajectory/joint_trajectory.h"
#include "../../ServoDrive/JointDrive/joint_drive_port.h"
#include "../../ServoDrive/AvatarM/avatar_m_position.h"

#include <stdbool.h>
#include <stdint.h>


/* ============================================================================
 * APPROACH CONSTANTS
 * ============================================================================
 *
 * Approach moves the robot from its CURRENT measured joint position to
 * sample 0 of an already validated 2 ms / 500 Hz AVATAR trajectory.
 *
 * The final command uses the exact int32_t AVATAR target-position values
 * stored by Path Validation so Preview / Welding can start without a
 * position jump.
 * ============================================================================
 */

#define APPROACH_MAX_CLEARANCE_POSES   3U
#define APPROACH_MAX_LEGS              (APPROACH_MAX_CLEARANCE_POSES + 1U)


/* ============================================================================
 * REQUEST / OPERATION
 * ============================================================================ */

typedef enum
{
    APPROACH_OPERATION_NONE = 0,
    APPROACH_OPERATION_PREVIEW,
    APPROACH_OPERATION_WELD

} ApproachOperation;


/* ============================================================================
 * INTERNAL PHASES
 * ============================================================================ */

typedef enum
{
    APPROACH_PHASE_IDLE = 0,

    APPROACH_PHASE_CHECK_REQUEST,
    APPROACH_PHASE_LOAD_TARGET,
    APPROACH_PHASE_READ_START,

    APPROACH_PHASE_PREPARE_LEG,
    APPROACH_PHASE_VALIDATE_LEG,

    APPROACH_PHASE_WAIT_PERMISSION,
    APPROACH_PHASE_EXECUTE_LEG,
    APPROACH_PHASE_PAUSED,

    APPROACH_PHASE_VERIFY_TARGET,

    APPROACH_PHASE_COMPLETE,
    APPROACH_PHASE_ABORTED,
    APPROACH_PHASE_FAILED

} ApproachPhase;


/* ============================================================================
 * ERRORS
 * ============================================================================ */

typedef enum
{
    APPROACH_ERR_NONE = 0,

    APPROACH_ERR_NULL_ARGUMENT,
    APPROACH_ERR_INVALID_CONFIG,
    APPROACH_ERR_INVALID_REQUEST,

    APPROACH_ERR_TRAJECTORY_NOT_VALID,
    APPROACH_ERR_TRAJECTORY_MISMATCH,
    APPROACH_ERR_EMPTY_TRAJECTORY,
    APPROACH_ERR_STORAGE_READ,

    APPROACH_ERR_POSITION_CONVERSION,
    APPROACH_ERR_TARGET_LIMIT,

    /* Numeric slot preserved from the old EtherCAT PDO error. */
    APPROACH_ERR_COMMUNICATION,
    APPROACH_ERR_FEEDBACK,
    APPROACH_ERR_DRIVE_NOT_READY,

    APPROACH_ERR_PLAN_LIMIT,
    APPROACH_ERR_COLLISION,
    APPROACH_ERR_COLLISION_CHECK_MISSING,

    APPROACH_ERR_VERIFY_TIMEOUT,
    APPROACH_ERR_FOLLOWING_ERROR,
    /* Numeric slot preserved from the old EtherCAT WKC error. */
    APPROACH_ERR_CYCLIC_FEEDBACK,

    APPROACH_ERR_EXTERNAL_FAULT,
    APPROACH_ERR_ESTOP,

    APPROACH_ERR_RESET_REQUESTED,
    APPROACH_ERR_HOME_REQUESTED

} ApproachError;


/* ============================================================================
 * RESULT
 * ============================================================================ */

typedef enum
{
    APPROACH_RESULT_NONE = 0,
    APPROACH_RESULT_RUNNING,
    APPROACH_RESULT_COMPLETE,
    APPROACH_RESULT_ABORTED,
    APPROACH_RESULT_FAILED

} ApproachResult;


/* ============================================================================
 * APPROACH REQUEST
 * ============================================================================
 *
 * "trajectory_ready" is the supervisor/storage-level validity indication.
 *
 * For a just-completed Path Validation state this should be:
 *
 *      path_validation_outputs.trajectory_ready
 *
 * For a previously stored program, the storage/program manager must establish
 * the same guarantee before Approach is entered.
 *
 * The expected identity fields prevent Approach from accidentally using a
 * stale or different artifact.
 */

typedef struct
{
    ApproachOperation operation;

    const ValidatedTrajectory *trajectory;

    bool trajectory_ready;

    uint32_t expected_program_id;
    uint32_t expected_source_revision;
    uint32_t expected_artifact_crc;

    /*
     * Optional installation-specific prevalidated clearance configurations.
     *
     * Route:
     *
     *      measured q
     *          ->
     *      clearance pose(s)
     *          ->
     *      trajectory sample 0
     */
    const JointVector *clearance_poses;
    uint8_t clearance_pose_count;

} ApproachRequest;


/* ============================================================================
 * SUPERVISORY INPUTS
 * ============================================================================ */

typedef struct
{
    bool motion_permission;

    bool pause_requested;
    bool protective_stop_active;

    bool reset_requested;
    bool home_requested;

    bool estop_active;
    bool external_fault_active;

} ApproachControlInputs;


/* ============================================================================
 * APPROACH-SPECIFIC CONFIGURATION
 * ============================================================================
 *
 * Joint position, velocity and acceleration limits are NOT duplicated here.
 *
 * They come from RobotConfig:
 *
 *      qMin / qMax
 *      qdMax
 *      qddMax when defined
 *
 * Only Approach-specific policy remains here.
 */

typedef struct
{
    real_t duration_safety_factor;

    real_t minimum_leg_duration_s;
    real_t maximum_leg_duration_s;

    real_t final_position_tolerance_rad;
    real_t following_error_limit_rad;

    /*
     * RobotConfig currently has no jerk limits.
     *
     * Keep jerk limits Approach-specific until they become part of the shared
     * robot configuration.
     */
    bool use_jerk_limits;
    real_t jerk_max_rad_s3[ROBOT_DOF];

    uint32_t required_stable_cycles;
    uint32_t maximum_verification_cycles;

    /*
     * Number of route samples collision-checked per call while Approach is
     * validating the route BEFORE motion begins.
     */
    uint16_t validation_samples_per_step;

    bool require_collision_check;

} ApproachConfig;


/* ============================================================================
 * SERVICES NOT YET OWNED BY A COMMON PROJECT MODULE
 * ============================================================================
 *
 * The JointDrivePort handles feedback, readiness and target cycles.
 * Only two application-level capabilities remain as callbacks:
 *
 *      1. read one sample from committed Path Validation storage
 *      2. installation/cell collision checking
 */

typedef bool (*ApproachReadValidatedSampleFn)(
    uint32_t sample_index,
    PvExecutionSample *sample,
    void *context
);


typedef bool (*ApproachCollisionFreeFn)(
    const JointVector *joints_rad,
    void *context
);


typedef struct
{
    ApproachReadValidatedSampleFn read_validated_sample;
    void *storage_context;

    ApproachCollisionFreeFn collision_free;
    void *collision_context;

} ApproachServices;


/* ============================================================================
 * REPORT / HMI OUTPUT
 * ============================================================================ */

typedef struct
{
    ApproachPhase phase;
    ApproachResult result;
    ApproachError error;

    uint8_t failed_joint; /* 0 = global, 1..6 = axis */

    uint8_t active_leg;
    uint8_t total_legs;

    uint32_t sample_index;
    uint32_t samples_sent;

    uint32_t stable_cycles;
    uint32_t verification_cycles;

    uint32_t replan_count;

    real_t leg_duration_s;
    real_t progress_0_to_1;

    JointVector q_start;
    JointVector q_goal;
    JointVector final_target;

    int32_t final_target_units[ROBOT_DOF];

} ApproachReport;


typedef struct
{
    ApproachReport report;

} ApproachOutputs;


/* ============================================================================
 * STATE DATA
 * ============================================================================ */

typedef struct
{
    ApproachPhase phase;
    ApproachResult result;
    ApproachError error;

    bool initialized;

    uint8_t failed_joint;

    const RobotConfig *robot;

    JointDrivePort drive;
    const AvatarMPositionScale *position_scales;

    ApproachRequest request;
    ApproachConfig config;
    ApproachServices services;

    /*
     * State-owned copy of optional clearance poses.
     *
     * This prevents the Approach state from depending on the lifetime of the
     * supervisor's request/input buffer after state_approach_enter() returns.
     */
    JointVector clearance_poses[APPROACH_MAX_CLEARANCE_POSES];

    uint8_t active_leg;
    uint8_t total_legs;

    uint32_t sample_index;
    uint32_t validation_index;

    uint32_t samples_sent;

    uint32_t stable_cycles;
    uint32_t verification_cycles;

    uint32_t replan_count;

    /*
     * q0 = measured start
     * q1.. = optional clearance poses
     * final = validated trajectory sample 0
     */
    JointVector route_pose[APPROACH_MAX_LEGS + 1U];

    /*
     * Reuse the existing ControlCore synchronized quintic joint trajectory.
     *
     * No second quintic implementation lives in Approach.
     */
    JointTrajectory route_trajectory[APPROACH_MAX_LEGS];

    JointVector q_start;
    JointVector final_target;

    JointVector last_command;
    bool last_command_valid;

    int32_t final_target_units[ROBOT_DOF];

    /*
     * CANopen cyclic synchronization bookkeeping.
     * Every command is six RPDO4 targets followed by one SYNC, and the next
     * motion command is not issued until fresh TPDO4 feedback arrives.
     */
    uint32_t command_period_ms;
    uint32_t last_command_ms;
    bool command_clock_started;
    bool awaiting_feedback;
    uint32_t command_tpdo_count[ROBOT_DOF];

    bool paused_during_verification;

} ApproachState;


/* ============================================================================
 * PUBLIC API
 * ============================================================================ */


/*
 * Enter APPROACH.
 *
 * This initializes the state only. No motion command is sent from enter().
 */
void state_approach_enter(
    ApproachState *state,
    const RobotConfig *robot,
    const JointDrivePort *drive_port,
    const AvatarMPositionScale position_scales[ROBOT_DOF],
    const ApproachRequest *request,
    const ApproachConfig *config,
    const ApproachServices *services
);


/*
 * Execute one nonblocking Approach state step.
 *
 * The surrounding control task should call this once per supervisory/control
 * cycle. During trajectory execution targets are paced at 2 ms / 500 Hz.
 *
 * Mapping:
 *
 *      APPROACH_RESULT_RUNNING
 *          -> STATE_STEP_RUNNING
 *
 *      APPROACH_RESULT_COMPLETE / ABORTED
 *          -> STATE_STEP_COMPLETE
 *
 *      APPROACH_RESULT_FAILED
 *          -> STATE_STEP_FAILED
 */
StateStepResult state_approach_step(
    ApproachState *state,
    const ApproachControlInputs *inputs,
    uint32_t now_ms,
    ApproachOutputs *outputs
);


void state_approach_get_outputs(
    const ApproachState *state,
    ApproachOutputs *outputs
);


const char *state_approach_phase_name(
    ApproachPhase phase
);


const char *state_approach_error_name(
    ApproachError error
);


#ifdef __cplusplus
}
#endif


#endif /* STATE_APPROACH_H */
