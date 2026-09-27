#ifndef STATE_PATH_VALIDATION_H
#define STATE_PATH_VALIDATION_H

#include "../state_machine_types.h"
#include "state_teaching.h"

#include "../../ControlCore/Config/robot_config.h"
#include "../../ControlCore/Math/control_types.h"
#include "../../ControlCore/Kinematics/adls_ik.h"
#include "../../ControlCore/Pipeline/single_segment_line_stream.h"
#include "../../ControlCore/Pipeline/single_segment_circular_stream.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PATH_VALIDATION_DOF               ROBOT_DOF
#define PATH_VALIDATION_SAMPLE_PERIOD_US  (1000UL)
#define PATH_VALIDATION_SAMPLE_PERIOD_S   (0.001)

/* ============================================================================
 * PATH VALIDATION STATE / RESULT
 * ============================================================================ */

typedef enum
{
    PV_PHASE_IDLE = 0,
    PV_PHASE_SNAPSHOT_CHECK,
    PV_PHASE_STRUCTURAL_CHECK,
    PV_PHASE_PREPARE_SEGMENT,
    PV_PHASE_GENERATE_AND_VALIDATE_SAMPLE,
    PV_PHASE_FINALIZE,
    PV_PHASE_VALID,
    PV_PHASE_INVALID

} PathValidationPhase;


typedef enum
{
    PV_ERR_NONE = 0,
    PV_ERR_NULL_ARGUMENT,
    PV_ERR_BUSY,
    PV_ERR_EMPTY_PROGRAM,
    PV_ERR_REVISION_MISMATCH,
    PV_ERR_DRAFT_CRC_MISMATCH,
    PV_ERR_UNSUPPORTED_SEGMENT,
    PV_ERR_INCOMPLETE_SEGMENT,
    PV_ERR_INVALID_POINT,
    PV_ERR_DEGENERATE_GEOMETRY,
    PV_ERR_ARC_DIRECTION,
    PV_ERR_INVALID_CIRCLE_DIRECTION,
    PV_ERR_FRAME_MISMATCH,
    PV_ERR_TOOL_MISMATCH,
    PV_ERR_CALIBRATION_MISMATCH,
    PV_ERR_INVALID_PARAMETER,
    PV_ERR_ZERO_LENGTH_SEGMENT,
    PV_ERR_SAMPLE_CAPACITY,
    PV_ERR_IK_FAILED,
    PV_ERR_FK_POSITION,
    PV_ERR_FK_ORIENTATION,
    PV_ERR_JOINT_POSITION,
    PV_ERR_JOINT_VELOCITY,
    PV_ERR_JOINT_ACCELERATION,
    PV_ERR_JOINT_DISCONTINUITY,
    PV_ERR_POSITION_CONVERSION,
    PV_ERR_SINGULARITY_MARGIN,
    PV_ERR_COLLISION,
    PV_ERR_STORAGE,
    PV_ERR_CALLBACK_MISSING,
    PV_ERR_CANCELLED

} PathValidationError;


typedef enum
{
    PV_RESULT_NONE = 0,
    PV_RESULT_RUNNING,
    PV_RESULT_VALID,
    PV_RESULT_INVALID,
    PV_RESULT_CANCELLED

} PathValidationResult;


/* ============================================================================
 * VALIDATED EXECUTION ARTIFACT
 * ============================================================================
 *
 * One sample = one future 1 ms CSP Target Position command for all six axes.
 * The sample carries no timestamp because time is sample_index * 1 ms.
 */

typedef struct
{
    int32_t target_position_units[PATH_VALIDATION_DOF];

} PvExecutionSample;


typedef struct
{
    uint32_t first_sample;
    uint32_t sample_count;
    uint8_t segment_type;
    uint8_t reserved;

} PvSegmentIndex;


typedef struct
{
    uint32_t program_id;
    uint32_t source_revision;
    uint32_t source_crc;

    uint32_t artifact_crc;
    uint32_t sample_data_crc;

    uint32_t sample_count;
    uint16_t segment_count;

    uint32_t sample_period_us;

    real_t duration_s;
    real_t path_length_m;

    PvSegmentIndex segments[TEACHING_MAX_SEGMENTS];

} ValidatedTrajectory;


/* ============================================================================
 * STORAGE BOUNDARY
 * ============================================================================
 *
 * This is deliberately NOT ControlCore/Execution/TrajectoryBuffer.
 *
 * TrajectoryBuffer:
 *      short live FIFO for planner -> execution streaming.
 *
 * PathValidationStorage:
 *      complete immutable validated program artifact for later Preview/Welding.
 */

typedef bool (*PvStorageBeginFn)(void *context);

typedef bool (*PvStorageWriteFn)(
    uint32_t sample_index,
    const PvExecutionSample *sample,
    void *context
);

typedef bool (*PvStorageCommitFn)(
    const ValidatedTrajectory *metadata,
    void *context
);

typedef void (*PvStorageAbortFn)(void *context);


typedef struct
{
    PvStorageBeginFn begin;
    PvStorageWriteFn write_sample;
    PvStorageCommitFn commit;
    PvStorageAbortFn abort;

    uint32_t capacity_samples;
    void *context;

} PathValidationStorage;


/* ============================================================================
 * PATH VALIDATION SERVICES
 * ============================================================================
 *
 * Kinematics and trajectory generation are NOT callbacks here because this
 * integrated state uses our existing ControlCore directly.
 *
 * Collision remains a callback because no final collision/environment module
 * is owned by this state.
 */

typedef bool (*PathValidationCollisionFn)(
    const JointVector *joints_rad,
    uint16_t source_segment,
    uint32_t sample_index,
    void *context
);


typedef struct
{
    PathValidationCollisionFn collision_free;
    void *context;

} PathValidationServices;


/* ============================================================================
 * FIXED WORKSPACE SUPPLIED BY THE CALLER
 * ============================================================================
 *
 * The ControlCore streaming planners already avoid trajectory-sized arrays,
 * but they still need fixed geometry scratch memory and one ADLSInfo scratch
 * object. The caller owns those buffers so this state does not dynamically
 * allocate memory and does not duplicate ControlCore internals.
 */

typedef struct
{
    size_t geometry_capacity;

    Vec3 *raw_geometry;
    Vec3 *arc_geometry;

    real_t *l_original;
    real_t *l_arc;

    ADLSInfo *ik_scratch;

} PathValidationWorkspace;


/* ============================================================================
 * CONFIGURATION
 * ============================================================================
 *
 * Robot joint position/velocity limits, DH parameters and TCP are taken from
 * RobotConfig. They are intentionally not duplicated here.
 */

typedef struct
{
    real_t default_tcp_speed_mps;
    real_t max_tcp_speed_mps;
    real_t max_tcp_acceleration_mps2;
    real_t max_tcp_jerk_mps3;

    real_t minimum_segment_length_m;

    real_t maximum_fk_position_error_m;
    real_t maximum_fk_orientation_error_rad;
    real_t minimum_singularity_sigma;

    real_t maximum_joint_step_rad;
    real_t maximum_position_quantization_error_rad;

    size_t geometry_points_per_segment;
    real_t arc_length_spacing_m;

    ADLSParameters ik_parameters;

    /*
     * If RobotConfig.limits.qddMaxDefined is true, RobotConfig owns the
     * acceleration limits and this fallback is ignored.
     *
     * Otherwise these values are used only when check_joint_acceleration=true.
     */
    bool check_joint_acceleration;
    real_t joint_acceleration_max_rad_s2[PATH_VALIDATION_DOF];

    bool require_collision_callback;

} PathValidationConfig;


/* ============================================================================
 * REPORT / HMI-FACING OUTPUT
 * ============================================================================ */

typedef struct
{
    PathValidationPhase phase;
    PathValidationResult result;
    PathValidationError error;

    uint16_t failed_segment;
    uint32_t failed_sample;
    uint8_t failed_joint;

    real_t progress_0_to_1;

    real_t max_fk_position_error_m;
    real_t max_fk_orientation_error_rad;
    real_t minimum_sigma_seen;

    real_t peak_joint_velocity_rad_s[PATH_VALIDATION_DOF];
    real_t peak_joint_acceleration_rad_s2[PATH_VALIDATION_DOF];

} PathValidationReport;


typedef struct
{
    PathValidationReport report;
    bool trajectory_ready;

} PathValidationOutputs;


/* ============================================================================
 * STATE DATA
 * ============================================================================ */

typedef struct
{
    PathValidationPhase phase;
    PathValidationResult result;
    PathValidationError error;

    bool initialized;
    bool storage_open;

    const RobotConfig *robot;
    const TaughtProgram *source_program;

    uint32_t expected_revision;
    uint32_t expected_crc;

    PathValidationConfig config;
    PathValidationServices services;
    PathValidationStorage storage;
    PathValidationWorkspace *workspace;

    ValidatedTrajectory *artifact;

    uint16_t segment_index;
    uint32_t sample_index;

    real_t current_segment_length_m;

    bool active_stream_is_line;

    SingleLineStream line_stream;
    SingleCircularStream circular_stream;

    SingleLineStreamWorkspace line_workspace;
    SingleCircularStreamWorkspace circular_workspace;

    JointVector previous_q;
    JointVector previous_qd;

    bool previous_q_valid;
    bool previous_qd_valid;

    real_t previous_time_s;

    uint32_t sample_crc_state;

    PathValidationReport report;

} PathValidationState;


/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

/*
 * Enter PATH VALIDATION with the immutable draft submitted by Teaching.
 *
 * The state does not read live EtherCAT feedback and does not command drives.
 * It uses ControlCore to generate motion, validates the generated commands,
 * converts accepted joint values to the existing A6-EC CSP position units,
 * then writes the immutable artifact through PathValidationStorage.
 */
void state_path_validation_enter(
    PathValidationState *state,
    const RobotConfig *robot,
    const PathValidationConfig *config,
    const PathValidationServices *services,
    PathValidationWorkspace *workspace,
    const PathValidationStorage *storage,
    ValidatedTrajectory *artifact_storage,
    const TaughtProgram *frozen_program,
    uint32_t submitted_revision,
    uint32_t submitted_crc
);


/*
 * Execute at most sample_budget trajectory samples.
 *
 * RUNNING:
 *      validation still in progress.
 *
 * COMPLETE:
 *      validation reached VALID, INVALID or CANCELLED. Inspect outputs.result.
 *      An invalid path is an expected validation outcome, not automatically a
 *      global controller fault.
 *
 * FAILED:
 *      the state object itself is unusable/not initialized.
 */
StateStepResult state_path_validation_step(
    PathValidationState *state,
    uint16_t sample_budget,
    PathValidationOutputs *outputs
);


void state_path_validation_cancel(
    PathValidationState *state
);


void state_path_validation_get_outputs(
    const PathValidationState *state,
    PathValidationOutputs *outputs
);


/* Must remain field-for-field identical to Teaching's submitted draft CRC. */
uint32_t state_path_validation_calculate_draft_crc(
    const TaughtProgram *program
);


const char *state_path_validation_phase_name(
    PathValidationPhase phase
);


const char *state_path_validation_error_name(
    PathValidationError error
);


#ifdef __cplusplus
}
#endif

#endif /* STATE_PATH_VALIDATION_H */
