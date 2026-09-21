#ifndef STATE_TEACHING_H
#define STATE_TEACHING_H


#include "../state_machine_types.h"

#include "../../ControlCore/Config/robot_config.h"

#include <stdbool.h>
#include <stdint.h>


/* ============================================================================
 * TEACHING CONSTANTS
 * ============================================================================
 *
 * Preserved from the teammate implementation.
 */

#define TEACHING_DOF                 (6U)
#define TEACHING_MAX_SEGMENTS        (32U)
#define TEACHING_MAX_POINTS_PER_SEG  (3U)


/* ============================================================================
 * TEACHING TYPES
 * ============================================================================
 *
 * The following enums and structures preserve the teammate's Teaching logic
 * and stored variables. They are moved here only so Teaching follows the same
 * StateMachine/States structure as BOOT, HOMING and IDLE.
 */

typedef enum
{
    TEACH_SEGMENT_NONE = 0,
    TEACH_SEGMENT_LINE,
    TEACH_SEGMENT_ARC,
    TEACH_SEGMENT_CIRCLE

} TeachingSegmentType;


typedef enum
{
    TEACH_CIRCLE_DIRECTION_UNSPECIFIED = 0,
    TEACH_CIRCLE_CCW = 1,
    TEACH_CIRCLE_CW = -1

} TeachingCircleDirection;


typedef enum
{
    TEACH_ORIENTATION_CONSTANT = 0,
    TEACH_ORIENTATION_INTERPOLATED

} TeachingOrientationMode;


typedef enum
{
    TEACH_PHASE_WAIT_SEGMENT_SELECTION = 0,
    TEACH_PHASE_WAIT_POINT,
    TEACH_PHASE_SEGMENT_COMPLETE,
    TEACH_PHASE_SUBMITTING_DRAFT

} TeachingPhase;


typedef enum
{
    TEACH_DRAFT_EMPTY = 0,
    TEACH_DRAFT_EDITING,
    TEACH_DRAFT_COMPLETE,
    TEACH_DRAFT_SUBMITTED

} TeachingDraftStatus;


typedef enum
{
    TEACH_ERR_NONE = 0,
    TEACH_ERR_NULL_ARGUMENT,
    TEACH_ERR_NO_SEGMENT_SELECTED,
    TEACH_ERR_ROBOT_NOT_HOMED,
    TEACH_ERR_DRIVES_NOT_READY,
    TEACH_ERR_ROBOT_NOT_SETTLED,
    TEACH_ERR_GUIDANCE_INACTIVE,
    TEACH_ERR_MOTION_NOT_PERMITTED,
    TEACH_ERR_SAFETY_ACTIVE,
    TEACH_ERR_GLOBAL_FAULT_ACTIVE,
    TEACH_ERR_DUPLICATE_POINT,
    TEACH_ERR_COLLINEAR_POINTS,
    TEACH_ERR_SEGMENT_INCOMPLETE,
    TEACH_ERR_DRAFT_EMPTY,
    TEACH_ERR_SEGMENT_CAPACITY,
    TEACH_ERR_INVALID_SPEED,
    TEACH_ERR_INVALID_MEASUREMENT,
    TEACH_ERR_FRAME_CHANGED,
    TEACH_ERR_TOOL_CHANGED,
    TEACH_ERR_CALIBRATION_CHANGED,
    TEACH_ERR_DRAFT_LOCKED

} TeachingError;


typedef enum
{
    TEACH_MSG_SELECT_SEGMENT = 0,
    TEACH_MSG_MOVE_TO_POINT,
    TEACH_MSG_POINT_RECORDED,
    TEACH_MSG_SEGMENT_RECORDED,
    TEACH_MSG_DRAFT_RESET,
    TEACH_MSG_DRAFT_SUBMITTED,
    TEACH_MSG_ACTION_REJECTED

} TeachingMessageId;


typedef enum
{
    TEACH_EVENT_NONE = 0,
    TEACH_EVENT_SELECT_LINE,
    TEACH_EVENT_SELECT_ARC,
    TEACH_EVENT_SELECT_CIRCLE,
    TEACH_EVENT_RECORD_POINT,
    TEACH_EVENT_VALIDATE_PATH,
    TEACH_EVENT_SPEED_INCREASE,
    TEACH_EVENT_SPEED_DECREASE,
    TEACH_EVENT_SPEED_DEFAULT,
    TEACH_EVENT_RESET

} TeachingEvent;


/* ============================================================================
 * TAUGHT DATA
 * ============================================================================
 */

typedef struct
{
    float position_m[3];

    /*
     * Quaternion convention:
     *
     *      [w, x, y, z]
     */
    float orientation_quat[4];

    float joint_position_rad[TEACHING_DOF];

    uint32_t record_timestamp_ms;
    uint32_t calibration_version;
    uint16_t frame_id;
    uint16_t tool_id;

    bool point_valid;

} TaughtPoint;


typedef struct
{
    uint16_t segment_id;

    TeachingSegmentType type;

    uint8_t point_count;

    TaughtPoint points[TEACHING_MAX_POINTS_PER_SEG];

    float speed_mps;

    TeachingCircleDirection circle_direction;

    TeachingOrientationMode orientation_mode;

    /*
     * Structurally complete.
     * Full feasibility validation belongs to PATH VALIDATION.
     */
    bool segment_valid;

} TaughtSegment;


typedef struct
{
    uint32_t program_id;
    uint32_t draft_revision;
    uint32_t draft_crc;

    uint16_t segment_count;

    TaughtSegment segments[TEACHING_MAX_SEGMENTS];

    float global_speed_scale;

    TeachingDraftStatus status;

} TaughtProgram;


/* ============================================================================
 * TEACHING INPUT SNAPSHOT
 * ============================================================================
 *
 * Preserved from the teammate implementation.
 *
 * In this integrated version, state_teaching.c builds this snapshot using:
 *
 *      A6-EC feedback       -> actual_joint_position_rad
 *      CiA-402 status       -> drives_ready
 *      ControlCore FK       -> actual_position_m + actual_orientation_quat
 *
 * The remaining supervisory flags are supplied through TeachingRuntimeInputs.
 */

typedef struct
{
    float actual_position_m[3];
    float actual_orientation_quat[4];
    float actual_joint_position_rad[TEACHING_DOF];

    uint32_t timestamp_ms;
    uint32_t calibration_version;

    uint16_t active_frame_id;
    uint16_t active_tool_id;

    bool measurement_valid;
    bool robot_motion_settled;
    bool manual_guidance_active;
    bool motion_permitted;
    bool estop_active;
    bool protective_stop_active;
    bool global_fault_active;
    bool drives_ready;
    bool robot_homed;

} TeachingInputs;


/* ============================================================================
 * RUNTIME BRIDGE TO THE REST OF OUR ROBOT
 * ============================================================================
 *
 * These values are not owned by the Teaching algorithm itself.
 *
 * They will eventually come from:
 *
 *      FSM / HMI
 *      guidance / admittance controller
 *      safety module
 *      calibration / frame / tool configuration
 *
 * Motor positions, drive readiness and TCP pose are intentionally NOT passed
 * here because state_teaching.c derives them from our existing robot modules.
 */

typedef struct
{
    uint32_t timestamp_ms;
    uint32_t calibration_version;

    uint16_t active_frame_id;
    uint16_t active_tool_id;

    bool robot_motion_settled;
    bool manual_guidance_active;
    bool motion_permitted;
    bool estop_active;
    bool protective_stop_active;
    bool global_fault_active;
    bool robot_homed;

} TeachingRuntimeInputs;


/* ============================================================================
 * TEACHING CONFIGURATION
 * ============================================================================
 */

typedef struct
{
    float default_speed_mps;
    float minimum_speed_mps;
    float maximum_speed_mps;
    float speed_step_mps;

    float minimum_point_separation_m;
    float collinearity_epsilon_m2;

} TeachingConfig;


/* ============================================================================
 * TEACHING OUTPUTS
 * ============================================================================
 */

typedef struct
{
    bool validation_request;

    uint32_t submitted_program_id;
    uint32_t submitted_revision;
    uint32_t submitted_crc;

    TeachingMessageId status_message_id;
    TeachingError error;
    TeachingPhase phase;

    uint16_t current_segment_number;
    uint8_t next_point_number;

    float displayed_speed_mps;

    bool record_allowed;
    bool validate_allowed;
    bool draft_was_reset;

} TeachingOutputs;


/* ============================================================================
 * TEACHING STATE DATA
 * ============================================================================
 *
 * All teammate-owned Teaching variables are preserved.
 *
 * "initialized" and "last_event_accepted" are integration bookkeeping only.
 */

typedef struct
{
    TeachingPhase phase;
    TeachingError error;
    TeachingMessageId status_message_id;

    TeachingSegmentType selected_segment_type;
    TeachingCircleDirection selected_circle_direction;
    TeachingOrientationMode selected_orientation_mode;

    uint8_t required_point_count;
    uint8_t captured_point_count;
    uint16_t active_segment_index;

    float teaching_speed_mps;

    bool draft_dirty;
    bool teaching_complete;

    TaughtSegment working_segment;
    TaughtProgram draft;
    TeachingConfig config;


    /* Integration-only bookkeeping. */
    bool initialized;
    bool last_event_accepted;

} TeachingState;


/* ============================================================================
 * PUBLIC API
 * ============================================================================
 */

/*
 * Enter the global TEACHING state.
 *
 * Initializes the teammate Teaching logic using the supplied configuration.
 */
void state_teaching_enter(
    TeachingState *state,
    const TeachingConfig *config,
    uint32_t programId
);


/*
 * Execute one supervisory Teaching event.
 *
 * The function:
 *
 *      1. reads all six A6-EC joint positions,
 *      2. checks all six CiA-402 drives,
 *      3. computes TCP pose using our ControlCore FK,
 *      4. builds the preserved TeachingInputs snapshot,
 *      5. runs the preserved Teaching event logic.
 *
 * Returns:
 *
 *      STATE_STEP_RUNNING
 *          Teaching remains active.
 *
 *      STATE_STEP_COMPLETE
 *          A valid draft raised validation_request and is ready for
 *          PATH VALIDATION.
 *
 *      STATE_STEP_FAILED
 *          Integration/init arguments are invalid.
 *
 * A rejected Teaching action is NOT a global state failure. The reason is
 * exposed through TeachingOutputs.error and the operator can retry.
 */
StateStepResult state_teaching_step(
    TeachingState *state,
    const RobotConfig *robot,
    const TeachingRuntimeInputs *runtime,
    TeachingEvent event,
    TeachingOutputs *outputs
);


/*
 * Refresh HMI/status outputs without processing a new Teaching event.
 */
void state_teaching_get_outputs(
    const TeachingState *state,
    TeachingOutputs *outputs
);


/*
 * Immediate operational reset of the RAM draft.
 *
 * Preserves the teammate reset semantics:
 *
 *      no confirmation
 *      no homing request
 *      no fault clear
 *      revision incremented
 */
void state_teaching_reset_draft(
    TeachingState *state
);


/*
 * Optional selections preserved from the teammate implementation.
 */
bool state_teaching_set_circle_direction(
    TeachingState *state,
    TeachingCircleDirection direction
);


bool state_teaching_set_orientation_mode(
    TeachingState *state,
    TeachingOrientationMode mode
);


/*
 * Debug / HMI name helpers.
 */
const char *state_teaching_phase_name(
    TeachingPhase phase
);


const char *state_teaching_error_name(
    TeachingError error
);


const char *state_teaching_segment_name(
    TeachingSegmentType type
);


#endif /* STATE_TEACHING_H */
