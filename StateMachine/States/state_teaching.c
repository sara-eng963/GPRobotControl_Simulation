#include "state_teaching.h"

#include "../../ControlCore/Kinematics/control_fk.h"
#include "../../ControlCore/Math/control_types.h"
#include "../../ControlCore/Math/math3d.h"

#include "../../EtherCATComm/ethercat_master.h"

#include "../../ServoDrive/A6EC/a6ec_drive.h"
#include "../../ServoDrive/CiA402/cia402.h"

#include <math.h>
#include <stddef.h>
#include <string.h>


_Static_assert(
    TEACHING_DOF == ROBOT_DOF,
    "Teaching DOF must match robot DOF."
);


#define TEACHING_CRC32_POLYNOMIAL (0xEDB88320UL)


/* ============================================================================
 * SMALL INTERNAL HELPERS
 * ============================================================================
 */

static void clear_outputs(
    TeachingOutputs *outputs
)
{
    memset(
        outputs,
        0,
        sizeof(*outputs)
    );
}


static uint8_t required_points(
    TeachingSegmentType type
)
{
    switch (type)
    {
        case TEACH_SEGMENT_LINE:
            return 2U;

        case TEACH_SEGMENT_ARC:
            return 3U;

        case TEACH_SEGMENT_CIRCLE:
            return 3U;

        default:
            return 0U;
    }
}


static bool finite_array(
    const float *values,
    size_t count
)
{
    for (
        size_t i = 0U;
        i < count;
        ++i
    )
    {
        if (!isfinite(values[i]))
        {
            return false;
        }
    }


    return true;
}


static float squared_distance3(
    const float a[3],
    const float b[3]
)
{
    const float dx =
        a[0] - b[0];

    const float dy =
        a[1] - b[1];

    const float dz =
        a[2] - b[2];


    return
        (dx * dx) +
        (dy * dy) +
        (dz * dz);
}


static float cross_squared(
    const float p1[3],
    const float p2[3],
    const float p3[3]
)
{
    const float ax =
        p2[0] - p1[0];

    const float ay =
        p2[1] - p1[1];

    const float az =
        p2[2] - p1[2];


    const float bx =
        p3[0] - p1[0];

    const float by =
        p3[1] - p1[1];

    const float bz =
        p3[2] - p1[2];


    const float cx =
        (ay * bz) -
        (az * by);

    const float cy =
        (az * bx) -
        (ax * bz);

    const float cz =
        (ax * by) -
        (ay * bx);


    return
        (cx * cx) +
        (cy * cy) +
        (cz * cz);
}


/* ============================================================================
 * CRC
 * ============================================================================
 *
 * Preserved teammate behavior:
 *
 * CRC is calculated field-by-field so compiler padding bytes are not included.
 * It is an integrity check, not a safety-certified diagnostic by itself.
 */

static uint32_t crc32_update(
    uint32_t crc,
    const uint8_t *data,
    size_t length
)
{
    for (
        size_t i = 0U;
        i < length;
        ++i
    )
    {
        crc ^=
            data[i];


        for (
            uint8_t bit = 0U;
            bit < 8U;
            ++bit
        )
        {
            const uint32_t mask =
                (uint32_t)-(int32_t)(crc & 1U);


            crc =
                (crc >> 1U) ^
                (TEACHING_CRC32_POLYNOMIAL & mask);
        }
    }


    return crc;
}


static uint32_t calculate_draft_crc(
    const TaughtProgram *program
)
{
    uint32_t crc =
        0xFFFFFFFFUL;


#define CRC_FIELD(field_) \
    do \
    { \
        crc = crc32_update( \
            crc, \
            (const uint8_t *)&(field_), \
            sizeof(field_) \
        ); \
    } while (0)


    CRC_FIELD(program->program_id);
    CRC_FIELD(program->draft_revision);
    CRC_FIELD(program->segment_count);
    CRC_FIELD(program->global_speed_scale);


    for (
        uint16_t s = 0U;
        s < program->segment_count;
        ++s
    )
    {
        const TaughtSegment *segment =
            &program->segments[s];


        CRC_FIELD(segment->segment_id);
        CRC_FIELD(segment->type);
        CRC_FIELD(segment->point_count);
        CRC_FIELD(segment->speed_mps);
        CRC_FIELD(segment->circle_direction);
        CRC_FIELD(segment->orientation_mode);


        for (
            uint8_t p = 0U;
            p < segment->point_count;
            ++p
        )
        {
            const TaughtPoint *point =
                &segment->points[p];


            crc =
                crc32_update(
                    crc,
                    (const uint8_t *)point->position_m,
                    sizeof(point->position_m)
                );


            crc =
                crc32_update(
                    crc,
                    (const uint8_t *)point->orientation_quat,
                    sizeof(point->orientation_quat)
                );


            crc =
                crc32_update(
                    crc,
                    (const uint8_t *)point->joint_position_rad,
                    sizeof(point->joint_position_rad)
                );


            CRC_FIELD(point->record_timestamp_ms);
            CRC_FIELD(point->calibration_version);
            CRC_FIELD(point->frame_id);
            CRC_FIELD(point->tool_id);
        }
    }


#undef CRC_FIELD


    return
        ~crc;
}


/* ============================================================================
 * ERROR / CONFIG HELPERS
 * ============================================================================
 */

static void set_error(
    TeachingState *state,
    TeachingError error
)
{
    state->error =
        error;

    state->status_message_id =
        TEACH_MSG_ACTION_REJECTED;
}


static bool config_is_valid(
    const TeachingConfig *config
)
{
    if (config == NULL)
    {
        return false;
    }


    return
        isfinite(config->default_speed_mps) &&
        isfinite(config->minimum_speed_mps) &&
        isfinite(config->maximum_speed_mps) &&
        isfinite(config->speed_step_mps) &&
        isfinite(config->minimum_point_separation_m) &&
        isfinite(config->collinearity_epsilon_m2) &&

        (config->minimum_speed_mps > 0.0F) &&

        (config->maximum_speed_mps >=
         config->minimum_speed_mps) &&

        (config->default_speed_mps >=
         config->minimum_speed_mps) &&

        (config->default_speed_mps <=
         config->maximum_speed_mps) &&

        (config->speed_step_mps > 0.0F) &&

        (config->minimum_point_separation_m > 0.0F) &&

        (config->collinearity_epsilon_m2 > 0.0F);
}


/* ============================================================================
 * BUILD TEACHING INPUTS FROM OUR ROBOT MODULES
 * ============================================================================
 *
 * This is the main integration change.
 *
 * No Teaching decision logic is changed here.
 *
 * We only replace the old external "someone fills TeachingInputs" boundary
 * with our existing robot interfaces:
 *
 *      A6-EC feedback
 *      CiA-402
 *      ControlCore FK
 */

static bool build_teaching_inputs(
    const RobotConfig *robot,
    const TeachingRuntimeInputs *runtime,
    TeachingInputs *inputs
)
{
    if (
        robot == NULL ||
        runtime == NULL ||
        inputs == NULL
    )
    {
        return false;
    }


    memset(
        inputs,
        0,
        sizeof(*inputs)
    );


    /* ------------------------------------------------------------------------
     * SUPERVISORY VALUES
     * ------------------------------------------------------------------------
     */

    inputs->timestamp_ms =
        runtime->timestamp_ms;

    inputs->calibration_version =
        runtime->calibration_version;

    inputs->active_frame_id =
        runtime->active_frame_id;

    inputs->active_tool_id =
        runtime->active_tool_id;

    inputs->robot_motion_settled =
        runtime->robot_motion_settled;

    inputs->manual_guidance_active =
        runtime->manual_guidance_active;

    inputs->motion_permitted =
        runtime->motion_permitted;

    inputs->estop_active =
        runtime->estop_active;

    inputs->protective_stop_active =
        runtime->protective_stop_active;

    inputs->global_fault_active =
        runtime->global_fault_active;

    inputs->robot_homed =
        runtime->robot_homed;


    /* ------------------------------------------------------------------------
     * A6-EC JOINT FEEDBACK + CiA-402 DRIVE READINESS
     * ------------------------------------------------------------------------
     */

    inputs->measurement_valid =
        true;

    inputs->drives_ready =
        true;


    double q[ROBOT_DOF] =
    {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };


    for (
        int slave = 1;
        slave <= ROBOT_DOF;
        ++slave
    )
    {
        /*
         * If no mapped input PDO exists, the measurement is not valid.
         */
        if (
            ethercat_master_slave_inputs(
                slave
            ) == NULL
        )
        {
            inputs->measurement_valid =
                false;

            inputs->drives_ready =
                false;

            continue;
        }


        A6ECPDOFeedback feedback =
        {
            0
        };


        a6ec_read_feedback(
            slave,
            &feedback
        );


        const double jointRad =
            a6ec_position_units_to_joint_rad(
                feedback.actualPosition
            );


        q[slave - 1] =
            jointRad;

        inputs->actual_joint_position_rad[
            slave - 1
        ] =
            (float)jointRad;


        if (!isfinite(jointRad))
        {
            inputs->measurement_valid =
                false;
        }


        const uint16_t driveState =
            cia402_get_state(
                feedback.statusword
            );


        if (
            driveState !=
            CIA402_STATE_OPERATION_ENABLED
        )
        {
            inputs->drives_ready =
                false;
        }
    }


    /* ------------------------------------------------------------------------
     * FORWARD KINEMATICS
     * ------------------------------------------------------------------------
     *
     * The teammate stored Cartesian position and quaternion orientation at
     * every taught point.
     *
     * We now derive those directly from our measured joint positions using
     * the existing ControlCore FK.
     */

    if (inputs->measurement_valid)
    {
        double T_B_TCP[4][4];


        control_fk(
            robot,
            q,
            T_B_TCP
        );


        inputs->actual_position_m[0] =
            (float)T_B_TCP[0][3];

        inputs->actual_position_m[1] =
            (float)T_B_TCP[1][3];

        inputs->actual_position_m[2] =
            (float)T_B_TCP[2][3];


        Mat3 rotation;


        for (
            int row = 0;
            row < 3;
            ++row
        )
        {
            for (
                int column = 0;
                column < 3;
                ++column
            )
            {
                rotation.m[row][column] =
                    T_B_TCP[row][column];
            }
        }


        const Quat orientation =
            rotm_to_quat(
                rotation
            );


        inputs->actual_orientation_quat[0] =
            (float)orientation.w;

        inputs->actual_orientation_quat[1] =
            (float)orientation.x;

        inputs->actual_orientation_quat[2] =
            (float)orientation.y;

        inputs->actual_orientation_quat[3] =
            (float)orientation.z;
    }


    /*
     * Keep the teammate's measurement validity meaning:
     *
     * all recorded numeric measurement arrays must contain finite values.
     */
    if (
        !finite_array(
            inputs->actual_position_m,
            3U
        )
        ||
        !finite_array(
            inputs->actual_orientation_quat,
            4U
        )
        ||
        !finite_array(
            inputs->actual_joint_position_rad,
            TEACHING_DOF
        )
    )
    {
        inputs->measurement_valid =
            false;
    }


    return true;
}


/* ============================================================================
 * ORIGINAL TEACHING LOGIC
 * ============================================================================
 */

static void begin_segment(
    TeachingState *state,
    TeachingSegmentType type
)
{
    memset(
        &state->working_segment,
        0,
        sizeof(state->working_segment)
    );


    state->selected_segment_type =
        type;

    state->required_point_count =
        required_points(type);

    state->captured_point_count =
        0U;

    state->active_segment_index =
        state->draft.segment_count;

    state->working_segment.segment_id =
        (uint16_t)(
            state->draft.segment_count +
            1U
        );

    state->working_segment.type =
        type;

    state->working_segment.speed_mps =
        state->teaching_speed_mps;

    state->working_segment.circle_direction =
        state->selected_circle_direction;

    state->working_segment.orientation_mode =
        state->selected_orientation_mode;

    state->phase =
        TEACH_PHASE_WAIT_POINT;

    state->draft.status =
        TEACH_DRAFT_EDITING;

    state->status_message_id =
        TEACH_MSG_MOVE_TO_POINT;

    state->error =
        TEACH_ERR_NONE;
}


static bool capture_is_permitted(
    TeachingState *state,
    const TeachingInputs *inputs
)
{
    if (
        inputs->estop_active ||
        inputs->protective_stop_active
    )
    {
        set_error(
            state,
            TEACH_ERR_SAFETY_ACTIVE
        );

        return false;
    }


    if (inputs->global_fault_active)
    {
        set_error(
            state,
            TEACH_ERR_GLOBAL_FAULT_ACTIVE
        );

        return false;
    }


    if (!inputs->robot_homed)
    {
        set_error(
            state,
            TEACH_ERR_ROBOT_NOT_HOMED
        );

        return false;
    }


    if (!inputs->drives_ready)
    {
        set_error(
            state,
            TEACH_ERR_DRIVES_NOT_READY
        );

        return false;
    }


    if (!inputs->motion_permitted)
    {
        set_error(
            state,
            TEACH_ERR_MOTION_NOT_PERMITTED
        );

        return false;
    }


    if (!inputs->manual_guidance_active)
    {
        set_error(
            state,
            TEACH_ERR_GUIDANCE_INACTIVE
        );

        return false;
    }


    if (!inputs->robot_motion_settled)
    {
        set_error(
            state,
            TEACH_ERR_ROBOT_NOT_SETTLED
        );

        return false;
    }


    if (
        !inputs->measurement_valid
        ||
        !finite_array(
            inputs->actual_position_m,
            3U
        )
        ||
        !finite_array(
            inputs->actual_orientation_quat,
            4U
        )
        ||
        !finite_array(
            inputs->actual_joint_position_rad,
            TEACHING_DOF
        )
    )
    {
        set_error(
            state,
            TEACH_ERR_INVALID_MEASUREMENT
        );

        return false;
    }


    return true;
}


static bool check_program_identity(
    TeachingState *state,
    const TeachingInputs *inputs
)
{
    const TaughtPoint *reference;


    if (
        state->draft.segment_count >
        0U
    )
    {
        reference =
            &state->draft.segments[0].points[0];
    }
    else if (
        state->captured_point_count >
        0U
    )
    {
        reference =
            &state->working_segment.points[0];
    }
    else
    {
        return true;
    }


    if (
        inputs->active_frame_id !=
        reference->frame_id
    )
    {
        set_error(
            state,
            TEACH_ERR_FRAME_CHANGED
        );

        return false;
    }


    if (
        inputs->active_tool_id !=
        reference->tool_id
    )
    {
        set_error(
            state,
            TEACH_ERR_TOOL_CHANGED
        );

        return false;
    }


    if (
        inputs->calibration_version !=
        reference->calibration_version
    )
    {
        set_error(
            state,
            TEACH_ERR_CALIBRATION_CHANGED
        );

        return false;
    }


    return true;
}


static bool record_point(
    TeachingState *state,
    const TeachingInputs *inputs
)
{
    const float minDistanceSquared =
        state->config.minimum_point_separation_m *
        state->config.minimum_point_separation_m;


    if (
        state->phase !=
        TEACH_PHASE_WAIT_POINT
        ||
        state->selected_segment_type ==
        TEACH_SEGMENT_NONE
    )
    {
        set_error(
            state,
            TEACH_ERR_NO_SEGMENT_SELECTED
        );

        return false;
    }


    if (
        !capture_is_permitted(
            state,
            inputs
        )
        ||
        !check_program_identity(
            state,
            inputs
        )
    )
    {
        return false;
    }


    if (
        state->captured_point_count >
        0U
        &&
        squared_distance3(
            inputs->actual_position_m,
            state->working_segment.points[
                state->captured_point_count - 1U
            ].position_m
        )
        <
        minDistanceSquared
    )
    {
        set_error(
            state,
            TEACH_ERR_DUPLICATE_POINT
        );

        return false;
    }


    TaughtPoint *point =
        &state->working_segment.points[
            state->captured_point_count
        ];


    memcpy(
        point->position_m,
        inputs->actual_position_m,
        sizeof(point->position_m)
    );


    memcpy(
        point->orientation_quat,
        inputs->actual_orientation_quat,
        sizeof(point->orientation_quat)
    );


    memcpy(
        point->joint_position_rad,
        inputs->actual_joint_position_rad,
        sizeof(point->joint_position_rad)
    );


    point->record_timestamp_ms =
        inputs->timestamp_ms;

    point->calibration_version =
        inputs->calibration_version;

    point->frame_id =
        inputs->active_frame_id;

    point->tool_id =
        inputs->active_tool_id;

    point->point_valid =
        true;


    ++state->captured_point_count;


    state->working_segment.point_count =
        state->captured_point_count;


    if (
        state->captured_point_count <
        state->required_point_count
    )
    {
        state->status_message_id =
            TEACH_MSG_POINT_RECORDED;

        state->error =
            TEACH_ERR_NONE;

        return true;
    }


    if (
        (
            state->selected_segment_type ==
            TEACH_SEGMENT_ARC
            ||
            state->selected_segment_type ==
            TEACH_SEGMENT_CIRCLE
        )
        &&
        (
            cross_squared(
                state->working_segment.points[0].position_m,
                state->working_segment.points[1].position_m,
                state->working_segment.points[2].position_m
            )
            <=
            state->config.collinearity_epsilon_m2
        )
    )
    {
        /*
         * Preserve teammate behavior:
         *
         * Keep P1 and P2 and discard only invalid P3 so the operator can retry.
         */
        memset(
            &state->working_segment.points[2],
            0,
            sizeof(TaughtPoint)
        );


        state->captured_point_count =
            2U;

        state->working_segment.point_count =
            2U;


        set_error(
            state,
            TEACH_ERR_COLLINEAR_POINTS
        );


        return false;
    }


    if (
        state->draft.segment_count >=
        TEACHING_MAX_SEGMENTS
    )
    {
        set_error(
            state,
            TEACH_ERR_SEGMENT_CAPACITY
        );

        return false;
    }


    state->working_segment.segment_valid =
        true;


    state->draft.segments[
        state->draft.segment_count
    ] =
        state->working_segment;


    ++state->draft.segment_count;

    ++state->draft.draft_revision;


    state->draft.draft_crc =
        0U;


    state->draft.status =
        TEACH_DRAFT_COMPLETE;


    state->draft_dirty =
        true;

    state->teaching_complete =
        true;


    state->phase =
        TEACH_PHASE_SEGMENT_COMPLETE;


    state->status_message_id =
        TEACH_MSG_SEGMENT_RECORDED;


    state->error =
        TEACH_ERR_NONE;


    return true;
}


static bool change_speed(
    TeachingState *state,
    float requestedSpeed
)
{
    if (!isfinite(requestedSpeed))
    {
        set_error(
            state,
            TEACH_ERR_INVALID_SPEED
        );

        return false;
    }


    if (
        requestedSpeed <
        state->config.minimum_speed_mps
    )
    {
        requestedSpeed =
            state->config.minimum_speed_mps;
    }


    if (
        requestedSpeed >
        state->config.maximum_speed_mps
    )
    {
        requestedSpeed =
            state->config.maximum_speed_mps;
    }


    state->teaching_speed_mps =
        requestedSpeed;


    state->error =
        TEACH_ERR_NONE;


    /*
     * Preserve teammate behavior:
     *
     * a segment snapshots its speed when selected.
     */
    return true;
}


static bool submit_for_validation(
    TeachingState *state,
    TeachingOutputs *outputs
)
{
    if (
        state->phase ==
        TEACH_PHASE_WAIT_POINT
        &&
        state->captured_point_count >
        0U
    )
    {
        set_error(
            state,
            TEACH_ERR_SEGMENT_INCOMPLETE
        );

        return false;
    }


    if (
        state->draft.segment_count ==
        0U
    )
    {
        set_error(
            state,
            TEACH_ERR_DRAFT_EMPTY
        );

        return false;
    }


    state->draft.status =
        TEACH_DRAFT_SUBMITTED;


    state->draft.draft_crc =
        calculate_draft_crc(
            &state->draft
        );


    state->draft_dirty =
        false;


    state->phase =
        TEACH_PHASE_SUBMITTING_DRAFT;


    state->status_message_id =
        TEACH_MSG_DRAFT_SUBMITTED;


    state->error =
        TEACH_ERR_NONE;


    outputs->validation_request =
        true;


    outputs->submitted_program_id =
        state->draft.program_id;


    outputs->submitted_revision =
        state->draft.draft_revision;


    outputs->submitted_crc =
        state->draft.draft_crc;


    return true;
}


/* ============================================================================
 * ENTER TEACHING
 * ============================================================================
 */

void state_teaching_enter(
    TeachingState *state,
    const TeachingConfig *config,
    uint32_t programId
)
{
    if (state == NULL)
    {
        return;
    }


    memset(
        state,
        0,
        sizeof(*state)
    );


    if (
        config == NULL ||
        !config_is_valid(config)
    )
    {
        state->error =
            TEACH_ERR_NULL_ARGUMENT;

        state->status_message_id =
            TEACH_MSG_ACTION_REJECTED;

        state->initialized =
            false;

        return;
    }


    state->config =
        *config;


    state->draft.program_id =
        programId;


    state->draft.global_speed_scale =
        1.0F;


    state->teaching_speed_mps =
        config->default_speed_mps;


    state->selected_orientation_mode =
        TEACH_ORIENTATION_CONSTANT;


    state->selected_circle_direction =
        TEACH_CIRCLE_CCW;


    state->phase =
        TEACH_PHASE_WAIT_SEGMENT_SELECTION;


    state->draft.status =
        TEACH_DRAFT_EMPTY;


    state->status_message_id =
        TEACH_MSG_SELECT_SEGMENT;


    state->initialized =
        true;


    state->last_event_accepted =
        false;
}


/* ============================================================================
 * RESET DRAFT
 * ============================================================================
 */

void state_teaching_reset_draft(
    TeachingState *state
)
{
    if (state == NULL)
    {
        return;
    }


    TeachingConfig config =
        state->config;


    const uint32_t programId =
        state->draft.program_id;


    const uint32_t nextRevision =
        state->draft.draft_revision +
        1U;


    const float speed =
        state->teaching_speed_mps;


    const bool initialized =
        state->initialized;


    memset(
        state,
        0,
        sizeof(*state)
    );


    state->config =
        config;


    state->draft.program_id =
        programId;


    state->draft.draft_revision =
        nextRevision;


    state->draft.global_speed_scale =
        1.0F;


    state->draft.status =
        TEACH_DRAFT_EMPTY;


    state->teaching_speed_mps =
        speed;


    state->selected_orientation_mode =
        TEACH_ORIENTATION_CONSTANT;


    state->selected_circle_direction =
        TEACH_CIRCLE_CCW;


    state->phase =
        TEACH_PHASE_WAIT_SEGMENT_SELECTION;


    state->status_message_id =
        TEACH_MSG_DRAFT_RESET;


    state->initialized =
        initialized;


    state->last_event_accepted =
        true;
}


/* ============================================================================
 * OUTPUTS
 * ============================================================================
 */

void state_teaching_get_outputs(
    const TeachingState *state,
    TeachingOutputs *outputs
)
{
    if (
        state == NULL ||
        outputs == NULL
    )
    {
        return;
    }


    clear_outputs(
        outputs
    );


    outputs->status_message_id =
        state->status_message_id;


    outputs->error =
        state->error;


    outputs->phase =
        state->phase;


    outputs->current_segment_number =
        (uint16_t)(
            state->draft.segment_count +
            1U
        );


    outputs->next_point_number =
        (
            state->phase ==
            TEACH_PHASE_WAIT_POINT
        )
        ?
        (uint8_t)(
            state->captured_point_count +
            1U
        )
        :
        0U;


    outputs->displayed_speed_mps =
        state->teaching_speed_mps;


    outputs->record_allowed =
        (
            state->phase ==
            TEACH_PHASE_WAIT_POINT
        );


    outputs->validate_allowed =
        (
            state->draft.segment_count >
            0U
        )
        &&
        !(
            state->phase ==
            TEACH_PHASE_WAIT_POINT
            &&
            state->captured_point_count >
            0U
        )
        &&
        (
            state->draft.status !=
            TEACH_DRAFT_SUBMITTED
        );
}


/* ============================================================================
 * TEACHING STEP
 * ============================================================================
 */

StateStepResult state_teaching_step(
    TeachingState *state,
    const RobotConfig *robot,
    const TeachingRuntimeInputs *runtime,
    TeachingEvent event,
    TeachingOutputs *outputs
)
{
    if (
        state == NULL ||
        robot == NULL ||
        runtime == NULL ||
        outputs == NULL
    )
    {
        return
            STATE_STEP_FAILED;
    }


    if (!state->initialized)
    {
        return
            STATE_STEP_FAILED;
    }


    TeachingInputs inputs;


    if (
        !build_teaching_inputs(
            robot,
            runtime,
            &inputs
        )
    )
    {
        return
            STATE_STEP_FAILED;
    }


    clear_outputs(
        outputs
    );


    /* ------------------------------------------------------------------------
     * RESET
     * ------------------------------------------------------------------------
     *
     * Preserved teammate behavior:
     *
     * reset is non-motion and has immediate priority.
     */

    if (
        event ==
        TEACH_EVENT_RESET
    )
    {
        state_teaching_reset_draft(
            state
        );


        outputs->draft_was_reset =
            true;


        state->last_event_accepted =
            true;


        state_teaching_get_outputs(
            state,
            outputs
        );


        outputs->draft_was_reset =
            true;


        return
            STATE_STEP_RUNNING;
    }


    /* ------------------------------------------------------------------------
     * LOCKED DRAFT
     * ------------------------------------------------------------------------
     */

    if (
        state->draft.status ==
        TEACH_DRAFT_SUBMITTED
    )
    {
        set_error(
            state,
            TEACH_ERR_DRAFT_LOCKED
        );


        state->last_event_accepted =
            false;


        state_teaching_get_outputs(
            state,
            outputs
        );


        /*
         * The global FSM should normally already have left TEACHING.
         */
        return
            STATE_STEP_COMPLETE;
    }


    bool accepted =
        false;


    /* ------------------------------------------------------------------------
     * EVENT PROCESSING
     * ------------------------------------------------------------------------
     *
     * This switch preserves the teammate event sequence and behavior.
     */

    switch (event)
    {
        case TEACH_EVENT_NONE:
        {
            accepted =
                true;

            break;
        }


        case TEACH_EVENT_SELECT_LINE:
        case TEACH_EVENT_SELECT_ARC:
        case TEACH_EVENT_SELECT_CIRCLE:
        {
            if (
                state->phase ==
                TEACH_PHASE_WAIT_POINT
                &&
                state->captured_point_count >
                0U
            )
            {
                set_error(
                    state,
                    TEACH_ERR_SEGMENT_INCOMPLETE
                );

                break;
            }


            begin_segment(
                state,
                (
                    event ==
                    TEACH_EVENT_SELECT_LINE
                )
                ?
                TEACH_SEGMENT_LINE
                :
                (
                    event ==
                    TEACH_EVENT_SELECT_ARC
                )
                ?
                TEACH_SEGMENT_ARC
                :
                TEACH_SEGMENT_CIRCLE
            );


            accepted =
                true;

            break;
        }


        case TEACH_EVENT_RECORD_POINT:
        {
            accepted =
                record_point(
                    state,
                    &inputs
                );

            break;
        }


        case TEACH_EVENT_VALIDATE_PATH:
        {
            accepted =
                submit_for_validation(
                    state,
                    outputs
                );

            break;
        }


        case TEACH_EVENT_SPEED_INCREASE:
        {
            accepted =
                change_speed(
                    state,
                    state->teaching_speed_mps +
                    state->config.speed_step_mps
                );

            break;
        }


        case TEACH_EVENT_SPEED_DECREASE:
        {
            accepted =
                change_speed(
                    state,
                    state->teaching_speed_mps -
                    state->config.speed_step_mps
                );

            break;
        }


        case TEACH_EVENT_SPEED_DEFAULT:
        {
            accepted =
                change_speed(
                    state,
                    state->config.default_speed_mps
                );

            break;
        }


        default:
        {
            set_error(
                state,
                TEACH_ERR_NULL_ARGUMENT
            );

            break;
        }
    }


    state->last_event_accepted =
        accepted;


    /* ------------------------------------------------------------------------
     * REFRESH OUTPUTS
     * ------------------------------------------------------------------------
     *
     * Preserve one-cycle validation pulse values exactly as in the teammate
     * implementation.
     */

    const bool validationRequest =
        outputs->validation_request;


    const uint32_t submittedProgramId =
        outputs->submitted_program_id;


    const uint32_t submittedRevision =
        outputs->submitted_revision;


    const uint32_t submittedCrc =
        outputs->submitted_crc;


    state_teaching_get_outputs(
        state,
        outputs
    );


    outputs->validation_request =
        validationRequest;


    outputs->submitted_program_id =
        submittedProgramId;


    outputs->submitted_revision =
        submittedRevision;


    outputs->submitted_crc =
        submittedCrc;


    /*
     * The only normal completion condition of the global TEACHING state:
     *
     *      teammate logic successfully submits a frozen draft
     *      and requests PATH VALIDATION.
     */
    if (outputs->validation_request)
    {
        return
            STATE_STEP_COMPLETE;
    }


    /*
     * A rejected Teaching action is intentionally NOT STATE_STEP_FAILED.
     * The TeachingError tells the HMI why it was rejected and the operator
     * may retry.
     */
    return
        STATE_STEP_RUNNING;
}


/* ============================================================================
 * OPTIONAL TEACHING SELECTIONS
 * ============================================================================
 */

bool state_teaching_set_circle_direction(
    TeachingState *state,
    TeachingCircleDirection direction
)
{
    if (
        state == NULL
        ||
        (
            direction !=
            TEACH_CIRCLE_CCW
            &&
            direction !=
            TEACH_CIRCLE_CW
        )
    )
    {
        return false;
    }


    state->selected_circle_direction =
        direction;


    return true;
}


bool state_teaching_set_orientation_mode(
    TeachingState *state,
    TeachingOrientationMode mode
)
{
    if (
        state == NULL
        ||
        (
            mode !=
            TEACH_ORIENTATION_CONSTANT
            &&
            mode !=
            TEACH_ORIENTATION_INTERPOLATED
        )
    )
    {
        return false;
    }


    state->selected_orientation_mode =
        mode;


    return true;
}


/* ============================================================================
 * NAME HELPERS
 * ============================================================================
 */

const char *state_teaching_phase_name(
    TeachingPhase phase
)
{
    switch (phase)
    {
        case TEACH_PHASE_WAIT_SEGMENT_SELECTION:
            return "WAIT_SEGMENT_SELECTION";

        case TEACH_PHASE_WAIT_POINT:
            return "WAIT_POINT";

        case TEACH_PHASE_SEGMENT_COMPLETE:
            return "SEGMENT_COMPLETE";

        case TEACH_PHASE_SUBMITTING_DRAFT:
            return "SUBMITTING_DRAFT";

        default:
            return "UNKNOWN_PHASE";
    }
}


const char *state_teaching_error_name(
    TeachingError error
)
{
    switch (error)
    {
        case TEACH_ERR_NONE:
            return "NONE";

        case TEACH_ERR_NULL_ARGUMENT:
            return "NULL_ARGUMENT";

        case TEACH_ERR_NO_SEGMENT_SELECTED:
            return "NO_SEGMENT_SELECTED";

        case TEACH_ERR_ROBOT_NOT_HOMED:
            return "ROBOT_NOT_HOMED";

        case TEACH_ERR_DRIVES_NOT_READY:
            return "DRIVES_NOT_READY";

        case TEACH_ERR_ROBOT_NOT_SETTLED:
            return "ROBOT_NOT_SETTLED";

        case TEACH_ERR_GUIDANCE_INACTIVE:
            return "GUIDANCE_INACTIVE";

        case TEACH_ERR_MOTION_NOT_PERMITTED:
            return "MOTION_NOT_PERMITTED";

        case TEACH_ERR_SAFETY_ACTIVE:
            return "SAFETY_ACTIVE";

        case TEACH_ERR_GLOBAL_FAULT_ACTIVE:
            return "GLOBAL_FAULT_ACTIVE";

        case TEACH_ERR_DUPLICATE_POINT:
            return "DUPLICATE_POINT";

        case TEACH_ERR_COLLINEAR_POINTS:
            return "COLLINEAR_POINTS";

        case TEACH_ERR_SEGMENT_INCOMPLETE:
            return "SEGMENT_INCOMPLETE";

        case TEACH_ERR_DRAFT_EMPTY:
            return "DRAFT_EMPTY";

        case TEACH_ERR_SEGMENT_CAPACITY:
            return "SEGMENT_CAPACITY";

        case TEACH_ERR_INVALID_SPEED:
            return "INVALID_SPEED";

        case TEACH_ERR_INVALID_MEASUREMENT:
            return "INVALID_MEASUREMENT";

        case TEACH_ERR_FRAME_CHANGED:
            return "FRAME_CHANGED";

        case TEACH_ERR_TOOL_CHANGED:
            return "TOOL_CHANGED";

        case TEACH_ERR_CALIBRATION_CHANGED:
            return "CALIBRATION_CHANGED";

        case TEACH_ERR_DRAFT_LOCKED:
            return "DRAFT_LOCKED";

        default:
            return "UNKNOWN_ERROR";
    }
}


const char *state_teaching_segment_name(
    TeachingSegmentType type
)
{
    switch (type)
    {
        case TEACH_SEGMENT_LINE:
            return "LINE";

        case TEACH_SEGMENT_ARC:
            return "ARC";

        case TEACH_SEGMENT_CIRCLE:
            return "CIRCLE";

        default:
            return "NONE";
    }
}
