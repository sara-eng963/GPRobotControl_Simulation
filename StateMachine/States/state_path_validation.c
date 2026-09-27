#include "state_path_validation.h"

#include "../../ControlCore/Kinematics/control_fk.h"
#include "../../ControlCore/Math/math3d.h"
#include "../../ServoDrive/A6EC/a6ec_drive.h"
#include "../../ControlCore/Analysis/singularity.h"
#include "../../ControlCore/Kinematics/control_jacobian.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PV_EPSILON          (1.0e-12)
#define PV_CRC32_POLYNOMIAL (0xEDB88320UL)


typedef struct
{
    JointVector q;
    Vec3 desired_position;
    Quat desired_orientation;

} PvGeneratedSample;


/* ============================================================================
 * CRC HELPERS
 * ============================================================================ */

static uint32_t crc32_update(
    uint32_t crc,
    const void *input,
    size_t length
)
{
    const uint8_t *data =
        (const uint8_t *)input;

    for (size_t i = 0U; i < length; ++i)
    {
        crc ^=
            data[i];

        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            const uint32_t mask =
                (uint32_t)-(int32_t)(crc & 1U);

            crc =
                (crc >> 1U) ^
                (PV_CRC32_POLYNOMIAL & mask);
        }
    }

    return crc;
}


uint32_t state_path_validation_calculate_draft_crc(
    const TaughtProgram *program
)
{
    if (program == NULL)
    {
        return 0U;
    }

    /*
     * A corrupted count must never make the validation-side CRC walk outside
     * the fixed Teaching arrays. Valid Teaching drafts are unaffected by these
     * guards and therefore produce exactly the same CRC as state_teaching.c.
     */
    if (program->segment_count > TEACHING_MAX_SEGMENTS)
    {
        return 0U;
    }

    uint32_t crc =
        0xFFFFFFFFUL;

#define CRC_FIELD(field_) \
    do \
    { \
        crc = crc32_update( \
            crc, \
            &(field_), \
            sizeof(field_) \
        ); \
    } while (0)

    CRC_FIELD(program->program_id);
    CRC_FIELD(program->draft_revision);
    CRC_FIELD(program->segment_count);
    CRC_FIELD(program->global_speed_scale);

    for (uint16_t s = 0U; s < program->segment_count; ++s)
    {
        const TaughtSegment *segment =
            &program->segments[s];

        if (segment->point_count > TEACHING_MAX_POINTS_PER_SEG)
        {
            return 0U;
        }

        CRC_FIELD(segment->segment_id);
        CRC_FIELD(segment->type);
        CRC_FIELD(segment->point_count);
        CRC_FIELD(segment->speed_mps);
        CRC_FIELD(segment->circle_direction);
        CRC_FIELD(segment->orientation_mode);

        for (uint8_t p = 0U; p < segment->point_count; ++p)
        {
            const TaughtPoint *point =
                &segment->points[p];

            crc =
                crc32_update(
                    crc,
                    point->position_m,
                    sizeof(point->position_m)
                );

            crc =
                crc32_update(
                    crc,
                    point->orientation_quat,
                    sizeof(point->orientation_quat)
                );

            crc =
                crc32_update(
                    crc,
                    point->joint_position_rad,
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


static uint32_t calculate_artifact_crc(
    const ValidatedTrajectory *artifact
)
{
    uint32_t crc =
        0xFFFFFFFFUL;

#define CRC_FIELD(field_) \
    do \
    { \
        crc = crc32_update( \
            crc, \
            &(field_), \
            sizeof(field_) \
        ); \
    } while (0)

    CRC_FIELD(artifact->program_id);
    CRC_FIELD(artifact->source_revision);
    CRC_FIELD(artifact->source_crc);
    CRC_FIELD(artifact->sample_data_crc);
    CRC_FIELD(artifact->sample_count);
    CRC_FIELD(artifact->segment_count);
    CRC_FIELD(artifact->sample_period_us);
    CRC_FIELD(artifact->duration_s);
    CRC_FIELD(artifact->path_length_m);

    for (uint16_t i = 0U; i < artifact->segment_count; ++i)
    {
        const PvSegmentIndex *segment =
            &artifact->segments[i];

        CRC_FIELD(segment->first_sample);
        CRC_FIELD(segment->sample_count);
        CRC_FIELD(segment->segment_type);
    }

#undef CRC_FIELD

    return
        ~crc;
}


/* ============================================================================
 * BASIC HELPERS
 * ============================================================================ */

static void clear_outputs(
    PathValidationOutputs *outputs
)
{
    if (outputs != NULL)
    {
        memset(
            outputs,
            0,
            sizeof(*outputs)
        );
    }
}


static void refresh_outputs(
    const PathValidationState *state,
    PathValidationOutputs *outputs
)
{
    if (state == NULL || outputs == NULL)
    {
        return;
    }

    clear_outputs(outputs);

    outputs->report =
        state->report;

    outputs->trajectory_ready =
        (state->result == PV_RESULT_VALID);
}


static void set_error(
    PathValidationState *state,
    PathValidationError error
)
{
    if (state == NULL)
    {
        return;
    }

    state->error =
        error;

    state->result =
        PV_RESULT_INVALID;

    state->phase =
        PV_PHASE_INVALID;

    state->report.error =
        error;

    state->report.result =
        PV_RESULT_INVALID;

    state->report.phase =
        PV_PHASE_INVALID;

    state->report.failed_segment =
        state->segment_index;

    state->report.failed_sample =
        state->sample_index;

    if (state->storage_open)
    {
        if (state->storage.abort != NULL)
        {
            state->storage.abort(
                state->storage.context
            );
        }

        state->storage_open =
            false;
    }
}


static bool finite_real(
    real_t value
)
{
    return
        isfinite(value) != 0;
}


static bool point_numeric_valid(
    const TaughtPoint *point
)
{
    if (point == NULL)
    {
        return false;
    }

    real_t quaternion_norm_squared =
        0.0;

    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (!isfinite(point->position_m[i]))
        {
            return false;
        }
    }

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (!isfinite(point->orientation_quat[i]))
        {
            return false;
        }

        quaternion_norm_squared +=
            (real_t)point->orientation_quat[i] *
            (real_t)point->orientation_quat[i];
    }

    for (uint8_t i = 0U; i < PATH_VALIDATION_DOF; ++i)
    {
        if (!isfinite(point->joint_position_rad[i]))
        {
            return false;
        }
    }

    return
        quaternion_norm_squared > PV_EPSILON;
}


static Vec3 taught_position_to_vec3(
    const TaughtPoint *point
)
{
    Vec3 result =
    {
        .v =
        {
            (real_t)point->position_m[0],
            (real_t)point->position_m[1],
            (real_t)point->position_m[2]
        }
    };

    return result;
}


static Quat taught_orientation_to_quat(
    const TaughtPoint *point
)
{
    Quat result =
    {
        .w = (real_t)point->orientation_quat[0],
        .x = (real_t)point->orientation_quat[1],
        .y = (real_t)point->orientation_quat[2],
        .z = (real_t)point->orientation_quat[3]
    };

    return
        quat_normalize(result);
}


static JointVector taught_joints_to_joint_vector(
    const TaughtPoint *point
)
{
    JointVector result =
    {
        .q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };

    for (uint8_t j = 0U; j < PATH_VALIDATION_DOF; ++j)
    {
        result.q[j] =
            (real_t)point->joint_position_rad[j];
    }

    return result;
}


static bool config_and_dependencies_valid(
    const RobotConfig *robot,
    const PathValidationConfig *config,
    const PathValidationServices *services,
    const PathValidationWorkspace *workspace,
    const PathValidationStorage *storage,
    const ValidatedTrajectory *artifact,
    const TaughtProgram *program
)
{
    if (
        robot == NULL ||
        config == NULL ||
        workspace == NULL ||
        storage == NULL ||
        artifact == NULL ||
        program == NULL
    )
    {
        return false;
    }

    if (
        robot->identity.dof != ROBOT_DOF ||
        !finite_real(config->default_tcp_speed_mps) ||
        config->default_tcp_speed_mps <= 0.0 ||
        !finite_real(config->max_tcp_speed_mps) ||
        config->max_tcp_speed_mps <= 0.0 ||
        config->default_tcp_speed_mps > config->max_tcp_speed_mps ||
        !finite_real(config->max_tcp_acceleration_mps2) ||
        config->max_tcp_acceleration_mps2 <= 0.0 ||
        !finite_real(config->max_tcp_jerk_mps3) ||
        config->max_tcp_jerk_mps3 <= 0.0 ||
        !finite_real(config->minimum_segment_length_m) ||
        config->minimum_segment_length_m <= 0.0 ||
        !finite_real(config->maximum_fk_position_error_m) ||
        config->maximum_fk_position_error_m <= 0.0 ||
        !finite_real(config->maximum_fk_orientation_error_rad) ||
        config->maximum_fk_orientation_error_rad <= 0.0 ||
        !finite_real(config->minimum_singularity_sigma) ||
        config->minimum_singularity_sigma < 0.0 ||
        !finite_real(config->maximum_joint_step_rad) ||
        config->maximum_joint_step_rad <= 0.0 ||
        !finite_real(config->maximum_position_quantization_error_rad) ||
        config->maximum_position_quantization_error_rad < 0.0 ||
        config->geometry_points_per_segment < 3U ||
        !finite_real(config->arc_length_spacing_m) ||
        config->arc_length_spacing_m <= 0.0 ||
        !finite_real(config->ik_parameters.lambdaMax) ||
        config->ik_parameters.lambdaMax < 0.0 ||
        !finite_real(config->ik_parameters.sigmaThreshold) ||
        config->ik_parameters.sigmaThreshold < 0.0
    )
    {
        return false;
    }

    if (
        workspace->geometry_capacity < config->geometry_points_per_segment ||
        workspace->raw_geometry == NULL ||
        workspace->arc_geometry == NULL ||
        workspace->l_original == NULL ||
        workspace->l_arc == NULL ||
        workspace->ik_scratch == NULL
    )
    {
        return false;
    }

    if (
        storage->begin == NULL ||
        storage->write_sample == NULL ||
        storage->commit == NULL ||
        storage->capacity_samples == 0U
    )
    {
        return false;
    }

    if (
        config->require_collision_callback &&
        (services == NULL || services->collision_free == NULL)
    )
    {
        return false;
    }

    for (uint8_t j = 0U; j < PATH_VALIDATION_DOF; ++j)
    {
        if (
            !isfinite(robot->limits.qMin[j]) ||
            !isfinite(robot->limits.qMax[j]) ||
            robot->limits.qMin[j] >= robot->limits.qMax[j] ||
            !isfinite(robot->limits.qdMax[j]) ||
            robot->limits.qdMax[j] <= 0.0
        )
        {
            return false;
        }

        if (config->check_joint_acceleration)
        {
            const real_t acceleration_limit =
                robot->limits.qddMaxDefined
                ? (real_t)robot->limits.qddMax[j]
                : config->joint_acceleration_max_rad_s2[j];

            if (
                !finite_real(acceleration_limit) ||
                acceleration_limit <= 0.0
            )
            {
                return false;
            }
        }
    }

    return true;
}


static void configure_controlcore_workspaces(
    PathValidationState *state
)
{
    state->line_workspace.geometryCapacity =
        state->workspace->geometry_capacity;

    state->line_workspace.rawGeometry =
        state->workspace->raw_geometry;

    state->line_workspace.arcGeometry =
        state->workspace->arc_geometry;

    state->line_workspace.lOriginal =
        state->workspace->l_original;

    state->line_workspace.lArc =
        state->workspace->l_arc;

    state->line_workspace.ikScratch =
        state->workspace->ik_scratch;


    state->circular_workspace.geometryCapacity =
        state->workspace->geometry_capacity;

    state->circular_workspace.rawGeometry =
        state->workspace->raw_geometry;

    state->circular_workspace.arcGeometry =
        state->workspace->arc_geometry;

    state->circular_workspace.lOriginal =
        state->workspace->l_original;

    state->circular_workspace.lArc =
        state->workspace->l_arc;

    state->circular_workspace.ikScratch =
        state->workspace->ik_scratch;
}


/* ============================================================================
 * CONTROLCORE STREAM PREPARATION
 * ============================================================================ */

static void map_line_init_failure(
    PathValidationState *state
)
{
    switch (single_line_stream_status(&state->line_stream))
    {
        case SINGLE_LINE_STREAM_GEOMETRY_FAILED:
            set_error(state, PV_ERR_DEGENERATE_GEOMETRY);
            break;

        case SINGLE_LINE_STREAM_IK_FAILED:
            set_error(state, PV_ERR_IK_FAILED);
            break;

        case SINGLE_LINE_STREAM_PROFILE_FAILED:
        case SINGLE_LINE_STREAM_INVALID_ARGUMENT:
        default:
            set_error(state, PV_ERR_INVALID_PARAMETER);
            break;
    }
}


static void map_circular_init_failure(
    PathValidationState *state
)
{
    switch (single_circular_stream_status(&state->circular_stream))
    {
        case SINGLE_CIRCULAR_STREAM_GEOMETRY_FAILED:
            set_error(state, PV_ERR_DEGENERATE_GEOMETRY);
            break;

        case SINGLE_CIRCULAR_STREAM_IK_FAILED:
            set_error(state, PV_ERR_IK_FAILED);
            break;

        case SINGLE_CIRCULAR_STREAM_PROFILE_FAILED:
        case SINGLE_CIRCULAR_STREAM_INVALID_ARGUMENT:
        default:
            set_error(state, PV_ERR_INVALID_PARAMETER);
            break;
    }
}


static bool prepare_segment(
    PathValidationState *state
)
{
    const TaughtSegment *segment =
        &state->source_program->segments[state->segment_index];

    real_t speed =
        segment->speed_mps > 0.0F
        ? (real_t)segment->speed_mps
        : state->config.default_tcp_speed_mps;

    speed *=
        (real_t)state->source_program->global_speed_scale;

    if (
        !finite_real(speed) ||
        speed <= 0.0 ||
        speed > state->config.max_tcp_speed_mps
    )
    {
        set_error(state, PV_ERR_INVALID_PARAMETER);
        return false;
    }

    const JointVector seed =
        state->previous_q_valid
        ? state->previous_q
        : taught_joints_to_joint_vector(&segment->points[0]);

    state->active_stream_is_line =
        false;

    state->current_segment_length_m =
        0.0;


    if (segment->type == TEACH_SEGMENT_LINE)
    {
        SingleLineRequest request;

        memset(
            &request,
            0,
            sizeof(request)
        );

        request.qSeed =
            seed;

        request.startPosition =
            taught_position_to_vec3(&segment->points[0]);

        request.endPosition =
            taught_position_to_vec3(&segment->points[1]);

        request.startOrientation =
            taught_orientation_to_quat(&segment->points[0]);

        request.endOrientation =
            taught_orientation_to_quat(&segment->points[1]);

        request.numGeometryPointsPerSegment =
            state->config.geometry_points_per_segment;

        request.arcLengthSpacing =
            state->config.arc_length_spacing_m;

        request.desiredTCPSpeed =
            speed;

        request.desiredTCPAccel =
            state->config.max_tcp_acceleration_mps2;

        request.desiredTCPJerk =
            state->config.max_tcp_jerk_mps3;

        request.dt =
            PATH_VALIDATION_SAMPLE_PERIOD_S;

        request.ikParameters =
            state->config.ik_parameters;

        if (
            !single_line_stream_init(
                &state->line_stream,
                state->robot,
                &request,
                &state->line_workspace
            )
        )
        {
            map_line_init_failure(state);
            return false;
        }

        state->active_stream_is_line =
            true;

        state->current_segment_length_m =
            state->line_stream.segmentLength;

        /*
         * The first sample of every later segment is the previous segment's
         * endpoint. The teammate validator intentionally skipped it so the
         * stored artifact stays on one global 1 ms time grid without a
         * duplicated zero-dt boundary sample.
         *
         * The stream structure is public and incremental, so advancing the
         * next sample index preserves that exact integration behavior while
         * still using ControlCore for all trajectory mathematics.
         */
        if (state->segment_index > 0U)
        {
            if (single_line_stream_sample_count(&state->line_stream) <= 1U)
            {
                set_error(state, PV_ERR_ZERO_LENGTH_SEGMENT);
                return false;
            }

            state->line_stream.nextSampleIndex =
                1U;
        }
    }
    else if (
        segment->type == TEACH_SEGMENT_ARC ||
        segment->type == TEACH_SEGMENT_CIRCLE
    )
    {
        if (
            segment->type == TEACH_SEGMENT_CIRCLE &&
            segment->circle_direction != TEACH_CIRCLE_CCW &&
            segment->circle_direction != TEACH_CIRCLE_CW
        )
        {
            set_error(state, PV_ERR_INVALID_CIRCLE_DIRECTION);
            return false;
        }

        SingleCircularRequest request;

        memset(
            &request,
            0,
            sizeof(request)
        );

        request.type =
            segment->type == TEACH_SEGMENT_ARC
            ? CIRCULAR_SEGMENT_ARC
            : CIRCULAR_SEGMENT_FULL_CIRCLE;

        request.qSeed =
            seed;

        request.point1 =
            taught_position_to_vec3(&segment->points[0]);

        request.point2 =
            taught_position_to_vec3(&segment->points[1]);

        request.point3 =
            taught_position_to_vec3(&segment->points[2]);

        request.direction =
            (int)segment->circle_direction;

        request.startOrientation =
            taught_orientation_to_quat(&segment->points[0]);

        /*
         * Preserve the teammate validator's full-circle rule: the endpoint of
         * a full circle has the same orientation as its start point.
         */
        request.endOrientation =
            segment->type == TEACH_SEGMENT_CIRCLE
            ? taught_orientation_to_quat(&segment->points[0])
            : taught_orientation_to_quat(&segment->points[2]);

        request.numGeometryPointsPerSegment =
            state->config.geometry_points_per_segment;

        request.arcLengthSpacing =
            state->config.arc_length_spacing_m;

        request.desiredTCPSpeed =
            speed;

        request.desiredTCPAccel =
            state->config.max_tcp_acceleration_mps2;

        request.desiredTCPJerk =
            state->config.max_tcp_jerk_mps3;

        request.dt =
            PATH_VALIDATION_SAMPLE_PERIOD_S;

        request.ikParameters =
            state->config.ik_parameters;

        if (
            !single_circular_stream_init(
                &state->circular_stream,
                state->robot,
                &request,
                &state->circular_workspace
            )
        )
        {
            map_circular_init_failure(state);
            return false;
        }

        state->current_segment_length_m =
            state->circular_stream.segmentLength;

        if (state->segment_index > 0U)
        {
            if (single_circular_stream_sample_count(&state->circular_stream) <= 1U)
            {
                set_error(state, PV_ERR_ZERO_LENGTH_SEGMENT);
                return false;
            }

            state->circular_stream.nextSampleIndex =
                1U;
        }
    }
    else
    {
        set_error(state, PV_ERR_UNSUPPORTED_SEGMENT);
        return false;
    }


    if (
        !finite_real(state->current_segment_length_m) ||
        state->current_segment_length_m < state->config.minimum_segment_length_m
    )
    {
        set_error(state, PV_ERR_ZERO_LENGTH_SEGMENT);
        return false;
    }


    state->artifact->segments[state->segment_index].first_sample =
        state->sample_index;

    state->artifact->segments[state->segment_index].sample_count =
        0U;

    state->artifact->segments[state->segment_index].segment_type =
        (uint8_t)segment->type;

    return true;
}


/* ============================================================================
 * GENERATE ONE SAMPLE THROUGH CONTROLCORE
 * ============================================================================ */

static bool generate_controlcore_sample(
    PathValidationState *state,
    PvGeneratedSample *generated
)
{
    if (state->active_stream_is_line)
    {
        SingleLineStreamSample sample;

        if (
            !single_line_stream_next(
                &state->line_stream,
                &sample
            )
        )
        {
            switch (single_line_stream_status(&state->line_stream))
            {
                case SINGLE_LINE_STREAM_IK_FAILED:
                    set_error(state, PV_ERR_IK_FAILED);
                    break;

                case SINGLE_LINE_STREAM_GEOMETRY_FAILED:
                    set_error(state, PV_ERR_DEGENERATE_GEOMETRY);
                    break;

                case SINGLE_LINE_STREAM_PROFILE_FAILED:
                case SINGLE_LINE_STREAM_INVALID_ARGUMENT:
                case SINGLE_LINE_STREAM_FINISHED:
                default:
                    set_error(state, PV_ERR_INVALID_PARAMETER);
                    break;
            }

            return false;
        }

        generated->q =
            sample.q;

        generated->desired_position =
            sample.pDesired;

        generated->desired_orientation =
            sample.quatDesired;

        return true;
    }


    SingleCircularStreamSample sample;

    if (
        !single_circular_stream_next(
            &state->circular_stream,
            &sample
        )
    )
    {
        switch (single_circular_stream_status(&state->circular_stream))
        {
            case SINGLE_CIRCULAR_STREAM_IK_FAILED:
                set_error(state, PV_ERR_IK_FAILED);
                break;

            case SINGLE_CIRCULAR_STREAM_GEOMETRY_FAILED:
                set_error(state, PV_ERR_DEGENERATE_GEOMETRY);
                break;

            case SINGLE_CIRCULAR_STREAM_PROFILE_FAILED:
            case SINGLE_CIRCULAR_STREAM_INVALID_ARGUMENT:
            case SINGLE_CIRCULAR_STREAM_FINISHED:
            default:
                set_error(state, PV_ERR_INVALID_PARAMETER);
                break;
        }

        return false;
    }

    generated->q =
        sample.q;

    generated->desired_position =
        sample.pDesired;

    generated->desired_orientation =
        sample.quatDesired;

    return true;
}


static bool active_stream_finished(
    const PathValidationState *state
)
{
    return
        state->active_stream_is_line
        ? single_line_stream_is_finished(&state->line_stream)
        : single_circular_stream_is_finished(&state->circular_stream);
}


static void reseed_active_stream_from_accepted_command(
    PathValidationState *state,
    JointVector accepted_q
)
{
    /*
     * ControlCore normally carries the previous raw IK solution forward.
     * Path Validation instead continues from the quantized joint command that
     * was actually accepted for CSP storage, preserving the teammate state's
     * "validate what will be commanded" behavior.
     */
    if (state->active_stream_is_line)
    {
        state->line_stream.qSeed =
            accepted_q;
    }
    else
    {
        state->circular_stream.qSeed =
            accepted_q;
    }
}


/* ============================================================================
 * VALIDATE + STORE ONE GENERATED SAMPLE
 * ============================================================================ */

static bool validate_and_store_sample(
    PathValidationState *state,
    const PvGeneratedSample *generated
)
{
    if (state->sample_index >= state->storage.capacity_samples)
    {
        set_error(state, PV_ERR_SAMPLE_CAPACITY);
        return false;
    }
    /* ------------------------------------------------------------------------
     * QUANTIZE THROUGH THE EXISTING A6-EC CSP POSITION CONVERSION
     * ------------------------------------------------------------------------ */

    PvExecutionSample execution_sample;
    JointVector quantized_q;

    memset(
        &execution_sample,
        0,
        sizeof(execution_sample)
    );

    memset(
        &quantized_q,
        0,
        sizeof(quantized_q)
    );

    for (uint8_t j = 0U; j < PATH_VALIDATION_DOF; ++j)
    {
        if (!finite_real(generated->q.q[j]))
        {
            state->report.failed_joint = j;
            set_error(state, PV_ERR_POSITION_CONVERSION);
            return false;
        }

        const int32_t units =
            a6ec_joint_rad_to_position_units(
                generated->q.q[j]
            );

        const real_t represented_q =
            (real_t)a6ec_position_units_to_joint_rad(
                units
            );

        if (
            !finite_real(represented_q) ||
            fabs(represented_q - generated->q.q[j]) >
                state->config.maximum_position_quantization_error_rad
        )
        {
            state->report.failed_joint = j;
            set_error(state, PV_ERR_POSITION_CONVERSION);
            return false;
        }

        execution_sample.target_position_units[j] =
            units;

        quantized_q.q[j] =
            represented_q;
    }
    /* ------------------------------------------------------------------------
 * SINGULARITY CHECK OF THE ACTUAL QUANTIZED COMMAND
 * ------------------------------------------------------------------------ */

real_t J[ROBOT_DOF][ROBOT_DOF];

control_jacobian(
    state->robot,
    quantized_q.q,
    J
);


SingularityResult singularity;

if (
    !singularity_analyze(
        J,
        state->config.minimum_singularity_sigma,
        &singularity
    )
)
{
    set_error(
        state,
        PV_ERR_SINGULARITY_MARGIN
    );

    return false;
}


if (!singularity.safe)
{
    set_error(
        state,
        PV_ERR_SINGULARITY_MARGIN
    );

    return false;
}


if (
    singularity.sigmaMin <
    state->report.minimum_sigma_seen
)
{
    state->report.minimum_sigma_seen =
        singularity.sigmaMin;
}


    /* ------------------------------------------------------------------------
     * INDEPENDENT FK CHECK OF THE QUANTIZED COMMAND
     * ------------------------------------------------------------------------
     *
     * ControlCore already performs an FK check for the raw IK solution. We do
     * this second check deliberately after A6-EC quantization because the
     * teammate validator validates the command that will actually be stored.
     */

    double achieved_pose_array[4][4];

    control_fk(
        state->robot,
        quantized_q.q,
        achieved_pose_array
    );

    Vec3 achieved_position =
    {
        .v =
        {
            achieved_pose_array[0][3],
            achieved_pose_array[1][3],
            achieved_pose_array[2][3]
        }
    };

    Mat3 achieved_rotation;

    memset(
        &achieved_rotation,
        0,
        sizeof(achieved_rotation)
    );

    for (uint8_t r = 0U; r < 3U; ++r)
    {
        for (uint8_t c = 0U; c < 3U; ++c)
        {
            achieved_rotation.m[r][c] =
                achieved_pose_array[r][c];
        }
    }

    const real_t position_error =
        vec3_norm(
            vec3_sub(
                generated->desired_position,
                achieved_position
            )
        );

    const real_t orientation_error =
        rotation_error(
            quat_to_rotm(generated->desired_orientation),
            achieved_rotation
        );

    if (position_error > state->report.max_fk_position_error_m)
    {
        state->report.max_fk_position_error_m =
            position_error;
    }

    if (orientation_error > state->report.max_fk_orientation_error_rad)
    {
        state->report.max_fk_orientation_error_rad =
            orientation_error;
    }

    if (
        !finite_real(position_error) ||
        position_error > state->config.maximum_fk_position_error_m
    )
    {
        set_error(state, PV_ERR_FK_POSITION);
        return false;
    }

    if (
        !finite_real(orientation_error) ||
        orientation_error > state->config.maximum_fk_orientation_error_rad
    )
    {
        set_error(state, PV_ERR_FK_ORIENTATION);
        return false;
    }


    /* ------------------------------------------------------------------------
     * JOINT POSITION / CONTINUITY / VELOCITY / ACCELERATION
     * ------------------------------------------------------------------------ */

    JointVector current_qd;

    memset(
        &current_qd,
        0,
        sizeof(current_qd)
    );

    const real_t current_time_s =
        (real_t)state->sample_index *
        PATH_VALIDATION_SAMPLE_PERIOD_S;

    for (uint8_t j = 0U; j < PATH_VALIDATION_DOF; ++j)
    {
        if (
            quantized_q.q[j] < state->robot->limits.qMin[j] ||
            quantized_q.q[j] > state->robot->limits.qMax[j]
        )
        {
            state->report.failed_joint = j;
            set_error(state, PV_ERR_JOINT_POSITION);
            return false;
        }

        if (state->previous_q_valid)
        {
            const real_t dt =
                current_time_s -
                state->previous_time_s;

            if (dt <= PV_EPSILON)
            {
                set_error(state, PV_ERR_INVALID_PARAMETER);
                return false;
            }

            const real_t delta_q =
                quantized_q.q[j] -
                state->previous_q.q[j];

            if (
                fabs(delta_q) >
                state->config.maximum_joint_step_rad
            )
            {
                state->report.failed_joint = j;
                set_error(state, PV_ERR_JOINT_DISCONTINUITY);
                return false;
            }

            current_qd.q[j] =
                delta_q / dt;

            const real_t abs_velocity =
                fabs(current_qd.q[j]);

            if (
                abs_velocity >
                state->report.peak_joint_velocity_rad_s[j]
            )
            {
                state->report.peak_joint_velocity_rad_s[j] =
                    abs_velocity;
            }

            if (
                abs_velocity >
                state->robot->limits.qdMax[j]
            )
            {
                state->report.failed_joint = j;
                set_error(state, PV_ERR_JOINT_VELOCITY);
                return false;
            }

            if (state->previous_qd_valid)
            {
                const real_t acceleration =
                    (
                        current_qd.q[j] -
                        state->previous_qd.q[j]
                    ) /
                    dt;

                const real_t abs_acceleration =
                    fabs(acceleration);

                if (
                    abs_acceleration >
                    state->report.peak_joint_acceleration_rad_s2[j]
                )
                {
                    state->report.peak_joint_acceleration_rad_s2[j] =
                        abs_acceleration;
                }

                if (state->config.check_joint_acceleration)
                {
                    const real_t acceleration_limit =
                        state->robot->limits.qddMaxDefined
                        ? (real_t)state->robot->limits.qddMax[j]
                        : state->config.joint_acceleration_max_rad_s2[j];

                    if (abs_acceleration > acceleration_limit)
                    {
                        state->report.failed_joint = j;
                        set_error(state, PV_ERR_JOINT_ACCELERATION);
                        return false;
                    }
                }
            }
        }
    }


    /* ------------------------------------------------------------------------
     * COLLISION CALLBACK
     * ------------------------------------------------------------------------ */

    if (
        state->services.collision_free != NULL &&
        !state->services.collision_free(
            &quantized_q,
            state->segment_index,
            state->sample_index,
            state->services.context
        )
    )
    {
        set_error(state, PV_ERR_COLLISION);
        return false;
    }


    /* ------------------------------------------------------------------------
     * STORE THE ACCEPTED 1 ms CSP SAMPLE
     * ------------------------------------------------------------------------ */

    if (
        !state->storage.write_sample(
            state->sample_index,
            &execution_sample,
            state->storage.context
        )
    )
    {
        set_error(state, PV_ERR_STORAGE);
        return false;
    }

    state->sample_crc_state =
        crc32_update(
            state->sample_crc_state,
            &execution_sample,
            sizeof(execution_sample)
        );


    /* ------------------------------------------------------------------------
     * UPDATE DERIVATIVE / IK SEED HISTORY
     * ------------------------------------------------------------------------ */

    state->previous_q =
        quantized_q;

    state->previous_qd =
        current_qd;

    state->previous_time_s =
        current_time_s;

    state->previous_qd_valid =
        state->previous_q_valid;

    state->previous_q_valid =
        true;

    reseed_active_stream_from_accepted_command(
        state,
        quantized_q
    );


    ++state->artifact->segments[state->segment_index].sample_count;
    ++state->sample_index;

    state->artifact->sample_count =
        state->sample_index;

    return true;
}


/* ============================================================================
 * ENTER
 * ============================================================================ */

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

    state->initialized =
        true;

    state->phase =
        PV_PHASE_IDLE;

    state->result =
        PV_RESULT_NONE;

    state->error =
        PV_ERR_NONE;

    state->report.minimum_sigma_seen =
        DBL_MAX;

    state->report.failed_joint =
        UINT8_MAX;


    if (
        robot == NULL ||
        config == NULL ||
        workspace == NULL ||
        storage == NULL ||
        artifact_storage == NULL ||
        frozen_program == NULL
    )
    {
        set_error(state, PV_ERR_NULL_ARGUMENT);
        return;
    }

    if (
        config->require_collision_callback &&
        (services == NULL || services->collision_free == NULL)
    )
    {
        set_error(state, PV_ERR_CALLBACK_MISSING);
        return;
    }

    if (
        !config_and_dependencies_valid(
            robot,
            config,
            services,
            workspace,
            storage,
            artifact_storage,
            frozen_program
        )
    )
    {
        set_error(state, PV_ERR_INVALID_PARAMETER);
        return;
    }


    state->robot =
        robot;

    state->config =
        *config;

    if (services != NULL)
    {
        state->services =
            *services;
    }
    else
    {
        memset(
            &state->services,
            0,
            sizeof(state->services)
        );
    }

    state->workspace =
        workspace;

    state->storage =
        *storage;

    state->artifact =
        artifact_storage;

    state->source_program =
        frozen_program;

    state->expected_revision =
        submitted_revision;

    state->expected_crc =
        submitted_crc;

    configure_controlcore_workspaces(
        state
    );

    memset(
        state->artifact,
        0,
        sizeof(*state->artifact)
    );

    state->sample_crc_state =
        0xFFFFFFFFUL;

    state->previous_q_valid =
        false;

    state->previous_qd_valid =
        false;


    if (
        !state->storage.begin(
            state->storage.context
        )
    )
    {
        set_error(state, PV_ERR_STORAGE);
        return;
    }

    state->storage_open =
        true;

    state->result =
        PV_RESULT_RUNNING;

    state->phase =
        PV_PHASE_SNAPSHOT_CHECK;

    state->report.result =
        PV_RESULT_RUNNING;

    state->report.phase =
        PV_PHASE_SNAPSHOT_CHECK;
}


/* ============================================================================
 * STEP
 * ============================================================================ */

StateStepResult state_path_validation_step(
    PathValidationState *state,
    uint16_t sample_budget,
    PathValidationOutputs *outputs
)
{
    if (
        state == NULL ||
        outputs == NULL ||
        !state->initialized
    )
    {
        return
            STATE_STEP_FAILED;
    }

    if (state->result != PV_RESULT_RUNNING)
    {
        refresh_outputs(
            state,
            outputs
        );

        return
            STATE_STEP_COMPLETE;
    }

    if (sample_budget == 0U)
    {
        sample_budget =
            1U;
    }

    uint16_t samples_used =
        0U;


    while (
        state->result == PV_RESULT_RUNNING &&
        samples_used < sample_budget
    )
    {
        switch (state->phase)
        {
            /* ----------------------------------------------------------------
             * VERIFY FROZEN TEACHING SNAPSHOT
             * ---------------------------------------------------------------- */
            case PV_PHASE_SNAPSHOT_CHECK:
            {
                if (
                    state->source_program->draft_revision !=
                    state->expected_revision
                )
                {
                    set_error(state, PV_ERR_REVISION_MISMATCH);
                    break;
                }

                const uint32_t calculated_crc =
                    state_path_validation_calculate_draft_crc(
                        state->source_program
                    );

                if (
                    state->source_program->draft_crc !=
                        state->expected_crc ||
                    calculated_crc !=
                        state->expected_crc
                )
                {
                    set_error(state, PV_ERR_DRAFT_CRC_MISMATCH);
                    break;
                }

                if (
                    state->source_program->segment_count == 0U
                )
                {
                    set_error(state, PV_ERR_EMPTY_PROGRAM);
                    break;
                }

                if (
                    state->source_program->segment_count >
                    TEACHING_MAX_SEGMENTS
                )
                {
                    set_error(state, PV_ERR_INVALID_PARAMETER);
                    break;
                }

                state->phase =
                    PV_PHASE_STRUCTURAL_CHECK;

                break;
            }


            /* ----------------------------------------------------------------
             * PROGRAM STRUCTURE / METADATA CONSISTENCY
             * ---------------------------------------------------------------- */
            case PV_PHASE_STRUCTURAL_CHECK:
            {
                bool reference_set =
                    false;

                uint16_t reference_frame =
                    0U;

                uint16_t reference_tool =
                    0U;

                uint32_t reference_calibration =
                    0U;


                for (
                    uint16_t s = 0U;
                    s < state->source_program->segment_count;
                    ++s
                )
                {
                    const TaughtSegment *segment =
                        &state->source_program->segments[s];

                    uint8_t required_points =
                        0U;

                    switch (segment->type)
                    {
                        case TEACH_SEGMENT_LINE:
                            required_points = 2U;
                            break;

                        case TEACH_SEGMENT_ARC:
                        case TEACH_SEGMENT_CIRCLE:
                            required_points = 3U;
                            break;

                        default:
                            set_error(state, PV_ERR_UNSUPPORTED_SEGMENT);
                            break;
                    }

                    if (state->result != PV_RESULT_RUNNING)
                    {
                        break;
                    }

                    if (
                        !segment->segment_valid ||
                        segment->point_count != required_points
                    )
                    {
                        set_error(state, PV_ERR_INCOMPLETE_SEGMENT);
                        break;
                    }

                    for (uint8_t p = 0U; p < required_points; ++p)
                    {
                        const TaughtPoint *point =
                            &segment->points[p];

                        if (
                            !point->point_valid ||
                            !point_numeric_valid(point)
                        )
                        {
                            set_error(state, PV_ERR_INVALID_POINT);
                            break;
                        }

                        if (!reference_set)
                        {
                            reference_frame =
                                point->frame_id;

                            reference_tool =
                                point->tool_id;

                            reference_calibration =
                                point->calibration_version;

                            reference_set =
                                true;
                        }

                        if (point->frame_id != reference_frame)
                        {
                            set_error(state, PV_ERR_FRAME_MISMATCH);
                            break;
                        }

                        if (point->tool_id != reference_tool)
                        {
                            set_error(state, PV_ERR_TOOL_MISMATCH);
                            break;
                        }

                        if (
                            point->calibration_version !=
                            reference_calibration
                        )
                        {
                            set_error(state, PV_ERR_CALIBRATION_MISMATCH);
                            break;
                        }
                    }

                    if (state->result != PV_RESULT_RUNNING)
                    {
                        break;
                    }
                }

                if (state->result == PV_RESULT_RUNNING)
                {
                    state->phase =
                        PV_PHASE_PREPARE_SEGMENT;
                }

                break;
            }


            /* ----------------------------------------------------------------
             * DISPATCH TAUGHT SEGMENT TO OUR CONTROLCORE STREAMING PIPELINE
             * ---------------------------------------------------------------- */
            case PV_PHASE_PREPARE_SEGMENT:
            {
                if (!prepare_segment(state))
                {
                    break;
                }

                state->phase =
                    PV_PHASE_GENERATE_AND_VALIDATE_SAMPLE;

                break;
            }


            /* ----------------------------------------------------------------
             * ONE CONTROLCORE SAMPLE -> VALIDATE -> CSP STORAGE
             * ---------------------------------------------------------------- */
            case PV_PHASE_GENERATE_AND_VALIDATE_SAMPLE:
            {
                PvGeneratedSample generated;

                memset(
                    &generated,
                    0,
                    sizeof(generated)
                );

                if (
                    !generate_controlcore_sample(
                        state,
                        &generated
                    )
                )
                {
                    break;
                }

                if (
                    !validate_and_store_sample(
                        state,
                        &generated
                    )
                )
                {
                    break;
                }

                ++samples_used;


                if (active_stream_finished(state))
                {
                    state->artifact->path_length_m +=
                        state->current_segment_length_m;

                    ++state->segment_index;

                    if (
                        state->segment_index >=
                        state->source_program->segment_count
                    )
                    {
                        state->phase =
                            PV_PHASE_FINALIZE;
                    }
                    else
                    {
                        state->phase =
                            PV_PHASE_PREPARE_SEGMENT;
                    }
                }

                break;
            }


            /* ----------------------------------------------------------------
             * COMMIT IMMUTABLE VALIDATED ARTIFACT
             * ---------------------------------------------------------------- */
            case PV_PHASE_FINALIZE:
            {
                state->artifact->program_id =
                    state->source_program->program_id;

                state->artifact->source_revision =
                    state->expected_revision;

                state->artifact->source_crc =
                    state->expected_crc;

                state->artifact->segment_count =
                    state->source_program->segment_count;

                state->artifact->sample_period_us =
                    PATH_VALIDATION_SAMPLE_PERIOD_US;

                state->artifact->duration_s =
                    state->artifact->sample_count > 0U
                    ? (
                        (real_t)(state->artifact->sample_count - 1U) *
                        PATH_VALIDATION_SAMPLE_PERIOD_S
                    )
                    : 0.0;

                state->artifact->sample_data_crc =
                    ~state->sample_crc_state;

                state->artifact->artifact_crc =
                    calculate_artifact_crc(
                        state->artifact
                    );

                if (
                    !state->storage.commit(
                        state->artifact,
                        state->storage.context
                    )
                )
                {
                    set_error(state, PV_ERR_STORAGE);
                    break;
                }

                state->storage_open =
                    false;

                state->result =
                    PV_RESULT_VALID;

                state->phase =
                    PV_PHASE_VALID;

                state->error =
                    PV_ERR_NONE;

                state->report.result =
                    PV_RESULT_VALID;

                state->report.phase =
                    PV_PHASE_VALID;

                state->report.error =
                    PV_ERR_NONE;

                state->report.progress_0_to_1 =
                    1.0;

                break;
            }


            default:
            {
                set_error(state, PV_ERR_INVALID_PARAMETER);
                break;
            }
        }


        state->report.phase =
            state->phase;

        state->report.result =
            state->result;

        state->report.error =
            state->error;

        if (
            state->result == PV_RESULT_RUNNING &&
            state->source_program->segment_count > 0U
        )
        {
            state->report.progress_0_to_1 =
                (real_t)state->segment_index /
                (real_t)state->source_program->segment_count;
        }
    }


    refresh_outputs(
        state,
        outputs
    );

    return
        state->result == PV_RESULT_RUNNING
        ? STATE_STEP_RUNNING
        : STATE_STEP_COMPLETE;
}


/* ============================================================================
 * CANCEL / OUTPUTS
 * ============================================================================ */

void state_path_validation_cancel(
    PathValidationState *state
)
{
    if (
        state == NULL ||
        state->result != PV_RESULT_RUNNING
    )
    {
        return;
    }

    if (
        state->storage_open &&
        state->storage.abort != NULL
    )
    {
        state->storage.abort(
            state->storage.context
        );
    }

    state->storage_open =
        false;

    state->error =
        PV_ERR_CANCELLED;

    state->result =
        PV_RESULT_CANCELLED;

    state->phase =
        PV_PHASE_INVALID;

    state->report.error =
        PV_ERR_CANCELLED;

    state->report.result =
        PV_RESULT_CANCELLED;

    state->report.phase =
        PV_PHASE_INVALID;
}


void state_path_validation_get_outputs(
    const PathValidationState *state,
    PathValidationOutputs *outputs
)
{
    refresh_outputs(
        state,
        outputs
    );
}


/* ============================================================================
 * NAME HELPERS
 * ============================================================================ */

const char *state_path_validation_phase_name(
    PathValidationPhase phase
)
{
    switch (phase)
    {
        case PV_PHASE_IDLE:
            return "IDLE";

        case PV_PHASE_SNAPSHOT_CHECK:
            return "SNAPSHOT_CHECK";

        case PV_PHASE_STRUCTURAL_CHECK:
            return "STRUCTURAL_CHECK";

        case PV_PHASE_PREPARE_SEGMENT:
            return "PREPARE_SEGMENT";

        case PV_PHASE_GENERATE_AND_VALIDATE_SAMPLE:
            return "GENERATE_AND_VALIDATE_SAMPLE";

        case PV_PHASE_FINALIZE:
            return "FINALIZE";

        case PV_PHASE_VALID:
            return "VALID";

        case PV_PHASE_INVALID:
            return "INVALID";

        default:
            return "UNKNOWN";
    }
}


const char *state_path_validation_error_name(
    PathValidationError error
)
{
    switch (error)
    {
        case PV_ERR_NONE:
            return "NONE";

        case PV_ERR_NULL_ARGUMENT:
            return "NULL_ARGUMENT";

        case PV_ERR_BUSY:
            return "BUSY";

        case PV_ERR_EMPTY_PROGRAM:
            return "EMPTY_PROGRAM";

        case PV_ERR_REVISION_MISMATCH:
            return "REVISION_MISMATCH";

        case PV_ERR_DRAFT_CRC_MISMATCH:
            return "DRAFT_CRC_MISMATCH";

        case PV_ERR_UNSUPPORTED_SEGMENT:
            return "UNSUPPORTED_SEGMENT";

        case PV_ERR_INCOMPLETE_SEGMENT:
            return "INCOMPLETE_SEGMENT";

        case PV_ERR_INVALID_POINT:
            return "INVALID_POINT";

        case PV_ERR_DEGENERATE_GEOMETRY:
            return "DEGENERATE_GEOMETRY";

        case PV_ERR_ARC_DIRECTION:
            return "ARC_DIRECTION";

        case PV_ERR_INVALID_CIRCLE_DIRECTION:
            return "INVALID_CIRCLE_DIRECTION";

        case PV_ERR_FRAME_MISMATCH:
            return "FRAME_MISMATCH";

        case PV_ERR_TOOL_MISMATCH:
            return "TOOL_MISMATCH";

        case PV_ERR_CALIBRATION_MISMATCH:
            return "CALIBRATION_MISMATCH";

        case PV_ERR_INVALID_PARAMETER:
            return "INVALID_PARAMETER";

        case PV_ERR_ZERO_LENGTH_SEGMENT:
            return "ZERO_LENGTH_SEGMENT";

        case PV_ERR_SAMPLE_CAPACITY:
            return "SAMPLE_CAPACITY";

        case PV_ERR_IK_FAILED:
            return "IK_FAILED";

        case PV_ERR_FK_POSITION:
            return "FK_POSITION";

        case PV_ERR_FK_ORIENTATION:
            return "FK_ORIENTATION";

        case PV_ERR_JOINT_POSITION:
            return "JOINT_POSITION";

        case PV_ERR_JOINT_VELOCITY:
            return "JOINT_VELOCITY";

        case PV_ERR_JOINT_ACCELERATION:
            return "JOINT_ACCELERATION";

        case PV_ERR_JOINT_DISCONTINUITY:
            return "JOINT_DISCONTINUITY";

        case PV_ERR_POSITION_CONVERSION:
            return "POSITION_CONVERSION";

        case PV_ERR_SINGULARITY_MARGIN:
            return "SINGULARITY_MARGIN";

        case PV_ERR_COLLISION:
            return "COLLISION";

        case PV_ERR_STORAGE:
            return "STORAGE";

        case PV_ERR_CALLBACK_MISSING:
            return "CALLBACK_MISSING";

        case PV_ERR_CANCELLED:
            return "CANCELLED";

        default:
            return "UNKNOWN";
    }
}
