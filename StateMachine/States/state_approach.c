#include "state_approach.h"


#include <math.h>
#include <stddef.h>
#include <string.h>


/* ============================================================================
 * QUINTIC PROFILE PEAK CONSTANTS
 * ============================================================================
 *
 * For:
 *
 *      s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5
 *
 * the normalized analytical peaks are:
 *
 *      max |ds/dtau|       = 1.875
 *      max |d2s/dtau2|     = 5.773502692...
 *      max |d3s/dtau3|     = 60
 *
 * These constants are used ONLY to choose a sufficiently long duration.
 *
 * Actual trajectory generation/evaluation is delegated to the existing
 * ControlCore JointTrajectory module.
 * ============================================================================
 */

#define APPROACH_QUINTIC_PEAK_VELOCITY      (1.875)
#define APPROACH_QUINTIC_PEAK_ACCELERATION  (5.773502692)
#define APPROACH_QUINTIC_PEAK_JERK          (60.0)

#define APPROACH_EPSILON                    (1.0e-12)
#define APPROACH_CYCLIC_FEEDBACK_TIMEOUT_MS (100U)


/* ============================================================================
 * INTERNAL HELPERS
 * ============================================================================ */

static bool finite_real(
    real_t value
)
{
    return
        isfinite(value);
}


static bool finite_joint_vector(
    const JointVector *q
)
{
    if (q == NULL)
    {
        return false;
    }


    for (int joint = 0;
         joint < ROBOT_DOF;
         ++joint)
    {
        if (!finite_real(q->q[joint]))
        {
            return false;
        }
    }


    return true;
}


static StateStepResult approach_fail(
    ApproachState *state,
    ApproachError error,
    uint8_t failed_joint
)
{
    if (state == NULL)
    {
        return
            STATE_STEP_FAILED;
    }


    state->error =
        error;

    state->failed_joint =
        failed_joint;

    state->phase =
        APPROACH_PHASE_FAILED;

    state->result =
        APPROACH_RESULT_FAILED;


    return
        STATE_STEP_FAILED;
}


static StateStepResult approach_abort(
    ApproachState *state,
    ApproachError reason
)
{
    if (state == NULL)
    {
        return
            STATE_STEP_FAILED;
    }


    state->error =
        reason;

    state->failed_joint =
        0U;

    state->phase =
        APPROACH_PHASE_ABORTED;

    state->result =
        APPROACH_RESULT_ABORTED;


    return
        STATE_STEP_COMPLETE;
}


static bool config_valid(
    const ApproachConfig *config,
    const RobotConfig *robot
)
{
    if (
        config == NULL ||
        robot == NULL ||
        robot->identity.dof != ROBOT_DOF ||
        !finite_real(config->duration_safety_factor) ||
        config->duration_safety_factor < 1.0 ||
        !finite_real(config->minimum_leg_duration_s) ||
        config->minimum_leg_duration_s <= 0.0 ||
        !finite_real(config->maximum_leg_duration_s) ||
        config->maximum_leg_duration_s <
            config->minimum_leg_duration_s ||
        !finite_real(config->final_position_tolerance_rad) ||
        config->final_position_tolerance_rad <= 0.0 ||
        !finite_real(config->following_error_limit_rad) ||
        config->following_error_limit_rad <= 0.0 ||
        config->required_stable_cycles == 0U ||
        config->maximum_verification_cycles == 0U ||
        config->validation_samples_per_step == 0U
    )
    {
        return false;
    }


    for (int joint = 0;
         joint < ROBOT_DOF;
         ++joint)
    {
        if (
            !finite_real(robot->limits.qMin[joint]) ||
            !finite_real(robot->limits.qMax[joint]) ||
            robot->limits.qMin[joint] >=
                robot->limits.qMax[joint] ||
            !finite_real(robot->limits.qdMax[joint]) ||
            robot->limits.qdMax[joint] <= 0.0
        )
        {
            return false;
        }


        if (
            robot->limits.qddMaxDefined &&
            (
                !finite_real(robot->limits.qddMax[joint]) ||
                robot->limits.qddMax[joint] <= 0.0
            )
        )
        {
            return false;
        }


        if (
            config->use_jerk_limits &&
            (
                !finite_real(config->jerk_max_rad_s3[joint]) ||
                config->jerk_max_rad_s3[joint] <= 0.0
            )
        )
        {
            return false;
        }
    }


    return true;
}


static bool services_valid(
    const ApproachServices *services,
    bool collision_required
)
{
    if (
        services == NULL ||
        services->read_validated_sample == NULL
    )
    {
        return false;
    }


    if (
        collision_required &&
        services->collision_free == NULL
    )
    {
        return false;
    }


    return true;
}


static bool joint_vector_in_limits(
    const ApproachState *state,
    const JointVector *q,
    uint8_t *failed_joint
)
{
    if (
        state == NULL ||
        state->robot == NULL ||
        q == NULL ||
        !finite_joint_vector(q)
    )
    {
        return false;
    }


    for (int joint = 0;
         joint < ROBOT_DOF;
         ++joint)
    {
        if (
            q->q[joint] <
                state->robot->limits.qMin[joint]
            ||
            q->q[joint] >
                state->robot->limits.qMax[joint]
        )
        {
            if (failed_joint != NULL)
            {
                *failed_joint =
                    (uint8_t)(joint + 1);
            }


            return false;
        }
    }


    return true;
}


/* ============================================================================
 * DRIVE-SERVICE ACCESS AND AVATAR JOINT-UNIT CONVERSION
 * ============================================================================ */

static bool position_scales_valid(
    const AvatarMPositionScale position_scales[ROBOT_DOF]
)
{
    if (position_scales == NULL)
    {
        return false;
    }

    for (size_t axis = 0U; axis < ROBOT_DOF; ++axis)
    {
        if (!avatar_m_position_scale_valid(
                &position_scales[axis]))
        {
            return false;
        }
    }

    return true;
}


static bool drive_port_valid(const ApproachState *state)
{
    return state != NULL && joint_drive_port_valid(&state->drive);
}


static bool read_feedback_ready(
    ApproachState *state,
    uint32_t now_ms,
    JointVector *actual_q,
    int32_t actual_units[ROBOT_DOF]
)
{
    if (
        state == NULL ||
        actual_q == NULL ||
        !drive_port_valid(state) ||
        !position_scales_valid(state->position_scales)
    )
    {
        return false;
    }

    if (!joint_drive_port_healthy(&state->drive, now_ms))
    {
        state->error =
            APPROACH_ERR_COMMUNICATION;

        state->failed_joint =
            0U;

        return false;
    }

    for (size_t axis = 0U;
         axis < ROBOT_DOF;
         ++axis)
    {
        JointDriveAxisFeedback feedback;
        if (!joint_drive_port_read_axis(&state->drive, axis, &feedback) ||
            !feedback.feedback_valid)
        {
            state->error =
                APPROACH_ERR_FEEDBACK;

            state->failed_joint =
                (uint8_t)(axis + 1U);

            return false;
        }

        if (!feedback.operation_enabled)
        {
            state->error =
                APPROACH_ERR_DRIVE_NOT_READY;

            state->failed_joint =
                (uint8_t)(axis + 1U);

            return false;
        }

        double q = 0.0;

        if (!avatar_m_position_units_to_joint_rad(
                &state->position_scales[axis],
                feedback.actual_position_units,
                &q) ||
            !isfinite(q))
        {
            state->error =
                APPROACH_ERR_FEEDBACK;

            state->failed_joint =
                (uint8_t)(axis + 1U);

            return false;
        }

        actual_q->q[axis] =
            (real_t)q;

        if (actual_units != NULL)
        {
            actual_units[axis] =
                feedback.actual_position_units;
        }
    }

    return true;
}


static bool capture_tpdo_counts(
    ApproachState *state
)
{
    for (size_t axis = 0U;
         axis < ROBOT_DOF;
         ++axis)
    {
        if (!joint_drive_port_feedback_sequence(&state->drive, axis, &state->command_tpdo_count[axis]))
            return false;
    }
    return true;
}


static bool command_feedback_arrived(
    const ApproachState *state
)
{
    for (size_t axis = 0U;
         axis < ROBOT_DOF;
         ++axis)
    {
        uint32_t current_sequence = 0U;
        if (
            !joint_drive_port_feedback_sequence(&state->drive, axis, &current_sequence) ||
            current_sequence <= state->command_tpdo_count[axis]
        )
        {
            return false;
        }
    }

    return true;
}


static bool command_due(
    const ApproachState *state,
    uint32_t now_ms
)
{
    return
        !state->command_clock_started ||
        (uint32_t)(
            now_ms -
            state->last_command_ms
        ) >=
            state->command_period_ms;
}


static bool write_target_units(
    ApproachState *state,
    const int32_t target_units[ROBOT_DOF],
    uint32_t now_ms
)
{
    if (
        state == NULL ||
        target_units == NULL ||
        !drive_port_valid(state)
    )
    {
        return false;
    }

    if (!capture_tpdo_counts(state))
    {
        state->error = APPROACH_ERR_COMMUNICATION;
        state->failed_joint = 0U;
        return false;
    }

    if (!joint_drive_port_send_targets(&state->drive, target_units))
    {
        state->error =
            APPROACH_ERR_COMMUNICATION;

        state->failed_joint =
            0U;

        return false;
    }

    state->last_command_ms =
        now_ms;

    state->command_clock_started =
        true;

    state->awaiting_feedback =
        true;

    return true;
}


static bool send_joint_target(
    ApproachState *state,
    const JointVector *target_q,
    bool use_exact_final_units,
    uint32_t now_ms
)
{
    if (
        state == NULL ||
        target_q == NULL ||
        !finite_joint_vector(target_q)
    )
    {
        return false;
    }

    int32_t target_units[ROBOT_DOF];

    if (use_exact_final_units)
    {
        memcpy(
            target_units,
            state->final_target_units,
            sizeof(target_units)
        );
    }
    else
    {
        for (size_t axis = 0U;
             axis < ROBOT_DOF;
             ++axis)
        {
            if (!avatar_m_joint_rad_to_position_units(
                    &state->position_scales[axis],
                    target_q->q[axis],
                    &target_units[axis]))
            {
                state->error =
                    APPROACH_ERR_POSITION_CONVERSION;

                state->failed_joint =
                    (uint8_t)(axis + 1U);

                return false;
            }
        }
    }

    if (!write_target_units(
            state,
            target_units,
            now_ms))
    {
        return false;
    }

    state->last_command =
        *target_q;

    state->last_command_valid =
        true;

    return true;
}


static bool hold_actual(
    ApproachState *state,
    uint32_t now_ms
)
{
    if (!command_due(state, now_ms))
    {
        return true;
    }

    JointVector actual_q;
    int32_t actual_units[ROBOT_DOF];

    memset(
        &actual_q,
        0,
        sizeof(actual_q)
    );

    if (!read_feedback_ready(
            state,
            now_ms,
            &actual_q,
            actual_units))
    {
        return false;
    }

    /*
     * Hold the exact measured AVATAR raw position.
     * This avoids a needless raw -> rad -> raw round trip.
     */
    if (!write_target_units(
            state,
            actual_units,
            now_ms))
    {
        return false;
    }

    state->last_command =
        actual_q;

    state->last_command_valid =
        true;

    return true;
}


static bool following_error_valid(
    ApproachState *state,
    const JointVector *actual_q
)
{
    if (
        state == NULL ||
        actual_q == NULL
    )
    {
        return false;
    }

    if (!state->last_command_valid)
    {
        return true;
    }

    for (int joint = 0;
         joint < ROBOT_DOF;
         ++joint)
    {
        if (
            fabs(
                actual_q->q[joint] -
                state->last_command.q[joint]
            )
            >
            state->config.following_error_limit_rad
        )
        {
            state->failed_joint =
                (uint8_t)(joint + 1);

            return false;
        }
    }

    return true;
}


/* ============================================================================
 * ROUTE / TRAJECTORY HELPERS
 * ============================================================================ */

static real_t calculate_leg_duration(
    const ApproachState *state,
    const JointVector *start,
    const JointVector *goal
)
{
    real_t duration =
        state->config.minimum_leg_duration_s;


    for (int joint = 0;
         joint < ROBOT_DOF;
         ++joint)
    {
        const real_t dq =
            fabs(
                goal->q[joint] -
                start->q[joint]
            );


        real_t required =
            APPROACH_QUINTIC_PEAK_VELOCITY *
            dq /
            state->robot->limits.qdMax[joint];


        if (required > duration)
        {
            duration =
                required;
        }


        if (state->robot->limits.qddMaxDefined)
        {
            required =
                sqrt(
                    APPROACH_QUINTIC_PEAK_ACCELERATION *
                    dq /
                    state->robot->limits.qddMax[joint]
                );


            if (required > duration)
            {
                duration =
                    required;
            }
        }


        if (state->config.use_jerk_limits)
        {
            required =
                cbrt(
                    APPROACH_QUINTIC_PEAK_JERK *
                    dq /
                    state->config.jerk_max_rad_s3[joint]
                );


            if (required > duration)
            {
                duration =
                    required;
            }
        }
    }


    duration *=
        state->config.duration_safety_factor;


    return duration;
}


static void build_route_from_request(
    ApproachState *state
)
{
    state->route_pose[0] =
        state->q_start;


    for (
        uint8_t i = 0U;
        i < state->request.clearance_pose_count;
        ++i
    )
    {
        state->route_pose[i + 1U] =
            state->request.clearance_poses[i];
    }


    state->route_pose[
        state->request.clearance_pose_count + 1U
    ] =
        state->final_target;


    state->total_legs =
        (uint8_t)(
            state->request.clearance_pose_count +
            1U
        );
}


static void replan_from_actual(
    ApproachState *state,
    const JointVector *actual_q
)
{
    JointVector remaining_goals[APPROACH_MAX_LEGS];


    const uint8_t remaining =
        (uint8_t)(
            state->total_legs -
            state->active_leg
        );


    for (
        uint8_t i = 0U;
        i < remaining;
        ++i
    )
    {
        remaining_goals[i] =
            state->route_pose[
                state->active_leg +
                i +
                1U
            ];
    }


    state->route_pose[0] =
        *actual_q;


    for (
        uint8_t i = 0U;
        i < remaining;
        ++i
    )
    {
        state->route_pose[i + 1U] =
            remaining_goals[i];
    }


    state->q_start =
        *actual_q;

    state->total_legs =
        remaining;

    state->active_leg =
        0U;

    state->sample_index =
        0U;

    state->validation_index =
        0U;

    state->last_command_valid =
        false;

    ++state->replan_count;
}


static bool prepare_leg(
    ApproachState *state,
    uint8_t leg
)
{
    if (
        state == NULL ||
        leg >= state->total_legs
    )
    {
        return false;
    }


    uint8_t failed_joint =
        0U;


    if (
        !joint_vector_in_limits(
            state,
            &state->route_pose[leg],
            &failed_joint
        )
        ||
        !joint_vector_in_limits(
            state,
            &state->route_pose[leg + 1U],
            &failed_joint
        )
    )
    {
        state->error =
            APPROACH_ERR_TARGET_LIMIT;

        state->failed_joint =
            failed_joint;

        return false;
    }


    real_t duration =
        calculate_leg_duration(
            state,
            &state->route_pose[leg],
            &state->route_pose[leg + 1U]
        );


    if (
        !finite_real(duration) ||
        duration <= 0.0
    )
    {
        state->error =
            APPROACH_ERR_PLAN_LIMIT;

        return false;
    }


    /*
     * Round UP to an exact number of 2 ms / 500 Hz intervals.
     *
     * This preserves the teammate Approach timing policy while using our
     * ControlCore JointTrajectory implementation.
     */
    const real_t dt =
        PATH_VALIDATION_SAMPLE_PERIOD_S;


    const real_t intervals_real =
        ceil(
            duration /
            dt
        );


    if (
        !finite_real(intervals_real) ||
        intervals_real < 1.0
    )
    {
        state->error =
            APPROACH_ERR_PLAN_LIMIT;

        return false;
    }


    duration =
        intervals_real *
        dt;


    if (
        duration >
        state->config.maximum_leg_duration_s +
        APPROACH_EPSILON
    )
    {
        state->error =
            APPROACH_ERR_PLAN_LIMIT;

        return false;
    }


    if (
        !joint_trajectory_init(
            &state->route_trajectory[leg],
            &state->route_pose[leg],
            &state->route_pose[leg + 1U],
            duration,
            dt
        )
    )
    {
        state->error =
            APPROACH_ERR_PLAN_LIMIT;

        return false;
    }


    /*
     * Independent limit verification using ControlCore's calculated peaks.
     */
    const JointTrajectory *trajectory =
        &state->route_trajectory[leg];


    for (int joint = 0;
         joint < ROBOT_DOF;
         ++joint)
    {
        if (
            trajectory->info
                .peakJointVelocity
                .q[joint]
            >
            state->robot->limits.qdMax[joint] +
            APPROACH_EPSILON
        )
        {
            state->error =
                APPROACH_ERR_PLAN_LIMIT;

            state->failed_joint =
                (uint8_t)(joint + 1);

            return false;
        }


        if (
            state->robot->limits.qddMaxDefined &&
            trajectory->info
                .peakJointAcceleration
                .q[joint]
            >
            state->robot->limits.qddMax[joint] +
            APPROACH_EPSILON
        )
        {
            state->error =
                APPROACH_ERR_PLAN_LIMIT;

            state->failed_joint =
                (uint8_t)(joint + 1);

            return false;
        }


        if (state->config.use_jerk_limits)
        {
            const real_t peak_joint_jerk =
                fabs(
                    trajectory->deltaQ.q[joint]
                )
                *
                trajectory->profile.info.peakPathJerk;


            if (
                peak_joint_jerk >
                state->config.jerk_max_rad_s3[joint] +
                APPROACH_EPSILON
            )
            {
                state->error =
                    APPROACH_ERR_PLAN_LIMIT;

                state->failed_joint =
                    (uint8_t)(joint + 1);

                return false;
            }
        }
    }


    return true;
}


/* ============================================================================
 * REPORT
 * ============================================================================ */

void state_approach_get_outputs(
    const ApproachState *state,
    ApproachOutputs *outputs
)
{
    if (
        state == NULL ||
        outputs == NULL
    )
    {
        return;
    }


    memset(
        outputs,
        0,
        sizeof(*outputs)
    );


    ApproachReport *report =
        &outputs->report;


    report->phase =
        state->phase;

    report->result =
        state->result;

    report->error =
        state->error;

    report->failed_joint =
        state->failed_joint;

    report->active_leg =
        state->active_leg;

    report->total_legs =
        state->total_legs;

    report->sample_index =
        state->sample_index;

    report->samples_sent =
        state->samples_sent;

    report->stable_cycles =
        state->stable_cycles;

    report->verification_cycles =
        state->verification_cycles;

    report->replan_count =
        state->replan_count;

    report->q_start =
        state->q_start;

    report->final_target =
        state->final_target;


    memcpy(
        report->final_target_units,
        state->final_target_units,
        sizeof(report->final_target_units)
    );


    if (
        state->active_leg <
        state->total_legs
    )
    {
        report->q_goal =
            state->route_pose[
                state->active_leg + 1U
            ];


        report->leg_duration_s =
            state->route_trajectory[
                state->active_leg
            ].info.duration;
    }
    else
    {
        report->q_goal =
            state->final_target;

        report->leg_duration_s =
            0.0;
    }


    size_t total_intervals =
        0U;

    size_t completed_intervals =
        0U;


    for (
        uint8_t leg = 0U;
        leg < state->total_legs;
        ++leg
    )
    {
        const size_t samples =
            joint_trajectory_sample_count(
                &state->route_trajectory[leg]
            );


        const size_t intervals =
            samples > 0U
            ? samples - 1U
            : 0U;


        total_intervals +=
            intervals;


        if (leg < state->active_leg)
        {
            completed_intervals +=
                intervals;
        }
    }


    completed_intervals +=
        state->sample_index;


    report->progress_0_to_1 =
        total_intervals == 0U
        ? 0.0
        : (real_t)completed_intervals /
          (real_t)total_intervals;


    if (
        report->progress_0_to_1 > 1.0 ||
        state->result ==
            APPROACH_RESULT_COMPLETE
    )
    {
        report->progress_0_to_1 =
            1.0;
    }
}


/* ============================================================================
 * ENTER
 * ============================================================================ */

void state_approach_enter(
    ApproachState *state,
    const RobotConfig *robot,
    const JointDrivePort *drive_port,
    const AvatarMPositionScale position_scales[ROBOT_DOF],
    const ApproachRequest *request,
    const ApproachConfig *config,
    const ApproachServices *services
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
        APPROACH_PHASE_IDLE;

    state->result =
        APPROACH_RESULT_NONE;

    state->error =
        APPROACH_ERR_NONE;


    if (
        robot == NULL ||
        drive_port == NULL ||
        position_scales == NULL ||
        request == NULL ||
        config == NULL ||
        services == NULL
    )
    {
        state->phase =
            APPROACH_PHASE_FAILED;

        state->result =
            APPROACH_RESULT_FAILED;

        state->error =
            APPROACH_ERR_NULL_ARGUMENT;

        return;
    }


    state->robot =
        robot;

    state->drive =
        *drive_port;

    state->position_scales =
        position_scales;

    state->command_period_ms =
        (uint32_t)(
            PATH_VALIDATION_SAMPLE_PERIOD_US /
            1000UL
        );

    state->request =
        *request;

    /*
     * Own the optional clearance-pose data after enter().
     *
     * The request itself may be a temporary supervisor object, so do not keep
     * a borrowed pointer to its clearance array.
     */
    if (
        request->clearance_pose_count <=
            APPROACH_MAX_CLEARANCE_POSES
        &&
        request->clearance_pose_count > 0U
        &&
        request->clearance_poses != NULL
    )
    {
        for (
            uint8_t i = 0U;
            i < request->clearance_pose_count;
            ++i
        )
        {
            state->clearance_poses[i] =
                request->clearance_poses[i];
        }

        state->request.clearance_poses =
            state->clearance_poses;
    }

    state->config =
        *config;

    state->services =
        *services;


    state->phase =
        APPROACH_PHASE_CHECK_REQUEST;

    state->result =
        APPROACH_RESULT_RUNNING;
}


/* ============================================================================
 * STEP
 * ============================================================================ */

StateStepResult state_approach_step(
    ApproachState *state,
    const ApproachControlInputs *inputs,
    uint32_t now_ms,
    ApproachOutputs *outputs
)
{
    if (
        state == NULL ||
        inputs == NULL ||
        outputs == NULL ||
        !state->initialized
    )
    {
        return
            STATE_STEP_FAILED;
    }


    if (
        !drive_port_valid(state) ||
        !position_scales_valid(state->position_scales) ||
        !joint_drive_port_poll(&state->drive, now_ms)
    )
    {
        const StateStepResult result =
            approach_fail(
                state,
                APPROACH_ERR_COMMUNICATION,
                0U
            );

        state_approach_get_outputs(
            state,
            outputs
        );

        return result;
    }


    if (
        state->result ==
        APPROACH_RESULT_COMPLETE
        ||
        state->result ==
        APPROACH_RESULT_ABORTED
    )
    {
        state_approach_get_outputs(
            state,
            outputs
        );

        return
            STATE_STEP_COMPLETE;
    }


    if (
        state->result ==
        APPROACH_RESULT_FAILED
    )
    {
        state_approach_get_outputs(
            state,
            outputs
        );

        return
            STATE_STEP_FAILED;
    }


    /* ------------------------------------------------------------------------
     * HIGHEST-PRIORITY SUPERVISORY CONDITIONS
     * ------------------------------------------------------------------------ */

    if (inputs->estop_active)
    {
        StateStepResult result =
            approach_fail(
                state,
                APPROACH_ERR_ESTOP,
                0U
            );


        state_approach_get_outputs(
            state,
            outputs
        );


        return result;
    }


    if (inputs->external_fault_active)
    {
        StateStepResult result =
            approach_fail(
                state,
                APPROACH_ERR_EXTERNAL_FAULT,
                0U
            );


        state_approach_get_outputs(
            state,
            outputs
        );


        return result;
    }


    if (inputs->reset_requested)
    {
        StateStepResult result =
            approach_abort(
                state,
                APPROACH_ERR_RESET_REQUESTED
            );


        state_approach_get_outputs(
            state,
            outputs
        );


        return result;
    }


    if (inputs->home_requested)
    {
        StateStepResult result =
            approach_abort(
                state,
                APPROACH_ERR_HOME_REQUESTED
            );


        state_approach_get_outputs(
            state,
            outputs
        );


        return result;
    }


    /*
     * Do not advance the motion sequence until every axis has returned a
     * fresh TPDO4 after the previous RPDO4 + SYNC command.
     */
    if (state->awaiting_feedback)
    {
        if (command_feedback_arrived(state))
        {
            state->awaiting_feedback =
                false;
        }
        else if (
            (uint32_t)(
                now_ms -
                state->last_command_ms
            ) >=
            APPROACH_CYCLIC_FEEDBACK_TIMEOUT_MS
        )
        {
            const StateStepResult result =
                approach_fail(
                    state,
                    APPROACH_ERR_CYCLIC_FEEDBACK,
                    0U
                );

            state_approach_get_outputs(
                state,
                outputs
            );

            return result;
        }
        else
        {
            state_approach_get_outputs(
                state,
                outputs
            );

            return
                STATE_STEP_RUNNING;
        }
    }


    /* ========================================================================
     * PHASE MACHINE
     * ======================================================================== */

    StateStepResult step_result =
        STATE_STEP_RUNNING;


    switch (state->phase)
    {
        /* --------------------------------------------------------------------
         * CHECK REQUEST
         * -------------------------------------------------------------------- */

        case APPROACH_PHASE_CHECK_REQUEST:
        {
            if (
                !config_valid(
                    &state->config,
                    state->robot
                )
                ||
                !drive_port_valid(state)
                ||
                !position_scales_valid(
                    state->position_scales
                )
                ||
                state->command_period_ms == 0U
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_INVALID_CONFIG,
                        0U
                    );

                break;
            }


            if (
                !services_valid(
                    &state->services,
                    state->config.require_collision_check
                )
            )
            {
                const ApproachError error =
                    (
                        state->config.require_collision_check &&
                        state->services.collision_free == NULL
                    )
                    ? APPROACH_ERR_COLLISION_CHECK_MISSING
                    : APPROACH_ERR_INVALID_CONFIG;


                step_result =
                    approach_fail(
                        state,
                        error,
                        0U
                    );

                break;
            }


            if (
                (
                    state->request.operation !=
                        APPROACH_OPERATION_PREVIEW
                    &&
                    state->request.operation !=
                        APPROACH_OPERATION_WELD
                )
                ||
                state->request.clearance_pose_count >
                    APPROACH_MAX_CLEARANCE_POSES
                ||
                (
                    state->request.clearance_pose_count > 0U
                    &&
                    state->request.clearance_poses == NULL
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_INVALID_REQUEST,
                        0U
                    );

                break;
            }


            state->phase =
                APPROACH_PHASE_LOAD_TARGET;

            break;
        }


        /* --------------------------------------------------------------------
         * LOAD SAMPLE 0 FROM COMMITTED VALIDATED TRAJECTORY
         * -------------------------------------------------------------------- */

        case APPROACH_PHASE_LOAD_TARGET:
        {
            const ValidatedTrajectory *trajectory =
                state->request.trajectory;


            if (
                trajectory == NULL ||
                !state->request.trajectory_ready
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_TRAJECTORY_NOT_VALID,
                        0U
                    );

                break;
            }


            if (
                trajectory->program_id !=
                    state->request.expected_program_id
                ||
                trajectory->source_revision !=
                    state->request.expected_source_revision
                ||
                trajectory->artifact_crc !=
                    state->request.expected_artifact_crc
                ||
                trajectory->sample_period_us !=
                    PATH_VALIDATION_SAMPLE_PERIOD_US
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_TRAJECTORY_MISMATCH,
                        0U
                    );

                break;
            }


            if (trajectory->sample_count == 0U)
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_EMPTY_TRAJECTORY,
                        0U
                    );

                break;
            }


            PvExecutionSample first_sample;


            memset(
                &first_sample,
                0,
                sizeof(first_sample)
            );


            if (
                !state->services.read_validated_sample(
                    0U,
                    &first_sample,
                    state->services.storage_context
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_STORAGE_READ,
                        0U
                    );

                break;
            }


            for (int joint = 0;
                 joint < ROBOT_DOF;
                 ++joint)
            {
                state->final_target_units[joint] =
                    first_sample
                        .target_position_units[joint];


                double target_q = 0.0;

                if (!avatar_m_position_units_to_joint_rad(
                        &state->position_scales[joint],
                        state->final_target_units[joint],
                        &target_q))
                {
                    step_result =
                        approach_fail(
                            state,
                            APPROACH_ERR_POSITION_CONVERSION,
                            (uint8_t)(joint + 1)
                        );

                    break;
                }

                state->final_target.q[joint] =
                    (real_t)target_q;
            }


            uint8_t failed_joint =
                0U;


            if (
                !joint_vector_in_limits(
                    state,
                    &state->final_target,
                    &failed_joint
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_TARGET_LIMIT,
                        failed_joint
                    );

                break;
            }


            state->phase =
                APPROACH_PHASE_READ_START;

            break;
        }


        /* --------------------------------------------------------------------
         * READ CURRENT ROBOT CONFIGURATION
         * -------------------------------------------------------------------- */

        case APPROACH_PHASE_READ_START:
        {
            int32_t actual_units[ROBOT_DOF];


            if (
                !read_feedback_ready(
                    state,
                    now_ms,
                    &state->q_start,
                    actual_units
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        state->error,
                        state->failed_joint
                    );

                break;
            }


            build_route_from_request(
                state
            );


            state->active_leg =
                0U;

            state->phase =
                APPROACH_PHASE_PREPARE_LEG;

            break;
        }


        /* --------------------------------------------------------------------
         * PREPARE EACH LEG WITH CONTROLCORE JOINT TRAJECTORY
         * -------------------------------------------------------------------- */

        case APPROACH_PHASE_PREPARE_LEG:
        {
            if (
                state->active_leg >=
                state->total_legs
            )
            {
                state->active_leg =
                    0U;

                state->validation_index =
                    0U;

                state->phase =
                    APPROACH_PHASE_VALIDATE_LEG;

                break;
            }


            if (
                !prepare_leg(
                    state,
                    state->active_leg
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        state->error,
                        state->failed_joint
                    );

                break;
            }


            ++state->active_leg;

            break;
        }


        /* --------------------------------------------------------------------
         * PREVALIDATE ENTIRE ROUTE BEFORE FIRST MOTION COMMAND
         * -------------------------------------------------------------------- */

        case APPROACH_PHASE_VALIDATE_LEG:
        {
            if (
                state->active_leg >=
                state->total_legs
            )
            {
                state->active_leg =
                    0U;

                state->sample_index =
                    0U;

                state->phase =
                    APPROACH_PHASE_WAIT_PERMISSION;

                break;
            }


            JointTrajectory *trajectory =
                &state->route_trajectory[
                    state->active_leg
                ];


            const size_t sample_count =
                joint_trajectory_sample_count(
                    trajectory
                );


            for (
                uint16_t work = 0U;
                work <
                    state->config.validation_samples_per_step;
                ++work
            )
            {
                if (
                    state->validation_index >=
                    sample_count
                )
                {
                    ++state->active_leg;

                    state->validation_index =
                        0U;

                    break;
                }


                JointTrajectorySample sample;


                if (
                    !joint_trajectory_evaluate_index(
                        trajectory,
                        state->validation_index,
                        &sample
                    )
                )
                {
                    step_result =
                        approach_fail(
                            state,
                            APPROACH_ERR_PLAN_LIMIT,
                            0U
                        );

                    break;
                }


                uint8_t failed_joint =
                    0U;


                if (
                    !joint_vector_in_limits(
                        state,
                        &sample.q,
                        &failed_joint
                    )
                )
                {
                    step_result =
                        approach_fail(
                            state,
                            APPROACH_ERR_TARGET_LIMIT,
                            failed_joint
                        );

                    break;
                }


                if (
                    state->services.collision_free != NULL
                    &&
                    !state->services.collision_free(
                        &sample.q,
                        state->services.collision_context
                    )
                )
                {
                    step_result =
                        approach_fail(
                            state,
                            APPROACH_ERR_COLLISION,
                            0U
                        );

                    break;
                }


                ++state->validation_index;
            }


            break;
        }


        /* --------------------------------------------------------------------
         * WAIT FOR MOTION PERMISSION
         * -------------------------------------------------------------------- */

        case APPROACH_PHASE_WAIT_PERMISSION:
        {
            if (
                inputs->pause_requested ||
                inputs->protective_stop_active ||
                !inputs->motion_permission
            )
            {
                if (!hold_actual(state, now_ms))
                {
                    step_result =
                        approach_fail(
                            state,
                            state->error,
                            state->failed_joint
                        );
                }

                break;
            }


            state->phase =
                APPROACH_PHASE_EXECUTE_LEG;

            break;
        }


        /* --------------------------------------------------------------------
         * EXECUTE CURRENT LEG
         * -------------------------------------------------------------------- */

        case APPROACH_PHASE_EXECUTE_LEG:
        {
            if (
                inputs->pause_requested ||
                inputs->protective_stop_active ||
                !inputs->motion_permission
            )
            {
                if (!hold_actual(state, now_ms))
                {
                    step_result =
                        approach_fail(
                            state,
                            state->error,
                            state->failed_joint
                        );

                    break;
                }


                state->phase =
                    APPROACH_PHASE_PAUSED;

                state->paused_during_verification =
                    false;

                break;
            }


            JointVector actual_q;


            if (
                !read_feedback_ready(
                    state,
                    now_ms,
                    &actual_q,
                    NULL
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        state->error,
                        state->failed_joint
                    );

                break;
            }


            if (
                !following_error_valid(
                    state,
                    &actual_q
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_FOLLOWING_ERROR,
                        state->failed_joint
                    );

                break;
            }


            if (
                state->active_leg >=
                state->total_legs
            )
            {
                state->phase =
                    APPROACH_PHASE_VERIFY_TARGET;

                break;
            }


            if (!command_due(
                    state,
                    now_ms))
            {
                break;
            }


            JointTrajectory *trajectory =
                &state->route_trajectory[
                    state->active_leg
                ];


            const size_t sample_count =
                joint_trajectory_sample_count(
                    trajectory
                );


            if (
                sample_count == 0U ||
                state->sample_index >=
                    sample_count
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_PLAN_LIMIT,
                        0U
                    );

                break;
            }


            JointTrajectorySample sample;


            if (
                !joint_trajectory_evaluate_index(
                    trajectory,
                    state->sample_index,
                    &sample
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_PLAN_LIMIT,
                        0U
                    );

                break;
            }


            const bool last_sample_of_leg =
                state->sample_index ==
                sample_count - 1U;


            const bool exact_final =
                (
                    state->active_leg + 1U ==
                    state->total_legs
                )
                &&
                last_sample_of_leg;


            if (
                !send_joint_target(
                    state,
                    &sample.q,
                    exact_final,
                    now_ms
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        state->error,
                        state->failed_joint
                    );

                break;
            }


            ++state->samples_sent;


            if (last_sample_of_leg)
            {
                ++state->active_leg;

                state->sample_index =
                    0U;

                state->last_command_valid =
                    false;


                if (
                    state->active_leg >=
                    state->total_legs
                )
                {
                    state->phase =
                        APPROACH_PHASE_VERIFY_TARGET;
                }
            }
            else
            {
                ++state->sample_index;
            }


            break;
        }


        /* --------------------------------------------------------------------
         * PAUSE / PROTECTIVE STOP
         * -------------------------------------------------------------------- */

        case APPROACH_PHASE_PAUSED:
        {
            if (!hold_actual(state, now_ms))
            {
                step_result =
                    approach_fail(
                        state,
                        state->error,
                        state->failed_joint
                    );

                break;
            }


            if (
                inputs->pause_requested ||
                inputs->protective_stop_active ||
                !inputs->motion_permission
            )
            {
                break;
            }


            if (state->paused_during_verification)
            {
                state->paused_during_verification =
                    false;

                state->phase =
                    APPROACH_PHASE_VERIFY_TARGET;

                break;
            }


            JointVector actual_q;


            if (
                !read_feedback_ready(
                    state,
                    now_ms,
                    &actual_q,
                    NULL
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        state->error,
                        state->failed_joint
                    );

                break;
            }


            replan_from_actual(
                state,
                &actual_q
            );


            state->phase =
                APPROACH_PHASE_PREPARE_LEG;

            break;
        }


        /* --------------------------------------------------------------------
         * VERIFY EXACT SAMPLE-0 TARGET
         * -------------------------------------------------------------------- */

        case APPROACH_PHASE_VERIFY_TARGET:
        {
            if (
                inputs->pause_requested ||
                inputs->protective_stop_active ||
                !inputs->motion_permission
            )
            {
                if (!hold_actual(state, now_ms))
                {
                    step_result =
                        approach_fail(
                            state,
                            state->error,
                            state->failed_joint
                        );

                    break;
                }


                state->phase =
                    APPROACH_PHASE_PAUSED;

                state->paused_during_verification =
                    true;

                break;
            }


            JointVector actual_q;


            if (
                !read_feedback_ready(
                    state,
                    now_ms,
                    &actual_q,
                    NULL
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        state->error,
                        state->failed_joint
                    );

                break;
            }


            bool within =
                true;


            for (int joint = 0;
                 joint < ROBOT_DOF;
                 ++joint)
            {
                if (
                    fabs(
                        actual_q.q[joint] -
                        state->final_target.q[joint]
                    )
                    >
                    state->config.final_position_tolerance_rad
                )
                {
                    within =
                        false;
                }
            }


            /*
             * Keep commanding the EXACT sample-0 AVATAR raw target during
             * final verification at the same 2 ms / 500 Hz command period.
             */
            if (!command_due(
                    state,
                    now_ms))
            {
                break;
            }

            if (
                !send_joint_target(
                    state,
                    &state->final_target,
                    true,
                    now_ms
                )
            )
            {
                step_result =
                    approach_fail(
                        state,
                        state->error,
                        state->failed_joint
                    );

                break;
            }


            ++state->verification_cycles;


            state->stable_cycles =
                within
                ? state->stable_cycles + 1U
                : 0U;


            if (
                state->stable_cycles >=
                state->config.required_stable_cycles
            )
            {
                state->phase =
                    APPROACH_PHASE_COMPLETE;

                state->result =
                    APPROACH_RESULT_COMPLETE;

                step_result =
                    STATE_STEP_COMPLETE;
            }
            else if (
                state->verification_cycles >=
                state->config.maximum_verification_cycles
            )
            {
                step_result =
                    approach_fail(
                        state,
                        APPROACH_ERR_VERIFY_TIMEOUT,
                        0U
                    );
            }


            break;
        }


        case APPROACH_PHASE_COMPLETE:
        {
            state->result =
                APPROACH_RESULT_COMPLETE;

            step_result =
                STATE_STEP_COMPLETE;

            break;
        }


        case APPROACH_PHASE_ABORTED:
        {
            state->result =
                APPROACH_RESULT_ABORTED;

            step_result =
                STATE_STEP_COMPLETE;

            break;
        }


        case APPROACH_PHASE_FAILED:
        {
            state->result =
                APPROACH_RESULT_FAILED;

            step_result =
                STATE_STEP_FAILED;

            break;
        }


        case APPROACH_PHASE_IDLE:
        default:
        {
            step_result =
                approach_fail(
                    state,
                    APPROACH_ERR_INVALID_REQUEST,
                    0U
                );

            break;
        }
    }


    state_approach_get_outputs(
        state,
        outputs
    );


    return
        step_result;
}


/* ============================================================================
 * NAME HELPERS
 * ============================================================================ */

const char *state_approach_phase_name(
    ApproachPhase phase
)
{
    switch (phase)
    {
        case APPROACH_PHASE_IDLE:
            return "IDLE";

        case APPROACH_PHASE_CHECK_REQUEST:
            return "CHECK_REQUEST";

        case APPROACH_PHASE_LOAD_TARGET:
            return "LOAD_TARGET";

        case APPROACH_PHASE_READ_START:
            return "READ_START";

        case APPROACH_PHASE_PREPARE_LEG:
            return "PREPARE_LEG";

        case APPROACH_PHASE_VALIDATE_LEG:
            return "VALIDATE_LEG";

        case APPROACH_PHASE_WAIT_PERMISSION:
            return "WAIT_PERMISSION";

        case APPROACH_PHASE_EXECUTE_LEG:
            return "EXECUTE_LEG";

        case APPROACH_PHASE_PAUSED:
            return "PAUSED";

        case APPROACH_PHASE_VERIFY_TARGET:
            return "VERIFY_TARGET";

        case APPROACH_PHASE_COMPLETE:
            return "COMPLETE";

        case APPROACH_PHASE_ABORTED:
            return "ABORTED";

        case APPROACH_PHASE_FAILED:
            return "FAILED";

        default:
            return "UNKNOWN";
    }
}


const char *state_approach_error_name(
    ApproachError error
)
{
    switch (error)
    {
        case APPROACH_ERR_NONE:
            return "NONE";

        case APPROACH_ERR_NULL_ARGUMENT:
            return "NULL_ARGUMENT";

        case APPROACH_ERR_INVALID_CONFIG:
            return "INVALID_CONFIG";

        case APPROACH_ERR_INVALID_REQUEST:
            return "INVALID_REQUEST";

        case APPROACH_ERR_TRAJECTORY_NOT_VALID:
            return "TRAJECTORY_NOT_VALID";

        case APPROACH_ERR_TRAJECTORY_MISMATCH:
            return "TRAJECTORY_MISMATCH";

        case APPROACH_ERR_EMPTY_TRAJECTORY:
            return "EMPTY_TRAJECTORY";

        case APPROACH_ERR_STORAGE_READ:
            return "STORAGE_READ";

        case APPROACH_ERR_POSITION_CONVERSION:
            return "POSITION_CONVERSION";

        case APPROACH_ERR_TARGET_LIMIT:
            return "TARGET_LIMIT";

        case APPROACH_ERR_COMMUNICATION:
            return "COMMUNICATION";

        case APPROACH_ERR_FEEDBACK:
            return "FEEDBACK";

        case APPROACH_ERR_DRIVE_NOT_READY:
            return "DRIVE_NOT_READY";

        case APPROACH_ERR_PLAN_LIMIT:
            return "PLAN_LIMIT";

        case APPROACH_ERR_COLLISION:
            return "COLLISION";

        case APPROACH_ERR_COLLISION_CHECK_MISSING:
            return "COLLISION_CHECK_MISSING";

        case APPROACH_ERR_VERIFY_TIMEOUT:
            return "VERIFY_TIMEOUT";

        case APPROACH_ERR_FOLLOWING_ERROR:
            return "FOLLOWING_ERROR";

        case APPROACH_ERR_CYCLIC_FEEDBACK:
            return "CYCLIC_FEEDBACK";

        case APPROACH_ERR_EXTERNAL_FAULT:
            return "EXTERNAL_FAULT";

        case APPROACH_ERR_ESTOP:
            return "ESTOP";

        case APPROACH_ERR_RESET_REQUESTED:
            return "RESET_REQUESTED";

        case APPROACH_ERR_HOME_REQUESTED:
            return "HOME_REQUESTED";

        default:
            return "UNKNOWN";
    }
}
