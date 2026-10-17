#ifndef HAND_GUIDING_PIPELINE_H
#define HAND_GUIDING_PIPELINE_H

#include "admittance_control.h"
#include "cartesian_joint_mapper.h"
#include "wrench_frame.h"

/* Stage 4E1 software-only composition: absolutely no motor I/O or safety
 * authority. All sensor/robot geometry and tuning values must be provided. */
typedef struct {
    SensorConfig sensor;
    FtSensorBias bias;
    AdmittanceParams admittance;
    CartesianJointMapperConfig mapper;
    bool require_new_sample;
} HandGuidingPipelineConfig;

typedef struct {
    /* These are external claims, NOT independent safety-chain verification. */
    bool teaching_active;
    bool motion_permitted;
    bool robot_homed;
    bool drives_ready;
    bool feedback_fresh;
} HandGuidingPipelineGates;

typedef struct {
    const FtSensorSample *sample;
    const double *measured_joint_rad; /* six measured angles, radians */
    uint32_t now_us;
    uint32_t max_sensor_age_us;
    HandGuidingPipelineGates gates;
    double period_s; /* Step only; no implicit assumed period */
} HandGuidingPipelineInput;

typedef struct {
    bool active;
    bool fault_latched;
    bool have_last_sample;
    uint32_t last_sample_us;
    HandGuidingPipelineConfig config; /* Snapshot on arm; do not retune live */
    AdmittanceState admittance;
    CartesianJointMapperState mapper;
} HandGuidingPipelineState;

typedef struct {
    bool valid;
    bool joint_limited;
    double wrench_base_tcp[ADMITTANCE_AXES];
    double twist_base_tcp[ADMITTANCE_AXES];
    double joint_position_ref[ROBOT_DOF];
    double joint_velocity_ref[ROBOT_DOF];
    double sigma_min;
} HandGuidingPipelineOutput;

/* Explicit external authorization is required to reset a latched fault.
 * This API does not enforce that external permission. */
void hand_guiding_pipeline_reset(HandGuidingPipelineState *state);

/* Normal disengagement; disables both controllers but preserves any latch. */
void hand_guiding_pipeline_stop(HandGuidingPipelineState *state);

/* Validates live sample and external gates, captures configuration and sets
 * joint references to current measured feedback. No command is produced.
 * Failed attempts latch a fault until explicit reset. */
bool hand_guiding_pipeline_arm(
    HandGuidingPipelineState *state,
    const HandGuidingPipelineConfig *config,
    const RobotConfig *robot,
    const HandGuidingPipelineInput *input);

/* Chain 4B1/4B2 -> 4C -> 4D, all at one measured robot configuration.
 * On any failure: output zero/invalid, both controllers disabled, fault
 * latched. Zero output is NOT a real motor stop or hold command.
 * Output and input/state storage must not overlap. */
bool hand_guiding_pipeline_step(
    HandGuidingPipelineState *state,
    const RobotConfig *robot,
    const HandGuidingPipelineInput *input,
    HandGuidingPipelineOutput *output);

#endif
