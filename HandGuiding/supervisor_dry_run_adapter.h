#ifndef HAND_GUIDING_SUPERVISOR_DRY_RUN_ADAPTER_H
#define HAND_GUIDING_SUPERVISOR_DRY_RUN_ADAPTER_H

#include "hand_guiding_pipeline.h"
#include "../StateMachine/supervisor_guidance_dry_run_port.h"

/* Adapter is host-testable without FreeRTOS or CANopen and has no call to
 * send_targets. Physical F/T acquisition is NOT implemented here. */
typedef struct {
    HandGuidingPipelineConfig pipeline_config;
    uint32_t max_sensor_age_us;
    uint64_t max_feedback_age_us;
    /* Optional bound on newest-oldest of six feedback timestamps (us).
     * 0 disables this additional skew check until an approved bound exists;
     * existing individual freshness and monotonic checks remain active. */
    uint64_t max_feedback_skew_us;
    uint32_t max_runtime_age_ms;
    uint32_t max_safety_age_ms;
    bool require_new_feedback;
    bool (*read_sample)(void *context, FtSensorSample *sample);
    void *sample_context;
} HandGuidingDryRunConfig;

typedef struct {
    HandGuidingDryRunConfig config;
    HandGuidingPipelineState pipeline;
    HandGuidingPipelineOutput last_output;
    uint32_t last_feedback_sequence[ROBOT_DOF];
    bool feedback_sequence_initialized;
    bool configured;
    uint32_t accepted_cycles;
    uint32_t rejected_cycles;
} HandGuidingDryRunState;

/* Caller owns persistent memory and external authorization for reset. */
bool hand_guiding_dry_run_init(
    HandGuidingDryRunState *state, const HandGuidingDryRunConfig *config);
void hand_guiding_dry_run_reset(HandGuidingDryRunState *state);
bool hand_guiding_dry_run_tick(
    void *context, const SupervisorGuidanceDryRunCycle *cycle);
void hand_guiding_dry_run_stop(void *context);

/* No code in this adapter can send a target or enable a drive. */
SupervisorGuidanceDryRunPort hand_guiding_dry_run_port(
    HandGuidingDryRunState *state);

#endif
