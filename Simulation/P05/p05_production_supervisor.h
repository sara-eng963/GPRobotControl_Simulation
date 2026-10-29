#ifndef P05_PRODUCTION_SUPERVISOR_H
#define P05_PRODUCTION_SUPERVISOR_H
/* Host-only scheduling/device seam; no firmware target links this file. */
#include "../../CANComm/CANopen/canopen_master_types.h"
#include "../../StateMachine/States/state_path_validation.h"
typedef enum {
    P05_SUP_NORMAL, P05_SUP_REVOKED, P05_SUP_CM4_LOST,
    P05_SUP_DEADLINE, P05_SUP_LATCH_FAULT, P05_SUP_RESTART
} P05SupervisorStimulus;
typedef struct {
    bool permission, inhibited, fault;
    uint32_t fault_code, samples_read, state_samples, path_error;
} P05SupervisorObservation;
bool p05_production_begin(CanopenMaster *live,
                           const ValidatedTrajectory *metadata, uint32_t now_us);
bool p05_production_cycle(const PvExecutionSample *sample, uint32_t index,
                          uint32_t now_us, P05SupervisorStimulus stimulus,
                          P05SupervisorObservation *observation);
#endif
