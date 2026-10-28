#ifndef SUPERVISION_PROGRESS_H
#define SUPERVISION_PROGRESS_H
/* C2A software-only decision contract. Not a hardware watchdog or safety channel.
 * All timestamps are the OBSERVING CORE'S local 1 ms tick clock. No remote
 * timestamps or unsynchronised cross-core time arithmetic are accepted. */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    SUP_PROGRESS_STARTUP = 0,
    SUP_PROGRESS_ACTIVE = 1,
    SUP_PROGRESS_FAULT = 2,
    SUP_PROGRESS_SERVICE = 3
} SupervisionProgressMode;

typedef struct {
    uint32_t session;
    uint32_t sequence;
    uint32_t completed_tick_ms;
    SupervisionProgressMode mode;
    bool valid;
    bool completed_ok;
    bool response_ok; /* FAULT: fault response step reached valid completion. */
} SupervisionProgressSample;

typedef struct {
    uint32_t expected_session;
    uint32_t last_accepted_sequence;
    bool have_accepted_sequence;
} SupervisionProgressPolicy;

/* Strict modular ordering; deltas >= half the range are ambiguous/rejected. */
static inline bool supervision_sequence_forward(uint32_t next, uint32_t prior)
{
    const uint32_t advance = next - prior;
    return advance != 0U && advance < UINT32_C(0x80000000);
}

/* One-shot permission: the same completed cycle NEVER grants a second feed.
 * The policy state belongs exclusively to one observer task. The sample is a
 * snapshot, copied by its producer's same-core FreeRTOS critical section.
 * max_age_ms is PROVISIONAL until target timing is measured, not IWDG period. */
static inline bool supervision_progress_eligible(
    SupervisionProgressPolicy *policy, const SupervisionProgressSample *sample,
    uint32_t now_tick_ms, uint32_t max_age_ms)
{
    if (policy == NULL || sample == NULL || policy->expected_session == 0U ||
        sample->session != policy->expected_session ||
        !sample->valid || !sample->completed_ok ||
        max_age_ms == 0U || max_age_ms >= UINT32_C(0x80000000))
        return false;
    const uint32_t age = now_tick_ms - sample->completed_tick_ms;
    if (age >= UINT32_C(0x80000000) || age > max_age_ms)
        return false;
    if (sample->mode != SUP_PROGRESS_ACTIVE &&
        sample->mode != SUP_PROGRESS_SERVICE &&
        sample->mode != SUP_PROGRESS_FAULT)
        return false;
    if (sample->mode == SUP_PROGRESS_FAULT && !sample->response_ok)
        return false;
    if (policy->have_accepted_sequence &&
        !supervision_sequence_forward(sample->sequence,
                                      policy->last_accepted_sequence))
        return false;
    policy->last_accepted_sequence = sample->sequence;
    policy->have_accepted_sequence = true;
    return true;
}

/* Caller must hold task-level critical section if another task reads sample. */
static inline void supervision_record_completion(
    SupervisionProgressSample *sample, uint32_t session, uint32_t tick_ms,
    SupervisionProgressMode mode, bool completed_ok, bool response_ok)
{
    if (sample == NULL) return;
    sample->valid = false;
    sample->sequence += 1U; /* modulo 2^32; zero is a valid wrapped sequence */
    sample->session = session;
    sample->completed_tick_ms = tick_ms;
    sample->mode = mode;
    sample->completed_ok = completed_ok;
    sample->response_ok = response_ok;
    sample->valid = true;
}

typedef enum {
    SUP_RESET_UNDETERMINED = 0,
    SUP_RESET_EXPECTED_POWER_ON = 1,
    SUP_RESET_WATCHDOG = 2,
    SUP_RESET_BROWNOUT = 3,
    SUP_RESET_OTHER = 4
} SupervisionResetReason;

typedef struct {
    SupervisionResetReason reason;
    bool reset_cause_reviewed;
    bool fresh_ipc_session;
    bool stale_commands_discarded;
    bool independent_safety_ready;
    bool drive_feedback_revalidated;
    bool operator_reauthorized;
} SupervisionRestartInputs;

/* Pure proposed BOOT drive-enable gate. NOT connected to motor operation in C2A.
 * All resets require deliberate operator reauthorization; ambiguous or
 * unexpected resets additionally require explicit reset-cause investigation. */
static inline bool supervision_restart_authorized(
    const SupervisionRestartInputs *input)
{
    if (input == NULL ||
        !input->fresh_ipc_session || !input->stale_commands_discarded ||
        !input->independent_safety_ready ||
        !input->drive_feedback_revalidated ||
        !input->operator_reauthorized)
        return false;
    if (input->reason == SUP_RESET_UNDETERMINED ||
        input->reason == SUP_RESET_WATCHDOG ||
        input->reason == SUP_RESET_BROWNOUT ||
        input->reason == SUP_RESET_OTHER)
        return input->reset_cause_reviewed;
    return input->reason == SUP_RESET_EXPECTED_POWER_ON;
}
#endif /* SUPERVISION_PROGRESS_H */
