#ifndef SUPERVISOR_TIMING_QUALITY_H
#define SUPERVISOR_TIMING_QUALITY_H

/* Pure integer arithmetic used by both firmware and host fault injection.
 * Timestamps are TIM2 microseconds, not inferred hardware TIM3 event times. */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#define SUPERVISOR_PERIOD_US UINT32_C(2000)

typedef struct {
    uint64_t isr_service_us;    /* Serviced TIM3 interrupt, not timer update. */
    uint64_t wake_sample_us;    /* Task timestamp after wait_next returns. */
    uint32_t isr_service_gap_us;/* Interval between two serviced TIM3 IRQs. */
    bool timestamp_valid;       /* Sequence-matched ISR entry exists. */
    bool isr_gap_valid;         /* Previous service timestamp exists. */
} SupervisorTimingWake;

typedef struct {
    bool valid;
    bool service_gap_warning;   /* > 1 period; not proof of missed updates. */
    bool service_gap_severe;    /* >= 2 periods without serviced TIM3 IRQ. */
    bool wake_gap_warning;
    bool wake_gap_severe;
    bool late_wakeup;           /* >= 1 period since serviced TIM3 IRQ. */
    bool late_completion;       /* >= 1 period since serviced TIM3 IRQ. */
    uint32_t isr_to_wake_us;
    uint32_t isr_to_completion_us;
    uint32_t wake_gap_us;
} SupervisorTimingAssessment;

static inline uint32_t supervisor_timing_clamp_u32(uint64_t value)
{
    return value > UINT32_MAX ? UINT32_MAX : (uint32_t)value;
}

static inline SupervisorTimingAssessment supervisor_timing_assess(
    const SupervisorTimingWake *wake, uint64_t completion_us,
    bool previous_wake_valid, uint64_t previous_wake_us)
{
    SupervisorTimingAssessment result = {0};
    if (wake == NULL || !wake->timestamp_valid ||
        wake->isr_service_us > wake->wake_sample_us ||
        wake->wake_sample_us > completion_us ||
        (wake->isr_gap_valid && wake->isr_service_gap_us == 0U) ||
        (previous_wake_valid && previous_wake_us >= wake->wake_sample_us))
        return result;

    result.valid = true;
    result.isr_to_wake_us = supervisor_timing_clamp_u32(
        wake->wake_sample_us - wake->isr_service_us);
    result.isr_to_completion_us = supervisor_timing_clamp_u32(
        completion_us - wake->isr_service_us);
    result.late_wakeup =
        wake->wake_sample_us - wake->isr_service_us >= SUPERVISOR_PERIOD_US;
    result.late_completion =
        completion_us - wake->isr_service_us >= SUPERVISOR_PERIOD_US;

    if (wake->isr_gap_valid) {
        result.service_gap_warning = wake->isr_service_gap_us > SUPERVISOR_PERIOD_US;
        result.service_gap_severe =
            wake->isr_service_gap_us >= 2U * SUPERVISOR_PERIOD_US;
    }
    if (previous_wake_valid) {
        const uint64_t gap = wake->wake_sample_us - previous_wake_us;
        result.wake_gap_us = supervisor_timing_clamp_u32(gap);
        result.wake_gap_warning = gap > SUPERVISOR_PERIOD_US;
        result.wake_gap_severe = gap >= 2U * SUPERVISOR_PERIOD_US;
    }
    return result;
}

#endif /* SUPERVISOR_TIMING_QUALITY_H */
