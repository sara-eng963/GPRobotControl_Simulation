/* Executes the exact production header implementation using host C11. */
#include "supervision_progress.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define CHECK(x) assert(x)
static SupervisionProgressSample sample(uint32_t seq, uint32_t tick,
                                        SupervisionProgressMode mode)
{
    SupervisionProgressSample x = {0};
    supervision_record_completion(&x, 1U, tick, mode, true, true);
    x.sequence = seq;
    return x;
}

static void healthy_and_stall(void)
{
    SupervisionProgressPolicy p = { .expected_session = 1U };
    SupervisionProgressSample x = sample(1U, 100U, SUP_PROGRESS_ACTIVE);
    CHECK(supervision_progress_eligible(&p, &x, 102U, 30U));
    CHECK(!supervision_progress_eligible(&p, &x, 110U, 30U)); /* IRQs can run */
    CHECK(!supervision_progress_eligible(&p, &x, 131U, 30U));
    x.sequence = 2U; x.completed_tick_ms = 130U;
    CHECK(supervision_progress_eligible(&p, &x, 131U, 30U));
    x.sequence = 3U; x.completed_tick_ms = 99U; /* Late, stale */
    CHECK(!supervision_progress_eligible(&p, &x, 131U, 30U));
    x.completed_tick_ms = 132U; /* Future */
    CHECK(!supervision_progress_eligible(&p, &x, 131U, 30U));
    x.completed_tick_ms = 131U;
    CHECK(supervision_progress_eligible(&p, &x, 131U, 30U));
    CHECK(!supervision_progress_eligible(&p, &x, 131U, 30U));
}
static void ordering_and_reset(void)
{
    SupervisionProgressPolicy p = { .expected_session = 1U };
    SupervisionProgressSample x = sample(UINT32_MAX - 1U, 5U, SUP_PROGRESS_ACTIVE);
    CHECK(supervision_progress_eligible(&p, &x, 5U, 30U));
    x.sequence = UINT32_MAX;
    CHECK(supervision_progress_eligible(&p, &x, 6U, 30U));
    x.sequence = 0U;
    CHECK(supervision_progress_eligible(&p, &x, 7U, 30U));
    x.sequence = UINT32_MAX;
    CHECK(!supervision_progress_eligible(&p, &x, 7U, 30U));
    x.sequence = 1U; x.session = 2U; /* peer reset or old session */
    CHECK(!supervision_progress_eligible(&p, &x, 8U, 30U));
    x.session = 1U; x.sequence = 0x80000001U;
    CHECK(!supervision_progress_eligible(&p, &x, 8U, 30U));
    p.expected_session = 2U; p.have_accepted_sequence = false;
    x.session = 2U; x.sequence = 1U;
    CHECK(supervision_progress_eligible(&p, &x, 8U, 30U));
}
static void startup_and_fault(void)
{
    SupervisionProgressPolicy p = { .expected_session = 1U };
    SupervisionProgressSample x = sample(1U, 10U, SUP_PROGRESS_STARTUP);
    CHECK(!supervision_progress_eligible(&p, &x, 10U, 30U));
    x.mode = SUP_PROGRESS_FAULT; x.response_ok = false;
    CHECK(!supervision_progress_eligible(&p, &x, 10U, 30U));
    x.response_ok = true;
    CHECK(supervision_progress_eligible(&p, &x, 10U, 30U));
    CHECK(!supervision_progress_eligible(&p, &x, 11U, 30U));
    x.sequence = 2U; x.completed_ok = false;
    CHECK(!supervision_progress_eligible(&p, &x, 12U, 30U));
    x.completed_ok = true; x.valid = false;
    CHECK(!supervision_progress_eligible(&p, &x, 12U, 30U));
    x.valid = true; x.mode = SUP_PROGRESS_SERVICE;
    CHECK(supervision_progress_eligible(&p, &x, 12U, 30U));
    CHECK(!supervision_progress_eligible(&p, &x, 12U, 0U));
}
static void timemath(void)
{
    SupervisionProgressPolicy p = { .expected_session = 1U };
    SupervisionProgressSample x = sample(1U, UINT32_MAX - 10U, SUP_PROGRESS_SERVICE);
    CHECK(supervision_progress_eligible(&p, &x, 4U, 15U)); /* exactly 15 */
    x.sequence = 2U;
    CHECK(!supervision_progress_eligible(&p, &x, 5U, 15U)); /* 16 */
    x.completed_tick_ms = 6U; /* Future timestamp */
    CHECK(!supervision_progress_eligible(&p, &x, 5U, 15U));
    x.completed_tick_ms = 5U;
    CHECK(supervision_progress_eligible(&p, &x, 5U, 15U));
}
static void restart_gate(void)
{
    SupervisionRestartInputs r = {0};
    CHECK(!supervision_restart_authorized(&r));
    r.reason = SUP_RESET_WATCHDOG;
    r.fresh_ipc_session = true;
    r.stale_commands_discarded = true;
    r.independent_safety_ready = true;
    r.drive_feedback_revalidated = true;
    r.operator_reauthorized = true;
    CHECK(!supervision_restart_authorized(&r));
    r.reset_cause_reviewed = true;
    CHECK(supervision_restart_authorized(&r));
    r.operator_reauthorized = false;
    CHECK(!supervision_restart_authorized(&r));
    r.operator_reauthorized = true;
    r.reason = SUP_RESET_UNDETERMINED;
    CHECK(supervision_restart_authorized(&r)); /* reviewed, not automatic */
    r.reset_cause_reviewed = false;
    CHECK(!supervision_restart_authorized(&r));
    r.reason = SUP_RESET_EXPECTED_POWER_ON;
    CHECK(supervision_restart_authorized(&r));
    r.fresh_ipc_session = false;
    CHECK(!supervision_restart_authorized(&r));
    CHECK(!supervision_restart_authorized(NULL));
}
int main(void)
{
    healthy_and_stall(); ordering_and_reset(); startup_and_fault();
    timemath(); restart_gate();
    puts("PASS: production C policy - freshness, progress, fault, session, restart");
    return 0;
}
