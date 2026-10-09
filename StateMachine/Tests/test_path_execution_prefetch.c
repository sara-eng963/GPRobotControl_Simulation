/* Real execution state + SRAM prefetch + fake six-axis drive port.
 * No physical motor behavior or wall-clock deadline claims. */
#include "../States/state_path_execution.h"
#include "../../Simulation/Storage/trajectory_prefetch.h"
#include "fake_joint_drive.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

typedef struct {
    TrajectoryPrefetch stream;
    FakeDrive drive;
    PathExecutionState execution;
    ValidatedTrajectory metadata;
    uint32_t fail_at, read_calls, next_ms;
    unsigned wire_enables, releases, stop_calls;
    bool wire;
} Test;

static int32_t value(uint32_t index, unsigned joint)
{
    int32_t magnitude = (int32_t)(index * 97U + joint * 13U + 1000U);
    return joint % 2U ? -magnitude : magnitude;
}

static bool read_batch(uint32_t first, PvExecutionSample *out, uint32_t count, void *ctx)
{
    Test *t = ctx;
    ++t->read_calls;
    if (first >= t->fail_at) return false;
    for (uint32_t i = 0; i < count; ++i)
        for (unsigned j = 0; j < 6; ++j)
            out[i].target_position_units[j] = value(first + i, j);
    return true;
}

static bool take(uint32_t index, PvExecutionSample *sample, void *ctx)
{
    Test *t = ctx;
    uint32_t before = t->read_calls;
    bool ok = trajectory_prefetch_take(&t->stream, index, sample);
    assert(before == t->read_calls);
    return ok;
}

static bool ready(uint32_t index, uint32_t count, void *ctx)
{
    Test *t = ctx;
    uint32_t before = t->read_calls;
    bool ok = trajectory_prefetch_begin_execution(&t->stream, index, count);
    assert(before == t->read_calls);
    return ok;
}

static void end_stream(void *ctx)
{
    Test *t = ctx;
    ++t->releases;
    trajectory_prefetch_end_execution(&t->stream);
}

static bool relay(bool enable, void *ctx)
{
    Test *t = ctx;
    t->wire = enable;
    if (enable) ++t->wire_enables;
    return true;
}
static bool retract_prepare(void *ctx) { (void)ctx; return true; }
static StateStepResult retract_step(void *ctx) { (void)ctx; return STATE_STEP_COMPLETE; }
static bool clearance(void *ctx) { (void)ctx; return true; }
static bool stop(bool *stopped, void *ctx)
{
    ++((Test *)ctx)->stop_calls;
    *stopped = true;
    return true;
}

static void enter(Test *t, RobotExecutionMode mode)
{
    PathExecutionRequest request = {
        .mode = mode, .trajectory = &t->metadata, .trajectory_ready = true,
        .expected_program_id = t->metadata.program_id,
        .expected_source_revision = t->metadata.source_revision,
        .expected_artifact_crc = t->metadata.artifact_crc
    };
    PathExecutionConfig config = {1000, 1};
    PathExecutionServices services = {
        .read_sample = take, .set_wire_feed_enabled = relay,
        .prepare_retraction = retract_prepare, .step_retraction = retract_step,
        .clearance_verified = clearance, .controlled_stop = stop,
        .context = t, .stream_ready = ready, .end_stream = end_stream
    };
    JointDrivePort port = fake_joint_drive_make_port(&t->drive);
    state_path_execution_enter(&t->execution, &port, &request, &config, &services);
    assert(t->execution.command_period_ms == 2);
}

static void init(Test *t, uint32_t count, unsigned prefill, uint32_t fail_at)
{
    memset(t, 0, sizeof(*t));
    t->fail_at = fail_at;
    fake_joint_drive_reset(&t->drive);
    t->drive.auto_feedback = true;
    t->metadata = (ValidatedTrajectory){
        .program_id = 1, .source_revision = 2, .artifact_crc = 3,
        .sample_count = count, .sample_period_us = 2000
    };
    assert(trajectory_prefetch_init(&t->stream));
    assert(trajectory_prefetch_prepare(&t->stream, read_batch, t, count));
    for (unsigned i = 0; i < prefill; ++i)
        (void)trajectory_prefetch_refill(&t->stream);
    assert(trajectory_prefetch_resume_worker(&t->stream) == (fail_at != 0));
    enter(t, ROBOT_EXECUTION_PRODUCTION);
}

static StateStepResult step(Test *t, bool abort_home)
{
    PathExecutionInputs inputs = {
        .motion_permission = true, .now_ms = t->next_ms++, .home_requested = abort_home
    };
    PathExecutionOutputs outputs;
    unsigned before = t->drive.commands;
    uint32_t index = t->execution.sample_index;
    StateStepResult result = state_path_execution_step(&t->execution, &inputs, &outputs);
    if (t->drive.commands != before) {
        assert(t->drive.commands == before + 1U);
        assert(t->execution.sample_index == index + 1U);
        for (unsigned j = 0; j < 6; ++j)
            assert(t->drive.last_targets[j] == value(index, j));
    }
    return result;
}

static void complete(Test *t, bool refill)
{
    StateStepResult result = STATE_STEP_RUNNING;
    for (unsigned i = 0; i < 10000 && result == STATE_STEP_RUNNING; ++i) {
        result = step(t, false);
        if (refill) (void)trajectory_prefetch_worker_step(&t->stream);
    }
    assert(result == STATE_STEP_COMPLETE);
    assert(t->execution.result == PATH_EXEC_RESULT_COMPLETE);
    assert(!t->wire);
}

static void test_startup(void)
{
    const uint32_t counts[] = {1, 3, 256, 257, 511, 512, 513};
    for (unsigned i = 0; i < sizeof(counts) / sizeof(counts[0]); ++i) {
        Test t;
        init(&t, counts[i], 2, UINT32_MAX);
        complete(&t, true);
        assert(t.drive.commands == counts[i]);
    }
    Test t;
    init(&t, 513, 1, UINT32_MAX); /* 256 samples cannot satisfy fresh preflight */
    assert(step(&t, false) == STATE_STEP_FAILED);
    assert(t.execution.error == PATH_EXEC_ERR_TRAJECTORY_NOT_READY);
    assert(t.wire_enables == 0 && t.drive.commands == 0);
    init(&t, 3, 0, UINT32_MAX);
    assert(step(&t, false) == STATE_STEP_FAILED);
    assert(t.wire_enables == 0 && t.drive.commands == 0);
    init(&t, 3, 1, 0);
    assert(step(&t, false) == STATE_STEP_FAILED);
    assert(t.wire_enables == 0 && t.drive.commands == 0);
    init(&t, 3, 2, UINT32_MAX);
    assert(step(&t, false) == STATE_STEP_RUNNING);
    assert(t.execution.phase == PATH_EXEC_PHASE_CONFIGURE_WIRE_FEED);
    __atomic_store_n(&t.stream.read_failed, true, __ATOMIC_RELEASE);
    assert(step(&t, false) == STATE_STEP_FAILED);
    assert(t.wire_enables == 0 && t.drive.commands == 0);
    puts("[PASS] Short/full startup, insufficient/failed prefill and failure before relay enable");
}

static void test_final_feedback_replay_pause_abort(void)
{
    Test t;
    init(&t, 3, 2, UINT32_MAX);
    enter(&t, ROBOT_EXECUTION_PREVIEW);
    while (t.drive.commands < 2) assert(step(&t, false) == STATE_STEP_RUNNING);
    /* Complete feedback for command 2, then hold final feedback. */
    assert(fake_drive_poll(&t.drive, t.next_ms));
    t.drive.auto_feedback = false;
    while (t.drive.commands < 3) assert(step(&t, false) == STATE_STEP_RUNNING);
    for (unsigned i = 0; i < 5; ++i) {
        assert(trajectory_prefetch_worker_step(&t.stream));
        assert(step(&t, false) == STATE_STEP_RUNNING);
        assert(t.execution.awaiting_feedback);
        assert(__atomic_load_n(&t.stream.consumed, __ATOMIC_ACQUIRE) == 3);
    }
    assert(t.releases == 0 && t.wire_enables == 0);
    t.drive.auto_feedback = true;
    complete(&t, true);
    state_path_execution_release_stream(&t.execution); /* Supervisor exits execution */
    assert(t.releases == 1);
    assert(trajectory_prefetch_worker_step(&t.stream));
    enter(&t, ROBOT_EXECUTION_PRODUCTION);
    while (t.execution.sample_index < 1) assert(step(&t, false) == STATE_STEP_RUNNING);
    t.wire = false; /* existing PAUSED relay policy */
    assert(trajectory_prefetch_worker_step(&t.stream));
    state_path_execution_resume(&t.execution, t.next_ms);
    assert(t.releases == 1);
    assert(__atomic_load_n(&t.stream.consumed, __ATOMIC_ACQUIRE) == 1);
    assert(step(&t, false) == STATE_STEP_RUNNING);
    assert(t.wire && t.execution.sample_index == 1);
    complete(&t, true);
    state_path_execution_release_stream(&t.execution);
    assert(trajectory_prefetch_worker_step(&t.stream));
    enter(&t, ROBOT_EXECUTION_PRODUCTION);
    while (t.execution.sample_index < 1) assert(step(&t, false) == STATE_STEP_RUNNING);
    assert(step(&t, true) == STATE_STEP_RUNNING);
    assert(!t.wire);
    unsigned commands = t.drive.commands;
    while (step(&t, false) == STATE_STEP_RUNNING) { }
    assert(t.execution.result == PATH_EXEC_RESULT_ABORTED && t.stop_calls == 1);
    assert(t.drive.commands == commands);
    state_path_execution_release_stream(&t.execution);
    assert(trajectory_prefetch_worker_step(&t.stream));
    enter(&t, ROBOT_EXECUTION_PREVIEW);
    complete(&t, true);
    assert(t.drive.commands == commands + 3);
    puts("[PASS] Final feedback holds ownership; preview/production, pause/resume, abort/restart");
}

static void test_runtime_faults(void)
{
    for (unsigned failure = 0; failure < 2; ++failure) {
        Test t;
        init(&t, 1024, 2, UINT32_MAX);
        while (t.execution.sample_index < (failure ? 256U : 512U))
            assert(step(&t, false) == STATE_STEP_RUNNING);
        if (failure) {
            t.fail_at = 512;
            assert(!trajectory_prefetch_worker_step(&t.stream));
        }
        StateStepResult result = STATE_STEP_RUNNING;
        unsigned commands = t.drive.commands;
        for (unsigned i = 0; i < 10 && result == STATE_STEP_RUNNING; ++i)
            result = step(&t, false);
        assert(result == STATE_STEP_FAILED && t.execution.error == PATH_EXEC_ERR_STORAGE_READ);
        assert(!t.wire && t.drive.commands == commands);
        /* Fault-phase steps cannot read/advance another trajectory sample. */
        assert(step(&t, false) == STATE_STEP_FAILED);
        assert(t.drive.commands == commands);
        assert(t.stop_calls == 0); /* existing storage-fault motor policy preserved */
        state_path_execution_release_stream(&t.execution);
    }
    Test t;
    init(&t, 1024, 2, UINT32_MAX);
    while (t.execution.sample_index < 512) assert(step(&t, false) == STATE_STEP_RUNNING);
    t.wire = false;
    state_path_execution_resume(&t.execution, t.next_ms);
    unsigned enables = t.wire_enables;
    assert(step(&t, false) == STATE_STEP_FAILED);
    assert(t.wire_enables == enables && !t.wire);
    puts("[PASS] Runtime read failure/starvation stop progression; empty resume cannot enable relay");
}

int main(void)
{
    test_startup();
    test_final_feedback_replay_pause_abort();
    test_runtime_faults();
    puts("ALL PATH_EXECUTION PREFETCH TESTS PASS");
    return 0;
}
