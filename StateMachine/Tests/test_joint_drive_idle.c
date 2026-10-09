/* Host-only application-layer regression: IDLE must not know CANopen. */
#include "../States/state_idle.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    bool configured;
    bool network_healthy;
    bool feedback_ok;
    bool send_ok;
    JointDriveAxisFeedback axes[JOINT_DRIVE_AXES];
    uint32_t sequence[JOINT_DRIVE_AXES];
    int32_t last_targets[JOINT_DRIVE_AXES];
    unsigned send_count;
    unsigned poll_count;
} FakeDrive;

static int failures;

#define CHECK(cond, name) do { \
    if (cond) { printf("[PASS] %s\n", name); } \
    else { printf("[FAIL] %s\n", name); ++failures; } \
} while (0)

static bool configured(void *ctx)
{
    return ((FakeDrive *)ctx)->configured;
}
static bool poll(void *ctx, uint32_t now_ms)
{
    (void)now_ms;
    ++((FakeDrive *)ctx)->poll_count;
    return true;
}
static bool healthy(void *ctx, uint32_t now_ms)
{
    (void)now_ms;
    return ((FakeDrive *)ctx)->network_healthy;
}
static bool feedback_valid(void *ctx)
{
    return ((FakeDrive *)ctx)->feedback_ok;
}
static bool read_axis(void *ctx, size_t axis, JointDriveAxisFeedback *out)
{
    FakeDrive *fake = (FakeDrive *)ctx;
    if (axis >= JOINT_DRIVE_AXES || out == NULL) return false;
    *out = fake->axes[axis];
    return true;
}
static bool send_targets(void *ctx, const int32_t *targets, size_t count)
{
    FakeDrive *fake = (FakeDrive *)ctx;
    if (count != JOINT_DRIVE_AXES || !fake->send_ok) return false;
    memcpy(fake->last_targets, targets, sizeof(fake->last_targets));
    ++fake->send_count;
    return true;
}
static uint32_t sequence(void *ctx, size_t axis)
{
    return ((FakeDrive *)ctx)->sequence[axis];
}
static JointDrivePort make_port(FakeDrive *f)
{
    JointDrivePort p = {
        .context = f,
        .is_configured = configured,
        .poll = poll,
        .ready_for_motion = healthy,
        .healthy = healthy,
        .all_feedback_valid = feedback_valid,
        .read_axis = read_axis,
        .send_targets = send_targets,
        .feedback_sequence = sequence
    };
    return p;
}
static void reset_fake(FakeDrive *f)
{
    memset(f, 0, sizeof(*f));
    f->configured = true;
    f->network_healthy = true;
    f->feedback_ok = true;
    f->send_ok = true;
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) {
        f->axes[i].feedback_valid = true;
        f->axes[i].operation_enabled = true;
        f->axes[i].actual_position_units = (int32_t)(1000 + i * 100);
        f->sequence[i] = 1U;
    }
}
int main(void)
{
    FakeDrive fake;
    IdleState idle;

    reset_fake(&fake);
    JointDrivePort port = make_port(&fake);
    state_idle_enter(&idle);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0) == STATE_STEP_RUNNING &&
          idle.phase == IDLE_PHASE_HOLDING,
          "IDLE enters holding after valid six-axis feedback");
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 1) == STATE_STEP_RUNNING &&
          fake.send_count == 1U,
          "IDLE issues first six-axis hold cycle");
    bool held_exact = true;
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i)
        held_exact = held_exact &&
            fake.last_targets[i] == fake.axes[i].actual_position_units;
    CHECK(held_exact, "Hold commands equal raw feedback positions without conversion");

    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 2) == STATE_STEP_RUNNING &&
          fake.send_count == 1U && idle.awaitingFeedback,
          "IDLE waits for fresh TPDO4 data before another hold cycle");
    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) ++fake.sequence[i];
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 3) == STATE_STEP_RUNNING &&
          fake.send_count == 2U && idle.cyclesHeld == 1U,
          "Fresh feedback authorizes the next hold cycle");

    fake.network_healthy = false;
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 4) == STATE_STEP_FAILED &&
          idle.error == IDLE_ERROR_COMMUNICATION,
          "Network failure produces communication fault");

    reset_fake(&fake);
    fake.feedback_ok = false;
    state_idle_enter(&idle);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0) == STATE_STEP_FAILED &&
          idle.error == IDLE_ERROR_POSITION_FEEDBACK,
          "Missing feedback retains its distinct fault");

    reset_fake(&fake);
    fake.axes[1].operation_enabled = false;
    state_idle_enter(&idle);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0) == STATE_STEP_FAILED &&
          idle.error == IDLE_ERROR_DRIVE_NOT_ENABLED && idle.failedAxis == 2,
          "Disabled axis reports the correct joint number");

    reset_fake(&fake);
    state_idle_enter(&idle);
    (void)state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0);
    (void)state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 1);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 102) == STATE_STEP_FAILED &&
          idle.error == IDLE_ERROR_CYCLIC_FEEDBACK,
          "Missing post-command feedback trips watchdog");

    reset_fake(&fake);
    state_idle_enter(&idle);
    (void)state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 0);
    CHECK(state_idle_step(&idle, IDLE_COMMAND_TEACH, &port, 1) == STATE_STEP_RUNNING &&
          idle.phase == IDLE_PHASE_COMPLETE &&
          idle.exitCommand == IDLE_COMMAND_TEACH,
          "Teach command exits holding through state transition");
    CHECK(state_idle_step(&idle, IDLE_COMMAND_NONE, &port, 2) == STATE_STEP_COMPLETE,
          "Completed IDLE state reports completion");

    printf("\nIDLE tests failed: %d\n", failures);
    return failures != 0;
}
