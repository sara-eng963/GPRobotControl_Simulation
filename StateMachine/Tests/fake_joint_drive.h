#ifndef TEST_FAKE_JOINT_DRIVE_H
#define TEST_FAKE_JOINT_DRIVE_H

/* Shared host-only fake for state regression tests (not a motor simulator).
 * Tests customize the initial positions and feedback timing as needed.
 */
#include "../../ServoDrive/JointDrive/joint_drive_port.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    bool configured;
    bool network_ok;
    bool feedback_ok;
    bool poll_ok;
    bool send_ok;
    bool auto_feedback;
    bool pending_feedback;
    bool fail_axis_read;

    JointDriveAxisFeedback axis[JOINT_DRIVE_AXES];
    uint32_t sequence[JOINT_DRIVE_AXES];
    int32_t last_targets[JOINT_DRIVE_AXES];

    unsigned commands;
    unsigned polls;
    unsigned reads;
} FakeDrive;

static inline void fake_joint_drive_reset(FakeDrive *fake)
{
    memset(fake, 0, sizeof(*fake));
    fake->configured = true;
    fake->network_ok = true;
    fake->feedback_ok = true;
    fake->poll_ok = true;
    fake->send_ok = true;

    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) {
        fake->axis[i].feedback_valid = true;
        fake->axis[i].operation_enabled = true;
        fake->sequence[i] = 1U;
    }
}

static inline bool fake_drive_is_configured(void *ctx)
{
    return ((FakeDrive *)ctx)->configured;
}

static inline bool fake_drive_poll(void *ctx, uint32_t now_ms)
{
    (void)now_ms;
    FakeDrive *fake = (FakeDrive *)ctx;
    ++fake->polls;
    if (fake->pending_feedback && fake->auto_feedback) {
        for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) {
            fake->axis[i].actual_position_units = fake->last_targets[i];
            ++fake->sequence[i];
        }
        fake->pending_feedback = false;
    }
    return fake->poll_ok;
}

static inline bool fake_drive_healthy(void *ctx, uint32_t now_ms)
{
    (void)now_ms;
    return ((FakeDrive *)ctx)->network_ok;
}

static inline bool fake_drive_all_feedback(void *ctx)
{
    const FakeDrive *fake = (const FakeDrive *)ctx;
    if (!fake->feedback_ok)
        return false;

    for (size_t i = 0; i < JOINT_DRIVE_AXES; ++i) {
        if (!fake->axis[i].feedback_valid)
            return false;
    }
    return true;
}

static inline bool fake_drive_read_axis(
    void *ctx, size_t axis, JointDriveAxisFeedback *out)
{
    FakeDrive *fake = (FakeDrive *)ctx;
    if (axis >= JOINT_DRIVE_AXES || out == NULL ||
        (fake->fail_axis_read && axis == 2U))
        return false;

    ++fake->reads;
    *out = fake->axis[axis];
    return true;
}

static inline bool fake_drive_send_targets(
    void *ctx, const int32_t *targets, size_t count)
{
    FakeDrive *fake = (FakeDrive *)ctx;
    if (targets == NULL || count != JOINT_DRIVE_AXES || !fake->send_ok)
        return false;

    memcpy(fake->last_targets, targets, sizeof(fake->last_targets));
    ++fake->commands;
    fake->pending_feedback = true;
    return true;
}

static inline uint32_t fake_drive_feedback_sequence(void *ctx, size_t axis)
{
    return ((const FakeDrive *)ctx)->sequence[axis];
}

static inline JointDrivePort fake_joint_drive_make_port(FakeDrive *fake)
{
    const JointDrivePort port = {
        .context = fake,
        .is_configured = fake_drive_is_configured,
        .poll = fake_drive_poll,
        .ready_for_motion = fake_drive_healthy,
        .healthy = fake_drive_healthy,
        .all_feedback_valid = fake_drive_all_feedback,
        .read_axis = fake_drive_read_axis,
        .send_targets = fake_drive_send_targets,
        .feedback_sequence = fake_drive_feedback_sequence
    };
    return port;
}

#endif /* TEST_FAKE_JOINT_DRIVE_H */
