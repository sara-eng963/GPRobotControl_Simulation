#ifndef JOINT_DRIVE_PORT_H
#define JOINT_DRIVE_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Six-axis application-facing motion contract.
 *
 * The owner (currently the supervisor task) is responsible for serializing
 * calls with drive commissioning and other motion states. This is NOT an
 * inter-core shared-memory ABI or an ISR-safe interface.
 *
 * A send_targets call represents one complete coordinated command cycle.
 * For AVATAR/CANopen the adapter emits 6 RPDO4 frames followed by SYNC.
 * feedback_sequence counts fresh received feedback for each axis.
 */
#define JOINT_DRIVE_AXES 6U

typedef struct {
    bool feedback_valid;
    bool operation_enabled;
    int32_t actual_position_units;
} JointDriveAxisFeedback;

typedef struct {
    void *context;
    bool (*is_configured)(void *context);
    bool (*poll)(void *context, uint32_t now_ms);
    bool (*ready_for_motion)(void *context, uint32_t now_ms);
    bool (*healthy)(void *context, uint32_t now_ms);
    bool (*all_feedback_valid)(void *context);
    bool (*read_axis)(void *context, size_t axis,
                      JointDriveAxisFeedback *feedback);
    bool (*send_targets)(void *context, const int32_t *targets, size_t count);
    uint32_t (*feedback_sequence)(void *context, size_t axis);
} JointDrivePort;

static inline bool joint_drive_port_valid(const JointDrivePort *port)
{
    return port != NULL &&
           port->context != NULL &&
           port->is_configured != NULL &&
           port->poll != NULL &&
           port->ready_for_motion != NULL &&
           port->healthy != NULL &&
           port->all_feedback_valid != NULL &&
           port->read_axis != NULL &&
           port->send_targets != NULL &&
           port->feedback_sequence != NULL &&
           port->is_configured(port->context);
}

static inline bool joint_drive_port_poll(const JointDrivePort *port,
                                         uint32_t now_ms)
{
    return joint_drive_port_valid(port) && port->poll(port->context, now_ms);
}

static inline bool joint_drive_port_ready(const JointDrivePort *port,
                                          uint32_t now_ms)
{
    return joint_drive_port_valid(port) &&
           port->ready_for_motion(port->context, now_ms);
}

static inline bool joint_drive_port_healthy(const JointDrivePort *port,
                                           uint32_t now_ms)
{
    return joint_drive_port_valid(port) &&
           port->healthy(port->context, now_ms);
}

static inline bool joint_drive_port_all_feedback_valid(const JointDrivePort *port)
{
    return joint_drive_port_valid(port) &&
           port->all_feedback_valid(port->context);
}

static inline bool joint_drive_port_read_axis(const JointDrivePort *port,
                                               size_t axis,
                                               JointDriveAxisFeedback *feedback)
{
    return joint_drive_port_valid(port) && axis < JOINT_DRIVE_AXES &&
           feedback != NULL &&
           port->read_axis(port->context, axis, feedback);
}

/* BOOT-only enable-state diagnostic. Unlike CanopenMaster's
 * all_drives_operation_enabled(), this checks only decoded enable status,
 * not feedback_valid; BOOT checks feedback validity separately.
 * joint_drive_port_ready() is the canonical motion-readiness predicate.
 */
static inline bool joint_drive_port_all_enabled(const JointDrivePort *port)
{
    if (!joint_drive_port_valid(port))
        return false;
    for (size_t i = 0U; i < JOINT_DRIVE_AXES; ++i) {
        JointDriveAxisFeedback axis = {0};
        if (!joint_drive_port_read_axis(port, i, &axis) ||
            !axis.operation_enabled)
            return false;
    }
    return true;
}

static inline bool joint_drive_port_send_targets(const JointDrivePort *port,
                                                 const int32_t targets[JOINT_DRIVE_AXES])
{
    return joint_drive_port_valid(port) && targets != NULL &&
           port->send_targets(port->context, targets, JOINT_DRIVE_AXES);
}

/* 0 is a valid TPDO count. Out-of-range, null or unconfigured ports fail
 * without modifying *sequence. The vtable signature remains unchanged.
 */
static inline bool joint_drive_port_feedback_sequence(const JointDrivePort *port,
                                                      size_t axis,
                                                      uint32_t *sequence)
{
    if (!joint_drive_port_valid(port) ||
        axis >= JOINT_DRIVE_AXES || sequence == NULL)
        return false;
    *sequence = port->feedback_sequence(port->context, axis);
    return true;
}

#endif /* JOINT_DRIVE_PORT_H */
