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
    void *context;
    bool (*is_configured)(void *context);
    bool (*poll)(void *context, uint32_t now_ms);
    bool (*ready_for_motion)(void *context, uint32_t now_ms);
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

static inline bool joint_drive_port_send_targets(const JointDrivePort *port,
                                                 const int32_t targets[JOINT_DRIVE_AXES])
{
    return joint_drive_port_valid(port) && targets != NULL &&
           port->send_targets(port->context, targets, JOINT_DRIVE_AXES);
}

static inline uint32_t joint_drive_port_feedback_sequence(const JointDrivePort *port,
                                                           size_t axis)
{
    if (!joint_drive_port_valid(port) || axis >= JOINT_DRIVE_AXES)
        return 0U;
    return port->feedback_sequence(port->context, axis);
}

#endif /* JOINT_DRIVE_PORT_H */
