#include "canopen_joint_drive_port.h"

static bool configured(void *context)
{
    const CanopenMaster *master = (const CanopenMaster *)context;
    return master != NULL && master->initialized &&
           master->node_count == JOINT_DRIVE_AXES;
}

static bool poll_drive(void *context, uint32_t now_ms)
{
    return canopen_master_poll((CanopenMaster *)context, now_ms);
}

static bool ready_for_motion(void *context, uint32_t now_ms)
{
    return canopen_master_ready_for_motion((const CanopenMaster *)context, now_ms);
}

static bool send_targets(void *context, const int32_t *targets, size_t count)
{
    /* The coordinator retains the six-RPDO4-then-SYNC transaction contract. */
    return canopen_master_send_target_cycle((CanopenMaster *)context,
                                             targets, count);
}

static uint32_t feedback_sequence(void *context, size_t axis)
{
    return canopen_master_tpdo_rx_count((const CanopenMaster *)context, axis);
}

JointDrivePort canopen_joint_drive_port_make(CanopenMaster *master)
{
    const JointDrivePort port = {
        .context = master,
        .is_configured = configured,
        .poll = poll_drive,
        .ready_for_motion = ready_for_motion,
        .send_targets = send_targets,
        .feedback_sequence = feedback_sequence,
    };
    return port;
}
