#include "canopen_joint_drive_port.h"
#include "../CiA402/cia402.h"

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

static bool healthy(void *context, uint32_t now_ms)
{
    return canopen_master_healthy((const CanopenMaster *)context, now_ms);
}

static bool all_feedback_valid(void *context)
{
    return canopen_master_all_feedback_valid((const CanopenMaster *)context);
}

static bool read_axis(void *context, size_t axis, JointDriveAxisFeedback *feedback)
{
    if (feedback == NULL) return false;
    const AvatarMDrive *drive =
        canopen_master_drive((const CanopenMaster *)context, axis);
    if (drive == NULL) return false;
    feedback->feedback_valid = drive->feedback_valid;
    feedback->operation_enabled =
        drive->cia402_state == CIA402_STATE_OPERATION_ENABLED;
    feedback->actual_position_units = drive->feedback.actual_position;
    return true;
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
        .healthy = healthy,
        .all_feedback_valid = all_feedback_valid,
        .read_axis = read_axis,
        .send_targets = send_targets,
        .feedback_sequence = feedback_sequence,
    };
    return port;
}
