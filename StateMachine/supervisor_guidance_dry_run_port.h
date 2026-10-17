#ifndef SUPERVISOR_GUIDANCE_DRY_RUN_PORT_H
#define SUPERVISOR_GUIDANCE_DRY_RUN_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "../ServoDrive/JointDrive/joint_drive_port.h"
#include "../ServoDrive/AvatarM/avatar_m_position.h"
#include "../ControlCore/Config/robot_config.h"

/* Optional M7 Supervisor task-context observation hook.
 * NO command-output or enable-drive function exists in this contract.
 * A null port means Stage 4E2A is disabled; no hardware provider is wired.
 */
typedef struct {
    /* Joint feedback copied by the M7 owner; NO send_targets vtable. */
    JointDriveAxisFeedback feedback[JOINT_DRIVE_AXES];
    bool feedback_healthy;
    const AvatarMPositionScale *scales;
    const RobotConfig *robot;
    uint64_t now_us;
    uint32_t now_ms;
    double period_s;
    uint32_t runtime_timestamp_ms;
    uint32_t safety_timestamp_ms;
    bool teaching_active;
    bool manual_guidance_requested;
    bool motion_permitted;
    bool robot_homed;
    bool drives_ready;
    bool communication_healthy;
    bool safety_valid;
    bool estop_active;
    bool protective_stop_active;
    bool global_fault_active;
} SupervisorGuidanceDryRunCycle;

typedef struct {
    void *context;
    /* Called once per TEACHING cycle when manual guidance is requested.
     * False indicates rejected/faulted dry-run; NO motor command occurs. */
    bool (*tick)(void *context, const SupervisorGuidanceDryRunCycle *cycle);
    /* Called on disengagement and in states other than TEACHING. */
    void (*stop)(void *context);
} SupervisorGuidanceDryRunPort;

#endif
