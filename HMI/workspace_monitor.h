#ifndef HMI_WORKSPACE_MONITOR_H
#define HMI_WORKSPACE_MONITOR_H

#include <stdbool.h>
#include <stddef.h>

#define HMI_WORKSPACE_Z_BINS 80

typedef enum
{
    HMI_WORKSPACE_CHECK_UNAVAILABLE = 0,
    HMI_WORKSPACE_CHECK_INSIDE,
    HMI_WORKSPACE_CHECK_OUTSIDE
} HmiWorkspaceCheckStatus;

typedef struct
{
    HmiWorkspaceCheckStatus status;

    double rho;
    double rhoMin;
    double rhoMax;

    double z;
    double zMin;
    double zMax;
} HmiWorkspaceCheck;

typedef struct
{
    bool ready;

    double zMin;
    double zMax;

    double rhoMin[HMI_WORKSPACE_Z_BINS];
    double rhoMax[HMI_WORKSPACE_Z_BINS];
    size_t sampleCount[HMI_WORKSPACE_Z_BINS];

    size_t sourceSamples;
} HmiWorkspaceMonitor;

/*
 * Build a compact radial-vs-height envelope from ControlCore's sampled
 * kinematic workspace. The expensive FK sampling is done once at HMI startup.
 *
 * This is a POSITION-ONLY sampled-workspace warning helper. It is not an IK,
 * orientation, collision, singularity, or trajectory-validity check.
 */
bool hmi_workspace_monitor_init(HmiWorkspaceMonitor *monitor);

void hmi_workspace_monitor_shutdown(HmiWorkspaceMonitor *monitor);

HmiWorkspaceCheck hmi_workspace_monitor_check(
    const HmiWorkspaceMonitor *monitor,
    double x,
    double y,
    double z
);

#endif /* HMI_WORKSPACE_MONITOR_H */
