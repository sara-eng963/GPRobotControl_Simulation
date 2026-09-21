#include "workspace_monitor.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "robot_config.h"
#include "workspace.h"

#define HMI_WORKSPACE_SAMPLE_COUNT 10000U
#define HMI_WORKSPACE_Z_NEIGHBORS  2
#define HMI_WORKSPACE_Z_MARGIN     0.025
#define HMI_WORKSPACE_RHO_MARGIN   0.035

static size_t z_to_bin(
    const HmiWorkspaceMonitor *monitor,
    double z
)
{
    if (monitor == NULL || monitor->zMax <= monitor->zMin)
    {
        return 0;
    }

    double normalized =
        (z - monitor->zMin) /
        (monitor->zMax - monitor->zMin);

    if (normalized < 0.0) normalized = 0.0;
    if (normalized > 1.0) normalized = 1.0;

    size_t bin =
        (size_t)floor(normalized * (double)HMI_WORKSPACE_Z_BINS);

    if (bin >= HMI_WORKSPACE_Z_BINS)
    {
        bin = HMI_WORKSPACE_Z_BINS - 1U;
    }

    return bin;
}

bool hmi_workspace_monitor_init(HmiWorkspaceMonitor *monitor)
{
    if (monitor == NULL)
    {
        return false;
    }

    memset(monitor, 0, sizeof(*monitor));

    double (*q)[ROBOT_DOF] =
        malloc(HMI_WORKSPACE_SAMPLE_COUNT * sizeof(*q));

    double (*points)[3] =
        malloc(HMI_WORKSPACE_SAMPLE_COUNT * sizeof(*points));

    double *radius3D =
        malloc(HMI_WORKSPACE_SAMPLE_COUNT * sizeof(*radius3D));

    double *horizontalRadius =
        malloc(HMI_WORKSPACE_SAMPLE_COUNT * sizeof(*horizontalRadius));

    if (
        q == NULL ||
        points == NULL ||
        radius3D == NULL ||
        horizontalRadius == NULL
    )
    {
        free(q);
        free(points);
        free(radius3D);
        free(horizontalRadius);
        return false;
    }

    RobotConfig robot;
    robot_config_init_ur5(&robot);

    WorkspaceRequest request;
    workspace_default_request(&request);
    request.numSamples = HMI_WORKSPACE_SAMPLE_COUNT;
    request.randomSeed = 1U;

    WorkspaceStorage storage =
    {
        .capacity = HMI_WORKSPACE_SAMPLE_COUNT,
        .q = q,
        .points = points,
        .radius3D = radius3D,
        .horizontalRadius = horizontalRadius
    };

    WorkspaceResult result;

    bool ok =
        analysis_workspace(
            &robot,
            &request,
            &storage,
            &result
        );

    if (!ok)
    {
        free(q);
        free(points);
        free(radius3D);
        free(horizontalRadius);
        return false;
    }

    monitor->zMin = result.zRange[0];
    monitor->zMax = result.zRange[1];
    monitor->sourceSamples = result.numSamples;

    for (size_t bin = 0; bin < HMI_WORKSPACE_Z_BINS; bin++)
    {
        monitor->rhoMin[bin] = DBL_MAX;
        monitor->rhoMax[bin] = -DBL_MAX;
        monitor->sampleCount[bin] = 0U;
    }

    for (size_t i = 0; i < result.numSamples; i++)
    {
        const double z = result.points[i][2];
        const double rho = result.horizontalRadius[i];
        const size_t bin = z_to_bin(monitor, z);

        if (rho < monitor->rhoMin[bin])
        {
            monitor->rhoMin[bin] = rho;
        }

        if (rho > monitor->rhoMax[bin])
        {
            monitor->rhoMax[bin] = rho;
        }

        monitor->sampleCount[bin]++;
    }

    monitor->ready = true;

    free(q);
    free(points);
    free(radius3D);
    free(horizontalRadius);

    return true;
}

void hmi_workspace_monitor_shutdown(HmiWorkspaceMonitor *monitor)
{
    if (monitor == NULL)
    {
        return;
    }

    memset(monitor, 0, sizeof(*monitor));
}

HmiWorkspaceCheck hmi_workspace_monitor_check(
    const HmiWorkspaceMonitor *monitor,
    double x,
    double y,
    double z
)
{
    HmiWorkspaceCheck check;
    memset(&check, 0, sizeof(check));

    check.status = HMI_WORKSPACE_CHECK_UNAVAILABLE;
    check.z = z;
    check.rho = sqrt(x * x + y * y);

    if (monitor == NULL || !monitor->ready)
    {
        return check;
    }

    check.zMin = monitor->zMin;
    check.zMax = monitor->zMax;

    if (
        z < monitor->zMin - HMI_WORKSPACE_Z_MARGIN ||
        z > monitor->zMax + HMI_WORKSPACE_Z_MARGIN
    )
    {
        check.status = HMI_WORKSPACE_CHECK_OUTSIDE;
        return check;
    }

    const size_t center = z_to_bin(monitor, z);

    double localRhoMin = DBL_MAX;
    double localRhoMax = -DBL_MAX;
    bool haveEnvelope = false;

    for (
        int offset = -HMI_WORKSPACE_Z_NEIGHBORS;
        offset <= HMI_WORKSPACE_Z_NEIGHBORS;
        offset++
    )
    {
        const int candidate = (int)center + offset;

        if (
            candidate < 0 ||
            candidate >= HMI_WORKSPACE_Z_BINS
        )
        {
            continue;
        }

        const size_t bin = (size_t)candidate;

        if (monitor->sampleCount[bin] == 0U)
        {
            continue;
        }

        if (monitor->rhoMin[bin] < localRhoMin)
        {
            localRhoMin = monitor->rhoMin[bin];
        }

        if (monitor->rhoMax[bin] > localRhoMax)
        {
            localRhoMax = monitor->rhoMax[bin];
        }

        haveEnvelope = true;
    }

    if (!haveEnvelope)
    {
        check.status = HMI_WORKSPACE_CHECK_OUTSIDE;
        return check;
    }

    check.rhoMin = localRhoMin;
    check.rhoMax = localRhoMax;

    if (
        check.rho < localRhoMin - HMI_WORKSPACE_RHO_MARGIN ||
        check.rho > localRhoMax + HMI_WORKSPACE_RHO_MARGIN
    )
    {
        check.status = HMI_WORKSPACE_CHECK_OUTSIDE;
    }
    else
    {
        check.status = HMI_WORKSPACE_CHECK_INSIDE;
    }

    return check;
}
