#ifndef CONTROLCORE_ANALYSIS_WORKSPACE_H
#define CONTROLCORE_ANALYSIS_WORKSPACE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../Config/robot_config.h"


/* ============================================================================
 * WORKSPACE REQUEST
 * ============================================================================
 *
 * C equivalent of:
 *
 *      analysis.workspace(robotModel, robot, numSamples, randomSeed)
 *
 * MATLAB defaults:
 *
 *      numSamples = 10000
 *      randomSeed = 1
 */

typedef struct
{
    size_t numSamples;
    uint32_t randomSeed;

} WorkspaceRequest;


/* ============================================================================
 * CALLER-PROVIDED STORAGE
 * ============================================================================
 *
 * No dynamic allocation is performed inside ControlCore.
 *
 * q:
 *      [capacity][ROBOT_DOF]
 *
 * points:
 *      [capacity][3]
 *
 * radius3D:
 *      sqrt(x^2 + y^2 + z^2)
 *
 * horizontalRadius:
 *      sqrt(x^2 + y^2)
 */

typedef struct
{
    size_t capacity;

    double (*q)[ROBOT_DOF];
    double (*points)[3];

    double *radius3D;
    double *horizontalRadius;

} WorkspaceStorage;


/* ============================================================================
 * RESULT
 * ============================================================================ */

typedef struct
{
    size_t numSamples;
    uint32_t randomSeed;

    /*
     * References to caller-provided storage.
     */
    double (*q)[ROBOT_DOF];
    double (*points)[3];

    double *radius3D;
    double *horizontalRadius;


    /* Overall 3D reach */

    double minReach;
    double maxReach;


    /* Distance from robot base Z-axis */

    double minHorizontalReach;
    double maxHorizontalReach;


    /* Cartesian limits */

    double xRange[2];
    double yRange[2];
    double zRange[2];


    /* Configuration closest to base origin */

    size_t closestToBaseIndex;

    double closestToBasePoint[3];
    double closestToBaseQ[ROBOT_DOF];
    double closestToBaseDistance;


    /* Configuration closest to base Z-axis */

    size_t closestToBaseAxisIndex;

    double closestToBaseAxisPoint[3];
    double closestToBaseAxisQ[ROBOT_DOF];
    double closestToBaseAxisHorizontalDistance;

} WorkspaceResult;


/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

void workspace_default_request(
    WorkspaceRequest *request
);


/*
 * Generate the mathematical kinematic TCP workspace.
 *
 * Considers:
 *
 *      - robot geometry
 *      - joint limits
 *      - TCP definition
 *
 * Does NOT consider:
 *
 *      - self collision
 *      - table collision
 *      - workpiece collision
 *      - welding torch orientation
 *
 * Returns:
 *
 *      true  -> successful
 *      false -> invalid request/storage/configuration
 */
bool analysis_workspace(
    const RobotConfig *robot,
    const WorkspaceRequest *request,
    WorkspaceStorage *storage,
    WorkspaceResult *result
);


#endif /* CONTROLCORE_ANALYSIS_WORKSPACE_H */