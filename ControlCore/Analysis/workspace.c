#include "workspace.h"

#include "../Kinematics/control_fk.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>


/* ============================================================================
 * LOCAL MT19937 RANDOM GENERATOR
 * ============================================================================
 *
 * MATLAB source uses:
 *
 *      rng(randomSeed, "twister")
 *
 * This gives us deterministic Mersenne-Twister-style sampling without
 * modifying global C library random state.
 *
 * NOTE:
 * Exact floating-point random sequences are not guaranteed to be bit-for-bit
 * identical to MATLAB's implementation, but the analysis procedure is the same.
 */

#define MT_N 624
#define MT_M 397

typedef struct
{
    uint32_t state[MT_N];
    int index;

} MT19937;


static void mt_seed(
    MT19937 *mt,
    uint32_t seed
)
{
    mt->state[0] = seed;

    for (int i = 1; i < MT_N; i++)
    {
        mt->state[i] =
            1812433253U *
            (mt->state[i - 1] ^ (mt->state[i - 1] >> 30)) +
            (uint32_t)i;
    }

    mt->index = MT_N;
}


static void mt_twist(
    MT19937 *mt
)
{
    const uint32_t upperMask = 0x80000000U;
    const uint32_t lowerMask = 0x7fffffffU;

    for (int i = 0; i < MT_N; i++)
    {
        uint32_t y =
            (mt->state[i] & upperMask) |
            (mt->state[(i + 1) % MT_N] & lowerMask);

        uint32_t next =
            mt->state[(i + MT_M) % MT_N] ^
            (y >> 1);

        if (y & 1U)
        {
            next ^= 0x9908b0dfU;
        }

        mt->state[i] = next;
    }

    mt->index = 0;
}


static uint32_t mt_u32(
    MT19937 *mt
)
{
    if (mt->index >= MT_N)
    {
        mt_twist(mt);
    }

    uint32_t y = mt->state[mt->index++];

    y ^= y >> 11;
    y ^= (y << 7)  & 0x9d2c5680U;
    y ^= (y << 15) & 0xefc60000U;
    y ^= y >> 18;

    return y;
}


/*
 * 53-bit floating-point random number in [0, 1).
 */
static double mt_uniform01(
    MT19937 *mt
)
{
    uint32_t a = mt_u32(mt) >> 5;
    uint32_t b = mt_u32(mt) >> 6;

    return
        (
            (double)a * 67108864.0 +
            (double)b
        ) /
        9007199254740992.0;
}


/* ============================================================================
 * HELPERS
 * ============================================================================ */

static bool valid_joint_limits(
    const RobotConfig *robot
)
{
    if (robot == NULL)
    {
        return false;
    }

    for (int joint = 0; joint < ROBOT_DOF; joint++)
    {
        if (
            !isfinite(robot->limits.qMin[joint]) ||
            !isfinite(robot->limits.qMax[joint]) ||
            robot->limits.qMin[joint] >= robot->limits.qMax[joint]
        )
        {
            return false;
        }
    }

    return true;
}


static void copy_joint_vector(
    double destination[ROBOT_DOF],
    const double source[ROBOT_DOF]
)
{
    for (int joint = 0; joint < ROBOT_DOF; joint++)
    {
        destination[joint] = source[joint];
    }
}


static void copy_point(
    double destination[3],
    const double source[3]
)
{
    for (int axis = 0; axis < 3; axis++)
    {
        destination[axis] = source[axis];
    }
}


/* ============================================================================
 * DEFAULT REQUEST
 * ============================================================================ */

void workspace_default_request(
    WorkspaceRequest *request
)
{
    if (request == NULL)
    {
        return;
    }

    request->numSamples = 10000;
    request->randomSeed = 1U;
}


/* ============================================================================
 * WORKSPACE ANALYSIS
 * ============================================================================ */

bool analysis_workspace(
    const RobotConfig *robot,
    const WorkspaceRequest *request,
    WorkspaceStorage *storage,
    WorkspaceResult *result
)
{
    if (
        robot == NULL ||
        request == NULL ||
        storage == NULL ||
        result == NULL
    )
    {
        return false;
    }

    if (
        request->numSamples == 0 ||
        storage->capacity < request->numSamples ||
        storage->q == NULL ||
        storage->points == NULL ||
        storage->radius3D == NULL ||
        storage->horizontalRadius == NULL
    )
    {
        return false;
    }

    if (!valid_joint_limits(robot))
    {
        return false;
    }

    memset(result, 0, sizeof(*result));

    const size_t numSamples = request->numSamples;


    /* ------------------------------------------------------------------------
     * 1. Generate joint configurations
     * ------------------------------------------------------------------------
     *
     * MATLAB:
     *
     * for joint = 1:robot.dof
     *
     *     Q(joint,:) =
     *         qMin(joint) +
     *         (qMax(joint)-qMin(joint)) .* rand(1,numSamples);
     *
     * end
     */

    MT19937 random;
    mt_seed(&random, request->randomSeed);

    for (int joint = 0; joint < ROBOT_DOF; joint++)
    {
        const double qMin =
            robot->limits.qMin[joint];

        const double qMax =
            robot->limits.qMax[joint];

        const double range =
            qMax - qMin;

        for (size_t sample = 0; sample < numSamples; sample++)
        {
            const double u =
                mt_uniform01(&random);

            storage->q[sample][joint] =
                qMin + range * u;
        }
    }


    /* ------------------------------------------------------------------------
     * 2. Initialize workspace extrema
     * ------------------------------------------------------------------------ */

    double minReach = DBL_MAX;
    double maxReach = -DBL_MAX;

    double minHorizontalReach = DBL_MAX;
    double maxHorizontalReach = -DBL_MAX;

    double xMin = DBL_MAX;
    double xMax = -DBL_MAX;

    double yMin = DBL_MAX;
    double yMax = -DBL_MAX;

    double zMin = DBL_MAX;
    double zMax = -DBL_MAX;

    size_t indexMinReach = 0;
    size_t indexMinHorizontal = 0;


    /* ------------------------------------------------------------------------
     * 3. Forward kinematics for every configuration
     * ------------------------------------------------------------------------ */

    for (size_t sample = 0; sample < numSamples; sample++)
    {
        double T_B_TCP[4][4];

        control_fk(
            robot,
            storage->q[sample],
            T_B_TCP
        );

        const double x = T_B_TCP[0][3];
        const double y = T_B_TCP[1][3];
        const double z = T_B_TCP[2][3];

        storage->points[sample][0] = x;
        storage->points[sample][1] = y;
        storage->points[sample][2] = z;


        /* --------------------------------------------------------------------
         * 3D distance from base origin
         * -------------------------------------------------------------------- */

        const double radius3D =
            sqrt(
                x * x +
                y * y +
                z * z
            );

        storage->radius3D[sample] =
            radius3D;


        /* --------------------------------------------------------------------
         * Horizontal distance from base Z-axis
         * -------------------------------------------------------------------- */

        const double horizontalRadius =
            sqrt(
                x * x +
                y * y
            );

        storage->horizontalRadius[sample] =
            horizontalRadius;


        /* --------------------------------------------------------------------
         * Reach extrema
         * -------------------------------------------------------------------- */

        if (radius3D < minReach)
        {
            minReach = radius3D;
            indexMinReach = sample;
        }

        if (radius3D > maxReach)
        {
            maxReach = radius3D;
        }

        if (horizontalRadius < minHorizontalReach)
        {
            minHorizontalReach = horizontalRadius;
            indexMinHorizontal = sample;
        }

        if (horizontalRadius > maxHorizontalReach)
        {
            maxHorizontalReach = horizontalRadius;
        }


        /* --------------------------------------------------------------------
         * Cartesian ranges
         * -------------------------------------------------------------------- */

        if (x < xMin) xMin = x;
        if (x > xMax) xMax = x;

        if (y < yMin) yMin = y;
        if (y > yMax) yMax = y;

        if (z < zMin) zMin = z;
        if (z > zMax) zMax = z;
    }


    /* ------------------------------------------------------------------------
     * 4. Store result
     * ------------------------------------------------------------------------ */

    result->numSamples = numSamples;
    result->randomSeed = request->randomSeed;

    result->q = storage->q;
    result->points = storage->points;

    result->radius3D = storage->radius3D;
    result->horizontalRadius = storage->horizontalRadius;


    result->minReach = minReach;
    result->maxReach = maxReach;

    result->minHorizontalReach =
        minHorizontalReach;

    result->maxHorizontalReach =
        maxHorizontalReach;


    result->xRange[0] = xMin;
    result->xRange[1] = xMax;

    result->yRange[0] = yMin;
    result->yRange[1] = yMax;

    result->zRange[0] = zMin;
    result->zRange[1] = zMax;


    /* Closest to base origin */

    result->closestToBaseIndex =
        indexMinReach;

    copy_point(
        result->closestToBasePoint,
        storage->points[indexMinReach]
    );

    copy_joint_vector(
        result->closestToBaseQ,
        storage->q[indexMinReach]
    );

    result->closestToBaseDistance =
        minReach;


    /* Closest to base Z-axis */

    result->closestToBaseAxisIndex =
        indexMinHorizontal;

    copy_point(
        result->closestToBaseAxisPoint,
        storage->points[indexMinHorizontal]
    );

    copy_joint_vector(
        result->closestToBaseAxisQ,
        storage->q[indexMinHorizontal]
    );

    result->closestToBaseAxisHorizontalDistance =
        minHorizontalReach;


    return true;
}