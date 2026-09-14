#include "generate_line_waypoints.h"

#include <stddef.h>


bool generate_line_waypoints(
    Vec3 start,
    Vec3 end,
    size_t numWaypoints,
    Vec3 *waypoints
)
{
    if (
        waypoints == NULL ||
        numWaypoints < 2
    )
    {
        return false;
    }


    for (size_t i = 0;
         i < numWaypoints;
         i++)
    {
        real_t s =
            (real_t)i /
            (real_t)(numWaypoints - 1);


        for (int axis = 0;
             axis < 3;
             axis++)
        {
            waypoints[i].v[axis] =
                start.v[axis]
                +
                s *
                (
                    end.v[axis] -
                    start.v[axis]
                );
        }
    }


    return true;
}