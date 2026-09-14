#include "dh_transform.h"

#include <math.h>
#include <stddef.h>


/* ============================================================================
 * STANDARD DH HOMOGENEOUS TRANSFORMATION
 * ============================================================================
 *
 * Direct C equivalent of:
 *
 *      function A = dhTransform(a,d,alpha,theta)
 *
 * ============================================================================
 */

void dh_transform(
    double a,
    double d,
    double alpha,
    double theta,
    double A[4][4]
)
{
    /*
     * Protect against invalid output pointer.
     */
    if (A == NULL)
    {
        return;
    }


    /* ========================================================================
     * MATLAB:
     *
     *      ct = cos(theta);
     *      st = sin(theta);
     *
     *      ca = cos(alpha);
     *      sa = sin(alpha);
     * ========================================================================
     */

    double ct =
        cos(theta);

    double st =
        sin(theta);

    double ca =
        cos(alpha);

    double sa =
        sin(alpha);


    /* ========================================================================
     * STANDARD DH MATRIX
     * ========================================================================
     *
     * MATLAB:
     *
     * A = [
     *     ct,   -st*ca,    st*sa,    a*ct;
     *     st,    ct*ca,   -ct*sa,    a*st;
     *     0,     sa,       ca,       d;
     *     0,     0,        0,        1
     * ];
     */


    A[0][0] = ct;
    A[0][1] = -st * ca;
    A[0][2] =  st * sa;
    A[0][3] =  a * ct;


    A[1][0] = st;
    A[1][1] =  ct * ca;
    A[1][2] = -ct * sa;
    A[1][3] =  a * st;


    A[2][0] = 0.0;
    A[2][1] = sa;
    A[2][2] = ca;
    A[2][3] = d;


    A[3][0] = 0.0;
    A[3][1] = 0.0;
    A[3][2] = 0.0;
    A[3][3] = 1.0;
}