#include "admittance_matlab_reference.h"

#include <stddef.h>

bool admittance_matlab_reference_md(AdmittanceParams *params)
{
    if (params == NULL) return false;
    /* Full-precision optimized values exported from design_6axis_admittance.m.
     * Axes 0..2: kg, N*s/m. Axes 3..5: kg*m^2, N*m*s/rad.
     * This function does not enable motion or authorize a safety configuration.
     */
    static const double mass[ADMITTANCE_AXES] = {
        29.22801456612752, 26.361336639519262, 23.74469161916069,
        1.6319684618011627, 1.4732529319920249, 1.1148713535550765
    };
    static const double damping[ADMITTANCE_AXES] = {
        143.9195532171934, 132.5682958794217, 121.32087961505862,
        8.1027427354888548, 7.2923649840052081, 6.0692154632587449
    };
    for (size_t i = 0U; i < ADMITTANCE_AXES; ++i) {
        params->virtual_mass[i] = mass[i];
        params->damping[i] = damping[i];
    }
    return true;
}
