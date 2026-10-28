#ifndef HANDGUIDING_ADMITTANCE_MATLAB_REFERENCE_H
#define HANDGUIDING_ADMITTANCE_MATLAB_REFERENCE_H

#include "admittance_control.h"

/* MATLAB optimizer output, 2026-10-28; 6-axis X,Y,Z,Rx,Ry,Rz.
 * Numerical reference only, NOT approved physical robot tuning.
 * Populates ONLY virtual_mass and damping. In particular, it does not
 * invent the actuator-specific speed, acceleration, or deadline limits.
 * Other AdmittanceParams members are left untouched; caller must supply
 * validated limits before admittance_enable() can succeed.
 */
bool admittance_matlab_reference_md(AdmittanceParams *params);

#endif /* HANDGUIDING_ADMITTANCE_MATLAB_REFERENCE_H */
