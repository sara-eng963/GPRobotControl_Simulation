#include "virtual_ft_controls.h"
#include <math.h>

double gp_virtual_ft_x_request(bool plus_held, bool minus_held,
                               bool controls_enabled, double magnitude_N)
{
    if (!controls_enabled || !isfinite(magnitude_N) || magnitude_N < 0.0 ||
        plus_held == minus_held) return 0.0;
    return plus_held ? magnitude_N : -magnitude_N;
}
