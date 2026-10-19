#ifndef GP_VIRTUAL_FT_CONTROLS_H
#define GP_VIRTUAL_FT_CONTROLS_H

#include <stdbool.h>

/* Pure, testable press-and-hold model used by the GUI. Opposing inputs
 * cancel. Neither this module nor the GUI generates a motion target. */
double gp_virtual_ft_x_request(bool plus_held, bool minus_held,
                               bool controls_enabled, double magnitude_N);
#endif
