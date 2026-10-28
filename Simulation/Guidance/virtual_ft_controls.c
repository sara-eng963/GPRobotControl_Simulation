#include "virtual_ft_controls.h"
#include <math.h>
#include <stddef.h>

static bool any_held(const VirtualFtKeys *keys)
{
    if (keys == NULL) return false;
    for (size_t i = 0U; i < VIRTUAL_FT_AXIS_COUNT; ++i)
        if (keys->positive[i] || keys->negative[i]) return true;
    return false;
}

void virtual_ft_input_gate_update(VirtualFtInputGate *gate,
                                     bool focused, bool clear_pressed,
                                     const VirtualFtKeys *keys)
{
    if (gate == NULL) return;
    if (!focused || clear_pressed) gate->suppressed_until_release = true;
    else if (!any_held(keys)) gate->suppressed_until_release = false;
}

void virtual_ft_wrench_request(const VirtualFtKeys *keys,
                                  bool enabled, double force_N,
                                  double torque_Nm, double wrench[6])
{
    if (wrench == NULL) return;
    for (size_t i = 0U; i < VIRTUAL_FT_AXIS_COUNT; ++i) wrench[i] = 0.0;
    if (!enabled || keys == NULL || !isfinite(force_N) ||
        !isfinite(torque_Nm) || force_N < 0.0 || torque_Nm < 0.0)
        return;
    for (size_t i = 0U; i < VIRTUAL_FT_AXIS_COUNT; ++i) {
        if (keys->positive[i] == keys->negative[i]) continue;
        const double magnitude = i < 3U ? force_N : torque_Nm;
        wrench[i] = keys->positive[i] ? magnitude : -magnitude;
    }
}

double virtual_ft_x_request(bool plus_held, bool minus_held,
                               bool controls_enabled, double magnitude_N)
{
    if (!controls_enabled || !isfinite(magnitude_N) || magnitude_N < 0.0 ||
        plus_held == minus_held) return 0.0;
    return plus_held ? magnitude_N : -magnitude_N;
}
