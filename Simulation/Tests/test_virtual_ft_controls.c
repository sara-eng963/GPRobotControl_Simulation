#include "../Guidance/virtual_ft_controls.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void assert_zero(const double w[6])
{
    for (unsigned i = 0U; i < 6U; ++i) assert(w[i] == 0.0);
}

int main(void)
{
    assert(virtual_ft_x_request(true, false, true, 2.5) == 2.5);
    assert(virtual_ft_x_request(false, true, true, 2.5) == -2.5);
    assert(virtual_ft_x_request(true, true, true, 2.5) == 0.0);
    assert(virtual_ft_x_request(false, false, true, 2.5) == 0.0);
    assert(virtual_ft_x_request(true, false, false, 2.5) == 0.0);
    assert(virtual_ft_x_request(true, false, true, NAN) == 0.0);
    assert(virtual_ft_x_request(true, false, true, -1.0) == 0.0);

    VirtualFtKeys keys = {0};
    double w[6] = {0};
    /* All twelve key directions produce the correct channel and sign. */
    for (unsigned i = 0U; i < 6U; ++i) {
        const double magnitude = i < 3U ? 1.5 : 0.25;
        keys.positive[i] = true;
        virtual_ft_wrench_request(&keys, true, 1.5, 0.25, w);
        for (unsigned j = 0U; j < 6U; ++j)
            assert(w[j] == (i == j ? magnitude : 0.0));
        keys.positive[i] = false;
        keys.negative[i] = true;
        virtual_ft_wrench_request(&keys, true, 1.5, 0.25, w);
        for (unsigned j = 0U; j < 6U; ++j)
            assert(w[j] == (i == j ? -magnitude : 0.0));
        keys.negative[i] = false;
    }
    /* Six simultaneous components and per-axis opposing cancellation. */
    for (unsigned i = 0U; i < 6U; ++i) keys.positive[i] = true;
    virtual_ft_wrench_request(&keys, true, 1.5, 0.25, w);
    for (unsigned i = 0U; i < 6U; ++i)
        assert(w[i] == (i < 3U ? 1.5 : 0.25));
    keys.negative[0] = true;
    keys.negative[4] = true;
    virtual_ft_wrench_request(&keys, true, 1.5, 0.25, w);
    assert(w[0] == 0.0 && w[4] == 0.0);
    assert(w[1] == 1.5 && w[2] == 1.5 && w[3] == 0.25 && w[5] == 0.25);
    /* Individual release preserves other held contributions. */
    keys.positive[2] = false;
    virtual_ft_wrench_request(&keys, true, 2.5, 0.5, w);
    assert(w[2] == 0.0 && w[1] == 2.5 && w[3] == 0.5);
    memset(&keys, 0, sizeof(keys));
    virtual_ft_wrench_request(&keys, true, 2.5, 0.5, w);
    assert_zero(w);
    keys.positive[5] = true;
    virtual_ft_wrench_request(&keys, false, 2.5, 0.5, w);
    assert_zero(w);
    virtual_ft_wrench_request(&keys, true, NAN, 0.5, w);
    assert_zero(w);
    virtual_ft_wrench_request(&keys, true, 2.5, -1.0, w);
    assert_zero(w);

    VirtualFtInputGate gate = {0};
    virtual_ft_input_gate_update(&gate, true, false, &keys);
    assert(!gate.suppressed_until_release);
    virtual_ft_input_gate_update(&gate, false, false, &keys);
    assert(gate.suppressed_until_release);
    virtual_ft_input_gate_update(&gate, true, false, &keys);
    assert(gate.suppressed_until_release); /* held across focus regain */
    virtual_ft_wrench_request(&keys, !gate.suppressed_until_release,
                                 1.0, 0.25, w);
    assert_zero(w);
    memset(&keys, 0, sizeof(keys));
    virtual_ft_input_gate_update(&gate, true, false, &keys);
    assert(!gate.suppressed_until_release);
    keys.positive[3] = true;
    virtual_ft_input_gate_update(&gate, true, true, &keys);
    assert(gate.suppressed_until_release);
    virtual_ft_input_gate_update(&gate, true, false, &keys);
    assert(gate.suppressed_until_release);
    memset(&keys, 0, sizeof(keys));
    virtual_ft_input_gate_update(&gate, true, false, &keys);
    assert(!gate.suppressed_until_release);
    puts("PASS: six-axis held keys, signs, cancellation, release, sliders and focus/zero gate");
    return 0;
}
