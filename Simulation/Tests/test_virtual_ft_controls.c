#include "../Guidance/virtual_ft_controls.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    assert(gp_virtual_ft_x_request(true, false, true, 2.5) == 2.5);
    assert(gp_virtual_ft_x_request(false, true, true, 2.5) == -2.5);
    assert(gp_virtual_ft_x_request(true, true, true, 2.5) == 0.0);
    assert(gp_virtual_ft_x_request(false, false, true, 2.5) == 0.0);
    assert(gp_virtual_ft_x_request(true, false, false, 2.5) == 0.0);
    assert(gp_virtual_ft_x_request(true, false, true, NAN) == 0.0);
    assert(gp_virtual_ft_x_request(true, false, true, -1.0) == 0.0);
    puts("PASS: held X force direction, release, cancellation and inhibition");
    return 0;
}
