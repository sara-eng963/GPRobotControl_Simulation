#include "HMI/teaching_sim_app.h"

#include <sys/resource.h>

int main(void)
{
    (void)setpriority(
        PRIO_PROCESS,
        0,
        15
    );

    return
        teaching_sim_app_run();
}
