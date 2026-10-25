#ifndef APPLICATION_SUPERVISOR_H
#define APPLICATION_SUPERVISOR_H
#include <stdbool.h>
/* Use CubeMX's public HAL umbrella header: it loads hal_conf and the
 * enabled TIM module in the supported include order. */
#include "stm32h7xx_hal.h"
#include "../../Application/StateMachine/supervisor_task.h"

/* Fail-closed default until an actual STM32 board port supplies real I/O. */
bool application_supervisor_board_configure(SupervisorTaskConfig *config);
/* Target-specific composition of the existing supervisor, not a new RTOS. */
bool application_supervisor_start(const SupervisorTaskConfig *config);
/* Called from CubeMX HAL_TIM_PeriodElapsedCallback; ignores TIM6. */
void application_supervisor_timer_period_elapsed_isr(
    TIM_HandleTypeDef *timer);
#endif
