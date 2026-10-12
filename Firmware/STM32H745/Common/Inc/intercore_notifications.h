#ifndef INTERCORE_NOTIFICATIONS_H
#define INTERCORE_NOTIFICATIONS_H

#include <stdbool.h>

/* The STM32H745 dual-core IPC notification adapter, not a safety channel.
 * M7 configures EXTI0 (M7 -> M4) and EXTI1 (M4 -> M7) before HSEM-0
 * wakeup. M4 enables its receiving IRQ only after validating shared IPC.
 * Both cores reserve these EXTI lines exclusively for IPC.
 */
bool intercore_notifications_initialize(void);

/* Called by the local EXTI IRQ handler, never by a task. */
void intercore_notifications_handle_irq(void);

#endif /* INTERCORE_NOTIFICATIONS_H */
