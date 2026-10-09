#ifndef STM32H7_W25Q_BUS_H
#define STM32H7_W25Q_BUS_H
#include "w25q512jv_flash.h"
/* STM32H745 QUADSPI indirect-mode bus. The caller must initialize the board's
 * QUADSPI GPIO/AF, clocks, and use a reliable monotonic microsecond timer.
 * Only ONE task may use QUADSPI at a time; no DMA or cache configuration here.
 * W25Q_RENODE_VIRTUAL_TIME selects simulator-only protocol compatibility.
 * W25Q_TEST_MMIO selects a host mock peripheral, NEVER real hardware.
 */
bool stm32h7_w25q_bus_make(W25Q512JVBus *bus);
#endif
