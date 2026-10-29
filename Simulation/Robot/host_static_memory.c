/* POSIX kernel only: required when the real IPC message buffers use static allocation. */
#include "FreeRTOS.h"
#include "task.h"
void vApplicationGetIdleTaskMemory(StaticTask_t **task, StackType_t **stack,
                                   configSTACK_DEPTH_TYPE *depth)
{
    static StaticTask_t idle;
    static StackType_t memory[configMINIMAL_STACK_SIZE];
    *task=&idle; *stack=memory; *depth=configMINIMAL_STACK_SIZE;
}
