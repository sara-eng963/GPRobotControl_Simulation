#include <stdint.h>

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;

int main(void);

void Reset_Handler(void);
void Reset_Handler_C(void);
static void Default_Handler(void);

__attribute__((used, section(".isr_vector")))
const uintptr_t vector_table[] =
{
    (uintptr_t)&_estack,
    (uintptr_t)Reset_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    0U,
    0U,
    0U,
    0U,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler,
    0U,
    (uintptr_t)Default_Handler,
    (uintptr_t)Default_Handler
};

/*
 * Renode's ELF loader can enter the ELF entry point directly. Set MSP
 * explicitly before executing ordinary C so this test does not depend on
 * boot-ROM alias behavior at address 0x00000000.
 */
__attribute__((naked))
void Reset_Handler(void)
{
    __asm volatile (
        "ldr r0, =_estack\n"
        "msr msp, r0\n"
        "b Reset_Handler_C\n"
    );
}

void Reset_Handler_C(void)
{
    uint32_t *source = &_sidata;
    uint32_t *destination = &_sdata;

    while (destination < &_edata)
    {
        *destination++ = *source++;
    }

    destination = &_sbss;

    while (destination < &_ebss)
    {
        *destination++ = 0U;
    }

    (void)main();

    for (;;)
    {
        __asm volatile ("wfi");
    }
}

static void Default_Handler(void)
{
    for (;;)
    {
        __asm volatile ("wfi");
    }
}
