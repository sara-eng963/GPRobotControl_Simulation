ENTRY(Reset_Handler)

MEMORY
{
    /*
     * Renode maps ITCM at 0x00000000. Keeping the reset vector here avoids
     * depending on STM32 boot aliasing during this peripheral-only test.
     */
    ITCM  (rx)  : ORIGIN = 0x00000000, LENGTH = 64K

    /* STM32H7 internal flash bank 1 in the Renode platform. */
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1M

    /*
     * AXI SRAM is 512 KiB at 0x24000000.
     * Reserve the first 0x100 bytes for test result words read by Renode.
     */
    RAM   (rwx) : ORIGIN = 0x24000100, LENGTH = 0x7FF00
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > ITCM

    .text :
    {
        *(.text*)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
    } > FLASH

    .ARM.exidx :
    {
        *(.ARM.exidx*)
    } > FLASH

    .data :
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM AT > FLASH

    _sidata = LOADADDR(.data);

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
    }
}
