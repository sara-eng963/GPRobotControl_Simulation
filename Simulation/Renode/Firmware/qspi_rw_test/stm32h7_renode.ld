ENTRY(Reset_Handler)

MEMORY
{
    ITCM  (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1M

    /*
     * Reserve 0x24000000..0x240000FF for Renode-visible test result words.
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
