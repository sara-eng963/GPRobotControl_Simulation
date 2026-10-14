/*
******************************************************************************
**
**  File        : LinkerScript.ld (debug in RAM dedicated)
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                      1024Kbytes FLASH
**                       800Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2026 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1); /* end of "RAM_D1" Ram type memory */

_Min_Heap_Size = 0x200; /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Memories definition */
MEMORY
{
  RAM_D1 (xrw)   : ORIGIN = 0x24000000, LENGTH =  512K
  FLASH   (rx)   : ORIGIN = 0x08000000, LENGTH = 1024K    /* Memory is divided. Actual start is 0x8000000 and actual length is 2048K */
  DTCMRAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D2 (xrw)   : ORIGIN = 0x30000000, LENGTH = 288K
  /* Partition SRAM4 without overlapping physical MEMORY regions. */
  GP_SRAM4_IPC        (rw)  : ORIGIN = 0x38000000, LENGTH = 4K
  GP_SRAM4_TRAJECTORY (rw)  : ORIGIN = 0x38001000, LENGTH = 16K
  GP_SRAM4_RESERVED   (rw)  : ORIGIN = 0x38005000, LENGTH = 12K
  RAM_D3              (xrw) : ORIGIN = 0x38008000, LENGTH = 32K
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
}

/* Sections */
SECTIONS
{
  /* The startup code into "RAM" Ram type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >RAM_D1

  /* The program code and other data into "RAM" Ram type memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >RAM_D1

  /* Constant data into "RAM" Ram type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >RAM_D1

  .ARM.extab (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >RAM_D1
  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >RAM_D1

  .preinit_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >RAM_D1

  .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >RAM_D1

  .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >RAM_D1

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */

  } >RAM_D1

  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM_D1

  /* Stage 2C: shared SRAM4 objects, NOT part of either core's .bss clear.
   * These sections must hold only uninitialized objects. Cross-core
   * initialization and MPU cache policy are separate future gates. */
  .gp_shared_ipc (NOLOAD) :
  {
    . = ALIGN(32);
    __gp_shared_ipc_start__ = .;
    KEEP(*(.gp_shared_ipc))
    KEEP(*(.gp_shared_ipc.*))
    . = ALIGN(32);
    __gp_shared_ipc_end__ = .;
  } >GP_SRAM4_IPC

  .gp_shared_trajectory (NOLOAD) :
  {
    . = ALIGN(32);
    __gp_shared_trajectory_start__ = .;
    KEEP(*(.gp_shared_trajectory))
    KEEP(*(.gp_shared_trajectory.*))
    . = ALIGN(32);
    __gp_shared_trajectory_end__ = .;
  } >GP_SRAM4_TRAJECTORY

  /* Stage 3D reserved validation slot, inactive until protocol binding. */
  .gp_shared_validation (NOLOAD) :
  {
    . = ALIGN(32);
    __gp_shared_validation_start__ = .;
    KEEP(*(.gp_shared_validation))
    KEEP(*(.gp_shared_validation.*))
    . = ALIGN(32);
    __gp_shared_validation_end__ = .;
  } >GP_SRAM4_RESERVED

  ASSERT(ADDR(.gp_shared_validation) == ORIGIN(GP_SRAM4_RESERVED),
         "Validation address mismatch")
  ASSERT(SIZEOF(.gp_shared_validation) <= LENGTH(GP_SRAM4_RESERVED),
         "Validation exceeds 12 KiB")
  ASSERT(ADDR(.gp_shared_ipc) == ORIGIN(GP_SRAM4_IPC),
         "Shared IPC base does not match STM32H745 SRAM4 contract")
  ASSERT(ADDR(.gp_shared_trajectory) == ORIGIN(GP_SRAM4_TRAJECTORY),
         "Shared trajectory base does not match STM32H745 SRAM4 contract")
  ASSERT(SIZEOF(.gp_shared_ipc) <= LENGTH(GP_SRAM4_IPC),
         "Shared IPC exceeds 4 KiB")
  ASSERT(SIZEOF(.gp_shared_trajectory) <= LENGTH(GP_SRAM4_TRAJECTORY),
         "Shared trajectory exceeds 16 KiB")

  /* User_heap_stack section, used to check that there is enough "RAM" Ram  type memory left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
