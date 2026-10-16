/* oem/ld/app.ld: our app behind the stock bootloader (0x08000000-0x08004FFF). The bootloader stores the image length
   at 0x0801E000, so the app must end before it; oem_image.py appends an 8-byte trailer after the binary. */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08005000, LENGTH = 0x18FF8   /* 0x19000 minus the trailer */
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 16K       /* conservative: the stock app's stack top is 0x20001448 */
}

_stack_top = ORIGIN(RAM) + LENGTH(RAM);
_min_stack = 0x1000;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

  .text :
  {
    *(.text*)
    *(.rodata*)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    __preinit_array_start = .;
    KEEP(*(.preinit_array))
    __preinit_array_end = .;
    __init_array_start = .;
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    __init_array_end = .;
    . = ALIGN(4);
  } > FLASH

  .ARM.exidx : { *(.ARM.exidx*) } > FLASH

  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH

  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM

  ASSERT(_ebss + _min_stack <= _stack_top, "RAM: less than 4 KB left for the stack")
}
