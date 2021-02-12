MEMORY
{
    SRAM1 (xrw) : ORIGIN = 0x20000000, LENGTH = 96K
    SRAM2 (xrw) : ORIGIN = 0x10000000, LENGTH = 32K
    FLASH  (xr) : ORIGIN = 0x08000000, LENGTH = 1M
}

SECTIONS
{
    .text :
    {
        KEEP(*(.vector_table))
        *(.text*)
    } > FLASH

    .bss :
    {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end__ = .;
    } > SRAM1

    .data :
    {
        *(.data*)
    } > SRAM1 AT> FLASH

    . = ALIGN(8);
    _estack = .;
}