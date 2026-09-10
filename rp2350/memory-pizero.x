/* RP2350B on the Waveshare RP2350-PiZero: 16 MB flash, 512 KB SRAM.
 *
 * Same layout as the RP2350-USB-C apart from the flash size — see memory.x for
 * why `.start_block` has to go after the vector table with `_stext` moved past
 * it. The larger part differs only in package and pin count; nothing about the
 * memory map or the boot process changes. */
MEMORY {
    FLASH : ORIGIN = 0x10000000, LENGTH = 16384K
    RAM   : ORIGIN = 0x20000000, LENGTH = 512K
}

SECTIONS {
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH
} INSERT AFTER .vector_table;

_stext = ADDR(.start_block) + SIZEOF(.start_block);
