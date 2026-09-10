/* RP2350A on the Waveshare RP2350-USB-C: 2 MB flash (W25Q16), 512 KB SRAM.
 *
 * Unlike the RP2040 there is no second-stage bootloader to place. The RP2350
 * bootrom instead looks for an IMAGE_DEF block in the first 4 KB of flash, and
 * embassy-rp emits a secure-exe one into `.start_block` automatically (see its
 * `select_imagedef!`, whose default is `secure_exe`). What memory.x has to do
 * is put that block immediately after the vector table and then push `.text`
 * out of its way — cortex-m-rt positions `.text` at `_stext`, so moving the
 * block without moving `_stext` links them on top of each other and the build
 * fails outright.
 *
 * RAM is the 512 KB main bank. The RP2350's extra 8 KB is a separate bank and
 * is left out rather than pretending the address space is contiguous. */
MEMORY {
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
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
