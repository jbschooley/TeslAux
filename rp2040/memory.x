/* RP2040: 2 MB flash, 264 KB SRAM.
 *
 * The bootrom copies the first 256 bytes of flash into SRAM and runs them as
 * the second-stage bootloader (it configures XIP/QSPI). embassy-rp supplies
 * that blob in a `.boot2` section, but nothing places it — without the SECTIONS
 * block below it links wherever there is room (it landed at 0x100050a0) and the
 * board never boots. */
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

SECTIONS {
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
