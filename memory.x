/* Heltec Mesh Node T114 (nRF52840) flash/RAM layout.
 *
 * The T114 ships with the Adafruit/Nordic bootloader + SoftDevice S140
 * v6.1.1 (the Meshtastic stock image).  Boot chain:
 *
 *   MBR (0x00000) -> SoftDevice reset (0x01000) -> app (0x26000)
 *
 * We place the application at 0x26000 exactly like the stock Meshtastic
 * app so the bootloader hands off to us cleanly.  We do NOT enable the
 * SoftDevice (this firmware needs no BLE), so it stays dormant — but its
 * flash region and RAM reservation must still be respected, hence the
 * app FLASH origin at 0x26000 and RAM origin at the SD-reserved address
 * (0x200032D8).  A dormant SD does not actually use that RAM, but keeping
 * the origin here matches the layout the stock bootloader was built for
 * and is known to boot on this board.
 *
 * Flash window: 0x26000 .. 0xE7000 (leaving the top of flash for the
 * bootloader + settings/DFU pages).  772 KB is far more than this tiny
 * USB firmware needs.
 *
 * UF2 packaging (see README): the app .bin is wrapped at base 0x26000
 * with the nRF52840 family id 0xADA52840.
 */
MEMORY
{
  FLASH : ORIGIN = 0x00026000, LENGTH = 0x000C1000
  RAM   : ORIGIN = 0x200032D8, LENGTH = 0x0003CD28
}
