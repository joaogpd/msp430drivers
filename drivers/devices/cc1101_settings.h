/*
 * cc1101_settings.h - CC1101 register images (SmartRF Studio 7 values)
 *
 * Each image holds all 47 configuration registers (0x00-0x2E), for
 * cc1101_write_config(). Registers a SmartRF preset does not list keep
 * their reset values, taken from SmartRF Studio's register_definition.xml
 * for the CC1101 (same as SWRS061 section 29).
 */
#ifndef DRIVERS_CC1101_SETTINGS_H_
#define DRIVERS_CC1101_SETTINGS_H_

#include <stdint.h>
#include "cc1101.h"

/*
 * SmartRF Studio 7, CC1101, "Typical settings" preset 1:
 *   Data rate 1.2 kBaud, deviation 5.2 kHz, GFSK, RX BW 58 kHz,
 *   optimized for sensitivity.
 * The preset ships at 868 MHz; FREQ2..0 are recomputed for 433.92 MHz
 * with the 26 MHz crystal, as SmartRF does when the frequency is changed:
 *   FREQ = 433.92e6 * 2^16 / 26e6 = 1093745 = 0x10B071 -> 433.9198 MHz
 *
 * Packet settings are the preset's (PKTCTRL0 = 0x00: fixed length
 * PKTLEN = 255, no CRC; GDO0/GDO2 3-state); cc1101_cfg_packet_mode()
 * below changes them for packet TX/RX.
 */
extern const uint8_t cc1101_cfg_433_gfsk_1k2[CC1101_CONFIG_SIZE];

/*
 * PATABLE for 433 MHz, no ramping, SmartRF Studio pa_settings.xml
 * (CC1101_434, default entry): 0 dBm = 0x60. Entry 0 is the only one
 * used with FREND0.PA_POWER = 0. Other levels from the same table:
 * 10 dBm 0xC0, 7 dBm 0xC8, 5 dBm 0x84, -10 dBm 0x34, -15 dBm 0x1D,
 * -20 dBm 0x0E, -30 dBm 0x12.
 */
extern const uint8_t cc1101_patable_433_0dbm[CC1101_PATABLE_SIZE];

/*
 * Turn a copy of a SmartRF image (in RAM) into this project's packet
 * mode, the one cc1101_tx_packet() / cc1101_rx_read_packet() expect:
 *   PKTCTRL0 = 0x05   variable length (first byte), CRC on, no whitening
 *   PKTLEN   = 61     longest accepted payload (CC1101_MAX_PAYLOAD)
 *   IOCFG0   = 0x06   GDO0 high from sync word to end of packet, in TX
 *                     and RX: its falling edge wakes the MCU
 *   IOCFG2   = 0x07   GDO2 high while a CRC-OK packet waits in the RX
 *                     FIFO (cross-check of the CRC_OK status bit)
 * Everything else (modem, frequency, PKTCTRL1 append-status, MCSM1
 * RX/TX -> IDLE) stays as in the image.
 */
void cc1101_cfg_packet_mode(uint8_t *cfg);

#endif /* DRIVERS_CC1101_SETTINGS_H_ */
