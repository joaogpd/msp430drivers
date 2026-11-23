/*
 * cc1101_settings.c - CC1101 register images (SmartRF Studio 7 values)
 *
 * Comment column: "preset" = value from the SmartRF preset, "reset" =
 * register reset value, "433" = recomputed for 433.92 MHz.
 */
#include "cc1101_settings.h"

const uint8_t cc1101_cfg_433_gfsk_1k2[CC1101_CONFIG_SIZE] = {
    /* 0x00 IOCFG2   */ 0x2E,   /* preset: GDO2 3-state */
    /* 0x01 IOCFG1   */ 0x2E,   /* reset:  GDO1 (= SO) 3-state when CSn high */
    /* 0x02 IOCFG0   */ 0x2E,   /* preset: GDO0 3-state (stops the reset
                                   CLK_XOSC/192 clock on the pin) */
    /* 0x03 FIFOTHR  */ 0x07,   /* reset:  FIFO threshold RX 32 / TX 33 bytes */
    /* 0x04 SYNC1    */ 0xD3,   /* reset:  sync word 0xD391 */
    /* 0x05 SYNC0    */ 0x91,   /* reset */
    /* 0x06 PKTLEN   */ 0xFF,   /* reset */
    /* 0x07 PKTCTRL1 */ 0x04,   /* reset:  append RSSI/LQI status, no addr check */
    /* 0x08 PKTCTRL0 */ 0x00,   /* preset: fixed length, CRC off, no whitening */
    /* 0x09 ADDR     */ 0x00,   /* reset */
    /* 0x0A CHANNR   */ 0x00,   /* preset: channel 0 */
    /* 0x0B FSCTRL1  */ 0x06,   /* preset: IF 152 kHz */
    /* 0x0C FSCTRL0  */ 0x00,   /* reset:  no frequency offset */
    /* 0x0D FREQ2    */ 0x10,   /* 433:    0x10B071 -> 433.9198 MHz */
    /* 0x0E FREQ1    */ 0xB0,   /* 433 */
    /* 0x0F FREQ0    */ 0x71,   /* 433 */
    /* 0x10 MDMCFG4  */ 0xF5,   /* preset: RX BW 58 kHz, DRATE_E = 5 */
    /* 0x11 MDMCFG3  */ 0x83,   /* preset: DRATE_M -> 1.1995 kBaud */
    /* 0x12 MDMCFG2  */ 0x13,   /* preset: GFSK, 30/32 sync bits */
    /* 0x13 MDMCFG1  */ 0x22,   /* reset:  4 preamble bytes, CHANSPC_E = 2 */
    /* 0x14 MDMCFG0  */ 0xF8,   /* reset:  channel spacing 199.95 kHz */
    /* 0x15 DEVIATN  */ 0x15,   /* preset: deviation 5.157 kHz */
    /* 0x16 MCSM2    */ 0x07,   /* reset */
    /* 0x17 MCSM1    */ 0x30,   /* reset:  CCA mode 3, RX/TX -> IDLE after packet */
    /* 0x18 MCSM0    */ 0x18,   /* preset: autocal IDLE -> RX/TX, PO_TIMEOUT 149 us */
    /* 0x19 FOCCFG   */ 0x16,   /* preset */
    /* 0x1A BSCFG    */ 0x6C,   /* reset */
    /* 0x1B AGCCTRL2 */ 0x03,   /* reset */
    /* 0x1C AGCCTRL1 */ 0x40,   /* reset */
    /* 0x1D AGCCTRL0 */ 0x91,   /* reset */
    /* 0x1E WOREVT1  */ 0x87,   /* reset */
    /* 0x1F WOREVT0  */ 0x6B,   /* reset */
    /* 0x20 WORCTRL  */ 0xFB,   /* preset */
    /* 0x21 FREND1   */ 0x56,   /* reset */
    /* 0x22 FREND0   */ 0x10,   /* preset: PA_POWER = 0 -> PATABLE[0] */
    /* 0x23 FSCAL3   */ 0xE9,   /* preset */
    /* 0x24 FSCAL2   */ 0x2A,   /* preset: VCO_CORE_H_EN */
    /* 0x25 FSCAL1   */ 0x00,   /* preset */
    /* 0x26 FSCAL0   */ 0x1F,   /* preset */
    /* 0x27 RCCTRL1  */ 0x41,   /* reset */
    /* 0x28 RCCTRL0  */ 0x00,   /* reset */
    /* 0x29 FSTEST   */ 0x59,   /* reset:  test register, not written */
    /* 0x2A PTEST    */ 0x7F,   /* reset:  test register, not written */
    /* 0x2B AGCTEST  */ 0x3F,   /* reset:  test register, not written */
    /* 0x2C TEST2    */ 0x81,   /* preset: RX BW < 325 kHz sensitivity setting */
    /* 0x2D TEST1    */ 0x35,   /* preset */
    /* 0x2E TEST0    */ 0x09,   /* preset: VCO_SEL_CAL_EN off */
};

const uint8_t cc1101_patable_433_0dbm[CC1101_PATABLE_SIZE] = {
    0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void cc1101_cfg_packet_mode(uint8_t *cfg)
{
    if (!cfg) { return; }
    cfg[CC1101_PKTCTRL0] = 0x05u;
    cfg[CC1101_PKTLEN]   = CC1101_MAX_PAYLOAD;
    cfg[CC1101_IOCFG0]   = CC1101_GDO_SYNC_EOP;
    cfg[CC1101_IOCFG2]   = CC1101_GDO_CRC_OK;
}
