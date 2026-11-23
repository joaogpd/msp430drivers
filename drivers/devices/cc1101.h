/*
 * cc1101.h - TI CC1101 sub-GHz transceiver, pin level and SPI access layer
 *
 * STATUS: pins and SPI access layer (strobes, single and burst register
 * access, status registers, chip status byte, manual power-on reset).
 * Radio configuration, packet TX/RX and GDO interrupts are still to be
 * written.
 *
 * Module: generic "CC1101 433 MHz V2.0" board (8-pin header: GND, VCC,
 * GDO0, CSN, SCK, MOSI, MISO/GDO1, GDO2), 26 MHz crystal, matching
 * network for 387-464 MHz. VCC is 1.8-3.6 V: 3V3 only, never 5 V.
 *
 * Wiring, taken from the earlier CC1101-FR5994 project (module pin ->
 * LaunchPad MCU pin), so the existing hardware can be reused:
 *
 *   VCC  -> 3V3
 *   GND  -> GND
 *   SI   -> P6.0  UCA3SIMO
 *   SO   -> P6.1  UCA3SOMI   also GDO1; goes low when the crystal is
 *                            running after CSn falls (CHIP_RDYn). PxIN
 *                            reads the pin even in SPI function mode.
 *   SCLK -> P6.2  UCA3CLK    SPI mode 0, 4 MHz (CC1101 limit: 10 MHz
 *                            single access, 6.5 MHz burst)
 *   CSn  -> P3.0  GPIO out   active low
 *   GDO2 -> P1.2  GPIO in    port 1, so it can interrupt and wake LPM3
 *   GDO0 -> P1.3  GPIO in    proposed: not wired in the old projects;
 *                            check before relying on it
 *
 * SPI frame (SWRS061 section 10): every transaction starts with a header
 * byte  [R/W | BURST | A5..A0].  While the header is shifted in, the chip
 * shifts out the chip status byte:
 *
 *   bit 7     CHIP_RDYn   0 = crystal running, chip ready
 *   bits 6:4  STATE       main radio state (CC1101_STATE_*)
 *   bits 3:0  FIFO_BYTES  free TX FIFO bytes if R/W = 0,
 *                         RX FIFO bytes      if R/W = 1  (15 means >= 15)
 *
 * Address map with the header bits:
 *   0x00-0x2E  configuration registers, single or burst
 *   0x30-0x3D  with R/W=0: command strobes (one byte, no data)
 *              with R/W=1 and BURST=0: also strobes
 *              with R/W=1 and BURST=1: read-only status registers
 *   0x3E       PATABLE (burst for all 8 entries)
 *   0x3F       FIFO: write = TX FIFO, read = RX FIFO
 *
 * Reference: CC1101 datasheet (SWRS061I), sections 10 (SPI), 19.1 (reset),
 * 29 (registers); errata SWRZ020 (status register read glitch).
 */
#ifndef DRIVERS_CC1101_H_
#define DRIVERS_CC1101_H_

#include <stdint.h>
#include <stdbool.h>
#include "drivers/msp430/gpio.h"
#include "drivers/msp430/spi.h"

#define CC1101_SPI          SPI_A3
#define CC1101_SPI_HZ       (4000000ul)

#define CC1101_SPI_PORT     GPIO_PORT_6
#define CC1101_SIMO_PIN     GPIO_PIN0
#define CC1101_SOMI_PIN     GPIO_PIN1
#define CC1101_CLK_PIN      GPIO_PIN2
#define CC1101_CS_PORT      GPIO_PORT_3
#define CC1101_CS_PIN       GPIO_PIN0
#define CC1101_GDO2_PORT    GPIO_PORT_1
#define CC1101_GDO2_PIN     GPIO_PIN2
#define CC1101_GDO0_PORT    GPIO_PORT_1
#define CC1101_GDO0_PIN     GPIO_PIN3

/*
 * How long to wait for SO to go low after CSn falls, or after SRES. The
 * crystal normally starts in ~150 us with a 26 MHz crystal; a
 * missing module or broken wire shows up as a timeout instead of a hang.
 * This is a lower bound: each polling step takes a little over 1 us.
 */
#define CC1101_SO_TIMEOUT_US    (2000u)

/* ---- Header bits ---------------------------------------------------- */
#define CC1101_WRITE_SINGLE     (0x00u)
#define CC1101_WRITE_BURST      (0x40u)
#define CC1101_READ_SINGLE      (0x80u)
#define CC1101_READ_BURST       (0xC0u)

/* ---- Configuration registers ---------------------- */
#define CC1101_IOCFG2       (0x00u)     /* GDO2 output pin configuration */
#define CC1101_IOCFG1       (0x01u)     /* GDO1 output pin configuration */
#define CC1101_IOCFG0       (0x02u)     /* GDO0 output pin configuration */
#define CC1101_FIFOTHR      (0x03u)     /* RX FIFO and TX FIFO thresholds */
#define CC1101_SYNC1        (0x04u)     /* sync word, high byte */
#define CC1101_SYNC0        (0x05u)     /* sync word, low byte */
#define CC1101_PKTLEN       (0x06u)     /* packet length */
#define CC1101_PKTCTRL1     (0x07u)     /* packet automation control */
#define CC1101_PKTCTRL0     (0x08u)     /* packet automation control */
#define CC1101_ADDR         (0x09u)     /* device address */
#define CC1101_CHANNR       (0x0Au)     /* channel number */
#define CC1101_FSCTRL1      (0x0Bu)     /* frequency synthesizer control */
#define CC1101_FSCTRL0      (0x0Cu)
#define CC1101_FREQ2        (0x0Du)     /* frequency control word, high */
#define CC1101_FREQ1        (0x0Eu)     /* frequency control word, middle */
#define CC1101_FREQ0        (0x0Fu)     /* frequency control word, low */
#define CC1101_MDMCFG4      (0x10u)     /* modem configuration */
#define CC1101_MDMCFG3      (0x11u)
#define CC1101_MDMCFG2      (0x12u)
#define CC1101_MDMCFG1      (0x13u)
#define CC1101_MDMCFG0      (0x14u)
#define CC1101_DEVIATN      (0x15u)     /* modem deviation setting */
#define CC1101_MCSM2        (0x16u)     /* main radio control state machine */
#define CC1101_MCSM1        (0x17u)
#define CC1101_MCSM0        (0x18u)
#define CC1101_FOCCFG       (0x19u)     /* frequency offset compensation */
#define CC1101_BSCFG        (0x1Au)     /* bit synchronization */
#define CC1101_AGCCTRL2     (0x1Bu)     /* AGC control */
#define CC1101_AGCCTRL1     (0x1Cu)
#define CC1101_AGCCTRL0     (0x1Du)
#define CC1101_WOREVT1      (0x1Eu)     /* event 0 timeout, high */
#define CC1101_WOREVT0      (0x1Fu)     /* event 0 timeout, low */
#define CC1101_WORCTRL      (0x20u)     /* wake on radio control */
#define CC1101_FREND1       (0x21u)     /* front end RX configuration */
#define CC1101_FREND0       (0x22u)     /* front end TX configuration */
#define CC1101_FSCAL3       (0x23u)     /* frequency synthesizer calibration */
#define CC1101_FSCAL2       (0x24u)
#define CC1101_FSCAL1       (0x25u)
#define CC1101_FSCAL0       (0x26u)
#define CC1101_RCCTRL1      (0x27u)     /* RC oscillator configuration */
#define CC1101_RCCTRL0      (0x28u)
#define CC1101_FSTEST       (0x29u)     /* test registers: SmartRF values only */
#define CC1101_PTEST        (0x2Au)
#define CC1101_AGCTEST      (0x2Bu)
#define CC1101_TEST2        (0x2Cu)
#define CC1101_TEST1        (0x2Du)
#define CC1101_TEST0        (0x2Eu)
#define CC1101_CONFIG_LAST  CC1101_TEST0

/* ---- Command strobes ----------------------------- */
#define CC1101_SRES         (0x30u)     /* reset chip */
#define CC1101_SFSTXON      (0x31u)     /* enable and calibrate synthesizer */
#define CC1101_SXOFF        (0x32u)     /* crystal off (when CSn goes high) */
#define CC1101_SCAL         (0x33u)     /* calibrate synthesizer and turn off */
#define CC1101_SRX          (0x34u)     /* enable RX */
#define CC1101_STX          (0x35u)     /* enable TX */
#define CC1101_SIDLE        (0x36u)     /* exit RX/TX, turn off synthesizer */
#define CC1101_SWOR         (0x38u)     /* start wake-on-radio */
#define CC1101_SPWD         (0x39u)     /* power down (when CSn goes high) */
#define CC1101_SFRX         (0x3Au)     /* flush RX FIFO (IDLE or overflow only) */
#define CC1101_SFTX         (0x3Bu)     /* flush TX FIFO (IDLE or underflow only) */
#define CC1101_SWORRST      (0x3Cu)     /* reset real-time clock to Event1 */
#define CC1101_SNOP         (0x3Du)     /* no operation: returns status byte */

/* ---- Status registers, read-only, need READ_BURST -------- */
#define CC1101_PARTNUM      (0x30u)     /* part number, expect 0x00 */
#define CC1101_VERSION      (0x31u)     /* chip version, typically 0x14 */
#define CC1101_FREQEST      (0x32u)     /* frequency offset estimate */
#define CC1101_LQI          (0x33u)     /* link quality estimate */
#define CC1101_RSSI         (0x34u)     /* received signal strength */
#define CC1101_MARCSTATE    (0x35u)     /* main radio state machine state */
#define CC1101_WORTIME1     (0x36u)     /* WOR timer, high */
#define CC1101_WORTIME0     (0x37u)     /* WOR timer, low */
#define CC1101_PKTSTATUS    (0x38u)     /* current GDOx status and packet status */
#define CC1101_VCO_VC_DAC   (0x39u)     /* PLL calibration current setting */
#define CC1101_TXBYTES      (0x3Au)     /* underflow flag and bytes in TX FIFO */
#define CC1101_RXBYTES      (0x3Bu)     /* overflow flag and bytes in RX FIFO */
#define CC1101_RCCTRL1_STATUS (0x3Cu)   /* last RC oscillator calibration */
#define CC1101_RCCTRL0_STATUS (0x3Du)

/* ---- Multi-byte registers ------------------------------------------- */
#define CC1101_PATABLE      (0x3Eu)     /* 8-byte PA power table */
#define CC1101_FIFO         (0x3Fu)     /* TX FIFO (write) / RX FIFO (read) */
#define CC1101_FIFO_SIZE    (64u)

#define CC1101_PARTNUM_EXPECTED (0x00u)

/* TXBYTES/RXBYTES: bit 7 = underflow/overflow, bits 6:0 = byte count */
#define CC1101_FIFO_OVF     (0x80u)
#define CC1101_FIFO_COUNT   (0x7Fu)

/* ---- Chip status byte ----------------------------------------------- */
#define CC1101_STATUS_CHIP_RDYn     (0x80u)
#define CC1101_STATUS_STATE_MASK    (0x70u)
#define CC1101_STATUS_FIFO_MASK     (0x0Fu)

/*
 * Returned in place of a status byte when SO did not go low within
 * CC1101_SO_TIMEOUT_US (no chip, bad wiring, no supply). A real status
 * byte never has CHIP_RDYn set, because the driver only clocks after SO
 * (= CHIP_RDYn) went low, so the two can never be confused.
 */
#define CC1101_STATUS_TIMEOUT       (0xFFu)

/* STATE field of the status byte, already shifted down */
typedef enum
{
    CC1101_STATE_IDLE = 0,
    CC1101_STATE_RX,
    CC1101_STATE_TX,
    CC1101_STATE_FSTXON,
    CC1101_STATE_CALIBRATE,
    CC1101_STATE_SETTLING,
    CC1101_STATE_RXFIFO_OVERFLOW,
    CC1101_STATE_TXFIFO_UNDERFLOW
} cc1101_state_t;

static inline bool cc1101_status_ok(uint8_t status)
{
    return (status & CC1101_STATUS_CHIP_RDYn) == 0u;
}

static inline cc1101_state_t cc1101_status_state(uint8_t status)
{
    return (cc1101_state_t)((status & CC1101_STATUS_STATE_MASK) >> 4);
}

static inline uint8_t cc1101_status_fifo(uint8_t status)
{
    return status & CC1101_STATUS_FIFO_MASK;
}

/* ---- Pins ----------------------------------------------------------- */

/*
 * Put the radio pins in a safe state: CSn output high (deselected),
 * SO/GDO0/GDO2 inputs without pulls (the CC1101 drives them), SI/SCLK
 * left as board_init made them (outputs low, which the CC1101 reads as
 * inputs). GPIO only, so it can run before cs_init(). Pass it to
 * board_init() as before_unlock: gpio_init_all_low_power() makes every
 * pin an output driving low, and without this the MSP430 would fight
 * the CC1101 on GDO0 (a CLK_XOSC/192 clock after power-on) and on SO
 * (driven while CSn is low) from the I/O unlock until cc1101_pins_init().
 */
void cc1101_pins_safe(void);

/*
 * Bring up SPI (needs cs_init() first) and the pins: CSn high
 * (deselected), SIMO/SOMI/CLK to eUSCI_A3, GDO0/GDO2 inputs without pulls
 * (the CC1101 drives them). Returns false if SPI could not be configured.
 */
bool cc1101_pins_init(void);

static inline bool cc1101_so_high(void) { return gpio_read(CC1101_SPI_PORT, CC1101_SOMI_PIN) != 0u; }
static inline bool cc1101_gdo0(void)    { return gpio_read(CC1101_GDO0_PORT, CC1101_GDO0_PIN) != 0u; }
static inline bool cc1101_gdo2(void)    { return gpio_read(CC1101_GDO2_PORT, CC1101_GDO2_PIN) != 0u; }

/* ---- SPI access layer ----------------------------------------------- */
/*
 * Every function below selects the chip, waits for CHIP_RDYn (SO low),
 * does one SPI transaction and deselects. They return the chip status
 * byte clocked out with the header, or CC1101_STATUS_TIMEOUT; on timeout
 * nothing is clocked and output buffers are left untouched.
 *
 * Main context only: the transfers are polled and not reentrant. ISRs in
 * this project only set flags, so this costs nothing.
 */

/*
 * Manual power-on reset, SWRS061 19.1.2: SCLK=1/SI=0, strobe CSn low and
 * high, keep CSn high >= 40 us, pull CSn low, wait for SO low, send SRES,
 * wait for SO low again (reset done, chip in IDLE with default registers).
 * Call once after cc1101_pins_init() and after every supply ramp.
 */
uint8_t cc1101_reset(void);

/* cc1101_pins_init() + cc1101_reset(). false if either failed. */
bool cc1101_init(void);

/*
 * One-byte command strobe (CC1101_SRES..CC1101_SNOP). The status byte
 * reports free TX FIFO bytes; use cc1101_strobe_rx_status() to get the
 * RX FIFO count instead (same strobe with the R/W bit set).
 * Note: SPWD and SXOFF take effect when CSn goes high, at the end of
 * this call; the next access then waits for the crystal to restart.
 */
uint8_t cc1101_strobe(uint8_t strobe);
uint8_t cc1101_strobe_rx_status(uint8_t strobe);

/* Configuration registers 0x00-0x2E. */
uint8_t cc1101_write_reg(uint8_t addr, uint8_t value);
uint8_t cc1101_read_reg(uint8_t addr, uint8_t *value);

/*
 * Burst access starting at addr, address auto-incrementing (except the
 * FIFO, which stays put, and the PATABLE, which steps its own index).
 * Use for a block of config registers, the PATABLE (8 bytes) and the
 * FIFOs (up to 64 bytes).
 * len 0 just returns the status byte.
 */
uint8_t cc1101_write_burst(uint8_t addr, const uint8_t *data, uint8_t len);
uint8_t cc1101_read_burst(uint8_t addr, uint8_t *data, uint8_t len);

/*
 * Read-only status register 0x30-0x3D (PARTNUM, VERSION, RSSI,
 * MARCSTATE, RXBYTES...). Sends the burst bit, without which these
 * addresses would be strobes.
 */
uint8_t cc1101_read_status(uint8_t addr, uint8_t *value);

/*
 * Errata SWRZ020: a status register that changes while it is being read
 * (RXBYTES, TXBYTES, MARCSTATE, RSSI, FREQEST, LQI, WORTIME) can come back
 * corrupted. This reads it until two consecutive values agree (at most
 * a few tries; the timeout status is returned if the chip stops replying).
 */
uint8_t cc1101_read_status_stable(uint8_t addr, uint8_t *value);

/* SNOP: just the status byte (TX FIFO free count / RX FIFO count). */
static inline uint8_t cc1101_get_status(void)    { return cc1101_strobe(CC1101_SNOP); }
static inline uint8_t cc1101_get_rx_status(void) { return cc1101_strobe_rx_status(CC1101_SNOP); }

/* ---- Register configuration ----------------------------------------- */

/* MARCSTATE values (bits 4:0 of the register), SWRS061 section 29 */
#define CC1101_MARC_SLEEP       (0x00u)
#define CC1101_MARC_IDLE        (0x01u)
#define CC1101_MARC_XOFF        (0x02u)
#define CC1101_MARC_RX          (0x0Du)
#define CC1101_MARC_TX          (0x13u)
#define CC1101_MARC_RXFIFO_OVF  (0x11u)
#define CC1101_MARC_TXFIFO_UNF  (0x16u)
#define CC1101_MARC_MASK        (0x1Fu)

/*
 * A full configuration image: one byte per register, indexed by address
 * 0x00 (IOCFG2) .. 0x2E (TEST0). The entries for FSTEST, PTEST and
 * AGCTEST (0x29-0x2B) are not written: the datasheet says these are for
 * test only and must not be written. Keep them at their reset values in
 * the table so the image still documents the whole register file.
 */
#define CC1101_CONFIG_SIZE  (CC1101_CONFIG_LAST + 1u)   /* 47 */
#define CC1101_PATABLE_SIZE (8u)

/*
 * Write a configuration image in two bursts (0x00-0x28 and 0x2C-0x2E).
 * Call in IDLE (right after cc1101_reset() is fine). Returns the status
 * byte of the last burst, or CC1101_STATUS_TIMEOUT.
 */
uint8_t cc1101_write_config(const uint8_t *cfg);

/* Read all 47 configuration registers in one burst into out. */
uint8_t cc1101_read_config(uint8_t *out);

/*
 * Bits of register addr that keep the value written to them, for
 * comparing a readback with the image. 0x00 for the test registers that
 * are never written; FSCAL3[3:0], FSCAL2[4:0] and FSCAL1 are replaced by
 * the chip's own calibration results after every calibration.
 */
uint8_t cc1101_config_mask(uint8_t addr);

/*
 * PA power table (1..8 entries, burst). With FREND0.PA_POWER = 0 only
 * entry 0 is used, for every TX. The PATABLE is lost in SLEEP (SPWD):
 * rewrite it after waking.
 */
static inline uint8_t cc1101_write_patable(const uint8_t *pa, uint8_t len)
{
    return cc1101_write_burst(CC1101_PATABLE, pa, len);
}
static inline uint8_t cc1101_read_patable(uint8_t *pa, uint8_t len)
{
    return cc1101_read_burst(CC1101_PATABLE, pa, len);
}

/*
 * Poll MARCSTATE until it equals want (CC1101_MARC_*) or timeout_us has
 * passed (a lower bound, like CC1101_SO_TIMEOUT_US). *last, if not NULL,
 * gets the last state read. false on timeout or if the chip stopped
 * answering (then *last = 0xFF).
 */
bool cc1101_wait_marcstate(uint8_t want, uint16_t timeout_us, uint8_t *last);

/*
 * SCAL from IDLE: calibrate the frequency synthesizer, then back to IDLE.
 * Takes about 0.7 ms at 26 MHz. A good check that the configured
 * frequency is one the VCO can reach. false if IDLE did not come back
 * within timeout.
 */
bool cc1101_calibrate(void);

/* ---- Packets (variable length, appended status) ---------------------- */
/*
 * These assume the packet engine is set up as cc1101_cfg_packet_mode()
 * in cc1101_settings.h does: variable length (first FIFO byte = length),
 * CRC on, PKTCTRL1.APPEND_STATUS = 1 (RSSI and LQI/CRC_OK appended in
 * RX), RXOFF/TXOFF = IDLE. A whole packet fits the 64-byte FIFO:
 * length byte + payload + 2 status bytes, so payload <= 61.
 */
#define CC1101_MAX_PAYLOAD      (61u)

/* GDOx_CFG values used by this project (SWRS061 section 26) */
#define CC1101_GDO_SYNC_EOP     (0x06u)  /* high from sync word to end of packet */
#define CC1101_GDO_CRC_OK       (0x07u)  /* high from packet with CRC OK until
                                            the first RX FIFO byte is read */
#define CC1101_GDO_HIZ          (0x2Eu)

/*
 * Send one packet: SIDLE, SFTX, length byte + payload into the TX FIFO,
 * STX. Returns as soon as TX has started; the end of the packet is the
 * falling edge of GDO0 (CC1101_GDO_SYNC_EOP), after which the chip is in
 * IDLE again. Returns the status byte of STX, or CC1101_STATUS_TIMEOUT
 * (also for len 0 or > CC1101_MAX_PAYLOAD, with nothing sent).
 */
uint8_t cc1101_tx_packet(const uint8_t *payload, uint8_t len);

/* SIDLE, SFRX, SRX: start listening with an empty RX FIFO. */
uint8_t cc1101_rx_start(void);

typedef enum
{
    CC1101_RX_OK = 0,       /* packet read, CRC good */
    CC1101_RX_CRC_ERROR,    /* packet read, CRC bad (payload unreliable) */
    CC1101_RX_EMPTY,        /* nothing in the RX FIFO */
    CC1101_RX_OVERFLOW,     /* RX FIFO overflowed; flushed */
    CC1101_RX_BAD_LENGTH,   /* length byte 0, > max or > bytes present; flushed */
    CC1101_RX_TIMEOUT       /* chip did not answer */
} cc1101_rx_result_t;

typedef struct
{
    uint8_t len;            /* payload bytes copied */
    uint8_t rssi_raw;       /* appended RSSI byte (cc1101_rssi_dbm()) */
    uint8_t lqi;            /* link quality, 7 bits, lower = better */
    bool    crc_ok;
} cc1101_rx_info_t;

/*
 * Read one received packet after GDO0 fell (the chip is then IDLE, as
 * RXOFF = IDLE). payload must hold max bytes. On OVERFLOW / BAD_LENGTH
 * the RX FIFO is flushed. The caller restarts RX with cc1101_rx_start().
 */
cc1101_rx_result_t cc1101_rx_read_packet(uint8_t *payload, uint8_t max,
                                         cc1101_rx_info_t *info);

/*
 * RSSI register / appended byte to dBm (SWRS061 17.3): two's complement
 * in 0.5 dB steps minus the RSSI offset, 74 dB at 433 MHz / 1.2 kBaud
 * (SWRS061 section 17.3, RSSI offset table). Accuracy is a few dB.
 */
static inline int16_t cc1101_rssi_dbm(uint8_t raw)
{
    int16_t r = (raw >= 128u) ? (int16_t)raw - 256 : (int16_t)raw;
    return (int16_t)(r / 2 - 74);
}

#endif /* DRIVERS_CC1101_H_ */
