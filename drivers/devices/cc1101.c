/*
 * cc1101.c - TI CC1101 sub-GHz transceiver, pin level and SPI access layer
 */
#include "cc1101.h"
#include "drivers/msp430/cs.h"
#include <stddef.h>

#define SPI_PINS    (CC1101_SIMO_PIN | CC1101_SOMI_PIN | CC1101_CLK_PIN)

/* Tries for cc1101_read_status_stable(); two agreeing reads normally
 * come in the first two or three. */
#define STABLE_TRIES    (8u)

void cc1101_pins_safe(void)
{
    gpio_set_output(CC1101_CS_PORT, CC1101_CS_PIN, true);
    gpio_set_input(CC1101_SPI_PORT, CC1101_SOMI_PIN, GPIO_PULL_NONE);
    gpio_set_input(CC1101_GDO0_PORT, CC1101_GDO0_PIN, GPIO_PULL_NONE);
    gpio_set_input(CC1101_GDO2_PORT, CC1101_GDO2_PIN, GPIO_PULL_NONE);
}

bool cc1101_pins_init(void)
{
    cc1101_pins_safe();                 /* also when board_init did not */

    const spi_config_t cfg = { CC1101_SPI_HZ, SPI_MODE_0, false };
    if (!spi_init(CC1101_SPI, &cfg)) { return false; }

    gpio_set_function(CC1101_SPI_PORT, SPI_PINS, GPIO_FUNC_PRIMARY);
    return true;
}

/* ---- Chip select ---------------------------------------------------- */

static inline void cs_high(void) { gpio_set(CC1101_CS_PORT, CC1101_CS_PIN); }
static inline void cs_low(void)  { gpio_clear(CC1101_CS_PORT, CC1101_CS_PIN); }

/* Wait for SO (CHIP_RDYn) to go low. true = ready, false = timed out. */
static bool wait_so_low(void)
{
    uint16_t us = CC1101_SO_TIMEOUT_US;
    while (cc1101_so_high())
    {
        if (us-- == 0u) { return false; }
        cs_delay_us(1u);
    }
    return true;
}

/*
 * Pull CSn low and wait for the chip. On timeout CSn goes back high, so
 * the caller only has to return CC1101_STATUS_TIMEOUT. Per SWRS061 10.1
 * the first SCLK edge must not come before SO is low: from SLEEP/XOFF
 * the crystal has to start first, in IDLE/RX/TX SO is low at once.
 */
static bool chip_select(void)
{
    cs_low();
    if (!wait_so_low())
    {
        cs_high();
        return false;
    }
    return true;
}

/* ---- Reset ---------------------------------------------------------- */

uint8_t cc1101_reset(void)
{
    /*
     * SWRS061 19.1.2. SCLK=1 and SI=0 during the CSn strobe avoid the
     * pin-control mode (SWRS061 11.3); in SPI mode 0 the eUSCI would hold
     * SCLK low, so take SI and SCLK as plain GPIO for this part.
     */
    gpio_set_output(CC1101_SPI_PORT, CC1101_SIMO_PIN, false);
    gpio_set_output(CC1101_SPI_PORT, CC1101_CLK_PIN, true);
    gpio_set_function(CC1101_SPI_PORT, CC1101_SIMO_PIN | CC1101_CLK_PIN,
                      GPIO_FUNC_IO);

    cs_high();
    cs_delay_us(5u);
    cs_low();                           /* strobe CSn low ... */
    cs_delay_us(5u);
    cs_high();                          /* ... and high */
    cs_delay_us(45u);                   /* >= 40 us high (datasheet) */

    /* Back to the eUSCI with CSn still high; SCLK drops to idle low
     * before the chip is selected again. */
    gpio_set_function(CC1101_SPI_PORT, SPI_PINS, GPIO_FUNC_PRIMARY);

    if (!chip_select()) { return CC1101_STATUS_TIMEOUT; }

    uint8_t status = spi_transfer_byte(CC1101_SPI, CC1101_SRES);

    /* SO goes high while the reset runs and low again when it is done
     * and the chip sits in IDLE. CSn has to stay low until then. If SO
     * has not risen yet when this first looks, the wait ends early; that
     * is harmless, since every later access waits for CHIP_RDYn again
     * before the first SCLK edge. */
    bool done = wait_so_low();
    cs_high();
    return done ? status : CC1101_STATUS_TIMEOUT;
}

bool cc1101_init(void)
{
    if (!cc1101_pins_init()) { return false; }
    return cc1101_reset() != CC1101_STATUS_TIMEOUT;
}

/* ---- Register access ------------------------------------------------ */

/* One header byte, then len data bytes written from tx or read into rx
 * (one of them NULL; both NULL means header only). */
static uint8_t transaction(uint8_t header, const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    if (!chip_select()) { return CC1101_STATUS_TIMEOUT; }

    uint8_t status = spi_transfer_byte(CC1101_SPI, header);
    if (len != 0u)
    {
        if (tx)      { spi_write(CC1101_SPI, tx, len); }
        else if (rx) { spi_read(CC1101_SPI, rx, len, 0x00u); }
    }

    cs_high();
    return status;
}

uint8_t cc1101_strobe(uint8_t strobe)
{
    return transaction((uint8_t)(CC1101_WRITE_SINGLE | (strobe & 0x3Fu)), NULL, NULL, 0u);
}

uint8_t cc1101_strobe_rx_status(uint8_t strobe)
{
    return transaction((uint8_t)(CC1101_READ_SINGLE | (strobe & 0x3Fu)), NULL, NULL, 0u);
}

uint8_t cc1101_write_reg(uint8_t addr, uint8_t value)
{
    return transaction((uint8_t)(CC1101_WRITE_SINGLE | (addr & 0x3Fu)), &value, NULL, 1u);
}

uint8_t cc1101_read_reg(uint8_t addr, uint8_t *value)
{
    if (!value) { return CC1101_STATUS_TIMEOUT; }
    return transaction((uint8_t)(CC1101_READ_SINGLE | (addr & 0x3Fu)), NULL, value, 1u);
}

/*
 * Burst: the address auto-increments through the config registers. The
 * FIFO address does not move, and the PATABLE has its own internal index
 * that steps per byte and resets to 0 when CSn goes high.
 */
uint8_t cc1101_write_burst(uint8_t addr, const uint8_t *data, uint8_t len)
{
    if (!data && len != 0u) { return CC1101_STATUS_TIMEOUT; }
    return transaction((uint8_t)(CC1101_WRITE_BURST | (addr & 0x3Fu)), data, NULL, len);
}

uint8_t cc1101_read_burst(uint8_t addr, uint8_t *data, uint8_t len)
{
    if (!data && len != 0u) { return CC1101_STATUS_TIMEOUT; }
    return transaction((uint8_t)(CC1101_READ_BURST | (addr & 0x3Fu)), NULL, data, len);
}

uint8_t cc1101_read_status(uint8_t addr, uint8_t *value)
{
    if (!value) { return CC1101_STATUS_TIMEOUT; }
    /* Single byte, but with the burst bit: that is what turns 0x30-0x3D
     * from strobes into status registers. */
    return transaction((uint8_t)(CC1101_READ_BURST | (addr & 0x3Fu)), NULL, value, 1u);
}

uint8_t cc1101_read_status_stable(uint8_t addr, uint8_t *value)
{
    if (!value) { return CC1101_STATUS_TIMEOUT; }

    uint8_t prev;
    uint8_t i;
    uint8_t status = cc1101_read_status(addr, &prev);
    if (status == CC1101_STATUS_TIMEOUT) { return status; }

    for (i = STABLE_TRIES; i != 0u; i--)
    {
        uint8_t cur;
        status = cc1101_read_status(addr, &cur);
        if (status == CC1101_STATUS_TIMEOUT) { return status; }
        if (cur == prev)
        {
            *value = cur;
            return status;
        }
        prev = cur;
    }

    *value = prev;          /* still changing: best effort, latest value */
    return status;
}

/* ---- Register configuration ----------------------------------------- */

/* First and last address of the two bursts; 0x29-0x2B are skipped */
#define CFG_A_FIRST     CC1101_IOCFG2
#define CFG_A_LAST      CC1101_RCCTRL0
#define CFG_B_FIRST     CC1101_TEST2
#define CFG_B_LAST      CC1101_TEST0

uint8_t cc1101_write_config(const uint8_t *cfg)
{
    if (!cfg) { return CC1101_STATUS_TIMEOUT; }

    uint8_t status = cc1101_write_burst(CFG_A_FIRST, &cfg[CFG_A_FIRST],
                                        (uint8_t)(CFG_A_LAST - CFG_A_FIRST + 1u));
    if (status == CC1101_STATUS_TIMEOUT) { return status; }

    return cc1101_write_burst(CFG_B_FIRST, &cfg[CFG_B_FIRST],
                              (uint8_t)(CFG_B_LAST - CFG_B_FIRST + 1u));
}

uint8_t cc1101_read_config(uint8_t *out)
{
    return cc1101_read_burst(CC1101_IOCFG2, out, (uint8_t)CC1101_CONFIG_SIZE);
}

uint8_t cc1101_config_mask(uint8_t addr)
{
    switch (addr)
    {
    case CC1101_FSTEST:
    case CC1101_PTEST:
    case CC1101_AGCTEST: return 0x00u;     /* never written */
    case CC1101_FSCAL3:  return 0xF0u;     /* [3:0] calibration result */
    case CC1101_FSCAL2:  return 0xE0u;     /* [4:0] VCO current result */
    case CC1101_FSCAL1:  return 0x00u;     /* capacitor array result */
    default:             return (addr <= CC1101_CONFIG_LAST) ? 0xFFu : 0x00u;
    }
}

bool cc1101_wait_marcstate(uint8_t want, uint16_t timeout_us, uint8_t *last)
{
    uint8_t marc = 0xFFu;
    for (;;)
    {
        if (cc1101_read_status_stable(CC1101_MARCSTATE, &marc) == CC1101_STATUS_TIMEOUT)
        {
            if (last) { *last = 0xFFu; }
            return false;
        }
        marc &= CC1101_MARC_MASK;
        if (marc == want)       { break; }
        if (timeout_us < 10u)
        {
            if (last) { *last = marc; }
            return false;
        }
        timeout_us -= 10u;
        cs_delay_us(10u);
    }
    if (last) { *last = marc; }
    return true;
}

bool cc1101_calibrate(void)
{
    if (cc1101_strobe(CC1101_SCAL) == CC1101_STATUS_TIMEOUT) { return false; }

    /*
     * The first MARCSTATE read can still show IDLE if the state machine
     * has not moved yet, and waiting for IDLE would then return at once.
     * So first see it leave IDLE (calibration lasts ~0.7 ms, far longer
     * than this window), then wait for it to come back.
     */
    uint16_t us = 200u;
    uint8_t marc = CC1101_MARC_IDLE;
    for (;;)
    {
        if (cc1101_read_status_stable(CC1101_MARCSTATE, &marc) == CC1101_STATUS_TIMEOUT)
        {
            return false;
        }
        if ((marc & CC1101_MARC_MASK) != CC1101_MARC_IDLE) { break; }
        if (us < 5u) { return false; }          /* SCAL had no effect */
        us -= 5u;
        cs_delay_us(5u);
    }

    /* ~0.7 ms nominal; 5 ms leaves room for the delay loop's error */
    return cc1101_wait_marcstate(CC1101_MARC_IDLE, 5000u, NULL);
}

/* ---- Packets -------------------------------------------------------- */

uint8_t cc1101_tx_packet(const uint8_t *payload, uint8_t len)
{
    if (!payload || len == 0u || len > CC1101_MAX_PAYLOAD) { return CC1101_STATUS_TIMEOUT; }

    /* SFTX is only accepted in IDLE or TXFIFO_UNDERFLOW */
    if (cc1101_strobe(CC1101_SIDLE) == CC1101_STATUS_TIMEOUT) { return CC1101_STATUS_TIMEOUT; }
    if (!cc1101_wait_marcstate(CC1101_MARC_IDLE, 1000u, NULL)) { return CC1101_STATUS_TIMEOUT; }
    if (cc1101_strobe(CC1101_SFTX) == CC1101_STATUS_TIMEOUT)   { return CC1101_STATUS_TIMEOUT; }

    /* A single write to 0x3F goes to the TX FIFO: length byte first */
    if (cc1101_write_reg(CC1101_FIFO, len) == CC1101_STATUS_TIMEOUT)       { return CC1101_STATUS_TIMEOUT; }
    if (cc1101_write_burst(CC1101_FIFO, payload, len) == CC1101_STATUS_TIMEOUT) { return CC1101_STATUS_TIMEOUT; }

    /* From IDLE, STX calibrates (MCSM0.FS_AUTOCAL) and transmits */
    return cc1101_strobe(CC1101_STX);
}

uint8_t cc1101_rx_start(void)
{
    if (cc1101_strobe(CC1101_SIDLE) == CC1101_STATUS_TIMEOUT) { return CC1101_STATUS_TIMEOUT; }
    if (!cc1101_wait_marcstate(CC1101_MARC_IDLE, 1000u, NULL)) { return CC1101_STATUS_TIMEOUT; }
    if (cc1101_strobe(CC1101_SFRX) == CC1101_STATUS_TIMEOUT)   { return CC1101_STATUS_TIMEOUT; }
    return cc1101_strobe(CC1101_SRX);
}

/* SIDLE + SFRX; SFRX is only accepted in IDLE or RXFIFO_OVERFLOW */
static void rx_flush(void)
{
    (void)cc1101_strobe(CC1101_SIDLE);
    (void)cc1101_wait_marcstate(CC1101_MARC_IDLE, 1000u, NULL);
    (void)cc1101_strobe(CC1101_SFRX);
}

cc1101_rx_result_t cc1101_rx_read_packet(uint8_t *payload, uint8_t max,
                                         cc1101_rx_info_t *info)
{
    if (!payload || !info) { return CC1101_RX_TIMEOUT; }
    info->len = 0u;
    info->rssi_raw = 0u;
    info->lqi = 0x7Fu;
    info->crc_ok = false;

    uint8_t rxbytes;
    if (cc1101_read_status_stable(CC1101_RXBYTES, &rxbytes) == CC1101_STATUS_TIMEOUT)
    {
        return CC1101_RX_TIMEOUT;
    }
    if (rxbytes & CC1101_FIFO_OVF)
    {
        rx_flush();
        return CC1101_RX_OVERFLOW;
    }

    uint8_t count = (uint8_t)(rxbytes & CC1101_FIFO_COUNT);
    if (count == 0u) { return CC1101_RX_EMPTY; }

    uint8_t len;
    if (cc1101_read_reg(CC1101_FIFO, &len) == CC1101_STATUS_TIMEOUT) { return CC1101_RX_TIMEOUT; }

    /* length + payload + RSSI + LQI must all be in the FIFO already */
    if (len == 0u || len > max || (uint16_t)len + 3u > count)
    {
        rx_flush();
        return CC1101_RX_BAD_LENGTH;
    }

    uint8_t st[2];
    if (cc1101_read_burst(CC1101_FIFO, payload, len) == CC1101_STATUS_TIMEOUT ||
        cc1101_read_burst(CC1101_FIFO, st, 2u) == CC1101_STATUS_TIMEOUT)
    {
        return CC1101_RX_TIMEOUT;
    }

    info->len      = len;
    info->rssi_raw = st[0];
    info->lqi      = (uint8_t)(st[1] & 0x7Fu);
    info->crc_ok   = (st[1] & 0x80u) != 0u;

    /* Anything left over (a second packet cannot arrive: RXOFF = IDLE)
     * would be garbage; start the next RX with an empty FIFO anyway. */
    return info->crc_ok ? CC1101_RX_OK : CC1101_RX_CRC_ERROR;
}
