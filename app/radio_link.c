/*
 * radio_link.c - Console over the CC1101 radio, duty-cycled (radio build)
 */
#include "radio_link.h"
#include "console.h"
#include "../drivers/devices/cc1101.h"
#include "../drivers/devices/cc1101_settings.h"
#include "../drivers/msp430/gpio.h"
#include "../drivers/msp430/timer.h"
#include "../drivers/msp430/pmm.h"
#include "../drivers/msp430/wdt.h"
#include <msp430.h>
#include <string.h>

#define RADIO_TMR       TIMER_A2
#define OUT_SIZE        (256u)          /* reply text waiting to be sent */
#define ASYNC_SIZE      (128u)          /* async text waiting for a beacon */

typedef enum { ST_OFF = 0, ST_LISTEN } state_t;

static uint8_t  s_cfg[CC1101_CONFIG_SIZE];
static bool     s_ok;
static bool     s_awake;
static state_t  s_state;
static bool     s_extended;             /* window already stretched once */
static bool     s_in_cmd;               /* a received line has not been answered yet */
static uint16_t s_secs;                 /* since the last window */

static uint8_t  s_tx_seq;               /* our next DATA sequence number */
static uint8_t  s_rx_last = 0xFFu;      /* last host DATA sequence accepted */

static char     s_out[OUT_SIZE];
static uint16_t s_out_len;
static char     s_async[ASYNC_SIZE];
static uint16_t s_async_len;

static volatile bool s_gdo0_fell;
static volatile bool s_tmo;

/* ---------------------------------------------------------- interrupts */

static bool on_gdo0(gpio_port_t port, uint8_t pin)
{
    (void)port; (void)pin;
    s_gdo0_fell = true;                 /* end of a packet, TX or RX */
    return true;
}

static bool on_timer(timer_id_t id)
{
    (void)id;
    s_tmo = true;
    return true;
}

static void gdo0_irq(bool on)
{
    if (on)
    {
        gpio_set_callback(CC1101_GDO0_PORT, CC1101_GDO0_PIN, on_gdo0);
        gpio_set_interrupt_edge(CC1101_GDO0_PORT, CC1101_GDO0_PIN, GPIO_EDGE_FALLING);
        gpio_interrupt_clear(CC1101_GDO0_PORT, CC1101_GDO0_PIN);   /* IES change can set IFG */
        s_gdo0_fell = false;
        gpio_interrupt_enable(CC1101_GDO0_PORT, CC1101_GDO0_PIN);
    }
    else
    {
        gpio_interrupt_disable(CC1101_GDO0_PORT, CC1101_GDO0_PIN);
        gpio_interrupt_clear(CC1101_GDO0_PORT, CC1101_GDO0_PIN);
        s_gdo0_fell = false;
    }
}

/*
 * Sleep in LPM3 until GDO0 falls or ms pass. disable -> check -> sleep, so
 * an edge between the check and the sleep is not lost. Other interrupts
 * (the RTC tick) are serviced meanwhile; their flags wait for the main loop.
 */
static bool wait_gdo0(uint16_t ms)
{
    s_tmo = false;
    if (!timer_start_oneshot_ms(RADIO_TMR, ms, on_timer)) { return false; }
    for (;;)
    {
        __disable_interrupt();
        if (s_gdo0_fell || s_tmo) { __enable_interrupt(); break; }
        pmm_enter_lpm(PMM_LPM3);        /* sets GIE atomically */
    }
    timer_stop(RADIO_TMR);
    s_tmo = false;
    bool fell = s_gdo0_fell;
    s_gdo0_fell = false;
    return fell;
}

/* ---------------------------------------------------------------- chip */

void radio_link_pins_safe(void)
{
    cc1101_pins_safe();
}

static bool chip_configure(void)
{
    /* Registers are rewritten at every wake: SLEEP loses the PATABLE (and
       this keeps the chip consistent even if a register was disturbed). */
    return cc1101_write_config(s_cfg) != CC1101_STATUS_TIMEOUT &&
           cc1101_write_patable(cc1101_patable_433_0dbm, CC1101_PATABLE_SIZE) != CC1101_STATUS_TIMEOUT;
}

static bool chip_on(void)
{
    if (s_awake) { return true; }
    /* The first SPI access pulls CSn low, which wakes the chip; the driver
       waits for SO (CHIP_RDYn) before clocking. */
    if (cc1101_strobe(CC1101_SIDLE) == CC1101_STATUS_TIMEOUT || !chip_configure())
    {
        s_ok = false;
        return false;
    }
    gdo0_irq(true);
    s_awake = true;
    return true;
}

static void chip_off(void)
{
    gdo0_irq(false);
    timer_stop(RADIO_TMR);
    s_tmo = false;
    if (s_ok)
    {
        (void)cc1101_strobe(CC1101_SIDLE);
        (void)cc1101_strobe(CC1101_SPWD);   /* SLEEP once CSn goes high */
    }
    s_awake = false;
    s_state = ST_OFF;
}

/* --------------------------------------------------------------- packets */

static bool send(uint8_t type, uint8_t seq, const char *text, uint8_t len)
{
    uint8_t p[RADIO_HDR_LEN + RADIO_TEXT_MAX];
    if (len > RADIO_TEXT_MAX) { len = RADIO_TEXT_MAX; }
    p[0] = RADIO_ADDR_HOST;
    p[1] = RADIO_ADDR_LOGGER;
    p[2] = (uint8_t)(type | (seq & RADIO_SEQ_MASK));
    if (len) { memcpy(&p[RADIO_HDR_LEN], text, len); }

    s_gdo0_fell = false;
    if (cc1101_tx_packet(p, (uint8_t)(RADIO_HDR_LEN + len)) == CC1101_STATUS_TIMEOUT) { return false; }
    return wait_gdo0(RADIO_TX_MS);      /* GDO0 falls at the end of the packet */
}

/* Read one packet after GDO0 fell; true if it is for us, from the host */
static bool receive(uint8_t *p, uint8_t *len)
{
    cc1101_rx_info_t info;
    if (cc1101_rx_read_packet(p, CC1101_MAX_PAYLOAD, &info) != CC1101_RX_OK) { return false; }
    if (info.len < RADIO_HDR_LEN) { return false; }
    if (p[0] != RADIO_ADDR_LOGGER || p[1] != RADIO_ADDR_HOST) { return false; }
    *len = info.len;
    return true;
}

static void listen(uint16_t ms)
{
    s_gdo0_fell = false;
    s_tmo = false;
    if (cc1101_rx_start() == CC1101_STATUS_TIMEOUT ||
        !timer_start_oneshot_ms(RADIO_TMR, ms, on_timer))
    {
        chip_off();
        return;
    }
    s_extended = false;
    s_state = ST_LISTEN;
}

/*
 * Send everything in s_out as DATA packets, each acknowledged by the host
 * (stop and wait, RADIO_RETRIES retries). On a lost link the rest of the
 * reply is dropped. Leaves the chip in IDLE.
 */
static void flush_out(void)
{
    if (!s_awake) { s_out_len = 0u; return; }
    timer_stop(RADIO_TMR);              /* the listen window pauses while we talk */

    while (s_out_len)
    {
        uint8_t n = (s_out_len > RADIO_TEXT_MAX) ? (uint8_t)RADIO_TEXT_MAX : (uint8_t)s_out_len;
        bool acked = false;
        uint8_t tries;
        for (tries = 0u; tries <= RADIO_RETRIES && !acked; tries++)
        {
            wdt_kick();
            if (!send(RADIO_TYPE_DATA, s_tx_seq, s_out, n)) { break; }
            if (cc1101_rx_start() == CC1101_STATUS_TIMEOUT) { break; }
            if (!wait_gdo0(RADIO_ACK_MS)) { continue; }

            uint8_t p[CC1101_MAX_PAYLOAD], len;
            if (receive(p, &len) && (p[2] & RADIO_TYPE_MASK) == RADIO_TYPE_ACK &&
                (p[2] & RADIO_SEQ_MASK) == s_tx_seq)
            {
                acked = true;
            }
        }
        if (!acked)
        {
            s_out_len = 0u;             /* link lost: drop the rest */
            break;
        }
        s_tx_seq = (uint8_t)((s_tx_seq + 1u) & RADIO_SEQ_MASK);
        s_out_len = (uint16_t)(s_out_len - n);
        memmove(s_out, &s_out[n], s_out_len);
    }
}

/* Send queued async text in beacons (at least one beacon) */
static void send_beacons(void)
{
    uint16_t off = 0u;
    do
    {
        uint8_t n = (s_async_len - off > RADIO_TEXT_MAX) ? (uint8_t)RADIO_TEXT_MAX
                                                         : (uint8_t)(s_async_len - off);
        if (!send(RADIO_TYPE_BEACON, 0u, &s_async[off], n)) { break; }
        off = (uint16_t)(off + n);
    } while (off < s_async_len);
    s_async_len = 0u;
}

/* ------------------------------------------------------- console port */

/* Output during a radio command goes to the reply; anything else (main
   selecting the port for an async line) waits for the next beacon. */
static void port_write(const char *s, uint16_t n)
{
    if (s_in_cmd)
    {
        while (n)
        {
            if (s_out_len == OUT_SIZE) { flush_out(); }
            uint16_t k = (uint16_t)(OUT_SIZE - s_out_len);
            if (k > n) { k = n; }
            memcpy(&s_out[s_out_len], s, k);
            s_out_len = (uint16_t)(s_out_len + k);
            s += k;
            n = (uint16_t)(n - k);
        }
        return;
    }
    if (s_async_len + n > ASYNC_SIZE) { return; }      /* full: dropped */
    memcpy(&s_async[s_async_len], s, n);
    s_async_len = (uint16_t)(s_async_len + n);
}

static void port_done(void)
{
    flush_out();
    s_in_cmd = false;
    if (s_awake) { listen(RADIO_WINDOW_MS); }   /* room for the next command */
}

static void port_flush(void)
{
    flush_out();
}

/* ------------------------------------------------------------- public */

bool radio_link_init(void)
{
    memcpy(s_cfg, cc1101_cfg_433_gfsk_1k2, sizeof s_cfg);
    cc1101_cfg_packet_mode(s_cfg);

    s_ok = false;
    s_awake = false;
    s_state = ST_OFF;
    if (cc1101_init() && chip_configure() && cc1101_calibrate())
    {
        uint8_t partnum = 0xFFu;
        s_ok = cc1101_read_status(CC1101_PARTNUM, &partnum) != CC1101_STATUS_TIMEOUT &&
               partnum == CC1101_PARTNUM_EXPECTED;
    }
    s_awake = true;                     /* chip_off() puts it to sleep */
    chip_off();
    if (!s_ok) { cc1101_pins_safe(); }

    static const con_port_ops_t k_ops = { port_write, NULL, port_done, port_flush };
    console_attach(CON_PORT_RADIO, &k_ops);
    s_async_len = 0u;
    s_out_len = 0u;
    s_in_cmd = false;
    s_secs = 0u;
    return s_ok;
}

bool radio_link_ok(void)     { return s_ok; }
bool radio_link_awake(void)  { return s_awake; }

bool radio_link_due(uint16_t period_s)
{
    if (period_s == 0u || s_state != ST_OFF) { s_secs = 0u; return false; }
    s_secs = (uint16_t)(s_secs + 2u);
    if (s_secs < period_s) { return false; }
    s_secs = 0u;
    if (!s_ok && !radio_link_init()) { return false; }  /* retry a missing chip */
    return true;
}

void radio_link_open(void)
{
    if (!chip_on()) { s_async_len = 0u; return; }
    send_beacons();
    listen(RADIO_WINDOW_MS);
}

bool radio_link_pending(void)
{
    return s_state == ST_LISTEN && (s_gdo0_fell || s_tmo);
}

void radio_link_service(void)
{
    if (s_state != ST_LISTEN) { return; }

    if (s_gdo0_fell)
    {
        s_gdo0_fell = false;
        uint8_t p[CC1101_MAX_PAYLOAD], len;
        bool mine = receive(p, &len);
        if (mine && (p[2] & RADIO_TYPE_MASK) == RADIO_TYPE_DATA && !s_in_cmd)
        {
            uint8_t seq = (uint8_t)(p[2] & RADIO_SEQ_MASK);
            timer_stop(RADIO_TMR);
            (void)send(RADIO_TYPE_ACK, seq, NULL, 0u);
            if (seq != s_rx_last)       /* a repeat means our ACK was lost */
            {
                uint8_t i;
                s_rx_last = seq;
                for (i = RADIO_HDR_LEN; i < len; i++)
                {
                    if (console_rx(CON_PORT_RADIO, p[i], false)) { s_in_cmd = true; }
                }
            }
            /* A complete line: the chip idles until port_done() has sent
               the reply and opens the next window. A partial line (no
               CR yet): keep listening for the rest. */
            if (!s_in_cmd) { listen(RADIO_WINDOW_MS); }
            return;
        }
        /* not for us (or a stray ACK): keep listening for the rest of the window */
        (void)cc1101_rx_start();
        return;
    }

    if (s_tmo)
    {
        s_tmo = false;
        if (s_in_cmd) { return; }       /* the reply will re-open the window */
        if (!s_extended && cc1101_gdo0())
        {
            /* a packet is arriving right at the end of the window */
            s_extended = true;
            (void)timer_start_oneshot_ms(RADIO_TMR, RADIO_WINDOW_MAX_MS, on_timer);
            return;
        }
        chip_off();
    }
}

void radio_link_shutdown(void)
{
    if (s_ok && s_async_len && chip_on())
    {
        send_beacons();
    }
    if (s_awake || s_ok) { s_awake = true; chip_off(); }
    console_detach(CON_PORT_RADIO);
    cc1101_pins_safe();
}
