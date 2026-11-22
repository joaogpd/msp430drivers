/*
 * main.c - Layer 2 demo: power-aware logger for the MSP-EXP430FR5994
 *
 * Normal mode (VCC >= 2.2 V)
 *   - RTC wakes the CPU from LPM3 every 2 s.
 *   - Each wake: measure VCC (supercap on J8 "Use") and die temperature
 *     (average of 8 conversions), append them to a 16-entry log, save
 *     everything to FRAM (checkpoint), kick the watchdog (16 s timeout).
 *   - The green LED flashes for 2 ms when a sample lands in log slot 0,
 *     i.e. every 16th sample (~32 s), marking a new pass through the log.
 *
 * Low-energy mode (VCC < 2.2 V)
 *   - Save, then enter LPM3.5: only the RTC runs (~0.5 uA for the chip).
 *   - The RTC wakes the device once a minute (that wake is a reset); it
 *     measures VCC and goes straight back to LPM3.5 unless VCC >= 2.6 V,
 *     in which case normal logging resumes.
 *
 * State survives resets and power loss (ckpt.c): boot and power-loss
 * counters, sample count, minimum VCC seen and the log.
 *
 * Boot     : red LED flashes N times (20 ms each) for the start reason:
 *            1 power-on, 2 LPMx.5 wake (only when resuming), 3 reset pin,
 *            4 watchdog, 5 software, 6 fault.
 * S1       : supply gauge - green flashes, one per 100 mV above 2.0 V.
 * S2       : erase the saved state (counters and log), 3 red flashes.
 *
 * Console (layer 3): the backchannel UART (eUSCI_A0, P2.0/P2.1) at
 * 9600 8N1 on the "MSP Application UART1" COM port. Clocked from the
 * 32 kHz crystal, so it works while the CPU sleeps in LPM3. Commands:
 * help, status, log, time, stream, erase (see k_cmds below). It is not
 * started on LPM3.5 check-ins and is shut down before deep sleep.
 */
#include <msp430.h>
#include <string.h>
#include "drivers/board/board.h"
#include "drivers/msp430/cs.h"
#include "drivers/msp430/pmm.h"
#include "drivers/msp430/wdt.h"
#include "drivers/msp430/adc.h"
#include "drivers/msp430/rtc.h"
#include "drivers/msp430/timer.h"
#include "drivers/msp430/crc.h"
#include "drivers/msp430/ckpt.h"
#include "drivers/msp430/uart.h"
#include "app/console.h"
#include "app/history.h"

/*
 * Long history (app/history.c): one record every HIST_EVERY_SAMPLES
 * samples (2 min) while logging, plus boot, deep-sleep, check-in and
 * resume events. 1600 records cover a full supercap discharge (~21 h of
 * logging at 2 min, then ~8 h of minute check-ins).
 */
#define HIST_EVERY_SAMPLES  (60u)
#define HIST_NO_TEMP        (-32768)    /* temp_dc when not measured */

/* Where the RTC time came from at this boot */
typedef enum { CLOCK_KEPT = 0, CLOCK_RESTORED, CLOCK_DEFAULT, CLOCK_SET } clock_src_t;
static bool        s_rtc_kept;
static clock_src_t s_clock;

/*
 * Power-cut test build: set to 1 to replace the logger with a loop that
 * saves a checkpoint continuously, so a USB unplug almost always lands in
 * the middle of a save. Each boot checks what was restored. See
 * stress_test() below.
 */
#ifndef CKPT_STRESS_TEST
#define CKPT_STRESS_TEST    0
#endif
/* The fault-injection build (test 4) is switched by CKPT_FAULT_TEST in
   drivers/msp430/ckpt.h, because ckpt.c needs it too. */

/*
 * Deep-sleep test build (test 1): set to 1 to run the LPM3.5 path on USB
 * power. See the "deep-sleep test" section below.
 */
#ifndef LPM35_TEST
#define LPM35_TEST          0
#endif

/*
 * LED timing: short to save energy, long enough to see in the test build.
 * The green sample flash marks the start of each pass through the 16-entry
 * log (every 16th sample, ~32 s); the test build flashes every sample.
 */
#if LPM35_TEST
#define BOOT_FLASH_MS       (200u)
#define SAMPLE_FLASH_MS     (50u)
#else
#define BOOT_FLASH_MS       (20u)
#define SAMPLE_FLASH_MS     (2u)
#endif

/*
 * LED A/B test build (test for the E6/E8 floor difference): set to 1 to
 * flash the green LED on EVERY sample for the first LED_AB_SAMPLES samples
 * after boot (as the E6 build did), then only every 16th (as E8). One
 * EnergyTrace capture then shows both modes under the same calibration:
 * if the sleep floor changes at the switch, the firmware causes it.
 */
#ifndef LED_AB_TEST
#define LED_AB_TEST         0
#endif
#define LED_AB_SAMPLES      (75u)   /* ~150 s at 2 s per sample */

/*
 * Console energy test build (E10): set to 1 to run three phases in one
 * EnergyTrace capture, counted in samples from boot (2 s each):
 *   samples   1-50  (~0-100 s)   console off, pins idle
 *   samples  51-100 (~100-200 s) console on and idle (banner printed once)
 *   samples 101-    (~200 s on)  console on, stream on: one CSV line/sample
 * The green sample flash is off in this build, so every sample costs the
 * same and only the console differs between phases.
 */
#ifndef CONSOLE_E10_TEST
#define CONSOLE_E10_TEST    0
#endif
#define E10_PHASE_SAMPLES   (50u)

#if (CKPT_STRESS_TEST + CKPT_FAULT_TEST + LPM35_TEST + LED_AB_TEST + CONSOLE_E10_TEST) > 1
#error "Enable only one of CKPT_STRESS_TEST, CKPT_FAULT_TEST, LPM35_TEST, LED_AB_TEST and CONSOLE_E10_TEST"
#endif

#define VCC_LOW_MV      (2200u)     /* enter LPM3.5 below this */
#define VCC_RESUME_MV   (2600u)     /* leave LPM3.5 at or above this */
#define LOG_LEN         (16u)

#define TMR_SLEEP       TIMER_A0    /* timer_sleep_ms */
#define TMR_DEBOUNCE    TIMER_A1

typedef struct
{
    uint32_t t;         /* RTC time, seconds since 2026-01-01 00:00:00 */
    uint16_t vcc_mv;
    int16_t  temp_dc;   /* 0.1 C */
} log_entry_t;

typedef struct
{
    uint32_t    boots;          /* every start that runs normal mode */
    uint32_t    power_losses;   /* starts caused by BOR / SVSH */
    uint32_t    deep_sleeps;    /* entries into LPM3.5 for low VCC */
    uint32_t    samples;
    uint16_t    min_vcc_mv;
    uint8_t     log_head;       /* next slot to write */
    uint8_t     log_count;
    log_entry_t log[LOG_LEN];
} app_state_t;

app_state_t g_state;            /* RAM working copy, saved to FRAM by ckpt */

static volatile bool s_sample_due;
static volatile bool s_s1_event;
static volatile bool s_s2_event;
static bool          s_ckpt_ok;

/* ---------------------------------------------------------------- RTC */

static bool on_rtc(rtc_event_t ev)
{
    if (ev == RTC_EV_PERIODIC)
    {
        s_sample_due = true;
        return true;
    }
    return false;       /* minute event: only a wake source for LPM3.5 */
}

/* Runs inside board_init() before LOCKLPM5 is cleared (RTC rule for LPMx.5) */
static void before_unlock(void)
{
    s_rtc_kept = rtc_init(NULL, on_rtc);
    rtc_set_periodic(RTC_PERIOD_2S);
    rtc_set_time_event(RTC_TEV_OFF);
}

/* 2026-01-01 00:00:00 in seconds since 2000-01-01 (9497 days) */
#define EPOCH_2026      (820540800ul)

/* RTC time as seconds since 2026-01-01 (the RTC's power-on default) */
static uint32_t now_s(void)
{
    rtc_time_t t;
    if (!rtc_get_time(&t)) { return 0; }
    uint32_t e = rtc_to_epoch(&t);
    return (e >= EPOCH_2026) ? e - EPOCH_2026 : 0u;
}

/* ------------------------------------------------------------ buttons */

static bool on_debounce_done(timer_id_t id)
{
    (void)id;
    gpio_interrupt_clear(BOARD_S1_PORT, BOARD_S1_PIN | BOARD_S2_PIN);
    gpio_interrupt_enable(BOARD_S1_PORT, BOARD_S1_PIN | BOARD_S2_PIN);
    return false;
}

static bool on_button(gpio_port_t port, uint8_t pin)
{
    (void)port;
    gpio_interrupt_disable(BOARD_S1_PORT, BOARD_S1_PIN | BOARD_S2_PIN);
    timer_start_oneshot_ms(TMR_DEBOUNCE, 300u, on_debounce_done);

    if ((uint8_t)(1u << pin) == BOARD_S1_PIN) { s_s1_event = true; }
    if ((uint8_t)(1u << pin) == BOARD_S2_PIN) { s_s2_event = true; }
    return true;
}

/* -------------------------------------------------------------- LEDs */

static void flash(bool red, uint8_t times, uint16_t on_ms, uint16_t off_ms)
{
    while (times--)
    {
        if (red) { board_led1(true); } else { board_led2(true); }
        timer_sleep_ms(TMR_SLEEP, on_ms);
        board_led1(false);
        board_led2(false);
        if (times) { timer_sleep_ms(TMR_SLEEP, off_ms); }
    }
}

/* ----------------------------------------------------------- storage */

static void state_reset(void)
{
    memset(&g_state, 0, sizeof g_state);
    g_state.min_vcc_mv = 0xFFFFu;
}

static void state_save(void)
{
    if (s_ckpt_ok) { ckpt_save(&g_state, sizeof g_state); }
}

/* ------------------------------------------- deep-sleep test (test 1) */
#if LPM35_TEST
/*
 * On USB VCC is ~3.3 V, so the real 2.2 V threshold never trips. This
 * build fakes only the two decisions, on a fixed schedule:
 *   - after LPM35_AWAKE_SAMPLES samples (2 s each) the low threshold is
 *     raised above any VCC, so the logger enters LPM3.5 the normal way;
 *   - the first LPM35_HOLD_CHECKINS minute check-ins are told to stay
 *     asleep; the next one uses the real 2.6 V resume check, which passes
 *     on USB, and logging resumes.
 * This repeats LPM35_EPISODES times, then the logger runs normally.
 *
 * Every check-in is verified:
 *   bad_wake  the start was not an LPMx.5 wake (RTC event lost, or
 *             something else reset the chip)                  -> must be 0
 *   bad_time  the RTC did not keep time through LPM3.5: the wake was not
 *             at a minute rollover, or not 60 s after the previous one
 *                                                              -> must be 0
 * Results are in g_lpm35 (persistent FRAM). Reflash to run it again.
 */
#define LPM35_AWAKE_SAMPLES     (5u)    /* ~10 s awake per episode */
#define LPM35_HOLD_CHECKINS     (2u)    /* stay asleep for 2 check-ins */
#define LPM35_EPISODES          (3u)
#define LPM35_CHECKINS_MAX      (LPM35_EPISODES * (LPM35_HOLD_CHECKINS + 1u))

typedef struct
{
    uint16_t episodes;          /* completed sleep -> resume cycles */
    uint16_t checkins;          /* check-ins in the current episode */
    uint16_t checkins_total;
    uint16_t bad_wake;
    uint16_t bad_time;
    uint16_t awake_samples;     /* samples since the last resume */
    uint32_t t_sleep;           /* RTC seconds when LPM3.5 was entered */
    uint32_t t_last;            /* RTC seconds at the last check-in */
    uint32_t last_gap_s;        /* seconds between the last two wakes */
    /*
     * Timeline, one row per check-in (episode e, check-in c at index
     * e * (LPM35_HOLD_CHECKINS + 1) + c - 1). t values are RTC seconds
     * since power-on; why is a LPM35_BAD_* bitmask, 0 = good.
     */
    uint16_t sleep_t[LPM35_EPISODES];
    uint16_t ci_t[LPM35_CHECKINS_MAX];
    uint16_t ci_gap[LPM35_CHECKINS_MAX];
    uint16_t ci_rstiv[LPM35_CHECKINS_MAX];  /* first SYSRSTIV code (0x08 = LPMx.5 wake) */
    uint16_t ci_why[LPM35_CHECKINS_MAX];
} lpm35_test_t;

#define LPM35_BAD_NOT_LPM5      (0x01u)     /* start was not an LPMx.5 wake */
#define LPM35_BAD_NOT_ROLLOVER  (0x02u)     /* wake not at a minute rollover */
#define LPM35_BAD_GAP           (0x04u)     /* wrong spacing from previous wake */
#define LPM35_BAD_RTC_READ      (0x08u)     /* rtc_get_time() failed */

#pragma PERSISTENT(g_lpm35)
lpm35_test_t g_lpm35 = { 0u };

static bool lpm35_active(void) { return g_lpm35.episodes < LPM35_EPISODES; }

static uint16_t low_mv(void)
{
    if (lpm35_active() && ++g_lpm35.awake_samples >= LPM35_AWAKE_SAMPLES)
    {
        return 0xFFFFu;             /* "supply low": sleep now */
    }
    return VCC_LOW_MV;
}

static void lpm35_on_sleep(void)
{
    g_lpm35.t_sleep = now_s();
    g_lpm35.t_last = g_lpm35.t_sleep;
    g_lpm35.checkins = 0u;
    if (lpm35_active()) { g_lpm35.sleep_t[g_lpm35.episodes] = (uint16_t)g_lpm35.t_sleep; }
}

static uint16_t resume_mv(void)
{
    if (!lpm35_active()) { return VCC_RESUME_MV; }

    rtc_time_t now;
    bool read_ok = rtc_get_time(&now);
    uint32_t t = read_ok ? (uint32_t)(now.day - 1u) * 86400ul + rtc_seconds_of_day(&now) : 0ul;
    uint32_t gap = t - g_lpm35.t_last;
    bool first = (g_lpm35.checkins == 0u);
    uint16_t why = 0u;

    g_lpm35.checkins++;
    g_lpm35.checkins_total++;
    g_lpm35.last_gap_s = gap;

    if (!(pmm_reset_flags() & PMM_RST_LPM5WU)) { why |= LPM35_BAD_NOT_LPM5; }
    if (!read_ok) { why |= LPM35_BAD_RTC_READ; }
    /* 2 s of slack: the time is read after clock and ADC start-up. The
       minute event fires at the rollover: first wake within 60 s of entry,
       the next ones exactly 60 s apart. */
    if ((t % 60u) > 2u) { why |= LPM35_BAD_NOT_ROLLOVER; }
    if (first ? (gap < 1u || gap > 62u) : (gap < 58u || gap > 62u)) { why |= LPM35_BAD_GAP; }

    if (why & LPM35_BAD_NOT_LPM5) { g_lpm35.bad_wake++; }
    if (why & (uint16_t)~LPM35_BAD_NOT_LPM5) { g_lpm35.bad_time++; }

    uint16_t i = g_lpm35.episodes * (LPM35_HOLD_CHECKINS + 1u) + g_lpm35.checkins - 1u;
    if (i < LPM35_CHECKINS_MAX)
    {
        g_lpm35.ci_t[i] = (uint16_t)t;
        g_lpm35.ci_gap[i] = (uint16_t)gap;
        g_lpm35.ci_rstiv[i] = pmm_reset_iv_first();
        g_lpm35.ci_why[i] = why;
    }
    g_lpm35.t_last = t;

    if (g_lpm35.checkins <= LPM35_HOLD_CHECKINS) { return 0xFFFFu; }  /* stay asleep */
    return VCC_RESUME_MV;           /* the real check decides */
}

static void lpm35_on_resume(void)
{
    if (!lpm35_active()) { return; }
    g_lpm35.episodes++;
    g_lpm35.awake_samples = 0u;
}
#else
static inline uint16_t low_mv(void)    { return VCC_LOW_MV; }
static inline uint16_t resume_mv(void) { return VCC_RESUME_MV; }
static inline void lpm35_on_sleep(void)  { }
static inline void lpm35_on_resume(void) { }
#endif

/* ----------------------------------------------------------- console */

#define CON_UART        UART_A0
#define CON_BAUD        (9600ul)

static uint8_t           s_con_tx[256];
static bool              s_con_up;
static bool              s_stream;          /* print every sample */
static pmm_boot_reason_t s_why;

static const char *const k_why[] = { "power-on", "LPMx.5 wake", "reset pin", "watchdog", "software", "fault" };

static void erase_state(void)
{
    state_reset();
    if (s_ckpt_ok) { ckpt_erase(); }
    hist_clear();
}

static const char *const k_clock[] = { "kept running", "restored from the last log entry (behind by the outage)",
                                       "default 2026-01-01 (no log to restore from)", "set from the console" };

/* t: seconds since 2026-01-01 -> "YYYY-MM-DD hh:mm:ss" */
static void con_time(uint32_t t)
{
    rtc_time_t c;
    rtc_from_epoch(t + EPOCH_2026, &c);
    con_u_pad(c.year, 4u);  con_putc('-');
    con_u_pad(c.month, 2u); con_putc('-');
    con_u_pad(c.day, 2u);   con_putc(' ');
    con_u_pad(c.hour, 2u);  con_putc(':');
    con_u_pad(c.min, 2u);   con_putc(':');
    con_u_pad(c.sec, 2u);
}

static void con_log_line(const log_entry_t *e)
{
    con_u32(e->t);         con_putc(',');
    con_time(e->t);        con_putc(',');
    con_u32(e->vcc_mv);    con_putc(',');
    con_dec1(e->temp_dc);
    con_nl();
}

static void cmd_help(uint8_t argc, char *argv[])
{
    (void)argc; (void)argv;
    console_help();
}

static void cmd_status(uint8_t argc, char *argv[])
{
    (void)argc; (void)argv;
    uint16_t mv = 0;
    int16_t  dc = 0;
    bool v_ok = adc_read_vcc_mv(&mv) == ADC_OK;
    bool t_ok = adc_read_temp_dc(&dc) == ADC_OK;

    con_puts("time      "); con_time(now_s());
    con_puts(" ("); con_puts(k_clock[s_clock]); con_puts(")"); con_nl();
    con_puts("boot      #"); con_u32(g_state.boots);
    con_puts(", reason "); con_puts(k_why[s_why]); con_nl();
    con_puts("counters  power losses "); con_u32(g_state.power_losses);
    con_puts(", deep sleeps "); con_u32(g_state.deep_sleeps);
    con_puts(", samples "); con_u32(g_state.samples); con_nl();
    con_puts("supply    ");
    if (v_ok) { con_u32(mv); con_puts(" mV"); } else { con_puts("read failed"); }
    con_puts(", min ");
    if (g_state.min_vcc_mv != 0xFFFFu) { con_u32(g_state.min_vcc_mv); con_puts(" mV"); } else { con_puts("-"); }
    con_puts(", sleep below "); con_u32(VCC_LOW_MV);
    con_puts(", resume at "); con_u32(VCC_RESUME_MV); con_puts(" mV"); con_nl();
    con_puts("die temp  ");
    if (t_ok) { con_dec1(dc); con_puts(" C"); } else { con_puts("read failed"); }
    con_nl();
    con_puts("storage   ");
    con_puts(s_ckpt_ok ? "checkpoints on" : "CRC self-test failed, checkpoints off");
    con_puts(", log "); con_u32(g_state.log_count); con_puts("/"); con_u32(LOG_LEN);
    con_puts(", history "); con_u32(hist_count()); con_puts("/"); con_u32(HIST_LEN); con_nl();
}

static void cmd_hist(uint8_t argc, char *argv[])
{
    uint16_t count = hist_count();
    uint32_t n = count;
    if (argc == 2u && !con_parse_u32(argv[1], 65535u, &n))
    {
        con_puts("expected: hist [n] (newest n records)"); con_nl();
        return;
    }
    if (argc > 2u) { con_puts("expected: hist [n]"); con_nl(); return; }
    if (n > count) { n = count; }

    con_puts("seq,t_s,time,event,detail,vcc_mv,temp_c"); con_nl();
    uint16_t i;
    for (i = (uint16_t)(count - n); i < count; i++)
    {
        hist_rec_t r;
        if (!hist_get(i, count, &r)) { continue; }
        con_u32(r.seq);        con_putc(',');
        con_u32(r.t);          con_putc(',');
        con_time(r.t);         con_putc(',');
        con_puts(hist_kind_name(r.kind)); con_putc(',');
        if (r.kind == HIST_BOOT)
        {
            con_puts(k_why[r.arg & 0x0Fu]);
            if (r.arg & 0x80u) { con_puts(" (clock restored)"); }
        }
        con_putc(',');
        if (r.vcc_mv) { con_u32(r.vcc_mv); }
        con_putc(',');
        if (r.temp_dc != HIST_NO_TEMP) { con_dec1(r.temp_dc); }
        con_nl();
        wdt_kick();
    }
}

static void cmd_log(uint8_t argc, char *argv[])
{
    (void)argc; (void)argv;
    uint8_t n = g_state.log_count;
    uint8_t i = (n < LOG_LEN) ? 0u : g_state.log_head;     /* oldest first */
    con_puts("t_s,time,vcc_mv,temp_c"); con_nl();
    while (n--)
    {
        con_log_line(&g_state.log[i]);
        i = (uint8_t)((i + 1u) % LOG_LEN);
        wdt_kick();
    }
}

/* "YYYY-MM-DD" "hh:mm:ss" */
static bool parse_time(const char *d, const char *h, rtc_time_t *t)
{
    char buf[11];
    uint32_t y, mo, da, hh, mi, ss;
    if (strlen(d) != 10u || d[4] != '-' || d[7] != '-') { return false; }
    if (strlen(h) != 8u || h[2] != ':' || h[5] != ':') { return false; }
    memcpy(buf, d, 4);      buf[4] = '\0'; if (!con_parse_u32(buf, 9999u, &y))  { return false; }
    memcpy(buf, d + 5, 2);  buf[2] = '\0'; if (!con_parse_u32(buf, 99u, &mo))   { return false; }
    memcpy(buf, d + 8, 2);  buf[2] = '\0'; if (!con_parse_u32(buf, 99u, &da))   { return false; }
    memcpy(buf, h, 2);      buf[2] = '\0'; if (!con_parse_u32(buf, 99u, &hh))   { return false; }
    memcpy(buf, h + 3, 2);  buf[2] = '\0'; if (!con_parse_u32(buf, 99u, &mi))   { return false; }
    memcpy(buf, h + 6, 2);  buf[2] = '\0'; if (!con_parse_u32(buf, 99u, &ss))   { return false; }
    t->year = (uint16_t)y; t->month = (uint8_t)mo; t->day = (uint8_t)da;
    t->hour = (uint8_t)hh; t->min = (uint8_t)mi;   t->sec = (uint8_t)ss;
    t->dow = 0u;
    if (!rtc_time_valid(t) || t->year < 2026u) { return false; }
    rtc_time_t c;
    rtc_from_epoch(rtc_to_epoch(t), &c);
    t->dow = c.dow;
    return true;
}

static void cmd_time(uint8_t argc, char *argv[])
{
    if (argc == 3u)
    {
        rtc_time_t t;
        if (!parse_time(argv[1], argv[2], &t))
        {
            con_puts("expected: time YYYY-MM-DD hh:mm:ss (2026 or later)"); con_nl();
            return;
        }
        rtc_set_time(&t);
        s_clock = CLOCK_SET;
        hist_append(HIST_CLOCK_SET, 0u, now_s(), 0u, HIST_NO_TEMP);
    }
    else if (argc != 1u)
    {
        con_puts("expected: time [YYYY-MM-DD hh:mm:ss]"); con_nl();
        return;
    }
    con_time(now_s()); con_nl();
}

static void cmd_stream(uint8_t argc, char *argv[])
{
    if (argc == 2u && strcmp(argv[1], "on") == 0)  { s_stream = true; }
    else if (argc == 2u && strcmp(argv[1], "off") == 0) { s_stream = false; }
    else if (argc != 1u) { con_puts("expected: stream [on|off]"); con_nl(); return; }
    con_puts("stream "); con_puts(s_stream ? "on" : "off"); con_nl();
    if (s_stream) { con_puts("t_s,time,vcc_mv,temp_c"); con_nl(); }
}

static void cmd_erase(uint8_t argc, char *argv[])
{
    if (argc != 2u || strcmp(argv[1], "yes") != 0)
    {
        con_puts("this clears all counters, the log and the history; type: erase yes"); con_nl();
        return;
    }
    erase_state();
    con_puts("state erased"); con_nl();
}

static const console_cmd_t k_cmds[] =
{
    { "help",   "",                      "list commands",                         cmd_help   },
    { "status", "",                      "counters, supply, temperature, time",   cmd_status },
    { "log",    "",                      "dump the 16-sample log as CSV",         cmd_log    },
    { "hist",   "[n]",                   "dump the long FRAM history as CSV",     cmd_hist   },
    { "time",   "[YYYY-MM-DD hh:mm:ss]", "show or set the clock",                 cmd_time   },
    { "stream", "[on|off]",              "print every sample as it is taken",     cmd_stream },
    { "erase",  "yes",                   "clear counters, log, history (like S2)", cmd_erase  },
};

static bool on_uart_rx(uart_id_t id, uint8_t byte, bool error)
{
    (void)id;
    return console_rx(byte, error);
}

static void console_start(cs_status_t cs)
{
    const uart_config_t cfg =
    {
        CON_BAUD,
        (cs == CS_LFXT_FAULT) ? UART_CLK_SMCLK : UART_CLK_ACLK,    /* VLO cannot do 9600 */
        s_con_tx, sizeof s_con_tx,
        on_uart_rx
    };
    board_uart_pins(true);
    if (!uart_init(CON_UART, &cfg))
    {
        board_uart_pins(false);
        return;
    }
    console_init(CON_UART, k_cmds, (uint8_t)(sizeof k_cmds / sizeof k_cmds[0]));
    s_con_up = true;

    con_nl();
    con_puts("FR5994 power-aware logger"); con_nl();
    con_puts("boot #"); con_u32(g_state.boots);
    con_puts(", reason "); con_puts(k_why[s_why]);
    if (cs == CS_LFXT_FAULT) { con_puts(", crystal FAILED (UART on DCO)"); }
    con_nl();
    con_puts("type help for commands"); con_nl();
    con_prompt();
}

static void console_stop(void)
{
    if (!s_con_up) { return; }
    (void)uart_flush(CON_UART);
    uart_deinit(CON_UART);
    board_uart_pins(false);
    s_con_up = false;
}

/* ------------------------------------------------------------- modes */

static void enter_deep_sleep(void)
{
    g_state.deep_sleeps++;
    state_save();

    if (s_con_up)
    {
        con_nl();
        con_puts("supply low, entering LPM3.5 (checks every minute)"); con_nl();
        console_stop();
    }

    wdt_hold();
    timer_stop(TMR_DEBOUNCE);
    gpio_interrupt_disable(BOARD_S1_PORT, BOARD_S1_PIN | BOARD_S2_PIN);
    gpio_interrupt_clear(BOARD_S1_PORT, BOARD_S1_PIN | BOARD_S2_PIN);
    board_led1(false);
    board_led2(false);

    rtc_set_periodic(RTC_PERIOD_OFF);
    rtc_set_time_event(RTC_TEV_MINUTE);     /* the only wake source */
    pmm_enter_lpmx5();                      /* no return: wake is a reset */
}

/* Returns true when the sample went into log slot 0, i.e. a new pass
   through the 16-entry ring buffer has started. */
static bool take_sample(void)
{
    uint16_t mv = 0;
    int16_t  dc = 0;
    if (adc_read_vcc_mv(&mv) != ADC_OK) { return false; }
    (void)adc_read_temp_dc(&dc);

    bool new_pass = (g_state.log_head == 0u);
    log_entry_t *e = &g_state.log[g_state.log_head];
    e->t = now_s();
    e->vcc_mv = mv;
    e->temp_dc = dc;
    g_state.log_head = (uint8_t)((g_state.log_head + 1u) % LOG_LEN);
    if (g_state.log_count < LOG_LEN) { g_state.log_count++; }
    g_state.samples++;
    if (mv < g_state.min_vcc_mv) { g_state.min_vcc_mv = mv; }
    state_save();
    if (g_state.samples % HIST_EVERY_SAMPLES == 0u)
    {
        hist_append(HIST_SAMPLE, 0u, e->t, mv, dc);
    }

    if (mv < low_mv())
    {
        hist_append(HIST_SLEEP, 0u, e->t, mv, dc);
        lpm35_on_sleep();
        enter_deep_sleep();
    }
    return new_pass;
}

/* ------------------------------------------- checkpoint test helpers */
#if CKPT_STRESS_TEST || CKPT_FAULT_TEST
/* Every field derives from one counter, so any mix of old and new bytes
   in a restored copy is detectable. */
static void stress_fill(uint32_t n)
{
    uint8_t i;
    g_state.samples = n;
    g_state.boots = ~n;
    for (i = 0; i < LOG_LEN; i++)
    {
        g_state.log[i].t = n + i;
        g_state.log[i].vcc_mv = (uint16_t)(n * 7u + i);
        g_state.log[i].temp_dc = (int16_t)~(uint16_t)(n + i);
    }
}

static bool stress_consistent(void)
{
    uint32_t n = g_state.samples;
    uint8_t i;
    if (g_state.boots != ~n) { return false; }
    for (i = 0; i < LOG_LEN; i++)
    {
        if (g_state.log[i].t != n + i ||
            g_state.log[i].vcc_mv != (uint16_t)(n * 7u + i) ||
            g_state.log[i].temp_dc != (int16_t)~(uint16_t)(n + i)) { return false; }
    }
    return true;
}

#endif

/* ------------------------------------------------- power-cut stress test */
#if CKPT_STRESS_TEST
/*
 * Test results, in persistent FRAM so they survive the power cuts.
 * Read them in the CCS Expressions view (connect with Load Symbols only):
 *   boots_checked  boots after the first save, i.e. power cuts survived
 *   restore_fail   boots where no valid checkpoint was found   -> must be 0
 *   content_bad    boots where a checkpoint passed its CRC but held
 *                  inconsistent data (a half-written slot)     -> must be 0
 *   saves_lost_max largest drop between the counter seen at a boot and the
 *                  last counter value saved before the cut     -> should be 0 or 1
 */
typedef struct
{
    uint16_t armed;
    uint32_t boots_checked;
    uint32_t restore_fail;
    uint32_t content_bad;
    uint32_t saves_lost_max;
    uint32_t last_saved;
} stress_result_t;

#pragma PERSISTENT(g_stress)
stress_result_t g_stress = { 0u, 0u, 0u, 0u, 0u, 0u };

#define STRESS_ARMED    (0x5A5Au)

static void stress_test(void)
{
    memset(&g_state, 0, sizeof g_state);
    bool ok = ckpt_restore(&g_state, sizeof g_state, NULL);

    if (g_stress.armed == STRESS_ARMED)
    {
        g_stress.boots_checked++;
        if (!ok)
        {
            g_stress.restore_fail++;
        }
        else if (!stress_consistent())
        {
            g_stress.content_bad++;
        }
        else if (g_stress.last_saved > g_state.samples)
        {
            /* last_saved is two 16-bit writes, so a cut can tear it too:
               only trust small differences */
            uint32_t lost = g_stress.last_saved - g_state.samples;
            if (lost < 1000u && lost > g_stress.saves_lost_max) { g_stress.saves_lost_max = lost; }
        }
    }

    bool failed = (g_stress.restore_fail != 0u) || (g_stress.content_bad != 0u);
    flash(!failed, 1u, failed ? 2000u : 300u, 0u);     /* red 2 s = a failure was recorded */

    uint32_t n = ok ? g_state.samples : 0u;
    for (;;)
    {
        n++;
        stress_fill(n);
        if (ckpt_save(&g_state, sizeof g_state))
        {
            g_stress.last_saved = n;
            g_stress.armed = STRESS_ARMED;
        }
        if ((n & 0x3FFu) == 0u) { board_led2_toggle(); }   /* alive: slow green blink */
    }
}
#endif

/* ------------------------------------------- fault injection (test 4) */
#if CKPT_FAULT_TEST
/*
 * Deterministic version of the power-cut test. Each boot runs one save of
 * counter base+1 that resets the chip (software BOR) at a chosen point
 * inside ckpt_save(), then the next boot checks what restore returns:
 *   steps 1-5  cut before the commit -> must restore base (old copy)
 *   step  6    cut after the commit  -> must restore base+1 (new copy)
 * Then, without resets:
 *   step  7    flip a payload bit in the newest slot -> must fall back to
 *              the older copy
 *   step  8    flip a header bit in the remaining slot -> restore must
 *              report failure, not return bad data
 *   step  9    a new save after losing both copies -> restores correctly
 *
 * Results (CCS Expressions view, Load Symbols): g_fault.state == 0xD0E5
 * when finished, passed == 0x03FE (bits 1..9), failed == 0.
 * LED while running: one 300 ms red flash per cut. When finished: green on =
 * all passed, red on = a step failed. Reflash to run it again.
 */
typedef struct
{
    uint16_t state;             /* FAULT_RUNNING / FAULT_DONE */
    uint16_t step;              /* step in progress or last step run */
    uint32_t base;              /* counter in the newest complete save */
    uint16_t passed;            /* bit n: step n passed */
    uint16_t failed;            /* bit n: step n failed */
    uint16_t restarts;          /* runs restarted after an unplanned reset */
} fault_result_t;

#pragma PERSISTENT(g_fault)
fault_result_t g_fault = { 0u, 0u, 0u, 0u, 0u, 0u };

#define FAULT_RUNNING   (0xFA17u)
#define FAULT_DONE      (0xD0E5u)
#define FAULT_LAST_CUT  ((uint16_t)CKPT_CUT_AFTER_COMMIT)
#define FAULT_ALL       (0x03FEu)

static void fault_fire(void)
{
    pmm_software_bor();                 /* full reset, like a power cut */
}

/* true if restore returns a complete, consistent copy of counter n */
static bool restored_is(uint32_t n)
{
    memset(&g_state, 0, sizeof g_state);
    return ckpt_restore(&g_state, sizeof g_state, NULL) &&
           stress_consistent() && g_state.samples == n;
}

static void fault_record(uint16_t step, bool ok)
{
    if (ok) { g_fault.passed |= (uint16_t)(1u << step); }
    else    { g_fault.failed |= (uint16_t)(1u << step); }
}

static void fault_start(void)
{
    ckpt_erase();
    stress_fill(1u);
    ckpt_save(&g_state, sizeof g_state);
    stress_fill(2u);
    ckpt_save(&g_state, sizeof g_state);   /* both slots valid, newest = 2 */
    g_fault.base = 2u;
    g_fault.step = 0u;
    g_fault.passed = 0u;
    g_fault.failed = 0u;
    g_fault.state = FAULT_RUNNING;
}

static void fault_test(void)
{
    if (g_fault.state == FAULT_RUNNING && g_fault.step != 0u)
    {
        if (!(pmm_reset_flags() & PMM_RST_SWBOR))
        {
            /* Reset/unplug not caused by the test: slot contents unknown */
            g_fault.restarts++;
            fault_start();
        }
        else
        {
            uint16_t s = g_fault.step;
            uint32_t expect = (s == FAULT_LAST_CUT) ? g_fault.base + 1u : g_fault.base;
            fault_record(s, restored_is(expect));
            g_fault.base = expect;
        }
    }
    else if (g_fault.state != FAULT_RUNNING && g_fault.state != FAULT_DONE)
    {
        fault_start();                  /* first boot after download */
    }

    while (g_fault.state == FAULT_RUNNING && g_fault.step < FAULT_LAST_CUT)
    {
        g_fault.step++;
        flash(true, 1u, 300u, 0u);                 /* visible: one per cut */
        stress_fill(g_fault.base + 1u);
        ckpt_fault_arm((ckpt_cut_t)g_fault.step, fault_fire);
        ckpt_save(&g_state, sizeof g_state);       /* resets the chip */

        /* Only reached if the cut point never fired */
        ckpt_fault_arm(CKPT_CUT_NONE, NULL);
        fault_record(g_fault.step, false);
        g_fault.base++;                            /* that save completed */
    }

    if (g_fault.state == FAULT_RUNNING)
    {
        uint32_t b = g_fault.base;

        g_fault.step = 7u;
        stress_fill(b + 1u);
        ckpt_save(&g_state, sizeof g_state);       /* newest b+1, older b */
        ckpt_test_corrupt(CKPT_CORRUPT_PAYLOAD);
        fault_record(7u, restored_is(b));

        g_fault.step = 8u;
        ckpt_test_corrupt(CKPT_CORRUPT_SEQ);       /* the b copy goes bad too */
        memset(&g_state, 0, sizeof g_state);
        fault_record(8u, !ckpt_restore(&g_state, sizeof g_state, NULL));

        g_fault.step = 9u;
        stress_fill(b + 2u);
        bool saved = ckpt_save(&g_state, sizeof g_state);
        fault_record(9u, saved && restored_is(b + 2u));

        g_fault.base = b + 2u;
        g_fault.state = FAULT_DONE;
    }

    bool pass = (g_fault.passed == FAULT_ALL) && (g_fault.failed == 0u);
    board_led1(!pass);
    board_led2(pass);
    for (;;) { pmm_enter_lpm(PMM_LPM4); }         /* LEDs keep their state */
}
#endif

/* -------------------------------------------------------------- main */

int main(void)
{
    board_init(before_unlock);

    const cs_config_t clk = CS_CONFIG_DEFAULT;      /* 8 MHz, LFXT on */
    cs_status_t cs = cs_init(&clk);

    s_ckpt_ok = crc16_self_test();
#if CKPT_STRESS_TEST
    if (s_ckpt_ok) { stress_test(); }               /* never returns */
    flash(true, 10u, 50u, 50u);                     /* CRC broken: cannot test */
#endif
#if CKPT_FAULT_TEST
    if (s_ckpt_ok) { fault_test(); }                /* never returns */
    flash(true, 10u, 50u, 50u);                     /* CRC broken: cannot test */
#endif
    if (!s_ckpt_ok || !ckpt_restore(&g_state, sizeof g_state, NULL))
    {
        state_reset();
    }
    hist_init();

    if (pmm_woke_from_lpmx5())
    {
        /* Minute check-in from low-energy mode: stay asleep unless recovered */
        s_clock = CLOCK_KEPT;
        uint16_t mv = 0;
        bool ok = adc_read_vcc_mv(&mv) == ADC_OK;
        hist_append(HIST_CHECKIN, 0u, now_s(), mv, HIST_NO_TEMP);
        if (!ok || mv < resume_mv())
        {
            g_state.deep_sleeps--;          /* same sleep episode, not a new one */
#if LPM35_TEST
            flash(false, 2u, 100u, 150u);   /* check-in: staying asleep */
#endif
            enter_deep_sleep();
        }
        hist_append(HIST_RESUME, 0u, now_s(), mv, HIST_NO_TEMP);
        lpm35_on_resume();
    }
    else if (s_rtc_kept)
    {
        s_clock = CLOCK_KEPT;
    }
    else
    {
        /*
         * Brown-out / power loss reset the RTC to 2026-01-01. Restart it
         * from the newest logged time (+1 s) so timestamps never run
         * backwards; it is then behind by however long the outage lasted.
         */
        uint32_t last = 0u, h;
        if (g_state.log_count) { last = g_state.log[(g_state.log_head + LOG_LEN - 1u) % LOG_LEN].t; }
        if (hist_newest_t(&h) && h > last) { last = h; }
        if (last)
        {
            rtc_time_t c;
            rtc_from_epoch(last + 1u + EPOCH_2026, &c);
            rtc_set_time(&c);
            s_clock = CLOCK_RESTORED;
        }
        else
        {
            s_clock = CLOCK_DEFAULT;
        }
    }

    pmm_boot_reason_t why = pmm_boot_reason();
    s_why = why;
    if (!pmm_woke_from_lpmx5())
    {
        uint16_t bmv = 0;
        (void)adc_read_vcc_mv(&bmv);
        hist_append(HIST_BOOT, (uint8_t)((uint8_t)why | (s_clock == CLOCK_RESTORED ? 0x80u : 0u)),
                    now_s(), bmv, HIST_NO_TEMP);
    }
    g_state.boots++;
    if (why == PMM_BOOT_POWER_ON) { g_state.power_losses++; }
    state_save();

    flash(true, (uint8_t)why + 1u, BOOT_FLASH_MS, 250u);
    if (cs == CS_LFXT_FAULT || !s_ckpt_ok)
    {
        /* RTC needs the crystal; checkpoints need the CRC: report and go on */
        board_led1(true);
        board_led2(true);
        timer_sleep_ms(TMR_SLEEP, 1000u);
        board_led1(false);
        board_led2(false);
    }

    /*
     * Buttons are not LPMx.5 wake sources here, so a flag left from the
     * wake path (e.g. set by the PxIES write) is stale: clear it, or S2
     * could erase the saved state on its own.
     */
    gpio_set_callback(BOARD_S1_PORT, BOARD_S1_PIN | BOARD_S2_PIN, on_button);
    gpio_interrupt_clear(BOARD_S1_PORT, BOARD_S1_PIN | BOARD_S2_PIN);
    gpio_interrupt_enable(BOARD_S1_PORT, BOARD_S1_PIN | BOARD_S2_PIN);

#if !CONSOLE_E10_TEST
    console_start(cs);
#endif

    wdt_start_watchdog(WDT_CLK_ACLK, WDT_DIV_512K);     /* 16 s at 32 kHz */
    s_sample_due = true;                                /* first sample now */

    for (;;)
    {
        if (s_sample_due)
        {
            s_sample_due = false;
            wdt_kick();
            bool new_pass = take_sample();
#if LPM35_TEST
            new_pass = true;            /* test build: flash every sample */
#endif
#if LED_AB_TEST
            {
                static uint16_t s_boot_samples;     /* RAM: counts from this boot */
                if (s_boot_samples < LED_AB_SAMPLES) { s_boot_samples++; new_pass = true; }
            }
#endif
#if CONSOLE_E10_TEST
            {
                static uint16_t s_e10;                  /* samples since boot */
                new_pass = false;                       /* no LED in this test */
                s_e10++;
                if (s_e10 == E10_PHASE_SAMPLES + 1u) { console_start(cs); }
                if (s_e10 == 2u * E10_PHASE_SAMPLES + 1u && s_con_up)
                {
                    s_stream = true;
                    con_puts("stream on (E10 phase 3)"); con_nl();
                    con_puts("t_s,time,vcc_mv,temp_c"); con_nl();
                }
            }
#endif
            if (new_pass) { flash(false, 1u, SAMPLE_FLASH_MS, 0u); }
            if (s_stream && s_con_up && g_state.log_count)
            {
                con_log_line(&g_state.log[(g_state.log_head + LOG_LEN - 1u) % LOG_LEN]);
            }
        }

        if (console_pending())
        {
            wdt_kick();
            console_poll();
        }

        if (s_s1_event)
        {
            s_s1_event = false;
            uint16_t mv = 0;
            if (adc_read_vcc_mv(&mv) == ADC_OK && mv > 2000u)
            {
                flash(false, (uint8_t)((mv - 2000u) / 100u), 20u, 200u);
            }
        }

        if (s_s2_event)
        {
            s_s2_event = false;
            erase_state();
            flash(true, 3u, 20u, 200u);
        }

        /* Sleep unless an event arrived while we were busy */
        uint16_t sr = __get_SR_register();
        __disable_interrupt();
        if (!s_sample_due && !s_s1_event && !s_s2_event && !console_pending())
        {
            pmm_enter_lpm(PMM_LPM3);        /* sets GIE atomically with LPM bits */
        }
        else if (sr & GIE)
        {
            __enable_interrupt();
        }
    }
}
