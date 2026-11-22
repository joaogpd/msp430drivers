/*
 * history.h - Long, power-loss-safe event log in FRAM
 *
 * A ring of HIST_LEN fixed-size records in persistent FRAM, kept apart
 * from the checkpoint store (whose payload is limited to 256 bytes).
 * The logger writes one record every HIST_EVERY_SAMPLES samples, plus one
 * at each boot, deep-sleep entry, minute check-in and resume, so a full
 * supercapacitor discharge fits in the ring.
 *
 * Power-loss safety: each record is invalidated first (mark = 0), then
 * filled, then committed by writing its one-byte mark last. A cut at any
 * point leaves either the old record or a record that is skipped as
 * invalid; a half-written record is never read as valid.
 * Records carry a 16-bit sequence number; the newest valid record is the
 * one with the highest sequence (compared wrap-safely, since HIST_LEN is
 * far below 32768). A persistent hint keeps boots from scanning the whole
 * ring; the hint is verified and repaired at every hist_init().
 */
#ifndef APP_HISTORY_H_
#define APP_HISTORY_H_

#include <stdint.h>
#include <stdbool.h>

#define HIST_LEN            (1600u)     /* 1600 x 14 B = 22.4 KB of FRAM */

typedef enum
{
    HIST_SAMPLE = 1,    /* periodic sample while logging */
    HIST_CHECKIN,       /* minute wake from LPM3.5 (VCC measured) */
    HIST_SLEEP,         /* entering LPM3.5 (VCC below the threshold) */
    HIST_RESUME,        /* leaving LPM3.5 (VCC recovered) */
    HIST_BOOT,          /* start other than an LPMx.5 wake; arg = boot reason */
    HIST_CLOCK_SET      /* clock set from the console; t = new time */
} hist_kind_t;

typedef struct
{
    uint32_t t;         /* seconds since 2026-01-01 */
    uint16_t seq;
    uint16_t vcc_mv;
    int16_t  temp_dc;   /* 0.1 C */
    uint8_t  kind;      /* hist_kind_t */
    uint8_t  arg;       /* kind-specific (boot reason, clock flags) */
    volatile uint8_t mark;
} hist_rec_t;

/* Find the newest record (fast via the hint). Call once per boot. */
void hist_init(void);

/* Append one record (main context). */
void hist_append(hist_kind_t kind, uint8_t arg, uint32_t t, uint16_t vcc_mv, int16_t temp_dc);

/* Number of valid records (scans the ring: ~1 ms, console use only). */
uint16_t hist_count(void);

/* Record i of hist_count(), oldest first. false if out of range/invalid. */
bool hist_get(uint16_t i, uint16_t count, hist_rec_t *out);

/* Time of the newest record; false if the history is empty. */
bool hist_newest_t(uint32_t *t);

/* Invalidate every record. */
void hist_clear(void);

const char *hist_kind_name(uint8_t kind);

#endif /* APP_HISTORY_H_ */
