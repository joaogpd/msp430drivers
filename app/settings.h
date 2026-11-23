/*
 * settings.h - Logger settings kept in FRAM (interval, pause, offsets)
 *
 * One persistent copy protected by a CRC16. It is written only when a
 * setting changes, in the same order as the history records: invalidate,
 * fill, CRC, then the valid mark last. A power cut during a write leaves
 * an invalid copy, and settings_init() then falls back to the defaults;
 * a setting may be lost that way, but never read half-written.
 *
 * The settings survive resets, power loss and "erase" (which clears the
 * data, not the configuration). A new download resets them to the
 * defaults, like every other persistent variable.
 */
#ifndef APP_SETTINGS_H_
#define APP_SETTINGS_H_

#include <stdint.h>
#include <stdbool.h>

#define SET_INTERVAL_DEFAULT    (120u)  /* history interval, s */
#define SET_INTERVAL_MIN        (2u)    /* the sample tick */
#define SET_INTERVAL_MAX        (3600u)
#define SET_TOFF_LIMIT          (100)   /* |temperature offset| <= 10.0 C */
#define SET_VOFF_LIMIT          (200)   /* |VCC offset| <= 200 mV */
#define SET_RADIO_DEFAULT       (60u)   /* radio window period, s (radio build) */
#define SET_RADIO_MIN           (4u)
#define SET_RADIO_MAX           (3600u)

typedef struct
{
    uint16_t interval_s;    /* history record interval, even, 2..3600 */
    uint8_t  paused;        /* 1: no periodic history records */
    uint8_t  reserved;      /* 0; keeps the struct free of padding for the CRC */
    int16_t  toff_dc;       /* added to every temperature, 0.1 C */
    int16_t  voff_mv;       /* added to every VCC reading, mV */
    uint16_t radio_period_s;/* radio window every n s, 0 = off (radio build) */
} settings_t;

/* Load the stored copy, or the defaults if it is invalid. Call once per boot. */
void settings_init(void);

/* The current settings (RAM copy). */
const settings_t *settings(void);

/* Change one setting; validates, stores, returns false if out of range. */
bool settings_set_interval(uint16_t s);
void settings_set_paused(bool paused);
bool settings_set_toff(int16_t dc);
bool settings_set_voff(int16_t mv);
bool settings_set_radio(uint16_t period_s);   /* 0, or even SET_RADIO_MIN..MAX */

#endif /* APP_SETTINGS_H_ */
