/*
 * settings.c - Logger settings kept in FRAM (interval, pause, offsets)
 */
#include "settings.h"
#include "../drivers/msp430/crc.h"

#define SET_MARK    (0x5Eu)

typedef struct
{
    settings_t s;
    uint16_t   crc;
    volatile uint8_t mark;
} stored_t;

#pragma PERSISTENT(s_stored)
static stored_t s_stored = { { 0u, 0u, 0u, 0, 0 }, 0u, 0u };

static settings_t s_cur;

static uint16_t crc_of(const settings_t *s)
{
    return crc16(s, (uint16_t)sizeof *s, 0xFFFFu);
}

static bool valid(const settings_t *s)
{
    return s->paused <= 1u && s->reserved == 0u &&
           s->interval_s >= SET_INTERVAL_MIN && s->interval_s <= SET_INTERVAL_MAX &&
           (s->interval_s & 1u) == 0u &&
           s->toff_dc >= -SET_TOFF_LIMIT && s->toff_dc <= SET_TOFF_LIMIT &&
           s->voff_mv >= -SET_VOFF_LIMIT && s->voff_mv <= SET_VOFF_LIMIT;
}

void settings_init(void)
{
    if (s_stored.mark == SET_MARK && s_stored.crc == crc_of(&s_stored.s) && valid(&s_stored.s))
    {
        s_cur = s_stored.s;
        return;
    }
    s_cur.interval_s = SET_INTERVAL_DEFAULT;
    s_cur.paused     = 0u;
    s_cur.reserved   = 0u;
    s_cur.toff_dc    = 0;
    s_cur.voff_mv    = 0;
}

const settings_t *settings(void)
{
    return &s_cur;
}

static void store(void)
{
    volatile stored_t *p = &s_stored;   /* volatile: FRAM writes stay in order */
    p->mark = 0u;                       /* 1. invalidate */
    p->s.interval_s = s_cur.interval_s; /* 2. fill */
    p->s.paused     = s_cur.paused;
    p->s.reserved   = 0u;
    p->s.toff_dc    = s_cur.toff_dc;
    p->s.voff_mv    = s_cur.voff_mv;
    p->crc  = crc_of(&s_cur);           /* 3. CRC */
    p->mark = SET_MARK;                 /* 4. commit */
}

bool settings_set_interval(uint16_t s)
{
    if (s < SET_INTERVAL_MIN || s > SET_INTERVAL_MAX || (s & 1u)) { return false; }
    s_cur.interval_s = s;
    store();
    return true;
}

void settings_set_paused(bool paused)
{
    s_cur.paused = paused ? 1u : 0u;
    store();
}

bool settings_set_toff(int16_t dc)
{
    if (dc < -SET_TOFF_LIMIT || dc > SET_TOFF_LIMIT) { return false; }
    s_cur.toff_dc = dc;
    store();
    return true;
}

bool settings_set_voff(int16_t mv)
{
    if (mv < -SET_VOFF_LIMIT || mv > SET_VOFF_LIMIT) { return false; }
    s_cur.voff_mv = mv;
    store();
    return true;
}
