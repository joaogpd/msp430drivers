/*
 * history.c - Long, power-loss-safe event log in FRAM
 */
#include "history.h"

#define HIST_MARK   (0xA5u)

#ifdef HIST_HOST_TEST
/* Host build for unit tests: plain arrays stand in for FRAM */
static hist_rec_t s_ring[HIST_LEN];
static volatile uint16_t s_hint;
hist_rec_t *hist_test_ring(void) { return s_ring; }
volatile uint16_t *hist_test_hint(void) { return &s_hint; }
#else
#pragma PERSISTENT(s_ring)
static hist_rec_t s_ring[HIST_LEN] = { { 0 } };

#pragma PERSISTENT(s_hint)
static volatile uint16_t s_hint = 0u;   /* index of the newest record */
#endif

static uint16_t s_newest;               /* index of the newest record */
static uint16_t s_next_seq;
static bool     s_empty;

static bool valid(uint16_t i)
{
    return s_ring[i].mark == HIST_MARK;
}

static uint16_t next_idx(uint16_t i) { return (uint16_t)((i + 1u) % HIST_LEN); }

/* Full scan: newest = valid record with the highest sequence */
static void scan(void)
{
    uint16_t i, ref = 0u;
    bool have = false;
    for (i = 0; i < HIST_LEN; i++)
    {
        if (!valid(i)) { continue; }
        if (!have || (int16_t)(s_ring[i].seq - s_ring[ref].seq) > 0) { ref = i; have = true; }
    }
    s_empty  = !have;
    s_newest = have ? ref : (uint16_t)(HIST_LEN - 1u);
    s_next_seq = have ? (uint16_t)(s_ring[ref].seq + 1u) : 1u;
}

void hist_init(void)
{
    uint16_t h = s_hint;
    if (h >= HIST_LEN || !valid(h))
    {
        scan();
        return;
    }
    /* Repair a hint that lags behind (power cut between commit and hint) */
    uint16_t steps;
    for (steps = 0; steps < HIST_LEN; steps++)
    {
        uint16_t n = next_idx(h);
        if (!valid(n) || s_ring[n].seq != (uint16_t)(s_ring[h].seq + 1u)) { break; }
        h = n;
    }
    s_empty = false;
    s_newest = h;
    s_next_seq = (uint16_t)(s_ring[h].seq + 1u);
}

void hist_append(hist_kind_t kind, uint8_t arg, uint32_t t, uint16_t vcc_mv, int16_t temp_dc)
{
    uint16_t i = s_empty ? 0u : next_idx(s_newest);
    /* volatile: the compiler keeps these FRAM writes in program order */
    volatile hist_rec_t *r = &s_ring[i];

    r->mark    = 0u;                    /* 1. invalidate */
    r->t       = t;                     /* 2. fill */
    r->seq     = s_next_seq;
    r->vcc_mv  = vcc_mv;
    r->temp_dc = temp_dc;
    r->kind    = (uint8_t)kind;
    r->arg     = arg;
    r->mark    = HIST_MARK;             /* 3. commit (volatile: stays last) */
    s_hint     = i;                     /* 4. hint, repaired at init if cut */

    s_newest = i;
    s_next_seq++;
    s_empty = false;
}

uint16_t hist_count(void)
{
    uint16_t i, n = 0u;
    for (i = 0; i < HIST_LEN; i++) { if (valid(i)) { n++; } }
    return n;
}

bool hist_get(uint16_t i, uint16_t count, hist_rec_t *out)
{
    if (s_empty || i >= count || count > HIST_LEN) { return false; }
    /* valid records form one run ending at s_newest */
    uint16_t idx = (uint16_t)((s_newest + HIST_LEN - (count - 1u) + i) % HIST_LEN);
    if (!valid(idx)) { return false; }
    out->t = s_ring[idx].t;
    out->seq = s_ring[idx].seq;
    out->vcc_mv = s_ring[idx].vcc_mv;
    out->temp_dc = s_ring[idx].temp_dc;
    out->kind = s_ring[idx].kind;
    out->arg = s_ring[idx].arg;
    out->mark = s_ring[idx].mark;
    return true;
}

bool hist_newest_t(uint32_t *t)
{
    if (s_empty) { return false; }
    *t = s_ring[s_newest].t;
    return true;
}

void hist_clear(void)
{
    uint16_t i;
    for (i = 0; i < HIST_LEN; i++) { s_ring[i].mark = 0u; }
    s_hint = 0u;
    s_empty = true;
    s_newest = (uint16_t)(HIST_LEN - 1u);
    s_next_seq = 1u;
}

const char *hist_kind_name(uint8_t kind)
{
    switch (kind)
    {
    case HIST_SAMPLE:    return "sample";
    case HIST_CHECKIN:   return "checkin";
    case HIST_SLEEP:     return "sleep";
    case HIST_RESUME:    return "resume";
    case HIST_BOOT:      return "boot";
    case HIST_CLOCK_SET: return "clock";
    default:             return "?";
    }
}
