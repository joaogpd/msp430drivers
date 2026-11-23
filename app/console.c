/*
 * console.c - Line-based command console on one or more byte ports
 */
#include "console.h"
#include <string.h>

typedef struct
{
    con_port_ops_t   ops;
    bool             attached;
    volatile bool    machine;           /* no echo, no prompt, tagged replies */
    char             line[CONSOLE_LINE_MAX + 1u];
    volatile uint8_t len;
    volatile bool    ready;             /* a complete line waits for console_poll */
    bool             last_cr;           /* swallow the LF of a CR LF pair */
} port_t;

static const console_cmd_t *s_cmds;
static uint8_t              s_count;
static port_t               s_port[CON_PORT_COUNT];
static con_port_t           s_out;      /* where con_* output goes */

void console_init(const console_cmd_t *cmds, uint8_t count)
{
    s_cmds  = cmds;
    s_count = count;
    s_out   = CON_PORT_UART;
}

void console_attach(con_port_t port, const con_port_ops_t *ops)
{
    if ((unsigned)port >= CON_PORT_COUNT || !ops || !ops->write) { return; }
    port_t *p = &s_port[port];
    p->attached = false;                /* the RX side ignores it meanwhile */
    p->ops      = *ops;
    p->len      = 0u;
    p->ready    = false;
    p->last_cr  = false;
    p->machine  = false;                /* every attach starts in human mode */
    p->attached = true;
}

void console_detach(con_port_t port)
{
    if ((unsigned)port < CON_PORT_COUNT) { s_port[port].attached = false; }
}

bool console_attached(con_port_t port)
{
    return (unsigned)port < CON_PORT_COUNT && s_port[port].attached;
}

void console_select(con_port_t port)
{
    if ((unsigned)port < CON_PORT_COUNT) { s_out = port; }
}

con_port_t console_current(void)              { return s_out; }
void console_set_machine(bool on)             { s_port[s_out].machine = on; }
bool console_machine(void)                    { return s_port[s_out].machine; }
bool console_port_machine(con_port_t port)
{
    return (unsigned)port < CON_PORT_COUNT && s_port[port].machine;
}

static void echo(port_t *p, const char *s, uint16_t n)
{
    if (p->machine || !p->ops.echo) { return; }     /* the host does not want echo */
    p->ops.echo(s, n);                  /* non-blocking: may be ISR context */
}

bool console_rx(con_port_t port, uint8_t b, bool error)
{
    if ((unsigned)port >= CON_PORT_COUNT) { return false; }
    port_t *p = &s_port[port];
    if (!p->attached) { return false; }
    if (p->ready)
    {
        echo(p, "\a", 1u);              /* busy: line not consumed yet */
        return false;
    }
    if (error) { return false; }

    if (b == '\n' && p->last_cr) { p->last_cr = false; return false; }
    p->last_cr = (b == '\r');

    if (b == '\r' || b == '\n')
    {
        p->line[p->len] = '\0';
        echo(p, "\r\n", 2u);
        p->ready = true;
        return true;
    }
    if (b == 0x08u || b == 0x7Fu)       /* backspace / DEL */
    {
        if (p->len) { p->len--; echo(p, "\b \b", 3u); }
        return false;
    }
    if (b == 0x03u)                     /* Ctrl-C: drop the line */
    {
        p->len = 0u;
        echo(p, "^C\r\n> ", 6u);
        return false;
    }
    if (b < 0x20u || b > 0x7Eu) { return false; }
    if (p->len >= CONSOLE_LINE_MAX) { echo(p, "\a", 1u); return false; }

    p->line[p->len++] = (char)b;
    echo(p, (const char *)&b, 1u);
    return false;
}

bool console_pending(void)
{
    uint8_t i;
    for (i = 0u; i < CON_PORT_COUNT; i++)
    {
        if (s_port[i].attached && s_port[i].ready) { return true; }
    }
    return false;
}

static void run_line(port_t *p)
{
    char *argv[CONSOLE_ARGS_MAX];
    uint8_t argc = 0u;
    char *c = p->line;
    while (*c && argc < CONSOLE_ARGS_MAX)
    {
        while (*c == ' ') { *c++ = '\0'; }
        if (!*c) { break; }
        argv[argc++] = c;
        while (*c && *c != ' ') { c++; }
    }

    if (argc)
    {
        uint8_t i;
        bool found = false;
        for (i = 0; i < s_count; i++)
        {
            if (strcmp(argv[0], s_cmds[i].name) == 0)
            {
                s_cmds[i].fn(argc, argv);
                found = true;
                break;
            }
        }
        if (!found)
        {
            if (p->machine)
            {
                con_puts("@err,"); con_puts(argv[0]); con_puts(",unknown"); con_nl();
            }
            else
            {
                con_puts("unknown command '");
                con_puts(argv[0]);
                con_puts("', try help");
                con_nl();
            }
        }
    }
}

void console_poll(void)
{
    uint8_t i;
    for (i = 0u; i < CON_PORT_COUNT; i++)
    {
        port_t *p = &s_port[i];
        if (!p->attached || !p->ready) { continue; }

        con_port_t prev = s_out;
        s_out = (con_port_t)i;          /* replies go back where the line came from */
        run_line(p);
        p->len = 0u;
        p->ready = false;               /* accept input again */
        if (p->attached)                /* a command may have detached it */
        {
            con_prompt();
            if (p->ops.done) { p->ops.done(); }
        }
        s_out = prev;
    }
}

void console_flush(void)
{
    port_t *p = &s_port[s_out];
    if (p->attached && p->ops.flush) { p->ops.flush(); }
}

void console_help(void)
{
    uint8_t i;
    for (i = 0; i < s_count; i++)
    {
        uint8_t n = 0u;
        con_puts("  ");
        con_puts(s_cmds[i].name);
        n = (uint8_t)(strlen(s_cmds[i].name));
        if (s_cmds[i].args[0])
        {
            con_putc(' ');
            con_puts(s_cmds[i].args);
            n = (uint8_t)(n + 1u + strlen(s_cmds[i].args));
        }
        while (n++ < 22u) { con_putc(' '); }
        con_puts(s_cmds[i].help);
        con_nl();
    }
}

/* ------------------------------------------------------------- output */

void con_puts(const char *s)
{
    port_t *p = &s_port[s_out];
    if (p->attached) { p->ops.write(s, (uint16_t)strlen(s)); }
}

void con_putc(char c)
{
    port_t *p = &s_port[s_out];
    if (p->attached) { p->ops.write(&c, 1u); }
}

void con_nl(void)
{
    con_puts("\r\n");
}

void con_prompt(void)
{
    if (!s_port[s_out].machine) { con_puts("> "); }
}

void con_u32(uint32_t v)
{
    char b[11];
    uint8_t i = sizeof b;
    b[--i] = '\0';
    do { b[--i] = (char)('0' + v % 10u); v /= 10u; } while (v);
    con_puts(&b[i]);
}

void con_i32(int32_t v)
{
    if (v < 0) { con_putc('-'); con_u32((uint32_t)(-(v + 1)) + 1u); }
    else       { con_u32((uint32_t)v); }
}

void con_u_pad(uint32_t v, uint8_t width)
{
    char b[11];
    uint8_t i = sizeof b, n = 0u;
    b[--i] = '\0';
    do { b[--i] = (char)('0' + v % 10u); v /= 10u; n++; } while (v && i);
    while (n < width && i) { b[--i] = '0'; n++; }
    con_puts(&b[i]);
}

void con_dec1(int32_t tenths)
{
    uint32_t a;
    if (tenths < 0) { con_putc('-'); a = (uint32_t)(-(tenths + 1)) + 1u; }
    else            { a = (uint32_t)tenths; }
    con_u32(a / 10u);
    con_putc('.');
    con_putc((char)('0' + a % 10u));
}

void con_hex16(uint16_t v)
{
    static const char k_hex[] = "0123456789ABCDEF";
    char b[7] = { '0', 'x', 0, 0, 0, 0, '\0' };
    uint8_t i;
    for (i = 0; i < 4u; i++) { b[5u - i] = k_hex[v & 0xFu]; v >>= 4; }
    con_puts(b);
}

bool con_parse_u32(const char *s, uint32_t max, uint32_t *out)
{
    uint32_t v = 0u;
    if (!s || !*s) { return false; }
    for (; *s; s++)
    {
        if (*s < '0' || *s > '9') { return false; }
        uint32_t d = (uint32_t)(*s - '0');
        if (d > max || v > (max - d) / 10u) { return false; }
        v = v * 10u + d;
    }
    *out = v;
    return true;
}
