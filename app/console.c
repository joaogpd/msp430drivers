/*
 * console.c - Line-based command console over a UART
 */
#include "console.h"
#include <string.h>

static uart_id_t            s_uart;
static const console_cmd_t *s_cmds;
static uint8_t              s_count;
static bool                 s_bound;
static volatile bool        s_machine;  /* no echo, no prompt, tagged replies */

static char          s_line[CONSOLE_LINE_MAX + 1u];
static volatile uint8_t s_len;
static volatile bool s_ready;
static bool          s_last_cr;     /* swallow the LF of a CR LF pair */

void console_set_machine(bool on) { s_machine = on; }
bool console_machine(void)        { return s_machine; }

void console_init(uart_id_t uart, const console_cmd_t *cmds, uint8_t count)
{
    s_uart  = uart;
    s_cmds  = cmds;
    s_count = count;
    s_len   = 0u;
    s_ready = false;
    s_last_cr = false;
    s_machine = false;                  /* every start is in human mode */
    s_bound = true;
}

static void echo(const char *s, uint16_t n)
{
    if (s_machine) { return; }          /* the host does not want echo */
    (void)uart_write(s_uart, s, n);     /* non-blocking: ISR context */
}

bool console_rx(uint8_t b, bool error)
{
    if (!s_bound) { return false; }
    if (s_ready)
    {
        echo("\a", 1u);                 /* busy: line not consumed yet */
        return false;
    }
    if (error) { return false; }

    if (b == '\n' && s_last_cr) { s_last_cr = false; return false; }
    s_last_cr = (b == '\r');

    if (b == '\r' || b == '\n')
    {
        s_line[s_len] = '\0';
        echo("\r\n", 2u);
        s_ready = true;
        return true;
    }
    if (b == 0x08u || b == 0x7Fu)       /* backspace / DEL */
    {
        if (s_len) { s_len--; echo("\b \b", 3u); }
        return false;
    }
    if (b == 0x03u)                     /* Ctrl-C: drop the line */
    {
        s_len = 0u;
        echo("^C\r\n> ", 6u);
        return false;
    }
    if (b < 0x20u || b > 0x7Eu) { return false; }
    if (s_len >= CONSOLE_LINE_MAX) { echo("\a", 1u); return false; }

    s_line[s_len++] = (char)b;
    echo((const char *)&b, 1u);
    return false;
}

bool console_pending(void)
{
    return s_ready;
}

void console_poll(void)
{
    if (!s_ready) { return; }

    char *argv[CONSOLE_ARGS_MAX];
    uint8_t argc = 0u;
    char *p = s_line;
    while (*p && argc < CONSOLE_ARGS_MAX)
    {
        while (*p == ' ') { *p++ = '\0'; }
        if (!*p) { break; }
        argv[argc++] = p;
        while (*p && *p != ' ') { p++; }
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
            if (s_machine)
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

    s_len = 0u;
    s_ready = false;                    /* accept input again */
    con_prompt();
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
    uart_write_all(s_uart, s, (uint16_t)strlen(s));
}

void con_putc(char c)
{
    uart_write_all(s_uart, &c, 1u);
}

void con_nl(void)
{
    con_puts("\r\n");
}

void con_prompt(void)
{
    if (!s_machine) { con_puts("> "); }
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
