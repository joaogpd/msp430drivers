/*
 * console.h - Line-based command console on one or more byte ports
 *
 * A port is a byte stream with its own line buffer and mode: the UART
 * (every build) and the radio link (radio build). All ports share one
 * command table and behave the same; a command's replies go back to the
 * port its line came from.
 *
 * Input: the port's receiver hands bytes to console_rx() (the UART from
 * its RX interrupt, the radio link from main context). It echoes them,
 * handles backspace, and marks a line ready on Enter (CR or LF). The main
 * loop then calls console_poll(), which runs every waiting line: it splits
 * it into words and calls the matching command from the table.
 *
 * While a line waits for console_poll(), further input on that port is
 * dropped (and a BEL is echoed), so a command never sees a line that
 * changes under it.
 *
 * Machine mode (console_set_machine, per port): no echo, no BEL, no
 * prompt, and an unknown command is answered "@err,<cmd>,unknown".
 * Commands check console_machine() to print tagged replies
 * (docs/console_protocol.md). Attaching a port starts it in human mode.
 *
 * Output helpers (con_*) write to the current port (console_select; during
 * a command, the port it came from) through its blocking write function;
 * main context only. No printf: the helpers cover what the logger prints,
 * at a fraction of the code size.
 */
#ifndef APP_CONSOLE_H_
#define APP_CONSOLE_H_

#include <stdint.h>
#include <stdbool.h>

#define CONSOLE_LINE_MAX    (48u)
#define CONSOLE_ARGS_MAX    (6u)

typedef enum
{
    CON_PORT_UART = 0,
    CON_PORT_RADIO,
    CON_PORT_COUNT
} con_port_t;

typedef struct
{
    /* Queue output; may block (main context). Required. */
    void (*write)(const char *s, uint16_t n);
    /* Echo typed input; must not block (may run in an ISR). NULL: no echo. */
    void (*echo)(const char *s, uint16_t n);
    /* After each command on this port (e.g. send what was queued). May be NULL. */
    void (*done)(void);
    /* Wait until all queued output has left (e.g. before a reset). May be NULL. */
    void (*flush)(void);
} con_port_ops_t;

typedef void (*console_cmd_fn)(uint8_t argc, char *argv[]);

typedef struct
{
    const char    *name;
    const char    *args;        /* shown by help, e.g. "[on|off]"; may be "" */
    const char    *help;        /* one line */
    console_cmd_fn fn;
} console_cmd_t;

/* Set the command table (kept, not copied). Output goes to the UART port. */
void console_init(const console_cmd_t *cmds, uint8_t count);

/* Connect / disconnect a port. ops is copied. */
void console_attach(con_port_t port, const con_port_ops_t *ops);
void console_detach(con_port_t port);
bool console_attached(con_port_t port);

/* One received byte. Returns true when a line is ready. */
bool console_rx(con_port_t port, uint8_t byte, bool error);

/* true while a complete line waits for console_poll() on any port */
bool console_pending(void);

/* Main context: run every waiting line, each followed by its prompt. */
void console_poll(void);

/* Where con_* output goes outside of commands (async lines) */
void       console_select(con_port_t port);
con_port_t console_current(void);

/* Human (false, default) or machine (true) mode of the current port */
void console_set_machine(bool on);
bool console_machine(void);
bool console_port_machine(con_port_t port);

/* Wait until the current port's output has left (its flush op). */
void console_flush(void);

/* Print the command list (for a "help" command). */
void console_help(void);

/* Output helpers (main context), to the current port */
void con_puts(const char *s);
void con_putc(char c);
void con_nl(void);
void con_u32(uint32_t v);
void con_i32(int32_t v);
void con_u_pad(uint32_t v, uint8_t width);      /* zero-padded */
void con_dec1(int32_t tenths);                   /* 253 -> "25.3", -5 -> "-0.5" */
void con_hex16(uint16_t v);                      /* "0x1A2B" */
void con_prompt(void);

/* Parse an unsigned decimal; false on empty or non-digit text or > max */
bool con_parse_u32(const char *s, uint32_t max, uint32_t *out);

#endif /* APP_CONSOLE_H_ */
