/*
 * console.h - Line-based command console over a UART
 *
 * The UART receive interrupt hands bytes to console_rx(); it echoes them,
 * handles backspace, and marks a line ready on Enter (CR or LF). The main
 * loop then calls console_poll(), which splits the line into words and
 * runs the matching command from the table given to console_init().
 *
 * While a line waits for console_poll(), further input is dropped (and a
 * BEL is echoed), so a command never sees a line that changes under it.
 *
 * Output helpers (con_*) queue text on the UART and sleep while its buffer
 * is full; main context only. No printf: the helpers cover what the
 * logger prints, at a fraction of the code size.
 */
#ifndef APP_CONSOLE_H_
#define APP_CONSOLE_H_

#include <stdint.h>
#include <stdbool.h>
#include "../drivers/uart.h"

#define CONSOLE_LINE_MAX    (48u)
#define CONSOLE_ARGS_MAX    (6u)

typedef void (*console_cmd_fn)(uint8_t argc, char *argv[]);

typedef struct
{
    const char    *name;
    const char    *args;        /* shown by help, e.g. "[on|off]"; may be "" */
    const char    *help;        /* one line */
    console_cmd_fn fn;
} console_cmd_t;

/* Bind to a running UART and a command table (kept, not copied). */
void console_init(uart_id_t uart, const console_cmd_t *cmds, uint8_t count);

/* From the UART RX callback (ISR). Returns true when a line is ready. */
bool console_rx(uint8_t byte, bool error);

/* true while a complete line waits for console_poll() */
bool console_pending(void);

/* Main context: run a waiting line, then print the prompt. */
void console_poll(void);

/* Print the command list (for a "help" command). */
void console_help(void);

/* Output helpers (main context) */
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
