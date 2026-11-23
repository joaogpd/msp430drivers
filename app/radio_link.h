/*
 * radio_link.h - Console over the CC1101 radio, duty-cycled (radio build)
 *
 * The radio is a second console port (console.h, CON_PORT_RADIO) with the
 * same commands and replies as the UART. To save energy the CC1101 sleeps
 * (SPWD, ~0.2 uA) except during short windows:
 *
 *   every radio period (settable, 0 = off) the logger wakes the CC1101,
 *   sends one or more BEACON packets (carrying any queued asynchronous
 *   text: @boot, @sleep, @s), then listens for RADIO_WINDOW_MS. A host
 *   that wants to talk sends a DATA packet with one command line inside
 *   that window. The logger ACKs it, runs the command, sends the reply as
 *   DATA packets (each ACKed by the host, retried up to RADIO_RETRIES
 *   times), and listens again for RADIO_WINDOW_MS. A window that passes
 *   with nothing addressed to the logger puts the CC1101 back to sleep.
 *
 * The packet format and the host's side are specified in
 * docs/console_protocol.md, section "Radio link".
 *
 * Energy: an empty window (wake, one short beacon, 300 ms of RX) costs
 * roughly 6 mC at 1.2 kBaud, i.e. ~0.1 mA averaged over a 60 s period,
 * which is about 30 times the logger's own drain. Choose the period with
 * the supercapacitor in mind (docs/console_protocol.md).
 *
 * Uses: eUSCI_A3 SPI and the pins in cc1101.h, TIMER_A2, the P1.3 (GDO0)
 * port interrupt. Main context only.
 */
#ifndef APP_RADIO_LINK_H_
#define APP_RADIO_LINK_H_

#include <stdint.h>
#include <stdbool.h>

#define RADIO_ADDR_HOST     (0x00u)
#define RADIO_ADDR_LOGGER   (0x01u)

/* Packet: [dst][src][ctrl][text...]; ctrl = type | seq */
#define RADIO_TYPE_DATA     (0x00u)
#define RADIO_TYPE_ACK      (0x40u)
#define RADIO_TYPE_BEACON   (0x80u)
#define RADIO_TYPE_MASK     (0xC0u)
#define RADIO_SEQ_MASK      (0x0Fu)
#define RADIO_HDR_LEN       (3u)
#define RADIO_TEXT_MAX      (58u)       /* CC1101_MAX_PAYLOAD - header */

#define RADIO_WINDOW_MS     (300u)      /* listen time after a beacon or reply */
#define RADIO_WINDOW_MAX_MS (1000u)     /* extra wait for a packet already arriving */
#define RADIO_ACK_MS        (250u)      /* wait for the host's ACK after a DATA packet */
#define RADIO_TX_MS         (1000u)     /* longest packet (72 B) is 480 ms at 1.2 kBaud */
#define RADIO_RETRIES       (3u)

/*
 * Every boot, before the I/O unlock (board_init's before_unlock hook): CSn
 * high so the CC1101 stays asleep, SO and GDOx as inputs.
 */
void radio_link_pins_safe(void);

/* Bring the chip up (reset, packet-mode configuration, check), then put
   it to sleep and attach the radio console port. false: chip not found. */
bool radio_link_init(void);
bool radio_link_ok(void);

/*
 * Call once per 2 s sample tick. Returns true when a window is due now
 * (period_s elapsed, radio idle): the caller may queue asynchronous text
 * (e.g. the latest @s line) with console_select(CON_PORT_RADIO) and then
 * calls radio_link_open(). period_s 0 = radio off.
 */
bool radio_link_due(uint16_t period_s);
void radio_link_open(void);

/* Main loop: handle a received packet or the end of the window. */
void radio_link_service(void);

/* true when radio_link_service() has an event to handle */
bool radio_link_pending(void);

/* true while the CC1101 is awake (a window is open) */
bool radio_link_awake(void);

/* Before deep sleep: send queued asynchronous text in beacons (no
   listening), put the chip to sleep, detach the port. */
void radio_link_shutdown(void);

#endif /* APP_RADIO_LINK_H_ */
