# Logger console protocol (firmware 2.0)

The logger's command console on the LaunchPad backchannel UART, as used by
people in a terminal (human mode) and by the dashboard (machine mode).

- Link: eUSCI_A0, "MSP Application UART1" COM port, **9600 8N1**, no flow control.
- Lines: commands end with CR, LF or CR LF; at most 48 characters; words
  separated by spaces.
- One command at a time: while a line is being executed, further input is
  dropped. The host must wait for the reply terminator (`@ok`/`@err`) before
  sending the next command.
- The console is off while the logger is in deep sleep (LPM3.5, VCC below the
  low threshold). Commands get no reply until VCC recovers and the logger
  boots again (an `@boot` line).

## Modes

| Mode | Set with | Echo | Prompt | Replies |
|---|---|---|---|---|
| human (default after every reset) | `mode h` | yes | `> ` | readable text |
| machine | `mode m` | no | none | tagged lines, below |

The mode is kept in RAM only: after any reset the console starts in human
mode. The boot banner always includes an `@boot` line, so a host that sees
`@boot` knows the device restarted and sends `mode m` again.

## Machine-mode line grammar

Every machine line starts with `@`, then a tag, then comma-separated fields.
No spaces, no quoting; text fields never contain commas. Lines end with CR LF.

```
line     = "@" tag *( "," field ) CRLF
field    = value / key "=" value
```

- Record-like lines (`@h`, `@s`, `@snap`) use **positional** fields.
- Status-like lines (`@boot`, `@status`, `@cfg`) use **key=value** fields;
  hosts must ignore unknown keys (new keys may be added).
- Every command reply ends with exactly one terminator:
  - `@ok,<cmd>[,<field>...]`
  - `@err,<cmd>,<code>[,<detail>]`
- Hosts must ignore lines that do not start with `@`: the human banner, the
  echo of the `mode m` command itself, and any prompt sent before machine
  mode was entered.
- Data lines of a reply come before its terminator. Asynchronous lines
  (`@s`, `@sleep`, `@boot`) are never printed inside a reply, but may appear
  between replies at any time.

### Units and sentinels

| Quantity | Unit | Missing value |
|---|---|---|
| `t_s` time | seconds since 2026-01-01 00:00:00 (device RTC, local time) | – |
| `vcc_mv` | millivolts, unsigned | `0` |
| `temp_dc` | tenths of a degree Celsius, signed | `-32768` |

Offsets (below) are already applied to every reported VCC and temperature.

## Asynchronous lines

| Line | When |
|---|---|
| `@boot,fw=2.0,why=<reason>,clock=<src>,boots=<n>` | in the banner after every start (both modes) |
| `@s,<t_s>,<vcc_mv>,<temp_dc>` | every 2 s sample while `stream on` |
| `@sleep,<t_s>,<vcc_mv>` | machine mode: just before the logger enters deep sleep and the console stops (human mode prints a text line instead) |

`why`: `power-on`, `lpm5-wake`, `reset-pin`, `watchdog`, `sw-por`, `sw-bor`,
`software`, `fault`. `clock`: `kept`, `restored`, `default`, `set`.

## Commands

| Command | Effect | Machine reply |
|---|---|---|
| `help` | list commands (human text in both modes) | text, then `@ok,help` |
| `mode [h\|m]` | show or set the mode | `@ok,mode,m` |
| `status` | device, clock, counters, supply, memory, settings | `@status,...` `@ok,status` |
| `log` | the 16-sample checkpoint log (human CSV text in both modes) | text, then `@ok,log` |
| `snap` | take one reading now (not stored) | `@snap,<t_s>,<vcc_mv>,<temp_dc>` `@ok,snap` |
| `time [YYYY-MM-DD hh:mm:ss]` | show or set the RTC; setting logs a `clock` record | `@ok,time,<t_s>` |
| `hist [n]` | newest n records (all if omitted), oldest first | `@h` lines, `@ok,hist,<sent>,0` |
| `hist since <seq> [max]` | records newer than `seq`, oldest first, at most `max` (default 100, 1..400) | `@h` lines, `@ok,hist,<sent>,<remaining>` |
| `stream [on\|off]` | print every 2 s sample (`@s`) | `@ok,stream,on` |
| `pause` | stop periodic history records (persistent) | `@ok,pause` |
| `resume` | restart periodic history records | `@ok,resume` |
| `interval [s]` | show or set the history interval, even, 2..3600 s (default 120) | `@ok,interval,<s>` |
| `mark [code]` | add a bookmark record, code 0..255 (default 0) | `@ok,mark,<seq>` |
| `offset` | show the sensor offsets | `@ok,offset,t=<dc>,v=<mv>` |
| `offset t <dc>` | temperature offset in 0.1 C, -100..100 | `@ok,offset,t=<dc>,v=<mv>` |
| `offset v <mv>` | VCC offset in mV, -200..200 | `@ok,offset,t=<dc>,v=<mv>` |
| `erase yes` | clear counters, 16-sample log and history (settings kept) | `@ok,erase` |
| `reset por\|bor` | software POR (RTC keeps running) or BOR (RTC cleared, restored from the log) | `@ok,reset,<por\|bor>`, then the device restarts |

Error codes: `unknown` (no such command), `args` (wrong arguments),
`range` (value out of range), `fail` (the device could not do it, e.g. an
ADC read failed).

Signed values are written as `-5`; the console accepts a leading `-` only
where noted (offsets).

### `@status` keys

| Key | Meaning |
|---|---|
| `fw` | firmware protocol version (`2.0`) |
| `t` | current RTC time, `t_s` |
| `clock` | where the time came from at this boot (`kept`, `restored`, `default`, `set`) |
| `boots`, `why` | boot counter and this boot's reason (as in `@boot`) |
| `plosses`, `sleeps`, `samples` | power-loss starts, deep-sleep entries, 2 s samples taken |
| `vcc`, `vmin` | VCC now and lowest seen, mV (0 = unknown) |
| `vlow`, `vresume` | deep-sleep threshold and resume threshold, mV |
| `temp` | die temperature now, 0.1 C |
| `ckpt` | 1 = checkpoints working, 0 = CRC self-test failed |
| `hcount`, `hlen` | valid history records, ring capacity (1600) |
| `hfirst`, `hlast` | sequence numbers of the oldest and newest record (0 if empty) |
| `interval`, `paused` | history interval in seconds, 1 if periodic records are paused |
| `toff`, `voff` | temperature offset (0.1 C) and VCC offset (mV) |
| `stream` | 1 if streaming |

## History records

```
@h,<seq>,<t_s>,<kind>,<arg>,<vcc_mv>,<temp_dc>
```

`seq` is a 16-bit sequence number that increases by one per record and wraps
after 65535; compare it wrap-safely (`(int16_t)(a - b) > 0`). The ring holds
the newest 1600 records.

| kind | Name | arg | vcc / temp |
|---|---|---|---|
| 1 | `sample` | 0 | reading |
| 2 | `checkin` | 0 | VCC at a deep-sleep minute wake, no temp |
| 3 | `sleep` | 0 | reading that triggered deep sleep |
| 4 | `resume` | 0 | VCC that ended deep sleep, no temp |
| 5 | `boot` | bits 0-3 reason (0 power-on, 1 LPMx.5 wake, 2 reset pin, 3 watchdog, 4 software, 5 fault); bit 6 = software BOR (else POR when reason is software); bit 7 = clock restored from the log | VCC at boot, no temp |
| 6 | `clock` | 0 | clock set; `t_s` is the new time; no readings |
| 7 | `pause` | 0 | reading when periodic logging paused |
| 8 | `unpause` | 0 | reading when it resumed |
| 9 | `mark` | bookmark code 0..255 | reading at the mark |
| 10 | `config` | 1 interval, 2 temp offset, 3 VCC offset | new value in `vcc_mv` (interval s) or as a signed 16-bit value in the field of its unit (offset t in `temp_dc`, offset v in `vcc_mv` read as int16); other field missing |

`hist` in human mode prints the same records as CSV with the header
`seq,t_s,time,event,detail,vcc_mv,temp_c` (the format of the report's
`*_hist.csv` files).

## Fetching the history

- **Full:** `status` gives `hfirst`; send `hist since <hfirst-1> 400`
  repeatedly, each time from the last `seq` received, until `remaining` is 0.
- **Incremental:** the same loop, starting from the newest `seq` the host
  already has.
- **Erased or replaced history:** `erase` restarts the sequence at 1. If
  `hlast` from `status` is behind the newest `seq` the host has (wrap-safe),
  the history was erased; start a full fetch instead of an incremental one.
- Chunks keep each reply short (400 records take about 14 s at 9600 baud).
  While a reply is being sent the logger does not sample; the 2 s samples
  missed are not made up (issue I-4 of the test report), so prefer small
  chunks while live data matters.

## Settings storage

`interval`, `pause`/`resume` and the two offsets are stored in FRAM with a
CRC and survive resets, power loss and `erase`. Each change is also logged
as a history record (`config`, `pause`, `unpause`). If the stored settings
are invalid (first boot after a download, or a write cut by power loss), the
defaults are used: interval 120 s, not paused, offsets 0.

The VCC offset also applies to the deep-sleep decision (2.2 V / 2.6 V),
since it is a calibration of the reading; the range is limited to ±200 mV
for that reason.
