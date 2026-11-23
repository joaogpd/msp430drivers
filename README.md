# msp430drivers

Register-level drivers and a power-loss-tolerant data logger for the
MSP-EXP430FR5994 LaunchPad, running from the board's 0.22 F supercapacitor.

- `drivers/msp430/` – MSP430 peripheral drivers: hal_common, gpio, cs, pmm,
  wdt, ref, adc, rtc, timer, crc, uart, and ckpt (two-slot FRAM checkpoint
  store)
- `drivers/board/` – LaunchPad bring-up and pin map
- `drivers/devices/` – drivers for external devices (none in this project;
  see epaper_display and cc1101_radio)
- `app/` – command console, the FRAM history log and the FRAM settings
- `docs/console_protocol.md` – the console commands and the machine-mode
  protocol used by the dashboard (msp430_dashboard)
- `main.c` – the logger, console commands and the test modes
- `report/` – test report (`lab_report.tex`/`.pdf`), the earlier paper-style
  write-up (`paper.tex`/`.pdf`) and the measurement data in `report/data/`

Toolchain: Code Composer Studio 12.5, TI MSP430 code generation tools
21.6.1.LTS. Import the folder in CCS with File → Import → CCS Projects.
