# msp430drivers

Register-level drivers and a power-loss-tolerant data logger for the
MSP-EXP430FR5994 LaunchPad, running from the board's 0.22 F supercapacitor.

- `drivers/` – Layer 1–3 drivers: gpio, board, cs, pmm, wdt, ref, adc, rtc,
  timer, crc, ckpt (two-slot FRAM checkpoint store), uart
- `app/` – command console and the FRAM history log
- `main.c` – the logger, console commands and the test modes
- `report/` – test report (`lab_report.tex`/`.pdf`), the earlier paper-style
  write-up (`paper.tex`/`.pdf`) and the measurement data in `report/data/`

Toolchain: Code Composer Studio 12.5, TI MSP430 code generation tools
21.6.1.LTS. Import the folder in CCS with File → Import → CCS Projects.
