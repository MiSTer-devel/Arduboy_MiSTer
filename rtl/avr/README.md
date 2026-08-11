# ATmega32U4 module

A reusable ATmega32U4 CPU + peripheral implementation with real datasheet pin names, meant to be
dropped into other cores. Written for the Arduboy_MiSTer core originally; the Arduboy-specific
wiring lives outside this directory, in `../arduboy_board.v`, which is a worked example of how a
board instantiates this module — not part of the reusable bundle itself.

Full derivation, with datasheet citations, is in `projects/arduboy-atmega-module/PROJECT.md` in
the parent workspace. This file is the short version: what to connect and what to expect.

## Taking this module into a new core

Bring the whole directory, not individual files — `atmega32u4.v` has hard dependencies on every
peripheral file below it. `atmega.qip` is the manifest; reference it as one `QIP_FILE` the same
way `files.qip` does here, and it pulls in the complete implementation.

## Ports

Source of truth for every real pin identity: `Atmel-7766J-USB-ATmega16U4/32U4-Datasheet_04/2016`
(ATmega16U4/32U4 datasheet), Figure 1-1 (p.3) and §10.3 alternate-function tables (pp.74-83).

### Not real chip pins — FPGA-modeling artifacts

| Port | Direction | What it actually is |
|---|---|---|
| `clk`, `clk_pll` | input | MiSTer supplies the clock directly. Real hardware has `XTAL1`/`XTAL2` or the internal 8MHz oscillator; neither is modeled. |
| `rst` | input | Functionally corresponds to the real `RESET` pin, but modeled as a plain synchronous input, not the pin's real electrical behavior. |
| `pgm_addr`, `pgm_data` | output / input | How this Verilog model loads flash contents. No real chip pin. |

### Real chip pins, correctly named

| Port | Direction | Real pin | Alternate function(s) | Status |
|---|---|---|---|---|
| `PB1` | output | SCK | Hardware SPI clock | Implemented — SPI master |
| `PB2` | output | MOSI | Hardware SPI data out | Implemented — SPI master |
| `PB4` | input | PCINT4/ADC11 | — | Digital GPIO only |
| `PB5` | output | OC1A/PCINT5/ADC12 | Timer1 PWM, falls through to GPIO when disconnected | Implemented |
| `PB6` | output | OC1B/PCINT6/ADC13 | Timer1 PWM, falls through to GPIO when disconnected | Implemented |
| `PB7` | output | OC0A/OC1C/PCINT7/RTS | Timer0 PWM, falls through to GPIO when disconnected | Implemented |
| `PC6` | output | OC3A | Timer3 PWM, falls through to GPIO when disconnected | Implemented |
| `PC7` | output | ICP3/CLKO/OC4A | Timer4 PWM, falls through to GPIO when disconnected | Implemented |
| `PD2` | input | RXD1 | Hardware USART1 receive | Implemented |
| `PD3` | output | TXD1 | Hardware USART1 transmit | Implemented |
| `PD4` | output | ICP1/ADC8 | — | Digital GPIO only; ICP1/ADC8 alternate functions not implemented |
| `PE6` | input | INT6/AIN0 | — | Digital GPIO only |
| `PF4`–`PF7` | input | ADC4-7/JTAG (TCK/TMS/TDO/TDI) | — | Digital GPIO only |

**Not yet exposed at all:** `PB0` (SS), `PB3` (MISO — internal `spi_miso` wire exists but isn't a
port; SPI receive is disabled by default, `USE_RX("FALSE")`), and every other real pin not listed
above (`PD0`, `PD1`, `PD5`, `PD6`, `PD7`, `PF0`, `PF1`). Nothing wired to them yet, not because
they're inaccurate — they were simply never needed by the Arduboy board so far.

### Deliberately not real-pin-accurate — see Step 3, `projects/arduboy-atmega-module/PROJECT.md`

| Port | Direction | Why it's still here |
|---|---|---|
| `joystick_analog` | input | Feeds `unstable_counters`, a fake-ADC stand-in wired directly into the CPU's internal bus (`data_addr`/`data_read`), not a boundary pin. Cleanly separating it means either implementing a real ADC peripheral or exposing internal CPU-bus signals as ports. Deferred as its own future project — not done in this rename/reorganization phase. |
| `status` | input | Same as above — gates whether `joystick_analog` or floating-pin noise answers an ADC register read. |

## What's real hardware but not implemented at all

Confirmed by tracing every interrupt source in `atmega32u4.v` — these are permanently tied to `0`,
never fire, regardless of what a game does:

| Real peripheral | Evidence |
|---|---|
| USB 2.0 device controller | `int_usb_general = 0`, `int_usb_endpoint = 0`. No SIE/endpoint logic anywhere; no `D+`/`D-`/`VBUS`/`UCAP` ports. |
| ADC (12-channel, 10-bit) | `int_adc = 0`. `unstable_counters.v` fakes register reads, not a real converter. |
| TWI / I2C | `int_twi = 0`. No file implements it. |
| Analog Comparator | `int_analog_comp = 0`. Nothing found. |
| JTAG / on-chip debug | Nothing found. |

**Present but disabled, unlike the rest:** the Watchdog Timer — `` `define WATCHDOG_CNT_WIDTH 0``
(`atmega32u4.v`), with a real width (`27`) commented out right next to it. Most of the counter
logic already exists.

**Unverified either way:** SPI slave mode — a mode bit and status wire exist, but slave
shift-register behavior was never traced end to end.

The interrupt vector table is correctly sized for the real chip (`VECTOR_INT_TABLE_SIZE = 42`)
even where the peripheral behind a given vector doesn't exist yet.
