# BQ25756

Hardware-agnostic driver for the TI BQ25756 — a 1S-28S NVDC synchronous
switch-mode battery charger with bidirectional (OTG / reverse-boost)
operation, an integrated 7-channel ADC (VBUS / IBUS / VBAT / IBAT / VSYS /
TS / TDIE), JEITA temperature handling, watchdog, and safety/charge
timers.

## Layout

```
include/
  bq25756.h            public API (~280 LOC)
  bq25756_registers.h  register addresses, bit masks, LSB step sizes
  bq25756_types.h      enums, HAL struct, device handle, config struct
src/
  bq25756.c            implementation
CMakeLists.txt         tri-mode build (ESP-IDF / Zephyr / plain CMake)
```

## Integrating

1. Provide a `BQ25756HAL` with three callbacks:
   - `readReg(i2c_addr, reg, data, length)`
   - `writeReg(i2c_addr, reg, data, length)`
   - `delayMs(ms)`
   All three must return 0 on success.
2. Call `bq25756Init(&dev, &hal, BQ25756_I2C_ADDR_DEFAULT)`. The driver
   reads `PART_INFO` and verifies the PN nibble.
3. Build a `BQ25756Config` and pass it to `bq25756ApplyConfig()`, or call
   the individual setters one at a time.
4. Poll `bq25756GetStatus()` / `bq25756GetFaults()` and pet the watchdog
   with `bq25756KickWatchdog()` on whatever cadence you've programmed.

## BENCH-VERIFY

The driver was authored without access to the live TI SLUSF42 datasheet
(WebFetch was unavailable). Every register address, bit position, and
LSB step is annotated `BENCH-VERIFY` in `bq25756_registers.h`. On first
power-up: dump the 0x00..0x55 register map and diff against the printed
datasheet before trusting any setter in production.

## Reference

- TI BQ25756 datasheet (SLUSF42): https://www.ti.com/lit/ds/symlink/bq25756.pdf
