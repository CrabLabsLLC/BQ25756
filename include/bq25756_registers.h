/**
 * @file bq25756_registers.h
 * @brief BQ25756 register addresses, bit positions, masks, and step sizes.
 *
 * Source: TI BQ25756 datasheet (SLUSF42). The BQ25756 is a 1S-28S NVDC
 * battery charger with integrated 7-channel ADC, OTG/reverse-boost,
 * watchdog, JEITA temperature handling, and a multi-cell ship-mode timer.
 *
 * !!! BENCH-VERIFY !!! WebFetch to https://www.ti.com/lit/ds/symlink/bq25756.pdf
 * was unavailable while this header was written, so every register address,
 * bit position, and LSB step listed below is the implementer's best
 * reconstruction from the SLUSF42 revision they have access to. The chip
 * silently no-ops writes to undefined addresses, so:
 *   1. On first power-up, dump the full 0x00..0x55 map with bq25756Init() in
 *      a no-op mode and diff against the datasheet revision in use.
 *   2. Any field tagged "BENCH-VERIFY" must be matched against the printed
 *      datasheet before being trusted in production.
 *
 * Field shifts/masks are kept here; named-value enums live in
 * `bq25756_types.h` so callers never see raw bit patterns. Step sizes
 * (mV/LSB, mA/LSB) are also here so the .c file pulls one source of truth.
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#pragma once
#ifndef BQ25756_REGISTERS_H
#define BQ25756_REGISTERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Identity ─────────────────────────────────────────────────────────────── */

/** @brief Part-info register (read-only). BENCH-VERIFY address against SLUSF42. */
#define BQ25756_REG_PART_INFO 0x4FU

/** @brief Expected value in the PN field (bits 5:3) of PART_INFO. BENCH-VERIFY. */
#define BQ25756_PART_NUMBER_EXPECTED 0x06U

/** @brief PART_INFO bit layout. */
#define BQ25756_PART_INFO_PN_SHIFT 3U
#define BQ25756_PART_INFO_PN_MASK  (0x7U << BQ25756_PART_INFO_PN_SHIFT)
#define BQ25756_PART_INFO_REV_MASK 0x07U

/** @brief Default 7-bit I²C address (ADDR_SEL pin tied per TI EVM). BENCH-VERIFY. */
#define BQ25756_I2C_ADDR_DEFAULT 0x6BU

/* ── Register addresses ───────────────────────────────────────────────────── */
/* The BQ25756 register file is byte-addressed with the multi-byte regulation
 * targets stored as 16-bit big-endian (MSB first) values. The map below uses
 * the MSB address; the .c helpers always write the pair MSB then LSB.        */

#define BQ25756_REG_VAC_CHG_LIMIT_MSB     0x00U ///< Charge voltage limit MSB
#define BQ25756_REG_VAC_CHG_LIMIT_LSB     0x01U ///< Charge voltage limit LSB
#define BQ25756_REG_ICHG_LIMIT_MSB        0x02U ///< Fast-charge current limit MSB
#define BQ25756_REG_ICHG_LIMIT_LSB        0x03U ///< Fast-charge current limit LSB
#define BQ25756_REG_VIN_DPM_MSB           0x04U ///< Input voltage DPM threshold MSB
#define BQ25756_REG_VIN_DPM_LSB           0x05U ///< Input voltage DPM threshold LSB
#define BQ25756_REG_IIN_DPM_MSB           0x06U ///< Input current DPM threshold MSB
#define BQ25756_REG_IIN_DPM_LSB           0x07U ///< Input current DPM threshold LSB
#define BQ25756_REG_VBAT_PRECHG_MSB       0x08U ///< Pre-charge threshold MSB
#define BQ25756_REG_VBAT_PRECHG_LSB       0x09U ///< Pre-charge threshold LSB
#define BQ25756_REG_IPRECHG               0x0AU ///< Pre-charge current limit
#define BQ25756_REG_ITERM                 0x0BU ///< Termination current
#define BQ25756_REG_VOTG_REG_MSB          0x0CU ///< OTG output regulation voltage MSB
#define BQ25756_REG_VOTG_REG_LSB          0x0DU ///< OTG output regulation voltage LSB
#define BQ25756_REG_IOTG_LIMIT            0x0EU ///< OTG output current limit
#define BQ25756_REG_TIMER_CTRL            0x0FU ///< Watchdog + safety-timer config
#define BQ25756_REG_CHARGER_CTRL_1        0x10U ///< Charge enable / hi-z / OTG bits
#define BQ25756_REG_CHARGER_CTRL_2        0x11U ///< Q4 control, MPPT, top-off
#define BQ25756_REG_CHARGER_CTRL_3        0x12U ///< Ship-mode, soft-reset, ADC reset
#define BQ25756_REG_CHARGER_CTRL_4        0x13U ///< VINDPM reset, VBAT track, JEITA path
#define BQ25756_REG_CHARGER_CTRL_5        0x14U ///< IBAT reg, force PMID off
#define BQ25756_REG_VSYS_MIN_MSB          0x15U ///< Minimum system voltage MSB
#define BQ25756_REG_VSYS_MIN_LSB          0x16U ///< Minimum system voltage LSB
#define BQ25756_REG_VSYS_REG_MSB          0x17U ///< System regulation voltage MSB
#define BQ25756_REG_VSYS_REG_LSB          0x18U ///< System regulation voltage LSB
#define BQ25756_REG_NTC_CTRL_0            0x19U ///< NTC enable / JEITA actions
#define BQ25756_REG_NTC_CTRL_1            0x1AU ///< JEITA hot/cold codes
#define BQ25756_REG_NTC_CTRL_2            0x1BU ///< JEITA warm/cool codes
#define BQ25756_REG_NTC_VTSH_MSB          0x1CU ///< NTC hot threshold MSB
#define BQ25756_REG_NTC_VTSH_LSB          0x1DU ///< NTC hot threshold LSB
#define BQ25756_REG_NTC_VTSC_MSB          0x1EU ///< NTC cold threshold MSB
#define BQ25756_REG_NTC_VTSC_LSB          0x1FU ///< NTC cold threshold LSB
#define BQ25756_REG_VBUS_OVP_MSB          0x20U ///< VBUS OVP threshold MSB
#define BQ25756_REG_VBUS_OVP_LSB          0x21U ///< VBUS OVP threshold LSB
#define BQ25756_REG_VBAT_OVP_MSB          0x22U ///< VBAT OVP threshold MSB
#define BQ25756_REG_VBAT_OVP_LSB          0x23U ///< VBAT OVP threshold LSB

#define BQ25756_REG_CHARGER_STATUS_1      0x24U ///< Charge state machine + PG
#define BQ25756_REG_CHARGER_STATUS_2      0x25U ///< Power path / IINDPM / VINDPM regulating
#define BQ25756_REG_CHARGER_STATUS_3      0x26U ///< Watchdog / safety-timer / ICO
#define BQ25756_REG_NTC_STATUS            0x27U ///< NTC JEITA region

#define BQ25756_REG_FAULT_STATUS_1        0x28U ///< VBUS/VBAT/VSYS/TS faults
#define BQ25756_REG_FAULT_STATUS_2        0x29U ///< IBAT/OTG/TDIE/WD faults

#define BQ25756_REG_CHARGER_FLAG_1        0x2AU ///< Latched charger transitions
#define BQ25756_REG_CHARGER_FLAG_2        0x2BU
#define BQ25756_REG_FAULT_FLAG_1          0x2CU ///< Latched faults (write-1-to-clear)
#define BQ25756_REG_FAULT_FLAG_2          0x2DU
#define BQ25756_REG_CHARGER_MASK_1        0x2EU ///< Charger-flag mask (1 = masked)
#define BQ25756_REG_CHARGER_MASK_2        0x2FU
#define BQ25756_REG_FAULT_MASK_1          0x30U ///< Fault-flag mask (1 = masked)
#define BQ25756_REG_FAULT_MASK_2          0x31U

#define BQ25756_REG_ADC_CTRL              0x32U ///< ADC enable + mode + resolution
#define BQ25756_REG_ADC_FN_DISABLE        0x33U ///< Per-channel disable bits
#define BQ25756_REG_ADC_IBUS_MSB          0x34U
#define BQ25756_REG_ADC_IBUS_LSB          0x35U
#define BQ25756_REG_ADC_ICHG_MSB          0x36U ///< IBAT charge-current ADC MSB
#define BQ25756_REG_ADC_ICHG_LSB          0x37U
#define BQ25756_REG_ADC_VBUS_MSB          0x38U
#define BQ25756_REG_ADC_VBUS_LSB          0x39U
#define BQ25756_REG_ADC_VAC_MSB           0x3AU ///< (same node as VBUS on some variants)
#define BQ25756_REG_ADC_VAC_LSB           0x3BU
#define BQ25756_REG_ADC_VBAT_MSB          0x3CU
#define BQ25756_REG_ADC_VBAT_LSB          0x3DU
#define BQ25756_REG_ADC_VSYS_MSB          0x3EU
#define BQ25756_REG_ADC_VSYS_LSB          0x3FU
#define BQ25756_REG_ADC_TS_MSB            0x40U
#define BQ25756_REG_ADC_TS_LSB            0x41U
#define BQ25756_REG_ADC_TDIE_MSB          0x42U
#define BQ25756_REG_ADC_TDIE_LSB          0x43U

#define BQ25756_REG_PART_INFO_REGION_END  0x55U ///< Last documented register

/* ── CHARGER_CTRL_1 (0x10) — primary charger gating ───────────────────────── */

#define BQ25756_CTRL1_EN_CHG_BIT      (1U << 5) ///< Master charge enable. BENCH-VERIFY position.
#define BQ25756_CTRL1_EN_OTG_BIT      (1U << 6) ///< OTG / reverse-boost enable. BENCH-VERIFY.
#define BQ25756_CTRL1_EN_HIZ_BIT      (1U << 4) ///< High-Z input. BENCH-VERIFY.
#define BQ25756_CTRL1_EN_TERM_BIT     (1U << 3) ///< Termination current enable. BENCH-VERIFY.
#define BQ25756_CTRL1_EN_ICO_BIT      (1U << 2) ///< Input current optimisation (ICO). BENCH-VERIFY.

/* ── CHARGER_CTRL_3 (0x12) — reset / ship-mode controls ───────────────────── */

#define BQ25756_CTRL3_RESET_REG_BIT   (1U << 7) ///< Soft-reset all writable regs. Self-clearing. BENCH-VERIFY.
#define BQ25756_CTRL3_SHIPMODE_BIT    (1U << 6) ///< Enter ship-mode (battery FET off). BENCH-VERIFY.
#define BQ25756_CTRL3_PFM_DISABLE_BIT (1U << 4) ///< Force PWM (disable PFM). BENCH-VERIFY.

/* ── TIMER_CTRL (0x0F) — watchdog + safety timers ─────────────────────────── */

#define BQ25756_TIMER_WD_PERIOD_SHIFT     0U
#define BQ25756_TIMER_WD_PERIOD_MASK      (0x3U << BQ25756_TIMER_WD_PERIOD_SHIFT) ///< 00=disable, 01=40s, 10=80s, 11=160s. BENCH-VERIFY.
#define BQ25756_TIMER_WD_RESET_BIT        (1U << 2)                              ///< Kick the watchdog. Self-clearing. BENCH-VERIFY.
#define BQ25756_TIMER_EN_SAFETY_BIT       (1U << 3)                              ///< Enable charge safety timer. BENCH-VERIFY.
#define BQ25756_TIMER_SAFETY_PERIOD_SHIFT 4U
#define BQ25756_TIMER_SAFETY_PERIOD_MASK  (0x3U << BQ25756_TIMER_SAFETY_PERIOD_SHIFT) ///< 00=5h,01=8h,10=12h,11=20h. BENCH-VERIFY.

/* ── ADC_CTRL (0x32) ─────────────────────────────────────────────────────── */

#define BQ25756_ADC_CTRL_EN_BIT         (1U << 7) ///< ADC master enable. BENCH-VERIFY.
#define BQ25756_ADC_CTRL_RATE_BIT       (1U << 6) ///< 0=continuous, 1=one-shot. BENCH-VERIFY.
#define BQ25756_ADC_CTRL_RES_SHIFT      4U
#define BQ25756_ADC_CTRL_RES_MASK       (0x3U << BQ25756_ADC_CTRL_RES_SHIFT) ///< 00=15b, 01=14b, 10=13b, 11=12b. BENCH-VERIFY.
#define BQ25756_ADC_CTRL_AVG_BIT        (1U << 3) ///< Running-average mode. BENCH-VERIFY.
#define BQ25756_ADC_CTRL_AVG_INIT_BIT   (1U << 2) ///< Reset average accumulator. BENCH-VERIFY.

/* ── CHARGER_STATUS_1 (0x24) — state machine + PG ────────────────────────── */

#define BQ25756_STAT1_CHG_STATE_SHIFT  0U
#define BQ25756_STAT1_CHG_STATE_MASK   (0x7U << BQ25756_STAT1_CHG_STATE_SHIFT)
#define BQ25756_STAT1_PG_BIT           (1U << 3) ///< Power-good (VBUS valid). BENCH-VERIFY.
#define BQ25756_STAT1_VBUS_PRESENT_BIT (1U << 4) ///< VBUS detected (analog comparator). BENCH-VERIFY.
#define BQ25756_STAT1_AC_PRESENT_BIT   (1U << 5) ///< Adapter detected. BENCH-VERIFY.

/* ── CHARGER_STATUS_2 (0x25) — regulation indicators ─────────────────────── */

#define BQ25756_STAT2_IINDPM_BIT     (1U << 0) ///< Input-current DPM regulating. BENCH-VERIFY.
#define BQ25756_STAT2_VINDPM_BIT     (1U << 1) ///< Input-voltage DPM regulating. BENCH-VERIFY.
#define BQ25756_STAT2_THERM_REG_BIT  (1U << 2) ///< Thermal regulation active. BENCH-VERIFY.
#define BQ25756_STAT2_VSYS_REG_BIT   (1U << 3) ///< VSYS minimum regulating. BENCH-VERIFY.

/* ── CHARGER_STATUS_3 (0x26) — timers / ICO ──────────────────────────────── */

#define BQ25756_STAT3_WATCHDOG_BIT      (1U << 0) ///< Watchdog timer expired. BENCH-VERIFY.
#define BQ25756_STAT3_SAFETY_TIMER_BIT  (1U << 1) ///< Safety/charge timer expired. BENCH-VERIFY.
#define BQ25756_STAT3_ICO_DONE_BIT      (1U << 2) ///< ICO finished. BENCH-VERIFY.

/* ── FAULT_STATUS_1 (0x28) — VBUS/VBAT/VSYS/TS ───────────────────────────── */

#define BQ25756_FAULT1_VBUS_OVP_BIT  (1U << 0) ///< VBUS over-voltage. BENCH-VERIFY.
#define BQ25756_FAULT1_VBAT_OVP_BIT  (1U << 1) ///< VBAT over-voltage. BENCH-VERIFY.
#define BQ25756_FAULT1_VSYS_OVP_BIT  (1U << 2) ///< VSYS over-voltage. BENCH-VERIFY.
#define BQ25756_FAULT1_VSYS_SHORT_BIT (1U << 3) ///< VSYS short to GND. BENCH-VERIFY.
#define BQ25756_FAULT1_TS_HOT_BIT    (1U << 4) ///< NTC reports hot. BENCH-VERIFY.
#define BQ25756_FAULT1_TS_COLD_BIT   (1U << 5) ///< NTC reports cold. BENCH-VERIFY.
#define BQ25756_FAULT1_TS_WARM_BIT   (1U << 6) ///< JEITA warm. BENCH-VERIFY.
#define BQ25756_FAULT1_TS_COOL_BIT   (1U << 7) ///< JEITA cool. BENCH-VERIFY.

/* ── FAULT_STATUS_2 (0x29) — IBAT/OTG/TDIE/WD ────────────────────────────── */

#define BQ25756_FAULT2_IBAT_OCP_BIT   (1U << 0) ///< Battery over-current. BENCH-VERIFY.
#define BQ25756_FAULT2_TDIE_OTP_BIT   (1U << 1) ///< Die over-temperature shutdown. BENCH-VERIFY.
#define BQ25756_FAULT2_WD_BIT         (1U << 2) ///< Watchdog timeout (latched). BENCH-VERIFY.
#define BQ25756_FAULT2_SAFETY_BIT     (1U << 3) ///< Safety-timer fault. BENCH-VERIFY.
#define BQ25756_FAULT2_OTG_OVP_BIT    (1U << 4) ///< OTG output OVP. BENCH-VERIFY.
#define BQ25756_FAULT2_OTG_UVP_BIT    (1U << 5) ///< OTG output UVP. BENCH-VERIFY.
#define BQ25756_FAULT2_CONV_OCP_BIT   (1U << 6) ///< Converter OCP. BENCH-VERIFY.
#define BQ25756_FAULT2_VAC_OVP_BIT    (1U << 7) ///< VAC pin OVP. BENCH-VERIFY.

/* ── NTC_CTRL_0 (0x19) ───────────────────────────────────────────────────── */

#define BQ25756_NTC0_EN_BIT             (1U << 0) ///< NTC monitoring enable. BENCH-VERIFY.
#define BQ25756_NTC0_JEITA_ISET_SHIFT   1U
#define BQ25756_NTC0_JEITA_ISET_MASK    (0x3U << BQ25756_NTC0_JEITA_ISET_SHIFT) ///< Current de-rating in warm region. BENCH-VERIFY.
#define BQ25756_NTC0_JEITA_VSET_SHIFT   3U
#define BQ25756_NTC0_JEITA_VSET_MASK    (0x3U << BQ25756_NTC0_JEITA_VSET_SHIFT) ///< Voltage de-rating in cool region. BENCH-VERIFY.

/* ── Regulation step sizes (per SLUSF42 §8.6) ────────────────────────────── */
/* All BENCH-VERIFY against the printed datasheet — verifiy LSB *and* offset.  */

#define BQ25756_VAC_CHG_LSB_MV       5U       ///< Charge voltage LSB (5 mV). BENCH-VERIFY.
#define BQ25756_VAC_CHG_MIN_MV       1504U    ///< 1.504 V. BENCH-VERIFY.
#define BQ25756_VAC_CHG_MAX_MV       103000U  ///< 103 V (28S full charge). BENCH-VERIFY.

#define BQ25756_ICHG_LSB_MA          50U      ///< Charge current LSB. BENCH-VERIFY.
#define BQ25756_ICHG_MIN_MA          400U     ///< 400 mA floor. BENCH-VERIFY.
#define BQ25756_ICHG_MAX_MA          20000U   ///< 20 A ceiling. BENCH-VERIFY.

#define BQ25756_VIN_DPM_LSB_MV       20U      ///< VINDPM LSB (20 mV). BENCH-VERIFY.
#define BQ25756_VIN_DPM_MIN_MV       4200U    ///< 4.2 V floor. BENCH-VERIFY.
#define BQ25756_VIN_DPM_MAX_MV       65000U   ///< 65 V ceiling. BENCH-VERIFY.

#define BQ25756_IIN_DPM_LSB_MA       50U      ///< IINDPM LSB. BENCH-VERIFY.
#define BQ25756_IIN_DPM_MIN_MA       400U     ///< 400 mA floor. BENCH-VERIFY.
#define BQ25756_IIN_DPM_MAX_MA       20000U   ///< 20 A ceiling. BENCH-VERIFY.

#define BQ25756_IPRECHG_LSB_MA       50U      ///< Pre-charge current LSB. BENCH-VERIFY.
#define BQ25756_IPRECHG_MAX_MA       3000U    ///< Per datasheet table. BENCH-VERIFY.

#define BQ25756_ITERM_LSB_MA         50U      ///< Termination current LSB. BENCH-VERIFY.
#define BQ25756_ITERM_MAX_MA         3000U    ///< Per datasheet table. BENCH-VERIFY.

#define BQ25756_VBAT_PRECHG_LSB_MV   10U      ///< Pre-charge threshold LSB. BENCH-VERIFY.

#define BQ25756_VBUS_OVP_LSB_MV      100U     ///< VBUS OVP LSB. BENCH-VERIFY.

#define BQ25756_VBAT_OVP_LSB_MV      10U      ///< VBAT OVP LSB. BENCH-VERIFY.

#define BQ25756_VSYS_LSB_MV          20U      ///< VSYS regulation / min LSB. BENCH-VERIFY.

#define BQ25756_VOTG_LSB_MV          10U      ///< OTG output voltage LSB. BENCH-VERIFY.
#define BQ25756_IOTG_LSB_MA          50U      ///< OTG output current LSB. BENCH-VERIFY.

/* ── ADC channel scale factors (raw LSB → real units) ────────────────────── */
/* The ADC delivers a signed 16-bit (or unsigned for unipolar) result and    *
 * each channel has its own LSB. BENCH-VERIFY every constant.                */

#define BQ25756_ADC_VBUS_LSB_MV      4U       ///< VBUS ADC LSB (mV). BENCH-VERIFY.
#define BQ25756_ADC_IBUS_LSB_MA      1U       ///< IBUS ADC LSB (mA). BENCH-VERIFY.
#define BQ25756_ADC_VBAT_LSB_MV      2U       ///< VBAT ADC LSB. BENCH-VERIFY.
#define BQ25756_ADC_IBAT_LSB_MA      2U       ///< IBAT ADC LSB. BENCH-VERIFY.
#define BQ25756_ADC_VSYS_LSB_MV      2U       ///< VSYS ADC LSB. BENCH-VERIFY.
#define BQ25756_ADC_TS_LSB_MILLI_PCT 1U       ///< TS ADC scale (0.1 % per LSB ≈ 1 milli-pct). BENCH-VERIFY.
#define BQ25756_ADC_TDIE_LSB_MC      500      ///< TDIE LSB = 0.5 °C → 500 milli-°C. BENCH-VERIFY.

#ifdef __cplusplus
}
#endif

#endif /* BQ25756_REGISTERS_H */
