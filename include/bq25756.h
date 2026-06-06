/**
 * @file bq25756.h
 * @brief Public API for the TI BQ25756 NVDC battery charger driver.
 *
 * The BQ25756 is a stand-alone, I²C-controlled 1S-28S synchronous switch-mode
 * battery charger with NVDC power-path topology, bidirectional (OTG /
 * reverse-boost) operation, an integrated 7-channel 15-bit ADC
 * (VBUS / IBUS / VBAT / IBAT / VSYS / TS / TDIE), JEITA temperature handling,
 * a watchdog, a safety-/charge-timer pair, and full fault reporting.
 *
 * This driver is intentionally bus-agnostic: register access goes through
 * three HAL callbacks supplied by the application
 * (`readReg`, `writeReg`, `delayMs`). The application wraps those around
 * whichever I²C master it has (ESP-IDF `i2c_master`, Zephyr `i2c`, STM32
 * HAL, bit-banger, etc.). The driver never touches GPIO directly.
 *
 * **Thread safety:** the driver is *not* thread-safe. Callers must serialise
 * access to a given ::BQ25756Device handle.
 *
 * API layering:
 *   1. Initialization / Lifecycle / Identity
 *   2. Charger configuration (CV, CC, pre-charge, termination, etc.)
 *   3. Input / system configuration (VINDPM, IINDPM, VBUS OVP, VSYS)
 *   4. Power-path control (charge enable, hi-z, OTG, ship-mode)
 *   5. Safety (watchdog kick, safety-timer, thermal-regulation)
 *   6. Status / fault decode
 *   7. ADC control + telemetry
 *   8. JEITA / NTC control
 *   9. Interrupt-mask programming
 *
 * Reference: TI BQ25756 datasheet, SLUSF42 (latest revision):
 *   https://www.ti.com/lit/ds/symlink/bq25756.pdf
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#pragma once
#ifndef BQ25756_H
#define BQ25756_H

#include "bq25756_registers.h"
#include "bq25756_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── 1. Initialization / Lifecycle / Identity ─────────────────────────────── */

/**
 * @brief Bind a HAL to a device handle and verify chip presence.
 *
 * Sequence:
 *   1. Validate args (NULL check on @p dev, @p hal, @p hal->readReg/writeReg).
 *   2. Copy HAL into the device handle and cache @p i2c_addr.
 *   3. Read REG_PART_INFO and confirm the PN nibble matches the expected
 *      ::BQ25756_PART_NUMBER_EXPECTED value. On mismatch return
 *      ::BQ25756_ERROR_BAD_ID; on bus failure return ::BQ25756_ERROR_COMM_FAIL.
 *   4. Mark the device initialised so subsequent calls can proceed.
 *
 * @param[in,out] dev      Device handle. Must not be NULL.
 * @param[in]     hal      HAL callbacks (deep-copied into @p dev).
 * @param[in]     i2c_addr 7-bit I²C address (typically 0x6B).
 *
 * @return ::BQ25756_ERROR_OK on success; ::BQ25756_ERROR_INVALID_PARAM on
 *         NULL args; ::BQ25756_ERROR_COMM_FAIL on bus failure;
 *         ::BQ25756_ERROR_BAD_ID if the part nibble doesn't match.
 */
BQ25756Error bq25756Init(BQ25756Device* const    dev,
                         const BQ25756HAL* const hal,
                         const uint8_t           i2c_addr);

/**
 * @brief Apply a full ::BQ25756Config bundle in one shot.
 *
 * Convenience wrapper that calls every individual setter in order. Returns
 * on the first non-OK code, leaving the chip half-configured — callers that
 * need atomic apply must keep their own rollback.
 */
BQ25756Error bq25756ApplyConfig(BQ25756Device* const dev,
                                const BQ25756Config* const config);

/**
 * @brief Issue a soft-reset (writes CHARGER_CTRL_3.RESET_REG). Self-clearing.
 *        Returns once the bit reads back as 0 (or ::BQ25756_ERROR_TIMEOUT).
 */
BQ25756Error bq25756Reset(BQ25756Device* const dev);

/**
 * @brief Read the cached PART_INFO byte captured at init.
 *
 * @param[out] part_info Filled in by the call; bits per datasheet.
 */
BQ25756Error bq25756GetPartInfo(const BQ25756Device* const dev,
                                uint8_t* const             part_info);

/* ── 2. Charger configuration ─────────────────────────────────────────────── */

/**
 * @brief Set the fast-charge constant-voltage target.
 *
 * @p mv is rounded down to the nearest ::BQ25756_VAC_CHG_LSB_MV. Out-of-range
 * values return ::BQ25756_ERROR_INVALID_PARAM.
 */
BQ25756Error bq25756SetChargeVoltage(const BQ25756Device* const dev,
                                     const uint32_t             mv);

/**
 * @brief Set the fast-charge constant-current target (mA).
 *
 * Rounds to ::BQ25756_ICHG_LSB_MA. Range-checked against
 * ::BQ25756_ICHG_MIN_MA / ::BQ25756_ICHG_MAX_MA.
 */
BQ25756Error bq25756SetChargeCurrent(const BQ25756Device* const dev,
                                     const uint32_t             ma);

/**
 * @brief Set the pre-charge constant current (mA). Rounds to LSB.
 */
BQ25756Error bq25756SetPrechargeCurrent(const BQ25756Device* const dev,
                                        const uint32_t             ma);

/**
 * @brief Set the charge-termination current (mA). Rounds to LSB.
 *
 * Charge-complete is declared when |IBAT| falls below this threshold while
 * still in the CV phase.
 */
BQ25756Error bq25756SetTerminationCurrent(const BQ25756Device* const dev,
                                          const uint32_t             ma);

/**
 * @brief Set the pre-charge to fast-charge transition voltage (mV).
 */
BQ25756Error bq25756SetPrechargeVoltage(const BQ25756Device* const dev,
                                        const uint32_t             mv);

/**
 * @brief Master enable / disable for the charge state machine.
 *
 * Read-modify-write on CHARGER_CTRL_1.EN_CHG. Does *not* clear faults.
 */
BQ25756Error bq25756SetChargeEnabled(const BQ25756Device* const dev,
                                     const bool                 enable);

/**
 * @brief Enable / disable charge-current termination detection.
 */
BQ25756Error bq25756SetTerminationEnabled(const BQ25756Device* const dev,
                                          const bool                 enable);

/* ── 3. Input / system configuration ──────────────────────────────────────── */

/**
 * @brief Set the input under-voltage regulation threshold (VINDPM) in mV.
 *
 * Below this threshold the chip will reduce ICHG to keep VBUS from sagging.
 */
BQ25756Error bq25756SetInputVoltageMin(const BQ25756Device* const dev,
                                       const uint32_t             mv);

/**
 * @brief Set the input-current limit (IINDPM) in mA.
 */
BQ25756Error bq25756SetInputCurrentMax(const BQ25756Device* const dev,
                                       const uint32_t             ma);

/**
 * @brief Set the VBUS over-voltage shutdown threshold (mV).
 */
BQ25756Error bq25756SetVbusOvp(const BQ25756Device* const dev,
                               const uint32_t             mv);

/**
 * @brief Set the VBAT over-voltage shutdown threshold (mV).
 */
BQ25756Error bq25756SetVbatOvp(const BQ25756Device* const dev,
                               const uint32_t             mv);

/**
 * @brief Set the minimum-VSYS regulation target (mV).
 *
 * Below this the charger will pull from the input even if the battery is
 * deeply discharged, keeping the system rail alive.
 */
BQ25756Error bq25756SetVsysMin(const BQ25756Device* const dev,
                               const uint32_t             mv);

/**
 * @brief Set the system voltage regulation target (mV). Used when the
 *        charger drives the system rail directly.
 */
BQ25756Error bq25756SetVsysReg(const BQ25756Device* const dev,
                               const uint32_t             mv);

/* ── 4. Power-path control ────────────────────────────────────────────────── */

/**
 * @brief Place the input in high-impedance (BFET-only, no input current).
 */
BQ25756Error bq25756SetHiZ(const BQ25756Device* const dev,
                           const bool                 enable);

/**
 * @brief Enable / disable OTG (reverse-boost) mode. Programme the OTG
 *        voltage and current limit *before* asserting this.
 */
BQ25756Error bq25756SetOtgEnabled(const BQ25756Device* const dev,
                                  const bool                 enable);

/**
 * @brief Set the OTG output regulation voltage (mV).
 */
BQ25756Error bq25756SetOtgVoltage(const BQ25756Device* const dev,
                                  const uint32_t             mv);

/**
 * @brief Set the OTG output current limit (mA).
 */
BQ25756Error bq25756SetOtgCurrent(const BQ25756Device* const dev,
                                  const uint32_t             ma);

/**
 * @brief Enter ship mode. Disconnects the battery FET; the chip will only
 *        wake on a /CE pin event or VBUS attach.
 */
BQ25756Error bq25756EnterShipMode(const BQ25756Device* const dev);

/* ── 5. Safety: watchdog & timers ─────────────────────────────────────────── */

/**
 * @brief Programme the watchdog period (or disable). When enabled, the host
 *        must call ::bq25756KickWatchdog within @p period or the chip will
 *        reset its configuration to defaults.
 */
BQ25756Error bq25756SetWatchdog(const BQ25756Device* const  dev,
                                const BQ25756WatchdogPeriod period);

/** @brief Pet the watchdog (writes TIMER_CTRL.WD_RESET, self-clearing). */
BQ25756Error bq25756KickWatchdog(const BQ25756Device* const dev);

/** @brief Enable / disable the charge-safety timer. */
BQ25756Error bq25756SetSafetyTimerEnabled(const BQ25756Device* const dev,
                                          const bool                 enable);

/** @brief Programme the safety-timer period. */
BQ25756Error bq25756SetSafetyTimer(const BQ25756Device* const dev,
                                   const BQ25756SafetyTimer   period);

/* ── 6. Status / fault decode ─────────────────────────────────────────────── */

/**
 * @brief Read CHARGER_STATUS_1..3 + NTC_STATUS into a decoded snapshot.
 */
BQ25756Error bq25756GetStatus(const BQ25756Device* const dev,
                              BQ25756Status* const       out);

/**
 * @brief Read FAULT_STATUS_1 / _2 and decode into an OR-mask plus a
 *        dominant-fault category for UI.
 */
BQ25756Error bq25756GetFaults(const BQ25756Device* const dev,
                              BQ25756Faults* const       out);

/**
 * @brief Clear latched fault flags by writing 1s back to the FAULT_FLAG
 *        registers. Does not affect the live FAULT_STATUS bits.
 */
BQ25756Error bq25756ClearFaultFlags(const BQ25756Device* const dev);

/* ── 7. ADC control + telemetry ───────────────────────────────────────────── */

/**
 * @brief Enable / disable the integrated ADC.
 */
BQ25756Error bq25756SetAdcEnabled(const BQ25756Device* const dev,
                                  const bool                 enable);

/**
 * @brief Pick continuous vs. one-shot conversion mode.
 */
BQ25756Error bq25756SetAdcMode(const BQ25756Device* const dev,
                               const BQ25756AdcMode       mode);

/**
 * @brief Set ADC resolution (15/14/13/12 bits). Lower resolution = faster.
 */
BQ25756Error bq25756SetAdcResolution(BQ25756Device* const       dev,
                                     const BQ25756AdcResolution res);

/**
 * @brief Trigger a single ADC conversion (one-shot mode only) and block
 *        @p timeout_ms while it completes.
 */
BQ25756Error bq25756TriggerAdcOneShot(const BQ25756Device* const dev,
                                      const uint32_t             timeout_ms);

/**
 * @brief Read an ADC channel in real-world units (mV / mA / m°C / m%).
 *
 * Channel → units mapping:
 *   VBUS, VBAT, VSYS → milli-volts.
 *   IBUS, IBAT       → milli-amps (IBAT is signed: + = charging).
 *   TS               → milli-percent of REGN (0 = 0 %, 100000 = 100 %).
 *   TDIE             → milli-degrees Celsius.
 */
BQ25756Error bq25756ReadAdc(const BQ25756Device* const dev,
                            const BQ25756AdcChannel    channel,
                            int32_t* const             out_value);

/* ── 8. JEITA / NTC control ───────────────────────────────────────────────── */

/**
 * @brief Enable / disable NTC monitoring and JEITA-region behaviour.
 */
BQ25756Error bq25756SetNtcEnabled(const BQ25756Device* const dev,
                                  const bool                 enable);

/**
 * @brief Programme the JEITA cool-region voltage de-rating action.
 */
BQ25756Error bq25756SetJeitaCoolAction(const BQ25756Device* const dev,
                                       const BQ25756JeitaVcool    action);

/**
 * @brief Programme the JEITA warm-region current de-rating action.
 */
BQ25756Error bq25756SetJeitaWarmAction(const BQ25756Device* const dev,
                                       const BQ25756JeitaIwarm    action);

/* ── 9. Interrupt-mask programming ────────────────────────────────────────── */

/**
 * @brief Mask all charger/fault interrupts (writes 0xFF to all four mask
 *        registers). Call this immediately after init if the host doesn't
 *        wire the /INT pin yet.
 */
BQ25756Error bq25756MaskAllInterrupts(const BQ25756Device* const dev);

/**
 * @brief Per-fault unmask. @p fault_bits is an OR of ::BQ25756FaultBits
 *        values; bits set will be *unmasked* (i.e. allowed to assert /INT).
 */
BQ25756Error bq25756SetFaultMask(const BQ25756Device* const dev,
                                 const uint32_t             fault_bits);

#ifdef __cplusplus
}
#endif

#endif /* BQ25756_H */
