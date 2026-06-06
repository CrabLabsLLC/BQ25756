/**
 * @file bq25756_types.h
 * @brief Types, enums, HAL, and config structs for the BQ25756 driver.
 *
 * Bus-agnostic: the platform supplies three callbacks (`readReg`, `writeReg`,
 * `delayMs`). Both register callbacks take the 7-bit I²C address so a single
 * HAL can drive multiple chargers on the same bus.
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#pragma once
#ifndef BQ25756_TYPES_H
#define BQ25756_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Error codes ──────────────────────────────────────────────────────────── */

typedef enum
{
	BQ25756_ERROR_OK              = 0,  ///< Success
	BQ25756_ERROR_INVALID_PARAM   = -1, ///< NULL pointer or out-of-range argument
	BQ25756_ERROR_COMM_FAIL       = -2, ///< I²C transaction reported failure
	BQ25756_ERROR_NOT_INITIALIZED = -3, ///< Driver not yet bound to a chip
	BQ25756_ERROR_BAD_ID          = -4, ///< PART_INFO returned an unexpected value
	BQ25756_ERROR_NOT_SUPPORTED   = -5, ///< Operation needs a feature the chip rejects
	BQ25756_ERROR_TIMEOUT         = -6, ///< Polling loop timed out
} BQ25756Error;

/* ── Charger state machine (CHARGER_STATUS_1[2:0]) ────────────────────────── */
/* Bench-verify the encoding on first power-up — write a dummy charge profile
 * and walk the bits while watching the state transition. */

typedef enum
{
	BQ25756_CHARGE_STATE_NOT_CHARGING = 0x0U,
	BQ25756_CHARGE_STATE_TRICKLE      = 0x1U,
	BQ25756_CHARGE_STATE_PRECHARGE    = 0x2U,
	BQ25756_CHARGE_STATE_FAST_CC      = 0x3U, ///< Fast-charge constant-current
	BQ25756_CHARGE_STATE_TAPER_CV     = 0x4U, ///< Constant-voltage taper
	BQ25756_CHARGE_STATE_TOPOFF       = 0x5U, ///< Top-off timer running
	BQ25756_CHARGE_STATE_DONE         = 0x6U, ///< Termination reached
	BQ25756_CHARGE_STATE_RESERVED     = 0x7U,
	BQ25756_CHARGE_STATE_UNKNOWN      = 0xFFU,
} BQ25756ChargeState;

/* ── Fault bits (OR-mask returned by bq25756GetFaults) ────────────────────── */

typedef enum
{
	BQ25756_FAULT_NONE        = 0U,
	BQ25756_FAULT_VBUS_OVP    = (1U << 0),
	BQ25756_FAULT_VBAT_OVP    = (1U << 1),
	BQ25756_FAULT_VSYS_OVP    = (1U << 2),
	BQ25756_FAULT_VSYS_SHORT  = (1U << 3),
	BQ25756_FAULT_TS_HOT      = (1U << 4),
	BQ25756_FAULT_TS_COLD     = (1U << 5),
	BQ25756_FAULT_TS_WARM     = (1U << 6),
	BQ25756_FAULT_TS_COOL     = (1U << 7),
	BQ25756_FAULT_IBAT_OCP    = (1U << 8),
	BQ25756_FAULT_TDIE_OTP    = (1U << 9),
	BQ25756_FAULT_WATCHDOG    = (1U << 10),
	BQ25756_FAULT_SAFETY      = (1U << 11),
	BQ25756_FAULT_OTG_OVP     = (1U << 12),
	BQ25756_FAULT_OTG_UVP     = (1U << 13),
	BQ25756_FAULT_CONV_OCP    = (1U << 14),
	BQ25756_FAULT_VAC_OVP     = (1U << 15),
} BQ25756FaultBits;

/** @brief Dominant-fault helper for quick UI / log lines. */
typedef enum
{
	BQ25756_DOMINANT_FAULT_NONE = 0,
	BQ25756_DOMINANT_FAULT_INPUT,      ///< VBUS / VAC / VBUS OVP
	BQ25756_DOMINANT_FAULT_BATTERY,    ///< VBAT OVP, IBAT OCP
	BQ25756_DOMINANT_FAULT_SYSTEM,     ///< VSYS short/OVP, conv OCP
	BQ25756_DOMINANT_FAULT_THERMAL,    ///< TDIE OTP or NTC hot/cold
	BQ25756_DOMINANT_FAULT_TIMER,      ///< Watchdog or safety
	BQ25756_DOMINANT_FAULT_OTG,        ///< OTG OVP/UVP
} BQ25756DominantFault;

/* ── ADC channel ──────────────────────────────────────────────────────────── */

typedef enum
{
	BQ25756_ADC_CHANNEL_VBUS = 0,  ///< Input voltage, milli-volts
	BQ25756_ADC_CHANNEL_IBUS,      ///< Input current, milli-amps
	BQ25756_ADC_CHANNEL_VBAT,      ///< Battery voltage, milli-volts
	BQ25756_ADC_CHANNEL_IBAT,      ///< Battery current, milli-amps (signed: + = charging)
	BQ25756_ADC_CHANNEL_VSYS,      ///< System voltage, milli-volts
	BQ25756_ADC_CHANNEL_TS,        ///< NTC ratio, milli-percent of REGN
	BQ25756_ADC_CHANNEL_TDIE,      ///< Die temperature, milli-degrees Celsius
	BQ25756_ADC_CHANNEL_COUNT,
} BQ25756ADCChannel;

/* ── ADC conversion mode ──────────────────────────────────────────────────── */

typedef enum
{
	BQ25756_ADC_MODE_CONTINUOUS = 0, ///< Free-running, host reads latest
	BQ25756_ADC_MODE_ONE_SHOT   = 1, ///< One conversion per write
} BQ25756ADCMode;

/* ── ADC resolution ───────────────────────────────────────────────────────── */

typedef enum
{
	BQ25756_ADC_RES_15_BIT = 0x0U,
	BQ25756_ADC_RES_14_BIT = 0x1U,
	BQ25756_ADC_RES_13_BIT = 0x2U,
	BQ25756_ADC_RES_12_BIT = 0x3U,
} BQ25756ADCResolution;

/* ── Watchdog period (TIMER_CTRL[1:0]) ────────────────────────────────────── */

typedef enum
{
	BQ25756_WATCHDOG_DISABLED = 0x0U,
	BQ25756_WATCHDOG_40_S     = 0x1U,
	BQ25756_WATCHDOG_80_S     = 0x2U,
	BQ25756_WATCHDOG_160_S    = 0x3U,
} BQ25756WatchdogPeriod;

/* ── Safety / charge-timer period (TIMER_CTRL[5:4]) ───────────────────────── */

typedef enum
{
	BQ25756_SAFETY_TIMER_5_H  = 0x0U,
	BQ25756_SAFETY_TIMER_8_H  = 0x1U,
	BQ25756_SAFETY_TIMER_12_H = 0x2U,
	BQ25756_SAFETY_TIMER_20_H = 0x3U,
} BQ25756SafetyTimer;

/* ── JEITA region (NTC_STATUS decode) ─────────────────────────────────────── */

typedef enum
{
	BQ25756_JEITA_NORMAL = 0,   ///< Between cool and warm
	BQ25756_JEITA_COOL,         ///< Cool: reduce charge voltage
	BQ25756_JEITA_WARM,         ///< Warm: reduce charge current
	BQ25756_JEITA_COLD,         ///< Cold: charging suspended
	BQ25756_JEITA_HOT,          ///< Hot: charging suspended
} BQ25756JEITARegion;

/* ── JEITA cool-region voltage de-rating ──────────────────────────────────── */

typedef enum
{
	BQ25756_JEITA_VCOOL_UNCHANGED  = 0x0U,
	BQ25756_JEITA_VCOOL_MINUS_100  = 0x1U, ///< -100 mV
	BQ25756_JEITA_VCOOL_MINUS_200  = 0x2U,
	BQ25756_JEITA_VCOOL_SUSPENDED  = 0x3U,
} BQ25756JEITAVCool;

/* ── JEITA warm-region current de-rating ──────────────────────────────────── */

typedef enum
{
	BQ25756_JEITA_IWARM_UNCHANGED  = 0x0U,
	BQ25756_JEITA_IWARM_HALF       = 0x1U, ///< 50 %
	BQ25756_JEITA_IWARM_QUARTER    = 0x2U, ///< 25 %
	BQ25756_JEITA_IWARM_SUSPENDED  = 0x3U,
} BQ25756JEITAIWarm;

/* ── Top-level charger status snapshot ────────────────────────────────────── */

typedef struct
{
	BQ25756ChargeState   state;             ///< Decoded state-machine code
	bool                 power_good;        ///< /PG asserted
	bool                 vbus_present;      ///< VBUS analog comparator
	bool                 ac_present;        ///< Adapter detected
	bool                 charging;          ///< In trickle/pre/CC/CV/topoff
	bool                 input_v_regulating; ///< VINDPM clamping
	bool                 input_i_regulating; ///< IINDPM clamping
	bool                 vsys_regulating;   ///< VSYS minimum regulating
	bool                 thermal_regulating;///< TJ regulation active
	BQ25756JEITARegion   jeita_region;
} BQ25756Status;

/* ── Fault snapshot ───────────────────────────────────────────────────────── */

typedef struct
{
	uint32_t              fault_mask;      ///< OR of ::BQ25756FaultBits
	BQ25756DominantFault  dominant;        ///< Highest-priority category for UI
} BQ25756Faults;

/* ── HAL ──────────────────────────────────────────────────────────────────── */

/**
 * @brief Platform-supplied I²C register access.
 *
 * Both callbacks return 0 on success, non-zero on any bus failure. The driver
 * never inspects negative return codes — only zero / non-zero.
 *
 * The chip auto-increments the register pointer for multi-byte transfers when
 * the host issues a single START — the platform layer must produce a faithful
 * "address + N bytes" transaction on each call.
 */
typedef struct
{
	/**
	 * @brief Read `length` bytes from `reg` on the chip at `i2c_addr`.
	 *
	 * Typical implementation: START + write(addr<<1) + write(reg) + RESTART +
	 * write((addr<<1)|1) + read(length) + STOP.
	 */
	int (*readReg)(uint8_t i2c_addr, uint8_t reg, uint8_t* data, uint8_t length);

	/**
	 * @brief Write `length` bytes to `reg` on the chip at `i2c_addr`.
	 */
	int (*writeReg)(uint8_t i2c_addr, uint8_t reg, const uint8_t* data, uint8_t length);

	/**
	 * @brief Block the calling thread for at least `delay_ms` milliseconds.
	 *
	 * Required for soft-reset and ADC one-shot conversions. May be NULL only
	 * if the caller guarantees those code paths are never exercised.
	 */
	void (*delayMs)(uint32_t delay_ms);
} BQ25756HAL;

/* ── Charger / regulation config bundle ──────────────────────────────────── */

typedef struct
{
	uint32_t              charge_voltage_mv;     ///< Fast-charge CV target
	uint32_t              charge_current_ma;     ///< Fast-charge CC target
	uint32_t              precharge_current_ma;  ///< Pre-charge constant current
	uint32_t              termination_current_ma;///< Charge-done threshold
	uint32_t              input_voltage_min_mv;  ///< VINDPM threshold
	uint32_t              input_current_max_ma;  ///< IINDPM threshold
	uint32_t              vbus_ovp_mv;           ///< VBUS OVP trip
	uint32_t              vbat_ovp_mv;           ///< VBAT OVP trip
	uint32_t              vsys_min_mv;           ///< Minimum system rail
	uint32_t              vsys_reg_mv;           ///< System regulation target
	BQ25756WatchdogPeriod watchdog;
	BQ25756SafetyTimer    safety_timer;
	bool                  enable_termination;
	bool                  enable_ico;
	bool                  enable_ntc;
} BQ25756Config;

/* ── Device handle (one per chip) ─────────────────────────────────────────── */

typedef struct
{
	BQ25756HAL          hal;
	uint8_t             i2c_addr;
	uint8_t             part_info;        ///< Latched at init for debug
	BQ25756ADCResolution adc_resolution;  ///< Cached for raw → unit conversion
	bool                is_initialized;
} BQ25756;

#ifdef __cplusplus
}
#endif

#endif /* BQ25756_TYPES_H */
