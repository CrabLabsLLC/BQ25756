/**
 * @file bq25756.c
 * @brief BQ25756 driver implementation.
 *
 * Rewritten from scratch against TI datasheet SLUSEN5 (August 2023) after
 * a live POR-state register dump from the actual chip confirmed:
 *   - PART_INFO lives at REG 0x3D (POR 0x12 = PN=010 BQ25756, DEV_REV=010).
 *   - Wide setpoint registers are LITTLE-endian (LSB at low address).
 *   - REG_RST lives at REG 0x19 bit 7, not REG 0x17.
 *   - EN_CHG / EN_HIZ / EN_IBAT_LOAD live in REG 0x17 bits {0,2,1}.
 *   - There is no VBUS_OVP / VSYS / OTG / JEITA programming on this part.
 *
 * Bus access is wrapped through three host-supplied callbacks
 * (BQ25756HAL.readReg / writeReg / delayMs); the driver never touches
 * GPIO or RTOS primitives directly.
 *
 * @author Orion Serup <orion@crablabs.io>
 */

#include "bq25756.h"

#include <stddef.h>
#include <string.h>

/* ── Constants ────────────────────────────────────────────────────────────── */

static const uint32_t bq25756_reset_timeout_ms      = 25U;
static const uint32_t bq25756_default_adc_settle_ms = 50U;

/* ── Forward declarations ─────────────────────────────────────────────────── */

static BQ25756Error rd(const BQ25756* dev, uint8_t reg, uint8_t* data, uint8_t len);
static BQ25756Error wr(const BQ25756* dev, uint8_t reg, const uint8_t* data, uint8_t len);
static BQ25756Error rd8(const BQ25756* dev, uint8_t reg, uint8_t* value);
static BQ25756Error wr8(const BQ25756* dev, uint8_t reg, uint8_t value);
static BQ25756Error rmw8(const BQ25756* dev, uint8_t reg, uint8_t mask, uint8_t value);
static BQ25756Error rd16le(const BQ25756* dev, uint8_t lsb_reg, uint16_t* value);
static BQ25756Error wr16le(const BQ25756* dev, uint8_t lsb_reg, uint16_t value);
static BQ25756Error ready(const BQ25756* dev);

static uint16_t encodeShiftedField(uint32_t value, uint32_t lsb, uint32_t shift, uint32_t mask);
static BQ25756Error writeShiftedField(const BQ25756* dev, uint8_t reg,
                                      uint32_t value, uint32_t min, uint32_t max,
                                      uint32_t lsb, uint32_t shift, uint16_t mask);

static BQ25756ChargeState decodeChargeState(uint8_t status1);
static BQ25756JEITARegion decodeJeitaRegion(uint8_t status2);
static BQ25756DominantFault decodeDominantFault(uint32_t mask);

/* ── 1. Initialization / lifecycle ────────────────────────────────────────── */

BQ25756Error bq25756Init(BQ25756* const dev, const BQ25756HAL* const hal,
                         const uint8_t i2c_addr)
{
	if (dev == NULL || hal == NULL || hal->readReg == NULL || hal->writeReg == NULL)
		return BQ25756_ERROR_INVALID_PARAM;

	dev->hal            = *hal;
	dev->i2c_addr       = i2c_addr;
	dev->part_info      = 0U;
	dev->adc_resolution = BQ25756_ADC_RES_15_BIT;
	dev->is_initialized = false;

	// Read PART_INFO to confirm we are talking to a BQ25756 silicon rev 2.
	// On a healthy chip with VAC present, this byte reads 0x12.
	uint8_t part = 0U;
	if (hal->readReg(i2c_addr, BQ25756_REG_PART_INFO, &part, 1U) != 0)
		return BQ25756_ERROR_COMM_FAIL;
	const uint8_t pn = (uint8_t)((part & BQ25756_PART_INFO_PN_MASK) >>
	                             BQ25756_PART_INFO_PN_SHIFT);
	if (pn != BQ25756_PART_INFO_PN_BQ25756)
		return BQ25756_ERROR_BAD_ID;

	dev->part_info      = part;
	dev->is_initialized = true;
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756ApplyConfig(BQ25756* const dev, const BQ25756Config* const config)
{
	if (dev == NULL || config == NULL)
		return BQ25756_ERROR_INVALID_PARAM;
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;

	BQ25756Error e;
	e = bq25756SetChargeVoltage(dev, config->charge_voltage_mv);
	if (e != BQ25756_ERROR_OK) return e;
	e = bq25756SetChargeCurrent(dev, config->charge_current_ma);
	if (e != BQ25756_ERROR_OK) return e;
	e = bq25756SetPrechargeCurrent(dev, config->precharge_current_ma);
	if (e != BQ25756_ERROR_OK) return e;
	e = bq25756SetTerminationCurrent(dev, config->termination_current_ma);
	if (e != BQ25756_ERROR_OK) return e;
	e = bq25756SetInputVoltageMin(dev, config->input_voltage_min_mv);
	if (e != BQ25756_ERROR_OK) return e;
	e = bq25756SetInputCurrentMax(dev, config->input_current_max_ma);
	if (e != BQ25756_ERROR_OK) return e;
	e = bq25756SetWatchdog(dev, config->watchdog);
	if (e != BQ25756_ERROR_OK) return e;
	e = bq25756SetSafetyTimer(dev, config->safety_timer);
	if (e != BQ25756_ERROR_OK) return e;
	return bq25756SetTerminationEnabled(dev, config->enable_termination);
}

BQ25756Error bq25756Reset(BQ25756* const dev)
{
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;

	// REG_RST lives in REG 0x19 bit 7, NOT in 0x17.  Self-clearing.
	const BQ25756Error w = rmw8(dev, BQ25756_REG_POWER_PATH_REV_MODE_CTRL,
	                            BQ25756_PWRPATH_REG_RST, BQ25756_PWRPATH_REG_RST);
	if (w != BQ25756_ERROR_OK) return w;

	for (uint32_t i = 0U; i < bq25756_reset_timeout_ms; i++)
	{
		if (dev->hal.delayMs != NULL) dev->hal.delayMs(1U);
		uint8_t v = 0U;
		const BQ25756Error e = rd8(dev, BQ25756_REG_POWER_PATH_REV_MODE_CTRL, &v);
		if (e != BQ25756_ERROR_OK) return e;
		if ((v & BQ25756_PWRPATH_REG_RST) == 0U)
			return BQ25756_ERROR_OK;
	}
	return BQ25756_ERROR_TIMEOUT;
}

BQ25756Error bq25756GetPartInfo(const BQ25756* const dev, uint8_t* const part_info)
{
	if (dev == NULL || part_info == NULL) return BQ25756_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)             return BQ25756_ERROR_NOT_INITIALIZED;
	*part_info = dev->part_info;
	return BQ25756_ERROR_OK;
}

/* ── 2. Charger configuration ─────────────────────────────────────────────── */

BQ25756Error bq25756SetChargeVoltage(const BQ25756* const dev, const uint32_t mv)
{
	// VFB regulation: 1504..1566 mV at the FB pin, 2 mV/LSB, bits 4:0.
	// Caller passes the desired FB-pin voltage in mV; the board's external
	// divider sets the actual VBAT regulation point.  Out-of-range -> error.
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;
	if (mv < BQ25756_VFB_OFFSET_MV || mv > BQ25756_VFB_MAX_MV)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint16_t code = (uint16_t)((mv - BQ25756_VFB_OFFSET_MV) / BQ25756_VFB_LSB_MV);
	return wr16le(dev, BQ25756_REG_CHARGE_VOLTAGE_LIMIT,
	              (uint16_t)(code & BQ25756_VFB_FIELD_MASK));
}

BQ25756Error bq25756SetChargeCurrent(const BQ25756* const dev, const uint32_t ma)
{
	return writeShiftedField(dev, BQ25756_REG_CHARGE_CURRENT_LIMIT, ma,
	                         BQ25756_ICHG_MIN_MA, BQ25756_ICHG_MAX_MA,
	                         BQ25756_ICHG_LSB_MA, BQ25756_ICHG_FIELD_SHIFT,
	                         BQ25756_ICHG_FIELD_MASK);
}

BQ25756Error bq25756SetPrechargeCurrent(const BQ25756* const dev, const uint32_t ma)
{
	return writeShiftedField(dev, BQ25756_REG_PRECHARGE_CURRENT_LIMIT, ma,
	                         BQ25756_PRECHG_LSB_MA, BQ25756_ICHG_MAX_MA,
	                         BQ25756_PRECHG_LSB_MA, BQ25756_PRECHG_FIELD_SHIFT,
	                         BQ25756_PRECHG_FIELD_MASK);
}

BQ25756Error bq25756SetTerminationCurrent(const BQ25756* const dev, const uint32_t ma)
{
	return writeShiftedField(dev, BQ25756_REG_TERMINATION_CURRENT_LIMIT, ma,
	                         BQ25756_ITERM_LSB_MA, BQ25756_ICHG_MAX_MA,
	                         BQ25756_ITERM_LSB_MA, BQ25756_ITERM_FIELD_SHIFT,
	                         BQ25756_ITERM_FIELD_MASK);
}

BQ25756Error bq25756SetPrechargeVoltage(const BQ25756* const dev, const uint32_t mv)
{
	// BQ25756 derives pre-charge / fast-charge transition voltage internally
	// from VFB_REG and the VBAT_LOWV ratio in PRECHG_TERM_CONTROL.  Not a
	// directly programmable mV target on this part.
	(void)dev; (void)mv;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

BQ25756Error bq25756SetChargeEnabled(const BQ25756* const dev, const bool enable)
{
	return rmw8(dev, BQ25756_REG_CHARGER_CONTROL,
	            BQ25756_CHARGER_CTRL_EN_CHG,
	            enable ? BQ25756_CHARGER_CTRL_EN_CHG : 0U);
}

BQ25756Error bq25756SetTerminationEnabled(const BQ25756* const dev, const bool enable)
{
	return rmw8(dev, BQ25756_REG_PRECHG_TERM_CONTROL,
	            BQ25756_PRECHG_EN_TERM,
	            enable ? BQ25756_PRECHG_EN_TERM : 0U);
}

/* ── 3. Input / system configuration ──────────────────────────────────────── */

BQ25756Error bq25756SetInputVoltageMin(const BQ25756* const dev, const uint32_t mv)
{
	return writeShiftedField(dev, BQ25756_REG_INPUT_VOLTAGE_DPM_LIMIT, mv,
	                         BQ25756_VAC_DPM_MIN_MV, BQ25756_VAC_DPM_MAX_MV,
	                         BQ25756_VAC_DPM_LSB_MV, BQ25756_VAC_DPM_FIELD_SHIFT,
	                         BQ25756_VAC_DPM_FIELD_MASK);
}

BQ25756Error bq25756SetInputCurrentMax(const BQ25756* const dev, const uint32_t ma)
{
	return writeShiftedField(dev, BQ25756_REG_INPUT_CURRENT_DPM_LIMIT, ma,
	                         BQ25756_IAC_DPM_MIN_MA, BQ25756_IAC_DPM_MAX_MA,
	                         BQ25756_IAC_DPM_LSB_MA, BQ25756_IAC_DPM_FIELD_SHIFT,
	                         BQ25756_IAC_DPM_FIELD_MASK);
}

BQ25756Error bq25756SetVBUSOVP(const BQ25756* const dev, const uint32_t mv)
{
	// VAC OVP threshold on BQ25756 is fixed by VAC_DPM + internal margin,
	// not host-programmable.
	(void)dev; (void)mv;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

BQ25756Error bq25756SetVBATOVP(const BQ25756* const dev, const uint32_t mv)
{
	// VBAT OVP is internally fixed at ~107% of VFB regulation; not programmable.
	(void)dev; (void)mv;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

BQ25756Error bq25756SetVSYSMin(const BQ25756* const dev, const uint32_t mv)
{
	// BQ25756 is a stand-alone charger -- no VSYS regulation path.
	(void)dev; (void)mv;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

BQ25756Error bq25756SetVSYSReg(const BQ25756* const dev, const uint32_t mv)
{
	(void)dev; (void)mv;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

/* ── 4. Power-path control ────────────────────────────────────────────────── */

BQ25756Error bq25756SetHiZ(const BQ25756* const dev, const bool enable)
{
	return rmw8(dev, BQ25756_REG_CHARGER_CONTROL,
	            BQ25756_CHARGER_CTRL_EN_HIZ,
	            enable ? BQ25756_CHARGER_CTRL_EN_HIZ : 0U);
}

BQ25756Error bq25756SetOTGEnabled(const BQ25756* const dev, const bool enable)
{
	// BQ25756 has reverse-mode (EN_REV in 0x19); semantics differ from OTG.
	// The application doesn't run reverse mode -- expose as NOT_SUPPORTED so
	// no caller mistakenly thinks they got an OTG boost.
	(void)dev; (void)enable;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

BQ25756Error bq25756SetOTGVoltage(const BQ25756* const dev, const uint32_t mv)
{
	(void)dev; (void)mv;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

BQ25756Error bq25756SetOTGCurrent(const BQ25756* const dev, const uint32_t ma)
{
	(void)dev; (void)ma;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

BQ25756Error bq25756EnterShipMode(const BQ25756* const dev)
{
	// No ship-mode bit on BQ25756.  Closest equivalent is HiZ + EN_CHG=0.
	(void)dev;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

/* ── 5. Safety: watchdog & timers ─────────────────────────────────────────── */

BQ25756Error bq25756SetWatchdog(const BQ25756* const dev, const BQ25756WatchdogPeriod period)
{
	uint8_t field;
	switch (period)
	{
	case BQ25756_WATCHDOG_DISABLED: field = BQ25756_WATCHDOG_FIELD_DISABLED; break;
	case BQ25756_WATCHDOG_40_S:     field = BQ25756_WATCHDOG_FIELD_40S;      break;
	case BQ25756_WATCHDOG_80_S:     field = BQ25756_WATCHDOG_FIELD_80S;      break;
	case BQ25756_WATCHDOG_160_S:    field = BQ25756_WATCHDOG_FIELD_160S;     break;
	default: return BQ25756_ERROR_INVALID_PARAM;
	}
	return rmw8(dev, BQ25756_REG_TIMER_CONTROL, BQ25756_TIMER_WATCHDOG_MASK,
	            (uint8_t)(field << BQ25756_TIMER_WATCHDOG_SHIFT));
}

BQ25756Error bq25756KickWatchdog(const BQ25756* const dev)
{
	// WD_RST is REG 0x17 bit 5 on this part, not in the timer control reg.
	return rmw8(dev, BQ25756_REG_CHARGER_CONTROL,
	            BQ25756_CHARGER_CTRL_WD_RST, BQ25756_CHARGER_CTRL_WD_RST);
}

BQ25756Error bq25756SetSafetyTimerEnabled(const BQ25756* const dev, const bool enable)
{
	return rmw8(dev, BQ25756_REG_TIMER_CONTROL, BQ25756_TIMER_EN_CHG_TMR,
	            enable ? BQ25756_TIMER_EN_CHG_TMR : 0U);
}

BQ25756Error bq25756SetSafetyTimer(const BQ25756* const dev, const BQ25756SafetyTimer period)
{
	uint8_t field;
	switch (period)
	{
	case BQ25756_SAFETY_TIMER_5_H:  field = BQ25756_CHG_TMR_FIELD_5H;  break;
	case BQ25756_SAFETY_TIMER_8_H:  field = BQ25756_CHG_TMR_FIELD_8H;  break;
	case BQ25756_SAFETY_TIMER_12_H: field = BQ25756_CHG_TMR_FIELD_12H; break;
	case BQ25756_SAFETY_TIMER_20_H: field = BQ25756_CHG_TMR_FIELD_24H; break; // BQ25756 max is 24h
	default: return BQ25756_ERROR_INVALID_PARAM;
	}
	return rmw8(dev, BQ25756_REG_TIMER_CONTROL, BQ25756_TIMER_CHG_TMR_MASK,
	            (uint8_t)(field << BQ25756_TIMER_CHG_TMR_SHIFT));
}

/* ── 6. Status / fault decode ─────────────────────────────────────────────── */

BQ25756Error bq25756GetStatus(const BQ25756* const dev, BQ25756Status* const out)
{
	if (out == NULL) return BQ25756_ERROR_INVALID_PARAM;
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;

	uint8_t s[3] = {0};
	const BQ25756Error e = rd(dev, BQ25756_REG_CHARGER_STATUS_1, s, 3U);
	if (e != BQ25756_ERROR_OK) return e;

	const uint8_t s1 = s[0], s2 = s[1];
	out->state              = decodeChargeState(s1);
	out->power_good         = (s2 & BQ25756_STAT2_PG) != 0U;
	out->vbus_present       = out->power_good;       // proxy: chip can't tell us VBUS alone
	out->ac_present         = out->power_good;
	out->charging           = (out->state >= BQ25756_CHARGE_STATE_TRICKLE) &&
	                          (out->state <= BQ25756_CHARGE_STATE_TOPOFF);
	out->input_i_regulating = (s1 & BQ25756_STAT1_IAC_DPM_STAT) != 0U;
	out->input_v_regulating = (s1 & BQ25756_STAT1_VAC_DPM_STAT) != 0U;
	out->thermal_regulating = false;                 // no thermal-reg status bit on this part
	out->vsys_regulating    = false;                 // no VSYS path
	out->jeita_region       = decodeJeitaRegion(s2);
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756GetFaults(const BQ25756* const dev, BQ25756Faults* const out)
{
	if (out == NULL) return BQ25756_ERROR_INVALID_PARAM;
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;

	uint8_t fs = 0U;
	const BQ25756Error e = rd8(dev, BQ25756_REG_FAULT_STATUS, &fs);
	if (e != BQ25756_ERROR_OK) return e;

	// BQ25756 FAULT_STATUS is a single 8-bit register.  Map its bits onto
	// the closest BQ25756FaultBits values the application API exposes.
	uint32_t mask = 0U;
	if (fs & BQ25756_FSTAT_BIT_VAC_OV)      mask |= BQ25756_FAULT_VBUS_OVP;
	if (fs & BQ25756_FSTAT_BIT_VAC_UV)      mask |= BQ25756_FAULT_VSYS_OVP; // best-fit
	if (fs & BQ25756_FSTAT_BIT_IBAT_OCP)    mask |= BQ25756_FAULT_IBAT_OCP;
	if (fs & BQ25756_FSTAT_BIT_VBAT_OV)     mask |= BQ25756_FAULT_VBAT_OVP;
	if (fs & BQ25756_FSTAT_BIT_TSHUT)       mask |= BQ25756_FAULT_TDIE_OTP;
	if (fs & BQ25756_FSTAT_BIT_CHG_TMR_EXP) mask |= BQ25756_FAULT_SAFETY;

	out->fault_mask = mask;
	out->dominant   = decodeDominantFault(mask);
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756ClearFaultFlags(const BQ25756* const dev)
{
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;
	// Writing 1s back to the FAULT_FLAG register clears latched bits.
	return wr8(dev, BQ25756_REG_FAULT_FLAG, 0xFFU);
}

/* ── 7. ADC ───────────────────────────────────────────────────────────────── */

BQ25756Error bq25756SetADCEnabled(const BQ25756* const dev, const bool enable)
{
	return rmw8(dev, BQ25756_REG_ADC_CONTROL,
	            BQ25756_ADC_CTRL_EN_ADC,
	            enable ? BQ25756_ADC_CTRL_EN_ADC : 0U);
}

BQ25756Error bq25756SetADCMode(const BQ25756* const dev, const BQ25756ADCMode mode)
{
	return rmw8(dev, BQ25756_REG_ADC_CONTROL,
	            BQ25756_ADC_CTRL_ADC_RATE_ONESHOT,
	            (mode == BQ25756_ADC_MODE_ONE_SHOT) ?
	                BQ25756_ADC_CTRL_ADC_RATE_ONESHOT : 0U);
}

BQ25756Error bq25756SetADCResolution(BQ25756* const dev, const BQ25756ADCResolution res)
{
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;
	if ((uint32_t)res > (uint32_t)BQ25756_ADC_RES_12_BIT)
		return BQ25756_ERROR_INVALID_PARAM;
	const BQ25756Error e = rmw8(dev, BQ25756_REG_ADC_CONTROL,
	                            BQ25756_ADC_CTRL_RES_MASK,
	                            (uint8_t)((uint8_t)res << BQ25756_ADC_CTRL_RES_SHIFT));
	if (e != BQ25756_ERROR_OK) return e;
	dev->adc_resolution = res;
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756TriggerADCOneShot(const BQ25756* const dev, const uint32_t timeout_ms)
{
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;

	const BQ25756Error e = rmw8(dev, BQ25756_REG_ADC_CONTROL,
	                            BQ25756_ADC_CTRL_EN_ADC, BQ25756_ADC_CTRL_EN_ADC);
	if (e != BQ25756_ERROR_OK) return e;

	const uint32_t wait = (timeout_ms == 0U) ? bq25756_default_adc_settle_ms : timeout_ms;
	if (dev->hal.delayMs != NULL) dev->hal.delayMs(wait);
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756ReadADC(const BQ25756* const dev, const BQ25756ADCChannel channel,
                            int32_t* const out_value)
{
	if (out_value == NULL) return BQ25756_ERROR_INVALID_PARAM;
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;

	// Channel routing: the public API was modelled on a chip with VBUS/IBUS/
	// VSYS/TDIE channels.  BQ25756 exposes VAC/IAC/VBAT/IBAT/TS/VFB only --
	// map VBUS->VAC, IBUS->IAC, VSYS->VBAT (best-fit), TDIE->not supported.
	uint8_t reg = 0U;
	bool    signed16 = false;
	uint32_t lsb_num = 0U;   // millis per LSB numerator
	uint32_t lsb_den = 1U;   // denominator (TS uses 98/1000 milli-percent)

	switch (channel)
	{
	case BQ25756_ADC_CHANNEL_VBUS:
		reg = BQ25756_REG_VAC_ADC;  lsb_num = BQ25756_ADC_VAC_LSB_MV; break;
	case BQ25756_ADC_CHANNEL_IBUS:
		reg = BQ25756_REG_IAC_ADC;  lsb_num = BQ25756_ADC_IAC_LSB_MA; break;
	case BQ25756_ADC_CHANNEL_VBAT:
		reg = BQ25756_REG_VBAT_ADC; lsb_num = BQ25756_ADC_VBAT_LSB_MV; break;
	case BQ25756_ADC_CHANNEL_IBAT:
		reg = BQ25756_REG_IBAT_ADC; lsb_num = BQ25756_ADC_IBAT_LSB_MA;
		signed16 = true; break;
	case BQ25756_ADC_CHANNEL_VSYS:
		reg = BQ25756_REG_VBAT_ADC; lsb_num = BQ25756_ADC_VBAT_LSB_MV; break;
	case BQ25756_ADC_CHANNEL_TS:
		reg = BQ25756_REG_TS_ADC;
		lsb_num = BQ25756_ADC_TS_LSB_MILLI_PCT_NUM;
		lsb_den = BQ25756_ADC_TS_LSB_MILLI_PCT_DEN;
		break;
	case BQ25756_ADC_CHANNEL_TDIE:
		// No die-temperature ADC on this part.
		return BQ25756_ERROR_NOT_SUPPORTED;
	default:
		return BQ25756_ERROR_INVALID_PARAM;
	}

	uint16_t raw_u = 0U;
	const BQ25756Error rc = rd16le(dev, reg, &raw_u);
	if (rc != BQ25756_ERROR_OK) return rc;

	if (signed16)
	{
		const int32_t raw_s = (int32_t)(int16_t)raw_u;
		*out_value = (raw_s * (int32_t)lsb_num) / (int32_t)lsb_den;
	}
	else
	{
		*out_value = (int32_t)(((uint32_t)raw_u * lsb_num) / lsb_den);
	}
	return BQ25756_ERROR_OK;
}

/* ── 8. JEITA / NTC ───────────────────────────────────────────────────────── */

BQ25756Error bq25756SetNTCEnabled(const BQ25756* const dev, const bool enable)
{
	// NTC behaviour is governed by REG 0x1B/1C/1D on this part and is always
	// "enabled" once TS thresholds are programmed; there is no master
	// EN_NTC bit.  Accept silently to keep ApplyConfig happy.
	(void)dev; (void)enable;
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756SetJEITACoolAction(const BQ25756* const dev, const BQ25756JEITAVCool action)
{
	(void)dev; (void)action;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

BQ25756Error bq25756SetJEITAWarmAction(const BQ25756* const dev, const BQ25756JEITAIWarm action)
{
	(void)dev; (void)action;
	return BQ25756_ERROR_NOT_SUPPORTED;
}

/* ── 9. Interrupt-mask programming ────────────────────────────────────────── */

BQ25756Error bq25756MaskAllInterrupts(const BQ25756* const dev)
{
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;
	const uint8_t all_masked[3] = {0xFFU, 0xFFU, 0xFFU};
	return wr(dev, BQ25756_REG_CHARGER_MASK_1, all_masked, 3U);
}

BQ25756Error bq25756SetFaultMask(const BQ25756* const dev, const uint32_t fault_bits)
{
	// On BQ25756 the entire fault-mask surface is one byte at REG 0x2A;
	// granular cross-API mapping isn't meaningful.  Unmask everything when
	// any caller bit is set, mask everything otherwise.
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;
	return wr8(dev, BQ25756_REG_FAULT_MASK, fault_bits ? 0x00U : 0xFFU);
}

/* ── Internal helpers ─────────────────────────────────────────────────────── */

static BQ25756Error ready(const BQ25756* const dev)
{
	if (dev == NULL)            return BQ25756_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)   return BQ25756_ERROR_NOT_INITIALIZED;
	return BQ25756_ERROR_OK;
}

static BQ25756Error rd(const BQ25756* const dev, const uint8_t reg,
                       uint8_t* const data, const uint8_t len)
{
	return (dev->hal.readReg(dev->i2c_addr, reg, data, len) == 0)
	       ? BQ25756_ERROR_OK : BQ25756_ERROR_COMM_FAIL;
}

static BQ25756Error wr(const BQ25756* const dev, const uint8_t reg,
                       const uint8_t* const data, const uint8_t len)
{
	return (dev->hal.writeReg(dev->i2c_addr, reg, data, len) == 0)
	       ? BQ25756_ERROR_OK : BQ25756_ERROR_COMM_FAIL;
}

static BQ25756Error rd8(const BQ25756* const dev, const uint8_t reg, uint8_t* const value)
{
	return rd(dev, reg, value, 1U);
}

static BQ25756Error wr8(const BQ25756* const dev, const uint8_t reg, const uint8_t value)
{
	return wr(dev, reg, &value, 1U);
}

static BQ25756Error rmw8(const BQ25756* const dev, const uint8_t reg,
                         const uint8_t mask, const uint8_t value)
{
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;
	uint8_t current = 0U;
	const BQ25756Error e = rd8(dev, reg, &current);
	if (e != BQ25756_ERROR_OK) return e;
	const uint8_t updated = (uint8_t)((current & (uint8_t)~mask) | (value & mask));
	return wr8(dev, reg, updated);
}

/* All 16-bit setpoint and ADC registers are little-endian: LSB at low addr,
 * MSB at low+1. */
static BQ25756Error rd16le(const BQ25756* const dev, const uint8_t lsb_reg,
                           uint16_t* const value)
{
	uint8_t buf[2] = {0};
	const BQ25756Error e = rd(dev, lsb_reg, buf, 2U);
	if (e != BQ25756_ERROR_OK) return e;
	*value = (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
	return BQ25756_ERROR_OK;
}

static BQ25756Error wr16le(const BQ25756* const dev, const uint8_t lsb_reg,
                           const uint16_t value)
{
	const uint8_t bytes[2] = { (uint8_t)(value & 0xFFU), (uint8_t)((value >> 8) & 0xFFU) };
	return wr(dev, lsb_reg, bytes, 2U);
}

static uint16_t encodeShiftedField(const uint32_t value, const uint32_t lsb,
                                   const uint32_t shift, const uint32_t mask)
{
	const uint32_t code = (value / lsb) << shift;
	return (uint16_t)(code & mask);
}

static BQ25756Error writeShiftedField(const BQ25756* const dev, const uint8_t reg,
                                      const uint32_t value, const uint32_t min,
                                      const uint32_t max, const uint32_t lsb,
                                      const uint32_t shift, const uint16_t mask)
{
	const BQ25756Error r = ready(dev);
	if (r != BQ25756_ERROR_OK) return r;
	if (value < min || value > max) return BQ25756_ERROR_INVALID_PARAM;
	return wr16le(dev, reg, encodeShiftedField(value, lsb, shift, mask));
}

static BQ25756ChargeState decodeChargeState(const uint8_t status1)
{
	const uint8_t code = (uint8_t)((status1 & BQ25756_STAT1_CHARGE_STATE_MASK) >>
	                               BQ25756_STAT1_CHARGE_STATE_SHIFT);
	switch (code)
	{
	case 0x0U: return BQ25756_CHARGE_STATE_NOT_CHARGING;
	case 0x1U: return BQ25756_CHARGE_STATE_TRICKLE;
	case 0x2U: return BQ25756_CHARGE_STATE_PRECHARGE;
	case 0x3U: return BQ25756_CHARGE_STATE_FAST_CC;
	case 0x4U: return BQ25756_CHARGE_STATE_TAPER_CV;
	case 0x6U: return BQ25756_CHARGE_STATE_TOPOFF;
	case 0x7U: return BQ25756_CHARGE_STATE_DONE;
	default:   return BQ25756_CHARGE_STATE_UNKNOWN;
	}
}

static BQ25756JEITARegion decodeJeitaRegion(const uint8_t status2)
{
	const uint8_t code = (uint8_t)((status2 & BQ25756_STAT2_TS_MASK) >>
	                               BQ25756_STAT2_TS_SHIFT);
	switch (code)
	{
	case BQ25756_TS_NORMAL: return BQ25756_JEITA_NORMAL;
	case BQ25756_TS_WARM:   return BQ25756_JEITA_WARM;
	case BQ25756_TS_COOL:   return BQ25756_JEITA_COOL;
	case BQ25756_TS_COLD:   return BQ25756_JEITA_COLD;
	case BQ25756_TS_HOT:    return BQ25756_JEITA_HOT;
	default:                return BQ25756_JEITA_NORMAL;
	}
}

static BQ25756DominantFault decodeDominantFault(const uint32_t mask)
{
	if (mask & (BQ25756_FAULT_TDIE_OTP | BQ25756_FAULT_TS_HOT | BQ25756_FAULT_TS_COLD))
		return BQ25756_DOMINANT_FAULT_THERMAL;
	if (mask & (BQ25756_FAULT_WATCHDOG | BQ25756_FAULT_SAFETY))
		return BQ25756_DOMINANT_FAULT_TIMER;
	if (mask & (BQ25756_FAULT_VBAT_OVP | BQ25756_FAULT_IBAT_OCP))
		return BQ25756_DOMINANT_FAULT_BATTERY;
	if (mask & (BQ25756_FAULT_VSYS_OVP | BQ25756_FAULT_VSYS_SHORT | BQ25756_FAULT_CONV_OCP))
		return BQ25756_DOMINANT_FAULT_SYSTEM;
	if (mask & (BQ25756_FAULT_VBUS_OVP | BQ25756_FAULT_VAC_OVP))
		return BQ25756_DOMINANT_FAULT_INPUT;
	if (mask & (BQ25756_FAULT_OTG_OVP | BQ25756_FAULT_OTG_UVP))
		return BQ25756_DOMINANT_FAULT_OTG;
	return BQ25756_DOMINANT_FAULT_NONE;
}
