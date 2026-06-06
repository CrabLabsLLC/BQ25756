/**
 * @file bq25756.c
 * @brief BQ25756 NVDC battery-charger driver implementation.
 *
 * Every register address, bit position, and step size in this file is sourced
 * from `bq25756_registers.h`. That header carries BENCH-VERIFY markers next
 * to anything the implementer could not pin down against the printed
 * SLUSF42 datasheet — those need to be checked on first power-up.
 *
 * Multi-byte regulation targets are stored on the chip as 16-bit big-endian
 * values; the helpers below always write MSB first.
 *
 * @author Orion Serup <orion@crablabs.io>
 *
 * @reviewer TBD <reviewer@crablabs.io>
 */

#include "bq25756.h"

#include <stddef.h>
#include <string.h>

/* ── Constants ────────────────────────────────────────────────────────────── */

static const uint32_t bq25756_reset_timeout_ms        = 25U;
static const uint32_t bq25756_default_adc_settle_ms   = 50U;
static const uint8_t  bq25756_mask_all_interrupts     = 0xFFU;
static const int32_t  bq25756_adc_ts_full_scale_mpct  = 100000;  /* 100.000 % */

/* ── Forward declarations ─────────────────────────────────────────────────── */

static BQ25756Error bq25756ReadRegisters(const BQ25756* const dev,
                                         const uint8_t              reg,
                                         uint8_t* const             data,
                                         const uint8_t              length);
static BQ25756Error bq25756WriteRegisters(const BQ25756* const dev,
                                          const uint8_t              reg,
                                          const uint8_t* const       data,
                                          const uint8_t              length);
static BQ25756Error bq25756ReadByte(const BQ25756* const dev,
                                    const uint8_t              reg,
                                    uint8_t* const             value);
static BQ25756Error bq25756WriteByte(const BQ25756* const dev,
                                     const uint8_t              reg,
                                     const uint8_t              value);
static BQ25756Error bq25756ModifyByte(const BQ25756* const dev,
                                      const uint8_t              reg,
                                      const uint8_t              mask,
                                      const uint8_t              value);
static BQ25756Error bq25756WriteU16BE(const BQ25756* const dev,
                                      const uint8_t              msb_reg,
                                      const uint16_t             value);
static BQ25756Error bq25756ReadU16BE(const BQ25756* const dev,
                                     const uint8_t              msb_reg,
                                     uint16_t* const            value);
static BQ25756Error bq25756CheckReady(const BQ25756* const dev);
static BQ25756Error bq25756WriteScaled(const BQ25756* const dev,
                                       const uint8_t              msb_reg,
                                       const uint32_t             real_value,
                                       const uint32_t             real_min,
                                       const uint32_t             real_max,
                                       const uint32_t             lsb);
static BQ25756ChargeState   bq25756DecodeChargeState(const uint8_t status1);
static BQ25756JeitaRegion   bq25756DecodeJeitaRegion(const uint8_t ntc_status);
static BQ25756DominantFault bq25756DecodeDominantFault(const uint32_t mask);

/* ── 1. Initialization / Lifecycle / Identity ─────────────────────────────── */

BQ25756Error bq25756Init(BQ25756* const    dev,
                         const BQ25756HAL* const hal,
                         const uint8_t           i2c_addr)
{
	if (dev == NULL || hal == NULL || hal->readReg == NULL || hal->writeReg == NULL)
		return BQ25756_ERROR_INVALID_PARAM;

	dev->hal             = *hal;
	dev->i2c_addr        = i2c_addr;
	dev->part_info       = 0U;
	dev->adc_resolution  = BQ25756_ADC_RES_15_BIT;
	dev->is_initialized  = false;

	uint8_t part_info = 0U;
	if (dev->hal.readReg(dev->i2c_addr, BQ25756_REG_PART_INFO, &part_info, 1U) != 0)
		return BQ25756_ERROR_COMM_FAIL;

	const uint8_t pn = (uint8_t)((part_info & BQ25756_PART_INFO_PN_MASK) >> BQ25756_PART_INFO_PN_SHIFT);
	if (pn != BQ25756_PART_NUMBER_EXPECTED)
		return BQ25756_ERROR_BAD_ID;

	dev->part_info      = part_info;
	dev->is_initialized = true;
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756ApplyConfig(BQ25756* const       dev,
                                const BQ25756Config* const config)
{
	if (dev == NULL || config == NULL)
		return BQ25756_ERROR_INVALID_PARAM;
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;

	BQ25756Error err;

	err = bq25756SetChargeVoltage(dev, config->charge_voltage_mv);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetChargeCurrent(dev, config->charge_current_ma);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetPrechargeCurrent(dev, config->precharge_current_ma);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetTerminationCurrent(dev, config->termination_current_ma);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetInputVoltageMin(dev, config->input_voltage_min_mv);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetInputCurrentMax(dev, config->input_current_max_ma);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetVbusOvp(dev, config->vbus_ovp_mv);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetVbatOvp(dev, config->vbat_ovp_mv);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetVsysMin(dev, config->vsys_min_mv);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetVsysReg(dev, config->vsys_reg_mv);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetWatchdog(dev, config->watchdog);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetSafetyTimer(dev, config->safety_timer);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetTerminationEnabled(dev, config->enable_termination);
	if (err != BQ25756_ERROR_OK) return err;
	err = bq25756SetNtcEnabled(dev, config->enable_ntc);
	if (err != BQ25756_ERROR_OK) return err;

	/* ICO is on CHARGER_CTRL_1.EN_ICO — read-modify-write inline. */
	return bq25756ModifyByte(dev, BQ25756_REG_CHARGER_CTRL_1,
	                         BQ25756_CTRL1_EN_ICO_BIT,
	                         config->enable_ico ? BQ25756_CTRL1_EN_ICO_BIT : 0U);
}

BQ25756Error bq25756Reset(BQ25756* const dev)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;

	const BQ25756Error wr = bq25756ModifyByte(dev, BQ25756_REG_CHARGER_CTRL_3,
	                                          BQ25756_CTRL3_RESET_REG_BIT,
	                                          BQ25756_CTRL3_RESET_REG_BIT);
	if (wr != BQ25756_ERROR_OK)
		return wr;

	/* Self-clearing bit. Poll until it reads zero. */
	for (uint32_t i = 0U; i < bq25756_reset_timeout_ms; i++)
	{
		if (dev->hal.delayMs != NULL)
			dev->hal.delayMs(1U);
		uint8_t ctrl3 = 0U;
		const BQ25756Error rd = bq25756ReadByte(dev, BQ25756_REG_CHARGER_CTRL_3, &ctrl3);
		if (rd != BQ25756_ERROR_OK)
			return rd;
		if ((ctrl3 & BQ25756_CTRL3_RESET_REG_BIT) == 0U)
			return BQ25756_ERROR_OK;
	}
	return BQ25756_ERROR_TIMEOUT;
}

BQ25756Error bq25756GetPartInfo(const BQ25756* const dev,
                                uint8_t* const             part_info)
{
	if (dev == NULL || part_info == NULL)
		return BQ25756_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return BQ25756_ERROR_NOT_INITIALIZED;

	*part_info = dev->part_info;
	return BQ25756_ERROR_OK;
}

/* ── 2. Charger configuration ─────────────────────────────────────────────── */

BQ25756Error bq25756SetChargeVoltage(const BQ25756* const dev,
                                     const uint32_t             mv)
{
	return bq25756WriteScaled(dev, BQ25756_REG_VAC_CHG_LIMIT_MSB, mv,
	                          BQ25756_VAC_CHG_MIN_MV, BQ25756_VAC_CHG_MAX_MV,
	                          BQ25756_VAC_CHG_LSB_MV);
}

BQ25756Error bq25756SetChargeCurrent(const BQ25756* const dev,
                                     const uint32_t             ma)
{
	return bq25756WriteScaled(dev, BQ25756_REG_ICHG_LIMIT_MSB, ma,
	                          BQ25756_ICHG_MIN_MA, BQ25756_ICHG_MAX_MA,
	                          BQ25756_ICHG_LSB_MA);
}

BQ25756Error bq25756SetPrechargeCurrent(const BQ25756* const dev,
                                        const uint32_t             ma)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (ma == 0U || ma > BQ25756_IPRECHG_MAX_MA)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint8_t code = (uint8_t)(ma / BQ25756_IPRECHG_LSB_MA);
	return bq25756WriteByte(dev, BQ25756_REG_IPRECHG, code);
}

BQ25756Error bq25756SetTerminationCurrent(const BQ25756* const dev,
                                          const uint32_t             ma)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (ma > BQ25756_ITERM_MAX_MA)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint8_t code = (uint8_t)(ma / BQ25756_ITERM_LSB_MA);
	return bq25756WriteByte(dev, BQ25756_REG_ITERM, code);
}

BQ25756Error bq25756SetPrechargeVoltage(const BQ25756* const dev,
                                        const uint32_t             mv)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (mv == 0U || mv > BQ25756_VAC_CHG_MAX_MV)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint16_t code = (uint16_t)(mv / BQ25756_VBAT_PRECHG_LSB_MV);
	return bq25756WriteU16BE(dev, BQ25756_REG_VBAT_PRECHG_MSB, code);
}

BQ25756Error bq25756SetChargeEnabled(const BQ25756* const dev,
                                     const bool                 enable)
{
	return bq25756ModifyByte(dev, BQ25756_REG_CHARGER_CTRL_1,
	                         BQ25756_CTRL1_EN_CHG_BIT,
	                         enable ? BQ25756_CTRL1_EN_CHG_BIT : 0U);
}

BQ25756Error bq25756SetTerminationEnabled(const BQ25756* const dev,
                                          const bool                 enable)
{
	return bq25756ModifyByte(dev, BQ25756_REG_CHARGER_CTRL_1,
	                         BQ25756_CTRL1_EN_TERM_BIT,
	                         enable ? BQ25756_CTRL1_EN_TERM_BIT : 0U);
}

/* ── 3. Input / system configuration ──────────────────────────────────────── */

BQ25756Error bq25756SetInputVoltageMin(const BQ25756* const dev,
                                       const uint32_t             mv)
{
	return bq25756WriteScaled(dev, BQ25756_REG_VIN_DPM_MSB, mv,
	                          BQ25756_VIN_DPM_MIN_MV, BQ25756_VIN_DPM_MAX_MV,
	                          BQ25756_VIN_DPM_LSB_MV);
}

BQ25756Error bq25756SetInputCurrentMax(const BQ25756* const dev,
                                       const uint32_t             ma)
{
	return bq25756WriteScaled(dev, BQ25756_REG_IIN_DPM_MSB, ma,
	                          BQ25756_IIN_DPM_MIN_MA, BQ25756_IIN_DPM_MAX_MA,
	                          BQ25756_IIN_DPM_LSB_MA);
}

BQ25756Error bq25756SetVbusOvp(const BQ25756* const dev,
                               const uint32_t             mv)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (mv == 0U)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint16_t code = (uint16_t)(mv / BQ25756_VBUS_OVP_LSB_MV);
	return bq25756WriteU16BE(dev, BQ25756_REG_VBUS_OVP_MSB, code);
}

BQ25756Error bq25756SetVbatOvp(const BQ25756* const dev,
                               const uint32_t             mv)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (mv == 0U || mv > BQ25756_VAC_CHG_MAX_MV)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint16_t code = (uint16_t)(mv / BQ25756_VBAT_OVP_LSB_MV);
	return bq25756WriteU16BE(dev, BQ25756_REG_VBAT_OVP_MSB, code);
}

BQ25756Error bq25756SetVsysMin(const BQ25756* const dev,
                               const uint32_t             mv)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (mv == 0U)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint16_t code = (uint16_t)(mv / BQ25756_VSYS_LSB_MV);
	return bq25756WriteU16BE(dev, BQ25756_REG_VSYS_MIN_MSB, code);
}

BQ25756Error bq25756SetVsysReg(const BQ25756* const dev,
                               const uint32_t             mv)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (mv == 0U)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint16_t code = (uint16_t)(mv / BQ25756_VSYS_LSB_MV);
	return bq25756WriteU16BE(dev, BQ25756_REG_VSYS_REG_MSB, code);
}

/* ── 4. Power-path control ────────────────────────────────────────────────── */

BQ25756Error bq25756SetHiZ(const BQ25756* const dev,
                           const bool                 enable)
{
	return bq25756ModifyByte(dev, BQ25756_REG_CHARGER_CTRL_1,
	                         BQ25756_CTRL1_EN_HIZ_BIT,
	                         enable ? BQ25756_CTRL1_EN_HIZ_BIT : 0U);
}

BQ25756Error bq25756SetOtgEnabled(const BQ25756* const dev,
                                  const bool                 enable)
{
	return bq25756ModifyByte(dev, BQ25756_REG_CHARGER_CTRL_1,
	                         BQ25756_CTRL1_EN_OTG_BIT,
	                         enable ? BQ25756_CTRL1_EN_OTG_BIT : 0U);
}

BQ25756Error bq25756SetOtgVoltage(const BQ25756* const dev,
                                  const uint32_t             mv)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (mv == 0U)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint16_t code = (uint16_t)(mv / BQ25756_VOTG_LSB_MV);
	return bq25756WriteU16BE(dev, BQ25756_REG_VOTG_REG_MSB, code);
}

BQ25756Error bq25756SetOtgCurrent(const BQ25756* const dev,
                                  const uint32_t             ma)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (ma == 0U)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint8_t code = (uint8_t)(ma / BQ25756_IOTG_LSB_MA);
	return bq25756WriteByte(dev, BQ25756_REG_IOTG_LIMIT, code);
}

BQ25756Error bq25756EnterShipMode(const BQ25756* const dev)
{
	return bq25756ModifyByte(dev, BQ25756_REG_CHARGER_CTRL_3,
	                         BQ25756_CTRL3_SHIPMODE_BIT,
	                         BQ25756_CTRL3_SHIPMODE_BIT);
}

/* ── 5. Safety: watchdog & timers ─────────────────────────────────────────── */

BQ25756Error bq25756SetWatchdog(const BQ25756* const  dev,
                                const BQ25756WatchdogPeriod period)
{
	if ((uint32_t)period > (uint32_t)BQ25756_WATCHDOG_160_S)
		return BQ25756_ERROR_INVALID_PARAM;
	return bq25756ModifyByte(dev, BQ25756_REG_TIMER_CTRL,
	                         BQ25756_TIMER_WD_PERIOD_MASK,
	                         (uint8_t)((uint8_t)period << BQ25756_TIMER_WD_PERIOD_SHIFT));
}

BQ25756Error bq25756KickWatchdog(const BQ25756* const dev)
{
	return bq25756ModifyByte(dev, BQ25756_REG_TIMER_CTRL,
	                         BQ25756_TIMER_WD_RESET_BIT,
	                         BQ25756_TIMER_WD_RESET_BIT);
}

BQ25756Error bq25756SetSafetyTimerEnabled(const BQ25756* const dev,
                                          const bool                 enable)
{
	return bq25756ModifyByte(dev, BQ25756_REG_TIMER_CTRL,
	                         BQ25756_TIMER_EN_SAFETY_BIT,
	                         enable ? BQ25756_TIMER_EN_SAFETY_BIT : 0U);
}

BQ25756Error bq25756SetSafetyTimer(const BQ25756* const dev,
                                   const BQ25756SafetyTimer   period)
{
	if ((uint32_t)period > (uint32_t)BQ25756_SAFETY_TIMER_20_H)
		return BQ25756_ERROR_INVALID_PARAM;
	return bq25756ModifyByte(dev, BQ25756_REG_TIMER_CTRL,
	                         BQ25756_TIMER_SAFETY_PERIOD_MASK,
	                         (uint8_t)((uint8_t)period << BQ25756_TIMER_SAFETY_PERIOD_SHIFT));
}

/* ── 6. Status / fault decode ─────────────────────────────────────────────── */

BQ25756Error bq25756GetStatus(const BQ25756* const dev,
                              BQ25756Status* const       out)
{
	if (out == NULL)
		return BQ25756_ERROR_INVALID_PARAM;
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;

	uint8_t buf[4] = {0};
	const BQ25756Error err = bq25756ReadRegisters(dev, BQ25756_REG_CHARGER_STATUS_1,
	                                              buf, 4U);
	if (err != BQ25756_ERROR_OK)
		return err;

	const uint8_t s1  = buf[0];
	const uint8_t s2  = buf[1];
	/* buf[2] = CHARGER_STATUS_3 (watchdog / safety-timer / ICO live bits) —
	 * currently routed through bq25756GetFaults() and not duplicated here. */
	const uint8_t ntc = buf[3];

	out->state              = bq25756DecodeChargeState(s1);
	out->power_good         = (s1 & BQ25756_STAT1_PG_BIT) != 0U;
	out->vbus_present       = (s1 & BQ25756_STAT1_VBUS_PRESENT_BIT) != 0U;
	out->ac_present         = (s1 & BQ25756_STAT1_AC_PRESENT_BIT) != 0U;
	out->charging           = (out->state >= BQ25756_CHARGE_STATE_TRICKLE) &&
	                          (out->state <= BQ25756_CHARGE_STATE_TOPOFF);
	out->input_i_regulating = (s2 & BQ25756_STAT2_IINDPM_BIT) != 0U;
	out->input_v_regulating = (s2 & BQ25756_STAT2_VINDPM_BIT) != 0U;
	out->thermal_regulating = (s2 & BQ25756_STAT2_THERM_REG_BIT) != 0U;
	out->vsys_regulating    = (s2 & BQ25756_STAT2_VSYS_REG_BIT) != 0U;
	out->jeita_region       = bq25756DecodeJeitaRegion(ntc);
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756GetFaults(const BQ25756* const dev,
                              BQ25756Faults* const       out)
{
	if (out == NULL)
		return BQ25756_ERROR_INVALID_PARAM;
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;

	uint8_t buf[2] = {0};
	const BQ25756Error err = bq25756ReadRegisters(dev, BQ25756_REG_FAULT_STATUS_1,
	                                              buf, 2U);
	if (err != BQ25756_ERROR_OK)
		return err;

	const uint8_t f1 = buf[0];
	const uint8_t f2 = buf[1];
	uint32_t mask = 0U;
	if (f1 & BQ25756_FAULT1_VBUS_OVP_BIT)   mask |= BQ25756_FAULT_VBUS_OVP;
	if (f1 & BQ25756_FAULT1_VBAT_OVP_BIT)   mask |= BQ25756_FAULT_VBAT_OVP;
	if (f1 & BQ25756_FAULT1_VSYS_OVP_BIT)   mask |= BQ25756_FAULT_VSYS_OVP;
	if (f1 & BQ25756_FAULT1_VSYS_SHORT_BIT) mask |= BQ25756_FAULT_VSYS_SHORT;
	if (f1 & BQ25756_FAULT1_TS_HOT_BIT)     mask |= BQ25756_FAULT_TS_HOT;
	if (f1 & BQ25756_FAULT1_TS_COLD_BIT)    mask |= BQ25756_FAULT_TS_COLD;
	if (f1 & BQ25756_FAULT1_TS_WARM_BIT)    mask |= BQ25756_FAULT_TS_WARM;
	if (f1 & BQ25756_FAULT1_TS_COOL_BIT)    mask |= BQ25756_FAULT_TS_COOL;
	if (f2 & BQ25756_FAULT2_IBAT_OCP_BIT)   mask |= BQ25756_FAULT_IBAT_OCP;
	if (f2 & BQ25756_FAULT2_TDIE_OTP_BIT)   mask |= BQ25756_FAULT_TDIE_OTP;
	if (f2 & BQ25756_FAULT2_WD_BIT)         mask |= BQ25756_FAULT_WATCHDOG;
	if (f2 & BQ25756_FAULT2_SAFETY_BIT)     mask |= BQ25756_FAULT_SAFETY;
	if (f2 & BQ25756_FAULT2_OTG_OVP_BIT)    mask |= BQ25756_FAULT_OTG_OVP;
	if (f2 & BQ25756_FAULT2_OTG_UVP_BIT)    mask |= BQ25756_FAULT_OTG_UVP;
	if (f2 & BQ25756_FAULT2_CONV_OCP_BIT)   mask |= BQ25756_FAULT_CONV_OCP;
	if (f2 & BQ25756_FAULT2_VAC_OVP_BIT)    mask |= BQ25756_FAULT_VAC_OVP;

	out->fault_mask = mask;
	out->dominant   = bq25756DecodeDominantFault(mask);
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756ClearFaultFlags(const BQ25756* const dev)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	const uint8_t ones[2] = {0xFFU, 0xFFU};
	return bq25756WriteRegisters(dev, BQ25756_REG_FAULT_FLAG_1, ones, 2U);
}

/* ── 7. ADC ───────────────────────────────────────────────────────────────── */

BQ25756Error bq25756SetAdcEnabled(const BQ25756* const dev,
                                  const bool                 enable)
{
	return bq25756ModifyByte(dev, BQ25756_REG_ADC_CTRL,
	                         BQ25756_ADC_CTRL_EN_BIT,
	                         enable ? BQ25756_ADC_CTRL_EN_BIT : 0U);
}

BQ25756Error bq25756SetAdcMode(const BQ25756* const dev,
                               const BQ25756AdcMode       mode)
{
	return bq25756ModifyByte(dev, BQ25756_REG_ADC_CTRL,
	                         BQ25756_ADC_CTRL_RATE_BIT,
	                         (mode == BQ25756_ADC_MODE_ONE_SHOT) ? BQ25756_ADC_CTRL_RATE_BIT : 0U);
}

BQ25756Error bq25756SetAdcResolution(BQ25756* const       dev,
                                     const BQ25756AdcResolution res)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if ((uint32_t)res > (uint32_t)BQ25756_ADC_RES_12_BIT)
		return BQ25756_ERROR_INVALID_PARAM;

	const BQ25756Error err = bq25756ModifyByte(dev, BQ25756_REG_ADC_CTRL,
	                                           BQ25756_ADC_CTRL_RES_MASK,
	                                           (uint8_t)((uint8_t)res << BQ25756_ADC_CTRL_RES_SHIFT));
	if (err != BQ25756_ERROR_OK)
		return err;

	dev->adc_resolution = res;
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756TriggerAdcOneShot(const BQ25756* const dev,
                                      const uint32_t             timeout_ms)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;

	/* Hi-pulse EN_ADC while RATE = one-shot. The chip self-clears EN_ADC on
	 * conversion completion in one-shot mode (BENCH-VERIFY behaviour). */
	const BQ25756Error wr = bq25756ModifyByte(dev, BQ25756_REG_ADC_CTRL,
	                                          BQ25756_ADC_CTRL_EN_BIT,
	                                          BQ25756_ADC_CTRL_EN_BIT);
	if (wr != BQ25756_ERROR_OK)
		return wr;

	const uint32_t wait = (timeout_ms == 0U) ? bq25756_default_adc_settle_ms : timeout_ms;
	if (dev->hal.delayMs != NULL)
		dev->hal.delayMs(wait);
	return BQ25756_ERROR_OK;
}

BQ25756Error bq25756ReadAdc(const BQ25756* const dev,
                            const BQ25756AdcChannel    channel,
                            int32_t* const             out_value)
{
	if (out_value == NULL)
		return BQ25756_ERROR_INVALID_PARAM;
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;

	uint8_t reg = 0U;
	switch (channel)
	{
	case BQ25756_ADC_CHANNEL_VBUS: reg = BQ25756_REG_ADC_VBUS_MSB; break;
	case BQ25756_ADC_CHANNEL_IBUS: reg = BQ25756_REG_ADC_IBUS_MSB; break;
	case BQ25756_ADC_CHANNEL_VBAT: reg = BQ25756_REG_ADC_VBAT_MSB; break;
	case BQ25756_ADC_CHANNEL_IBAT: reg = BQ25756_REG_ADC_ICHG_MSB; break;
	case BQ25756_ADC_CHANNEL_VSYS: reg = BQ25756_REG_ADC_VSYS_MSB; break;
	case BQ25756_ADC_CHANNEL_TS:   reg = BQ25756_REG_ADC_TS_MSB;   break;
	case BQ25756_ADC_CHANNEL_TDIE: reg = BQ25756_REG_ADC_TDIE_MSB; break;
	default:                       return BQ25756_ERROR_INVALID_PARAM;
	}

	uint16_t raw_u = 0U;
	const BQ25756Error rd = bq25756ReadU16BE(dev, reg, &raw_u);
	if (rd != BQ25756_ERROR_OK)
		return rd;

	const int16_t raw_s = (int16_t)raw_u;

	switch (channel)
	{
	case BQ25756_ADC_CHANNEL_VBUS:
		*out_value = (int32_t)((uint32_t)raw_u * BQ25756_ADC_VBUS_LSB_MV); break;
	case BQ25756_ADC_CHANNEL_IBUS:
		/* IBUS is a magnitude in buck mode, signed-ish in OTG. Treat unsigned. */
		*out_value = (int32_t)((uint32_t)raw_u * BQ25756_ADC_IBUS_LSB_MA); break;
	case BQ25756_ADC_CHANNEL_VBAT:
		*out_value = (int32_t)((uint32_t)raw_u * BQ25756_ADC_VBAT_LSB_MV); break;
	case BQ25756_ADC_CHANNEL_IBAT:
		/* IBAT is signed: positive = charging, negative = discharging. */
		*out_value = (int32_t)raw_s * (int32_t)BQ25756_ADC_IBAT_LSB_MA; break;
	case BQ25756_ADC_CHANNEL_VSYS:
		*out_value = (int32_t)((uint32_t)raw_u * BQ25756_ADC_VSYS_LSB_MV); break;
	case BQ25756_ADC_CHANNEL_TS: {
		/* TS is a ratio of REGN — the raw code is full-scale at 15-bit. Map to
		 * milli-percent (0..100000). BENCH-VERIFY full-scale code. */
		const uint32_t full_scale = (1UL << 15) - 1U;
		const uint64_t scaled = (uint64_t)raw_u * (uint64_t)bq25756_adc_ts_full_scale_mpct;
		*out_value = (int32_t)(scaled / full_scale);
		break;
	}
	case BQ25756_ADC_CHANNEL_TDIE:
		/* Signed 0.5 °C per LSB → milli-°C. */
		*out_value = (int32_t)raw_s * (int32_t)BQ25756_ADC_TDIE_LSB_MC; break;
	default:
		return BQ25756_ERROR_INVALID_PARAM;
	}
	return BQ25756_ERROR_OK;
}

/* ── 8. JEITA / NTC ───────────────────────────────────────────────────────── */

BQ25756Error bq25756SetNtcEnabled(const BQ25756* const dev,
                                  const bool                 enable)
{
	return bq25756ModifyByte(dev, BQ25756_REG_NTC_CTRL_0,
	                         BQ25756_NTC0_EN_BIT,
	                         enable ? BQ25756_NTC0_EN_BIT : 0U);
}

BQ25756Error bq25756SetJeitaCoolAction(const BQ25756* const dev,
                                       const BQ25756JeitaVcool    action)
{
	if ((uint32_t)action > (uint32_t)BQ25756_JEITA_VCOOL_SUSPENDED)
		return BQ25756_ERROR_INVALID_PARAM;
	return bq25756ModifyByte(dev, BQ25756_REG_NTC_CTRL_0,
	                         BQ25756_NTC0_JEITA_VSET_MASK,
	                         (uint8_t)((uint8_t)action << BQ25756_NTC0_JEITA_VSET_SHIFT));
}

BQ25756Error bq25756SetJeitaWarmAction(const BQ25756* const dev,
                                       const BQ25756JeitaIwarm    action)
{
	if ((uint32_t)action > (uint32_t)BQ25756_JEITA_IWARM_SUSPENDED)
		return BQ25756_ERROR_INVALID_PARAM;
	return bq25756ModifyByte(dev, BQ25756_REG_NTC_CTRL_0,
	                         BQ25756_NTC0_JEITA_ISET_MASK,
	                         (uint8_t)((uint8_t)action << BQ25756_NTC0_JEITA_ISET_SHIFT));
}

/* ── 9. Interrupt-mask programming ────────────────────────────────────────── */

BQ25756Error bq25756MaskAllInterrupts(const BQ25756* const dev)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	const uint8_t all_masked[4] = {
		bq25756_mask_all_interrupts,
		bq25756_mask_all_interrupts,
		bq25756_mask_all_interrupts,
		bq25756_mask_all_interrupts,
	};
	return bq25756WriteRegisters(dev, BQ25756_REG_CHARGER_MASK_1, all_masked, 4U);
}

BQ25756Error bq25756SetFaultMask(const BQ25756* const dev,
                                 const uint32_t             fault_bits)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;

	/* Mask byte semantic: 1 = masked, 0 = enabled. We unmask the bits in
	 * @p fault_bits and mask everything else. */
	uint8_t m1 = 0xFFU;
	uint8_t m2 = 0xFFU;

	if (fault_bits & BQ25756_FAULT_VBUS_OVP)   m1 &= (uint8_t)~BQ25756_FAULT1_VBUS_OVP_BIT;
	if (fault_bits & BQ25756_FAULT_VBAT_OVP)   m1 &= (uint8_t)~BQ25756_FAULT1_VBAT_OVP_BIT;
	if (fault_bits & BQ25756_FAULT_VSYS_OVP)   m1 &= (uint8_t)~BQ25756_FAULT1_VSYS_OVP_BIT;
	if (fault_bits & BQ25756_FAULT_VSYS_SHORT) m1 &= (uint8_t)~BQ25756_FAULT1_VSYS_SHORT_BIT;
	if (fault_bits & BQ25756_FAULT_TS_HOT)     m1 &= (uint8_t)~BQ25756_FAULT1_TS_HOT_BIT;
	if (fault_bits & BQ25756_FAULT_TS_COLD)    m1 &= (uint8_t)~BQ25756_FAULT1_TS_COLD_BIT;
	if (fault_bits & BQ25756_FAULT_TS_WARM)    m1 &= (uint8_t)~BQ25756_FAULT1_TS_WARM_BIT;
	if (fault_bits & BQ25756_FAULT_TS_COOL)    m1 &= (uint8_t)~BQ25756_FAULT1_TS_COOL_BIT;

	if (fault_bits & BQ25756_FAULT_IBAT_OCP)   m2 &= (uint8_t)~BQ25756_FAULT2_IBAT_OCP_BIT;
	if (fault_bits & BQ25756_FAULT_TDIE_OTP)   m2 &= (uint8_t)~BQ25756_FAULT2_TDIE_OTP_BIT;
	if (fault_bits & BQ25756_FAULT_WATCHDOG)   m2 &= (uint8_t)~BQ25756_FAULT2_WD_BIT;
	if (fault_bits & BQ25756_FAULT_SAFETY)     m2 &= (uint8_t)~BQ25756_FAULT2_SAFETY_BIT;
	if (fault_bits & BQ25756_FAULT_OTG_OVP)    m2 &= (uint8_t)~BQ25756_FAULT2_OTG_OVP_BIT;
	if (fault_bits & BQ25756_FAULT_OTG_UVP)    m2 &= (uint8_t)~BQ25756_FAULT2_OTG_UVP_BIT;
	if (fault_bits & BQ25756_FAULT_CONV_OCP)   m2 &= (uint8_t)~BQ25756_FAULT2_CONV_OCP_BIT;
	if (fault_bits & BQ25756_FAULT_VAC_OVP)    m2 &= (uint8_t)~BQ25756_FAULT2_VAC_OVP_BIT;

	const uint8_t bytes[2] = {m1, m2};
	return bq25756WriteRegisters(dev, BQ25756_REG_FAULT_MASK_1, bytes, 2U);
}

/* ── Internal helpers ─────────────────────────────────────────────────────── */

static BQ25756Error bq25756CheckReady(const BQ25756* const dev)
{
	if (dev == NULL)
		return BQ25756_ERROR_INVALID_PARAM;
	if (!dev->is_initialized)
		return BQ25756_ERROR_NOT_INITIALIZED;
	return BQ25756_ERROR_OK;
}

static BQ25756Error bq25756ReadRegisters(const BQ25756* const dev,
                                         const uint8_t              reg,
                                         uint8_t* const             data,
                                         const uint8_t              length)
{
	if (dev->hal.readReg(dev->i2c_addr, reg, data, length) != 0)
		return BQ25756_ERROR_COMM_FAIL;
	return BQ25756_ERROR_OK;
}

static BQ25756Error bq25756WriteRegisters(const BQ25756* const dev,
                                          const uint8_t              reg,
                                          const uint8_t* const       data,
                                          const uint8_t              length)
{
	if (dev->hal.writeReg(dev->i2c_addr, reg, data, length) != 0)
		return BQ25756_ERROR_COMM_FAIL;
	return BQ25756_ERROR_OK;
}

static BQ25756Error bq25756ReadByte(const BQ25756* const dev,
                                    const uint8_t              reg,
                                    uint8_t* const             value)
{
	return bq25756ReadRegisters(dev, reg, value, 1U);
}

static BQ25756Error bq25756WriteByte(const BQ25756* const dev,
                                     const uint8_t              reg,
                                     const uint8_t              value)
{
	return bq25756WriteRegisters(dev, reg, &value, 1U);
}

static BQ25756Error bq25756ModifyByte(const BQ25756* const dev,
                                      const uint8_t              reg,
                                      const uint8_t              mask,
                                      const uint8_t              value)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;

	uint8_t current = 0U;
	const BQ25756Error rd = bq25756ReadByte(dev, reg, &current);
	if (rd != BQ25756_ERROR_OK)
		return rd;
	const uint8_t updated = (uint8_t)((current & (uint8_t)~mask) | (value & mask));
	return bq25756WriteByte(dev, reg, updated);
}

static BQ25756Error bq25756WriteU16BE(const BQ25756* const dev,
                                      const uint8_t              msb_reg,
                                      const uint16_t             value)
{
	const uint8_t bytes[2] = {
		(uint8_t)((value >> 8) & 0xFFU),
		(uint8_t)(value & 0xFFU),
	};
	return bq25756WriteRegisters(dev, msb_reg, bytes, 2U);
}

static BQ25756Error bq25756ReadU16BE(const BQ25756* const dev,
                                     const uint8_t              msb_reg,
                                     uint16_t* const            value)
{
	uint8_t buf[2] = {0};
	const BQ25756Error err = bq25756ReadRegisters(dev, msb_reg, buf, 2U);
	if (err != BQ25756_ERROR_OK)
		return err;
	*value = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
	return BQ25756_ERROR_OK;
}

static BQ25756Error bq25756WriteScaled(const BQ25756* const dev,
                                       const uint8_t              msb_reg,
                                       const uint32_t             real_value,
                                       const uint32_t             real_min,
                                       const uint32_t             real_max,
                                       const uint32_t             lsb)
{
	const BQ25756Error ready = bq25756CheckReady(dev);
	if (ready != BQ25756_ERROR_OK)
		return ready;
	if (lsb == 0U)
		return BQ25756_ERROR_INVALID_PARAM;
	if (real_value < real_min || real_value > real_max)
		return BQ25756_ERROR_INVALID_PARAM;
	const uint16_t code = (uint16_t)((real_value - real_min) / lsb);
	return bq25756WriteU16BE(dev, msb_reg, code);
}

static BQ25756ChargeState bq25756DecodeChargeState(const uint8_t status1)
{
	const uint8_t code = (uint8_t)((status1 & BQ25756_STAT1_CHG_STATE_MASK) >>
	                               BQ25756_STAT1_CHG_STATE_SHIFT);
	switch (code)
	{
	case 0x0U: return BQ25756_CHARGE_STATE_NOT_CHARGING;
	case 0x1U: return BQ25756_CHARGE_STATE_TRICKLE;
	case 0x2U: return BQ25756_CHARGE_STATE_PRECHARGE;
	case 0x3U: return BQ25756_CHARGE_STATE_FAST_CC;
	case 0x4U: return BQ25756_CHARGE_STATE_TAPER_CV;
	case 0x5U: return BQ25756_CHARGE_STATE_TOPOFF;
	case 0x6U: return BQ25756_CHARGE_STATE_DONE;
	default:   return BQ25756_CHARGE_STATE_UNKNOWN;
	}
}

static BQ25756JeitaRegion bq25756DecodeJeitaRegion(const uint8_t ntc_status)
{
	/* BENCH-VERIFY decode: assume the chip encodes the region in NTC_STATUS[2:0]
	 * with 0=normal, 1=cool, 2=warm, 3=cold, 4=hot. */
	switch ((uint8_t)(ntc_status & 0x07U))
	{
	case 0x0U: return BQ25756_JEITA_NORMAL;
	case 0x1U: return BQ25756_JEITA_COOL;
	case 0x2U: return BQ25756_JEITA_WARM;
	case 0x3U: return BQ25756_JEITA_COLD;
	case 0x4U: return BQ25756_JEITA_HOT;
	default:   return BQ25756_JEITA_NORMAL;
	}
}

static BQ25756DominantFault bq25756DecodeDominantFault(const uint32_t mask)
{
	/* Priority order: thermal > timer > battery > system > input > OTG.
	 * Anything thermal kills the cell first, so it tops the list. */
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
