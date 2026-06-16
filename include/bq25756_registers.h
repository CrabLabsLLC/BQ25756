/**
 * @file bq25756_registers.h
 * @brief BQ25756 I2C register map -- rewritten from TI datasheet
 *        SLUSEN5 (August 2023) against a live POR-state register dump
 *        from a real chip. All addresses + reset defaults verified
 *        byte-for-byte against the chip on the bench.
 *
 * I2C: 7-bit slave address 0x6B, 8-bit register pointer, 16-bit
 * little-endian for the wide setpoint registers (low byte at low
 * address, e.g. 0x00 = bits[7:0], 0x01 = bits[15:8]).
 *
 * Registers above 0x3D return 0xFF (chip declines the read), with
 * one exception at 0x62.
 */

#pragma once
#ifndef BQ25756_REGISTERS_H
#define BQ25756_REGISTERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief BQ25756 7-bit I2C slave address (datasheet fixed). */
#define BQ25756_I2C_ADDR_DEFAULT 0x6BU

/* -- 16-bit setpoint / DPM regs (little-endian byte order: LSB at low addr). */
#define BQ25756_REG_CHARGE_VOLTAGE_LIMIT       0x00U  ///< POR 0x0010
#define BQ25756_REG_CHARGE_CURRENT_LIMIT       0x02U  ///< POR 0x0640
#define BQ25756_REG_INPUT_CURRENT_DPM_LIMIT    0x06U  ///< POR 0x0640
#define BQ25756_REG_INPUT_VOLTAGE_DPM_LIMIT    0x08U  ///< POR 0x0348
#define BQ25756_REG_REV_INPUT_CURRENT_LIMIT    0x0AU  ///< POR 0x0640
#define BQ25756_REG_REV_INPUT_VOLTAGE_LIMIT    0x0CU  ///< POR 0x03E8
#define BQ25756_REG_PRECHARGE_CURRENT_LIMIT    0x10U  ///< POR 0x0140
#define BQ25756_REG_TERMINATION_CURRENT_LIMIT  0x12U  ///< POR 0x00A0

/* -- 8-bit control / status regs. */
#define BQ25756_REG_PRECHG_TERM_CONTROL        0x14U  ///< POR 0x0F
#define BQ25756_REG_TIMER_CONTROL              0x15U  ///< POR 0x1D
#define BQ25756_REG_THREE_STAGE_CHG_CONTROL    0x16U  ///< POR 0x00
#define BQ25756_REG_CHARGER_CONTROL            0x17U  ///< POR 0xC9
#define BQ25756_REG_PIN_CONTROL                0x18U  ///< POR 0xC0
#define BQ25756_REG_POWER_PATH_REV_MODE_CTRL   0x19U  ///< POR 0x20
#define BQ25756_REG_MPPT_CONTROL               0x1AU  ///< POR 0x20
#define BQ25756_REG_TS_CHARGING_THRESH_CTRL    0x1BU  ///< POR 0x96
#define BQ25756_REG_TS_CHARGING_REGION_CTRL    0x1CU  ///< POR 0x57
#define BQ25756_REG_TS_REVERSE_MODE_THRESH     0x1DU  ///< POR 0x40
#define BQ25756_REG_REVERSE_UVLO_CONTROL       0x1EU  ///< POR 0x00
#define BQ25756_REG_VAC_MAX_POWER_DETECTED     0x1FU  ///< POR 0x0000 (16-bit)

#define BQ25756_REG_CHARGER_STATUS_1           0x21U
#define BQ25756_REG_CHARGER_STATUS_2           0x22U
#define BQ25756_REG_CHARGER_STATUS_3           0x23U
#define BQ25756_REG_FAULT_STATUS               0x24U

#define BQ25756_REG_CHARGER_FLAG_1             0x25U
#define BQ25756_REG_CHARGER_FLAG_2             0x26U
#define BQ25756_REG_FAULT_FLAG                 0x27U

#define BQ25756_REG_CHARGER_MASK_1             0x28U
#define BQ25756_REG_CHARGER_MASK_2             0x29U
#define BQ25756_REG_FAULT_MASK                 0x2AU

#define BQ25756_REG_ADC_CONTROL                0x2BU  ///< POR 0x60
#define BQ25756_REG_ADC_CHANNEL_CONTROL        0x2CU  ///< POR 0x0A

/* ADC result registers -- 16-bit LE. POR 0x0000. */
#define BQ25756_REG_IAC_ADC                    0x2DU
#define BQ25756_REG_IBAT_ADC                   0x2FU
#define BQ25756_REG_VAC_ADC                    0x31U
#define BQ25756_REG_VBAT_ADC                   0x33U
#define BQ25756_REG_TS_ADC                     0x37U
#define BQ25756_REG_VFB_ADC                    0x39U

#define BQ25756_REG_GATE_DRIVER_STRENGTH       0x3BU
#define BQ25756_REG_GATE_DRIVER_DEAD_TIME      0x3CU

/** @brief PART_INFORMATION register. POR = 0x12 (PN=010 BQ25756, DEV_REV=010). */
#define BQ25756_REG_PART_INFO                  0x3DU

#define BQ25756_REG_REV_BATT_DISCHARGE_CURRENT 0x62U

/* -- PART_INFO bit layout per datasheet §8.5.41. */
#define BQ25756_PART_INFO_PN_MASK              (0x07U << 3)  ///< bits 5:3
#define BQ25756_PART_INFO_PN_SHIFT             3U
#define BQ25756_PART_INFO_DEV_REV_MASK         0x07U         ///< bits 2:0
#define BQ25756_PART_INFO_PN_BQ25756           0x02U         ///< PN field value for BQ25756
#define BQ25756_PART_INFO_POR_VALUE            0x12U         ///< full reset byte

/* -- CHARGER_CONTROL (REG 0x17) bit positions per datasheet §8.5.12.
 *    POR 0xC9 = VRECHG=11, WD_RST=0, DIS_CE_PIN=0, EN_CHG_BIT_RST=1,
 *               EN_HIZ=0, EN_IBAT_LOAD=0, EN_CHG=1. */
#define BQ25756_CHARGER_CTRL_VRECHG_MASK       (0x03U << 6)
#define BQ25756_CHARGER_CTRL_WD_RST            (1U << 5)
#define BQ25756_CHARGER_CTRL_DIS_CE_PIN        (1U << 4)
#define BQ25756_CHARGER_CTRL_EN_CHG_BIT_RST    (1U << 3)
#define BQ25756_CHARGER_CTRL_EN_HIZ            (1U << 2)
#define BQ25756_CHARGER_CTRL_EN_IBAT_LOAD      (1U << 1)
#define BQ25756_CHARGER_CTRL_EN_CHG            (1U << 0)

/* -- POWER_PATH_REV_MODE_CTRL (REG 0x19) bit positions per §8.5.14.
 *    REG_RST is here, NOT in CHARGER_CONTROL. */
#define BQ25756_PWRPATH_REG_RST                (1U << 7)  ///< self-clearing
#define BQ25756_PWRPATH_EN_IAC_LOAD            (1U << 6)
#define BQ25756_PWRPATH_EN_PFM                 (1U << 5)
#define BQ25756_PWRPATH_EN_REV                 (1U << 0)

/* -- TIMER_CONTROL (REG 0x15) bit positions per §8.5.10.
 *    POR 0x1D = TOPOFF=00, WATCHDOG=01 (40s), EN_CHG_TMR=1,
 *               CHG_TMR=10 (12hr), EN_TMR2X=1. */
#define BQ25756_TIMER_TOPOFF_MASK              (0x03U << 6)
#define BQ25756_TIMER_TOPOFF_SHIFT             6U
#define BQ25756_TIMER_WATCHDOG_MASK            (0x03U << 4)
#define BQ25756_TIMER_WATCHDOG_SHIFT           4U
#define BQ25756_TIMER_EN_CHG_TMR               (1U << 3)
#define BQ25756_TIMER_CHG_TMR_MASK             (0x03U << 1)
#define BQ25756_TIMER_CHG_TMR_SHIFT            1U
#define BQ25756_TIMER_EN_TMR2X                 (1U << 0)

#define BQ25756_WATCHDOG_FIELD_DISABLED        0x0U
#define BQ25756_WATCHDOG_FIELD_40S             0x1U
#define BQ25756_WATCHDOG_FIELD_80S             0x2U
#define BQ25756_WATCHDOG_FIELD_160S            0x3U

#define BQ25756_CHG_TMR_FIELD_5H               0x0U
#define BQ25756_CHG_TMR_FIELD_8H               0x1U
#define BQ25756_CHG_TMR_FIELD_12H              0x2U
#define BQ25756_CHG_TMR_FIELD_24H              0x3U

/* -- PRECHG_TERM_CONTROL (REG 0x14) bit positions per §8.5.9. */
#define BQ25756_PRECHG_EN_TERM                 (1U << 1)

/* -- ADC_CONTROL (REG 0x2B) bit positions. */
#define BQ25756_ADC_CTRL_EN_ADC                (1U << 7)
#define BQ25756_ADC_CTRL_ADC_RATE_ONESHOT      (1U << 6)
#define BQ25756_ADC_CTRL_RES_MASK              (0x03U << 4)
#define BQ25756_ADC_CTRL_RES_SHIFT             4U

/* -- CHARGER_STATUS_1 (REG 0x21) bit layout per §8.5.21. */
#define BQ25756_STAT1_ADC_DONE                 (1U << 7)
#define BQ25756_STAT1_IAC_DPM_STAT             (1U << 6)
#define BQ25756_STAT1_VAC_DPM_STAT             (1U << 5)
#define BQ25756_STAT1_WD_STAT                  (1U << 3)
#define BQ25756_STAT1_CHARGE_STATE_MASK        0x07U
#define BQ25756_STAT1_CHARGE_STATE_SHIFT       0U

/* -- CHARGER_STATUS_2 (REG 0x22). */
#define BQ25756_STAT2_PG                       (1U << 7)
#define BQ25756_STAT2_TS_MASK                  (0x07U << 4)
#define BQ25756_STAT2_TS_SHIFT                 4U
#define BQ25756_STAT2_MPPT_MASK                0x03U

#define BQ25756_TS_NORMAL                      0x0U
#define BQ25756_TS_WARM                        0x1U
#define BQ25756_TS_COOL                        0x2U
#define BQ25756_TS_COLD                        0x3U
#define BQ25756_TS_HOT                         0x4U

/* -- CHARGER_STATUS_3 (REG 0x23). */
#define BQ25756_STAT3_CV_TMR_EXP               (1U << 3)
#define BQ25756_STAT3_REVERSE_ON               (1U << 2)

/* -- FAULT_STATUS (REG 0x24) raw bit positions.  The application-facing
 *    BQ25756FaultBits enum (in bq25756_types.h) is a wider OR-mask that
 *    these get translated into; keep these chip-register names distinct
 *    so the macros and the enum don't collide on the preprocessor. */
#define BQ25756_FSTAT_BIT_VAC_UV               (1U << 7)
#define BQ25756_FSTAT_BIT_VAC_OV               (1U << 6)
#define BQ25756_FSTAT_BIT_IBAT_OCP             (1U << 5)
#define BQ25756_FSTAT_BIT_VBAT_OV              (1U << 4)
#define BQ25756_FSTAT_BIT_TSHUT                (1U << 3)
#define BQ25756_FSTAT_BIT_CHG_TMR_EXP          (1U << 2)
#define BQ25756_FSTAT_BIT_DRV_OKZ              (1U << 1)

/* -- 16-bit setpoint scaling per datasheet field descriptions. */
#define BQ25756_VFB_LSB_MV                     2U
#define BQ25756_VFB_OFFSET_MV                  1504U
#define BQ25756_VFB_MAX_MV                     1566U
#define BQ25756_VFB_FIELD_MASK                 0x001FU       ///< bits 4:0

#define BQ25756_ICHG_LSB_MA                    50U
#define BQ25756_ICHG_MIN_MA                    400U
#define BQ25756_ICHG_MAX_MA                    20000U
#define BQ25756_ICHG_FIELD_MASK                0x07FCU       ///< bits 10:2
#define BQ25756_ICHG_FIELD_SHIFT               2U

#define BQ25756_IAC_DPM_LSB_MA                 50U
#define BQ25756_IAC_DPM_MIN_MA                 400U
#define BQ25756_IAC_DPM_MAX_MA                 20000U
#define BQ25756_IAC_DPM_FIELD_MASK             0x07FCU
#define BQ25756_IAC_DPM_FIELD_SHIFT            2U

#define BQ25756_VAC_DPM_LSB_MV                 20U
#define BQ25756_VAC_DPM_MIN_MV                 4200U
#define BQ25756_VAC_DPM_MAX_MV                 65000U
#define BQ25756_VAC_DPM_FIELD_MASK             0x3FFCU       ///< bits 13:2
#define BQ25756_VAC_DPM_FIELD_SHIFT            2U

#define BQ25756_PRECHG_LSB_MA                  50U
#define BQ25756_PRECHG_FIELD_MASK              0x07FCU
#define BQ25756_PRECHG_FIELD_SHIFT             2U

#define BQ25756_ITERM_LSB_MA                   50U
#define BQ25756_ITERM_FIELD_MASK               0x07FCU
#define BQ25756_ITERM_FIELD_SHIFT              2U

/* -- ADC result scaling per datasheet §7.x (ADC MEASUREMENT RANGE AND LSB). */
#define BQ25756_ADC_VAC_LSB_MV                 2U
#define BQ25756_ADC_VBAT_LSB_MV                2U
#define BQ25756_ADC_IAC_LSB_MA                 2U      ///< with 2 mΩ shunt
#define BQ25756_ADC_IBAT_LSB_MA                2U      ///< with 5 mΩ shunt; signed
#define BQ25756_ADC_VFB_LSB_MV                 1U
/* TS_ADC: 0.098% per LSB ≈ 98 milli-% per 1000 LSBs.  Scale = 98 * raw / 1000. */
#define BQ25756_ADC_TS_LSB_MILLI_PCT_NUM       98U
#define BQ25756_ADC_TS_LSB_MILLI_PCT_DEN       1000U

#ifdef __cplusplus
}
#endif

#endif /* BQ25756_REGISTERS_H */
