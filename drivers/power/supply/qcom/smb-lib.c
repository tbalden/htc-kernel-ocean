/* Copyright (c) 2016-2017 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/input/qpnp-power-on.h>
#include <linux/irq.h>
#include "smb-lib.h"
#include "smb-reg.h"
#include "storm-watch.h"
#include "pmic-voter.h"
#include <linux/nanohub_htc.h>
#include <linux/qpnp/qpnp-adc.h>

#define smblib_err(chg, fmt, ...)		\
	pr_err("%s: %s: " fmt, chg->name,	\
		__func__, ##__VA_ARGS__)	\

#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

#ifdef CONFIG_HTC_BATT
unsigned int g_cc_floating_cnt = 0;
static bool is_gc_charger_vote_usb_irq_enable = false;
#endif //CONFIG_HTC_BATT

static bool is_secure(struct smb_charger *chg, int addr)
{
	if (addr == SHIP_MODE_REG || addr == FREQ_CLK_DIV_REG)
		return true;
	/* assume everything above 0xA0 is secure */
	return (bool)((addr & 0xFF) >= 0xA0);
}

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val)
{
	unsigned int temp;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &temp);
	if (rc >= 0)
		*val = (u8)temp;

	return rc;
}

int smblib_multibyte_read(struct smb_charger *chg, u16 addr, u8 *val,
				int count)
{
	return regmap_bulk_read(chg->regmap, addr, val, count);
}

int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val)
{
	int rc = 0;
#ifdef CONFIG_HTC_BATT
	u8 debug = 0;
	int dbg_rc = 0;
#endif //CONFIG_HTC_BATT

	mutex_lock(&chg->write_lock);
	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & 0xFF00) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_update_bits(chg->regmap, addr, mask, val);

#ifdef CONFIG_HTC_BATT
	if (addr == USBIN_CMD_IL_REG){
		dbg_rc = smblib_read(chg, USBIN_CMD_IL_REG, &debug);
		if (dbg_rc >= 0)
			smblib_err(chg, "DEBUG: addr = 0x%X, mask = 0x%X, val = 0x%X, after write value = 0x%X\n", addr, mask, val, debug);
	}
#endif //CONFIG_HTC_BATT

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

int smblib_write(struct smb_charger *chg, u16 addr, u8 val)
{
	int rc = 0;
#ifdef CONFIG_HTC_BATT
	u8 debug = 0;
	int dbg_rc = 0;
#endif //CONFIG_HTC_BATT

	mutex_lock(&chg->write_lock);

	if (is_secure(chg, addr)) {
		rc = regmap_write(chg->regmap, (addr & ~(0xFF)) | 0xD0, 0xA5);
		if (rc < 0)
			goto unlock;
	}

	rc = regmap_write(chg->regmap, addr, val);

#ifdef CONFIG_HTC_BATT
	if (addr == USBIN_CMD_IL_REG){
		dbg_rc = smblib_read(chg, USBIN_CMD_IL_REG, &debug);
		if (dbg_rc >= 0)
			smblib_err(chg, "DEBUG: addr = 0x%X, val = 0x%X, after write value = 0x%X\n", addr, val, debug);
	}
#endif //CONFIG_HTC_BATT

unlock:
	mutex_unlock(&chg->write_lock);
	return rc;
}

static int smblib_get_step_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, step_state;
	u8 stat;

	if (!chg->step_chg_enabled) {
		*cc_delta_ua = 0;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	step_state = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;
	rc = smblib_get_charge_param(chg, &chg->param.step_cc_delta[step_state],
				     cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get step cc delta rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int smblib_get_jeita_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, cc_minus_ua;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}

	if (!(stat & BAT_TEMP_STATUS_SOFT_LIMIT_MASK)) {
		*cc_delta_ua = 0;
		return 0;
	}

	rc = smblib_get_charge_param(chg, &chg->param.jeita_cc_comp,
				     &cc_minus_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n", rc);
		return rc;
	}

	*cc_delta_ua = -cc_minus_ua;
	return 0;
}

int smblib_icl_override(struct smb_charger *chg, bool override)
{
	int rc;
	bool override_status;
	u8 stat;
	u16 reg;

	switch (chg->smb_version) {
	case PMI8998_SUBTYPE:
		reg = APSD_RESULT_STATUS_REG;
		break;
	case PM660_SUBTYPE:
		reg = AICL_STATUS_REG;
		break;
	default:
		smblib_dbg(chg, PR_MISC, "Unknown chip version=%x\n",
				chg->smb_version);
		return -EINVAL;
	}

	rc = smblib_read(chg, reg, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read reg=%x rc=%d\n", reg, rc);
		return rc;
	}
	override_status = (bool)(stat & ICL_OVERRIDE_LATCH_BIT);

	if (override != override_status) {
		rc = smblib_masked_write(chg, CMD_APSD_REG,
				ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't override ICL rc=%d\n", rc);
			return rc;
		}
	}
	return 0;
}

/********************
 * REGISTER GETTERS *
 ********************/

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read(chg, param->reg, &val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't read from 0x%04x rc=%d\n",
			param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, *val_u, val_raw);

	return rc;
}

int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}

struct apsd_result {
	const char * const name;
	const u8 bit;
	const enum power_supply_type pst;
};

enum {
	UNKNOWN,
	SDP,
	CDP,
	DCP,
	OCP,
	FLOAT,
	HVDCP2,
	HVDCP3,
	MAX_TYPES
};

static const struct apsd_result const smblib_apsd_results[] = {
	[UNKNOWN] = {
		.name	= "UNKNOWN",
		.bit	= 0,
#ifdef CONFIG_HTC_BATT
		.pst	= POWER_SUPPLY_TYPE_UNKNOWN
#else
		.pst	= POWER_SUPPLY_TYPE_USB
#endif //CONFIG_HTC_BATT
	},
	[SDP] = {
		.name	= "SDP",
		.bit	= SDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB
	},
	[CDP] = {
		.name	= "CDP",
		.bit	= CDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_CDP
	},
	[DCP] = {
		.name	= "DCP",
		.bit	= DCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[OCP] = {
		.name	= "OCP",
		.bit	= OCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[FLOAT] = {
		.name	= "FLOAT",
		.bit	= FLOAT_CHARGER_BIT,
#ifdef CONFIG_HTC_BATT
		.pst	= POWER_SUPPLY_TYPE_USB
#else
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
#endif //CONFIG_HTC_BATT
	},
	[HVDCP2] = {
		.name	= "HVDCP2",
		.bit	= DCP_CHARGER_BIT | QC_2P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP
	},
	[HVDCP3] = {
		.name	= "HVDCP3",
		.bit	= DCP_CHARGER_BIT | QC_3P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP_3,
	},
};

static const struct apsd_result *smblib_get_apsd_result(struct smb_charger *chg)
{
	int rc, i;
	u8 apsd_stat, stat;
	const struct apsd_result *result = &smblib_apsd_results[UNKNOWN];

	rc = smblib_read(chg, APSD_STATUS_REG, &apsd_stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return result;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", apsd_stat);

	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT))
		return result;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_RESULT_STATUS rc=%d\n",
			rc);
		return result;
	}
	stat &= APSD_RESULT_STATUS_MASK;

	for (i = 0; i < ARRAY_SIZE(smblib_apsd_results); i++) {
		if (smblib_apsd_results[i].bit == stat)
			result = &smblib_apsd_results[i];
	}

	if (apsd_stat & QC_CHARGER_BIT) {
		/* since its a qc_charger, either return HVDCP3 or HVDCP2 */
		if (result != &smblib_apsd_results[HVDCP3])
			result = &smblib_apsd_results[HVDCP2];
	}

	return result;
}

/********************
 * REGISTER SETTERS *
 ********************/

static int chg_freq_list[] = {
	9600, 9600, 6400, 4800, 3800, 3200, 2700, 2400, 2100, 1900, 1700,
	1600, 1500, 1400, 1300, 1200,
};

int smblib_set_chg_freq(struct smb_chg_param *param,
				int val_u, u8 *val_raw)
{
	u8 i;

	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	/* Charger FSW is the configured freqency / 2 */
	val_u *= 2;
	for (i = 0; i < ARRAY_SIZE(chg_freq_list); i++) {
		if (chg_freq_list[i] == val_u)
			break;
	}
	if (i == ARRAY_SIZE(chg_freq_list)) {
		pr_err("Invalid frequency %d Hz\n", val_u / 2);
		return -EINVAL;
	}

	*val_raw = i;

	return 0;
}

static int smblib_set_opt_freq_buck(struct smb_charger *chg, int fsw_khz)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_buck, fsw_khz);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_buck rc=%d\n", rc);

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fsw_khz;
		/*
		 * Some parallel charging implementations may not have
		 * PROP_BUCK_FREQ property - they could be running
		 * with a fixed frequency
		 */
		power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_BUCK_FREQ, &pval);
	}

	return rc;
}

int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u)
{
	int rc = 0;
	u8 val_raw;

	if (param->set_proc) {
		rc = param->set_proc(param, val_u, &val_raw);
		if (rc < 0)
			return -EINVAL;
	} else {
		if (val_u > param->max_u || val_u < param->min_u) {
			smblib_err(chg, "%s: %d is out of range [%d, %d]\n",
				param->name, val_u, param->min_u, param->max_u);
			return -EINVAL;
		}

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write(chg, param->reg, val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, val_u, val_raw);

	return rc;
}

static int step_charge_soc_update(struct smb_charger *chg, int capacity)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.step_soc, capacity);
	if (rc < 0) {
		smblib_err(chg, "Error in updating soc, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_write(chg, STEP_CHG_SOC_VBATT_V_UPDATE_REG,
			STEP_CHG_SOC_VBATT_V_UPDATE_BIT);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't set STEP_CHG_SOC_VBATT_V_UPDATE_REG rc=%d\n",
			rc);
		return rc;
	}

	return rc;
}

int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;
#ifdef CONFIG_HTC_BATT
	u8 debug = 0;
	int dbg_rc = 0;

	smblib_err(chg, "DEBUG: suspend = %d\n", suspend);
#endif //CONFIG_HTC_BATT

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				 suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

#ifdef CONFIG_HTC_BATT
	dbg_rc = smblib_read(chg, USBIN_CMD_IL_REG, &debug);
	if (dbg_rc >= 0)
		smblib_err(chg, "DEBUG: after suspend, 0x1340 = 0x%X\n", debug);
#endif //CONFIG_HTC_BATT

	return rc;
}

int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				 suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

static int smblib_set_adapter_allowance(struct smb_charger *chg,
					u8 allowed_voltage)
{
	int rc = 0;

	switch (allowed_voltage) {
	case USBIN_ADAPTER_ALLOW_12V:
	case USBIN_ADAPTER_ALLOW_5V_OR_12V:
	case USBIN_ADAPTER_ALLOW_9V_TO_12V:
	case USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V:
	case USBIN_ADAPTER_ALLOW_5V_TO_12V:
		/* PM660 only support max. 9V */
		if (chg->smb_version == PM660_SUBTYPE) {
			smblib_dbg(chg, PR_MISC, "voltage not supported=%d\n",
					allowed_voltage);
			allowed_voltage = USBIN_ADAPTER_ALLOW_5V_OR_9V;
		}
		break;
	}

	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG, allowed_voltage);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to USBIN_ADAPTER_ALLOW_CFG rc=%d\n",
			allowed_voltage, rc);
		return rc;
	}

	return rc;
}

#define MICRO_5V	5000000
#define MICRO_9V	9000000
#define MICRO_12V	12000000
static int smblib_set_usb_pd_allowed_voltage(struct smb_charger *chg,
					int min_allowed_uv, int max_allowed_uv)
{
	int rc;
	u8 allowed_voltage;

	if (min_allowed_uv == MICRO_5V && max_allowed_uv == MICRO_5V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V;
		smblib_set_opt_freq_buck(chg, chg->chg_freq.freq_5V);
	} else if (min_allowed_uv == MICRO_9V && max_allowed_uv == MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V;
		smblib_set_opt_freq_buck(chg, chg->chg_freq.freq_9V);
	} else if (min_allowed_uv == MICRO_12V && max_allowed_uv == MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_12V;
		smblib_set_opt_freq_buck(chg, chg->chg_freq.freq_12V);
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_9V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_9V;
	} else if (min_allowed_uv < MICRO_9V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_5V_TO_12V;
	} else if (min_allowed_uv < MICRO_12V && max_allowed_uv <= MICRO_12V) {
		allowed_voltage = USBIN_ADAPTER_ALLOW_9V_TO_12V;
	} else {
		smblib_err(chg, "invalid allowed voltage [%d, %d]\n",
			min_allowed_uv, max_allowed_uv);
		return -EINVAL;
	}

	rc = smblib_set_adapter_allowance(chg, allowed_voltage);
	if (rc < 0) {
		smblib_err(chg, "Couldn't configure adapter allowance rc=%d\n",
				rc);
		return rc;
	}

	return rc;
}

/********************
 * HELPER FUNCTIONS *
 ********************/

static int try_rerun_apsd_for_hvdcp(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result;

	/*
	 * PD_INACTIVE_VOTER on hvdcp_disable_votable indicates whether
	 * apsd rerun was tried earlier
	 */
	if (get_client_vote(chg->hvdcp_disable_votable_indirect,
						PD_INACTIVE_VOTER)) {
		vote(chg->hvdcp_disable_votable_indirect,
				PD_INACTIVE_VOTER, false, 0);
		/* ensure hvdcp is enabled */
		if (!get_effective_result(
				chg->hvdcp_disable_votable_indirect)) {
			apsd_result = smblib_get_apsd_result(chg);
			if (apsd_result->bit & (QC_2P0_BIT | QC_3P0_BIT)) {
				/* rerun APSD */
				smblib_dbg(chg, PR_MISC, "rerun APSD\n");
				smblib_masked_write(chg, CMD_APSD_REG,
						APSD_RERUN_BIT,
						APSD_RERUN_BIT);
			}
		}
	}
	return 0;
}

static const struct apsd_result *smblib_update_usb_type(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	/* if PD is active, APSD is disabled so won't have a valid result */
	if (chg->pd_active) {
		chg->usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
		return apsd_result;
	}

	chg->usb_psy_desc.type = apsd_result->pst;
	return apsd_result;
}

static int smblib_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct smb_charger *chg = container_of(nb, struct smb_charger, nb);

	if (!strcmp(psy->desc->name, "bms")) {
		if (!chg->bms_psy)
			chg->bms_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED)
			schedule_work(&chg->bms_update_work);
	}

	if (!chg->pl.psy && !strcmp(psy->desc->name, "parallel"))
		chg->pl.psy = psy;

	return NOTIFY_OK;
}

static int smblib_register_notifier(struct smb_charger *chg)
{
	int rc;

	chg->nb.notifier_call = smblib_notifier_call;
	rc = power_supply_reg_notifier(&chg->nb);
	if (rc < 0) {
		smblib_err(chg, "Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	*val_raw = val_u << 1;

	return 0;
}

int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw)
{
	int val_u  = val_raw * param->step_u + param->min_u;

	if (val_u > param->max_u)
		val_u -= param->max_u * 2;

	return val_u;
}

int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u - param->max_u)
		return -EINVAL;

	val_u += param->max_u * 2 - param->min_u;
	val_u %= param->max_u * 2;
	*val_raw = val_u / param->step_u;

	return 0;
}

static void smblib_uusb_removal(struct smb_charger *chg)
{
	int rc;

	/* reset both usbin current and voltage votes */
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, true, 0);

	cancel_delayed_work_sync(&chg->hvdcp_detect_work);

	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		/* re-enable AUTH_IRQ_EN_CFG_BIT */
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, AUTH_IRQ_EN_CFG_BIT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable QC auth setting rc=%d\n", rc);
	}

	/* reconfigure allowed voltage for HVDCP */
	rc = smblib_set_adapter_allowance(chg,
			USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V rc=%d\n",
			rc);

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
	chg->usb_icl_delta_ua = 0;
	chg->pulse_cnt = 0;

	/* clear USB ICL vote for USB_PSY_VOTER */
	rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote for USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for DCP_VOTER */
	rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote DCP from USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for PL_USBIN_USBIN_VOTER */
	rc = vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote PL_USBIN_USBIN from USB ICL rc=%d\n",
			rc);
}

static bool smblib_sysok_reason_usbin(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, SYSOK_REASON_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get SYSOK_REASON_STATUS rc=%d\n", rc);
		/* assuming 'not usbin' in case of read failure */
		return false;
	}

	return stat & SYSOK_REASON_USBIN_BIT;
}

void smblib_suspend_on_debug_battery(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval val;

	if (!chg->suspend_input_on_debug_batt)
		return;

	rc = power_supply_get_property(chg->bms_psy,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get debug battery prop rc=%d\n", rc);
		return;
	}

	vote(chg->usb_icl_votable, DEBUG_BOARD_VOTER, val.intval, 0);
	vote(chg->dc_suspend_votable, DEBUG_BOARD_VOTER, val.intval, 0);
	if (val.intval)
		pr_info("Input suspended: Fake battery\n");
}

int smblib_rerun_apsd_if_required(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result;
	union power_supply_propval val;
	int rc;

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb present rc = %d\n", rc);
		return rc;
	}

	if (!val.intval)
		return 0;

	apsd_result = smblib_get_apsd_result(chg);
	if ((apsd_result->pst == POWER_SUPPLY_TYPE_UNKNOWN)
		|| (apsd_result->pst == POWER_SUPPLY_TYPE_USB)) {
		/* rerun APSD */
		pr_info("Reruning APSD type = %s at bootup\n",
				apsd_result->name);
		rc = smblib_masked_write(chg, CMD_APSD_REG,
					APSD_RERUN_BIT,
					APSD_RERUN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't rerun APSD rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}

#ifdef CONFIG_HTC_BATT
int smblib_rerun_apsd(struct smb_charger *chg)
{
	int rc;

	pr_info("%s: RERUN-APSD\n", __func__);
	rc = smblib_masked_write(chg, CMD_APSD_REG,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't rerun APSD rc = %d\n", rc);
		return rc;
	}

	return 0;
}
#endif //CONFIG_HTC_BATT

static int smblib_get_pulse_cnt(struct smb_charger *chg, int *count)
{
	int rc;
	u8 val[2];

	switch (chg->smb_version) {
	case PMI8998_SUBTYPE:
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, val);
		if (rc) {
			pr_err("failed to read QC_PULSE_COUNT_STATUS_REG rc=%d\n",
					rc);
			return rc;
		}
		*count = val[0] & QC_PULSE_COUNT_MASK;
		break;
	case PM660_SUBTYPE:
		rc = smblib_multibyte_read(chg,
				QC_PULSE_COUNT_STATUS_1_REG, val, 2);
		if (rc) {
			pr_err("failed to read QC_PULSE_COUNT_STATUS_1_REG rc=%d\n",
					rc);
			return rc;
		}
		*count = (val[1] << 8) | val[0];
		break;
	default:
		smblib_dbg(chg, PR_PARALLEL, "unknown SMB chip %d\n",
				chg->smb_version);
		return -EINVAL;
	}

	return 0;
}

/*********************
 * VOTABLE CALLBACKS *
 *********************/

static int smblib_dc_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	/* resume input if suspend is invalid */
	if (suspend < 0)
		suspend = 0;

	return smblib_set_dc_suspend(chg, (bool)suspend);
}

#ifdef CONFIG_HTC_BATT
static int smblib_usb_icl_set_high_current (struct smb_charger *chg)
{
	int rc = 0;

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
					USBIN_MODE_CHG_BIT,
					USBIN_MODE_CHG_BIT);
	if (rc < 0){
		smblib_err(chg, "Couldn't set USB HC MODE rc=%d\n", rc);
		return rc;
	}

	/* override APSD ICL limit */
	rc = smblib_masked_write(chg, CMD_APSD_REG,
					ICL_OVERRIDE_BIT,
					ICL_OVERRIDE_BIT);
	if (rc < 0){
		smblib_err(chg, "Couldn't set ICL OVERRIDE rc=%d\n", rc);
		return rc;
	}

	return rc;

}
#endif //CONFIG_HTC_BATT

#define USBIN_25MA	25000
#define USBIN_100MA	100000
#define USBIN_150MA	150000
#define USBIN_500MA	500000
#define USBIN_900MA	900000

#ifdef CONFIG_HTC_BATT
#define TYPE_C_AICL_CHK_DELAY_MS	5000
#define USBIN_1000MA	1000000
#define USBIN_1500MA	1500000
#define USBIN_2000MA	2000000
#define USBIN_2500MA	2500000
#define USBIN_3000MA	3000000
#define USBIN_SUSPEND_MA	0
static int smblib_usb_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	u8 icl_options = 0;
	union power_supply_propval val = {0, };
	int cc_type = POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;

	smblib_err(chg, "DEBUG: client = %s, icl_ua = %d\n", client, icl_ua);

	/* suspend and return if 0mA is requested */
	if (client && (icl_ua == USBIN_SUSPEND_MA)) {
		smblib_err(chg, "Set ICL = 0, suspend input\n");
		return smblib_set_usb_suspend(chg, true);
	}

	smblib_set_usb_suspend(chg, false);

	rc = smblib_get_prop_typec_mode(chg, &val);
	if (rc < 0)
		smblib_err(chg, "Couldn't get prop typec mode rc=%d\n", rc);
	else
		cc_type = val.intval;

	if ((chg->pd_active) || ((chg->cc_first_det_sts != CC_DET_DEFAULT) &&
		(chg->usb_psy_desc.type != POWER_SUPPLY_TYPE_USB) && (chg->usb_psy_desc.type != POWER_SUPPLY_TYPE_USB_CDP))) {
		// Set PD current for PD and TYPE-C charger in high/medium mode
		if ((chg->pd_active) || (cc_type == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM) ||
				(cc_type == POWER_SUPPLY_TYPEC_SOURCE_HIGH)) {
			rc = smblib_get_prop_pd_current_max(chg, &val);
			if (rc < 0)
				smblib_err(chg, "Couldn't get prop pd current max rc=%d\n", rc);
			if (val.intval < USBIN_500MA)
				icl_ua = USBIN_500MA;
			else
				icl_ua = val.intval;

			rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);
			if (rc < 0) {
				smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
				return rc;
			}
			rc = smblib_usb_icl_set_high_current(chg);
			if (rc< 0)
				return rc;

			if (chg->pd_active)
				smblib_dbg(chg, PR_INTERRUPT, "Set PD ICL = %d\n", icl_ua);
			else {
				smblib_dbg(chg, PR_INTERRUPT, "Set TYPEC ICL = %d\n", icl_ua);
				if (delayed_work_pending(&chg->type_c_aicl_chk_work))
					cancel_delayed_work_sync(&chg->type_c_aicl_chk_work);
				schedule_delayed_work(&chg->type_c_aicl_chk_work, msecs_to_jiffies(TYPE_C_AICL_CHK_DELAY_MS));
			}
			return rc;
		}
	}

	// Set ICL Option
	switch (chg->usb_psy_desc.type) {
	case POWER_SUPPLY_TYPE_USB_CDP:
		if (icl_ua > USBIN_1500MA)
			icl_ua = USBIN_1500MA;
		else if (icl_ua < USBIN_25MA)
			icl_ua = USBIN_500MA;
		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
			return rc;
		}
		rc = smblib_usb_icl_set_high_current(chg);
		if (rc< 0)
			return rc;
		break;
	case POWER_SUPPLY_TYPE_USB:
		if (icl_ua == USBIN_100MA)
			icl_options = 0;
		else if (icl_ua == USBIN_150MA)
			icl_options = CFG_USB3P0_SEL_BIT;
		else if (icl_ua == USBIN_500MA)
			icl_options = USB51_MODE_BIT;
		else if (icl_ua == USBIN_900MA)
			icl_options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		else {
			smblib_err(chg, "ICL %duA isn't supported for SDP, force 500mA\n", icl_ua);
			icl_ua = USBIN_500MA;
			icl_options = USB51_MODE_BIT;
		}
		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);
		rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
					CFG_USB3P0_SEL_BIT | USB51_MODE_BIT | USBIN_MODE_CHG_BIT,
					icl_options);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set ICL opetions rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_USB_HVDCP:
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
			return rc;
		}
		rc = smblib_usb_icl_set_high_current(chg);
		if (rc< 0)
			return rc;
		break;
	default:
		break;
	}

	smblib_dbg(chg, PR_INTERRUPT, "Set ICL = %d\n", icl_ua);

	return rc;
}
#else
static int set_sdp_current(struct smb_charger *chg, int icl_ua)
{
	int rc;
	u8 icl_options;

	/* power source is SDP */
	switch (icl_ua) {
	case USBIN_100MA:
		/* USB 2.0 100mA */
		icl_options = 0;
		break;
	case USBIN_150MA:
		/* USB 3.0 150mA */
		icl_options = CFG_USB3P0_SEL_BIT;
		break;
	case USBIN_500MA:
		/* USB 2.0 500mA */
		icl_options = USB51_MODE_BIT;
		break;
	case USBIN_900MA:
		/* USB 3.0 900mA */
		icl_options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		break;
	default:
		smblib_err(chg, "ICL %duA isn't supported for SDP\n", icl_ua);
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
		CFG_USB3P0_SEL_BIT | USB51_MODE_BIT, icl_options);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL options rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int smblib_usb_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool override;
	union power_supply_propval pval;

	/* suspend and return if 25mA or less is requested */
	if (client && (icl_ua < USBIN_25MA))
		return smblib_set_usb_suspend(chg, true);

	disable_irq_nosync(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	if (!client)
		goto override_suspend_config;

	rc = smblib_get_prop_typec_mode(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get typeC mode rc = %d\n", rc);
		goto enable_icl_changed_interrupt;
	}

	/* configure current */
	if (pval.intval == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
		&& (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB)) {
		rc = set_sdp_current(chg, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set SDP ICL rc=%d\n", rc);
			goto enable_icl_changed_interrupt;
		}
	} else {
		rc = smblib_set_charge_param(chg, &chg->param.usb_icl,
				icl_ua - chg->icl_reduction_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
			goto enable_icl_changed_interrupt;
		}
	}

override_suspend_config:
	/* determine if override needs to be enforced */
	override = true;
	if (client == NULL) {
		/* remove override if no voters - hw defaults is desired */
		override = false;
	} else if (pval.intval == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) {
		if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB)
			/* For std cable with type = SDP never override */
			override = false;
		else if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_CDP
			&& icl_ua - chg->icl_reduction_ua == 1500000)
			/*
			 * For std cable with type = CDP override only if
			 * current is not 1500mA
			 */
			override = false;
	}

	/* enforce override */
	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
		USBIN_MODE_CHG_BIT, override ? USBIN_MODE_CHG_BIT : 0);

	rc = smblib_icl_override(chg, override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL override rc=%d\n", rc);
		goto enable_icl_changed_interrupt;
	}

	/* unsuspend after configuring current and override */
	rc = smblib_set_usb_suspend(chg, false);
	if (rc < 0) {
		smblib_err(chg, "Couldn't resume input rc=%d\n", rc);
		goto enable_icl_changed_interrupt;
	}

enable_icl_changed_interrupt:
	enable_irq(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	return rc;
}
#endif //CONFIG_HTC_BATT

static int smblib_dc_icl_vote_callback(struct votable *votable, void *data,
			int icl_ua, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;
	bool suspend;

	if (icl_ua < 0) {
		smblib_dbg(chg, PR_MISC, "No Voter hence suspending\n");
		icl_ua = 0;
	}

	suspend = (icl_ua < USBIN_25MA);
	if (suspend)
		goto suspend;

	rc = smblib_set_charge_param(chg, &chg->param.dc_icl, icl_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set DC input current limit rc=%d\n",
			rc);
		return rc;
	}

suspend:
	rc = vote(chg->dc_suspend_votable, USER_VOTER, suspend, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			suspend ? "suspend" : "resume", rc);
		return rc;
	}
	return rc;
}

static int smblib_pd_disallowed_votable_indirect_callback(
	struct votable *votable, void *data, int disallowed, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = vote(chg->pd_allowed_votable, PD_DISALLOWED_INDIRECT_VOTER,
		!disallowed, 0);

	return rc;
}

static int smblib_awake_vote_callback(struct votable *votable, void *data,
			int awake, const char *client)
{
	struct smb_charger *chg = data;

	if (awake)
		pm_stay_awake(chg->dev);
	else
		pm_relax(chg->dev);

	return 0;
}

static int smblib_chg_disable_vote_callback(struct votable *votable, void *data,
			int chg_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				 CHARGING_ENABLE_CMD_BIT,
				 chg_disable ? 0 : CHARGING_ENABLE_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}

static int smblib_pl_enable_indirect_vote_callback(struct votable *votable,
			void *data, int chg_enable, const char *client)
{
	struct smb_charger *chg = data;

	vote(chg->pl_disable_votable, PL_INDIRECT_VOTER, !chg_enable, 0);

	return 0;
}

static int smblib_hvdcp_enable_vote_callback(struct votable *votable,
			void *data,
			int hvdcp_enable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;
	u8 val = HVDCP_AUTH_ALG_EN_CFG_BIT | HVDCP_EN_BIT;

	/* vote to enable/disable HW autonomous INOV */
	vote(chg->hvdcp_hw_inov_dis_votable, client, !hvdcp_enable, 0);

	/*
	 * Disable the autonomous bit and auth bit for disabling hvdcp.
	 * This ensures only qc 2.0 detection runs but no vbus
	 * negotiation happens.
	 */
	if (!hvdcp_enable)
		val = HVDCP_EN_BIT;

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				 HVDCP_EN_BIT | HVDCP_AUTH_ALG_EN_CFG_BIT,
				 val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s hvdcp rc=%d\n",
			hvdcp_enable ? "enable" : "disable", rc);
		return rc;
	}

	return 0;
}

static int smblib_hvdcp_disable_indirect_vote_callback(struct votable *votable,
			void *data, int hvdcp_disable, const char *client)
{
	struct smb_charger *chg = data;

	vote(chg->hvdcp_enable_votable, HVDCP_INDIRECT_VOTER,
			!hvdcp_disable, 0);

	return 0;
}

static int smblib_apsd_disable_vote_callback(struct votable *votable,
			void *data,
			int apsd_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	if (apsd_disable) {
		/* Don't run APSD on CC debounce when APSD is disabled */
		rc = smblib_masked_write(chg, TYPE_C_CFG_REG,
							APSD_START_ON_CC_BIT,
							0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable APSD_START_ON_CC rc=%d\n",
									rc);
			return rc;
		}

		rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
							AUTO_SRC_DETECT_BIT,
							0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable APSD rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
							AUTO_SRC_DETECT_BIT,
							AUTO_SRC_DETECT_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable APSD rc=%d\n", rc);
			return rc;
		}

		rc = smblib_masked_write(chg, TYPE_C_CFG_REG,
							APSD_START_ON_CC_BIT,
							APSD_START_ON_CC_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable APSD_START_ON_CC rc=%d\n",
									rc);
			return rc;
		}
	}

#ifdef CONFIG_HTC_BATT
		if (!apsd_disable){
			smblib_dbg(chg, PR_INTERRUPT, "rerun APSD for apsd re-enable\n");
			smblib_rerun_apsd(chg);
		}
#endif //CONFIG_HTC_BATT

	return 0;
}

static int smblib_hvdcp_hw_inov_dis_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	if (disable) {
		/*
		 * the pulse count register get zeroed when autonomous mode is
		 * disabled. Track that in variables before disabling
		 */
		rc = smblib_get_pulse_cnt(chg, &chg->pulse_cnt);
		if (rc < 0) {
			pr_err("failed to read QC_PULSE_COUNT_STATUS_REG rc=%d\n",
					rc);
			return rc;
		}
	}

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
			HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT,
			disable ? 0 : HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s hvdcp rc=%d\n",
				disable ? "disable" : "enable", rc);
		return rc;
	}

	return rc;
}

/**********************
 * TPS61099 REGULATOR *
 * ********************/

int smblib_set_poweroff_sel(struct smb_charger *chg,
		const union power_supply_propval *val)
{
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);

	if (val->intval == 0 && regulator_is_enabled(chg->ext_power)) {
		rc = regulator_disable(chg->ext_power);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable TPS61099 power rc=%d\n", rc);
			goto unlock;
		}
	} else if (val->intval == 1) {
		rc = smblib_vbus_disable(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable PMI8998 vbus power rc=%d\n", rc);
			goto unlock;
		}
	}

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

int smblib_get_ext_otg_control(struct smb_charger *chg,
		union power_supply_propval *val)
{
	mutex_lock(&chg->otg_oc_lock);
	val->intval = regulator_is_enabled(chg->ext_power);
	mutex_unlock(&chg->otg_oc_lock);
	return 0;
}

int smblib_set_ext_otg_control(struct smb_charger *chg,
		const union power_supply_propval *val)
{
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (val->intval == 0 && regulator_is_enabled(chg->ext_power)) {
		rc = regulator_disable(chg->ext_power);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable TPS61099 power rc=%d\n", rc);
			goto unlock;
		}
	} else if (val->intval == 1 && !regulator_is_enabled(chg->ext_power)){
		rc = regulator_enable(chg->ext_power);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable TPS61099 power rc=%d\n", rc);
			goto unlock;
		}
	}

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

static int smblib_usb_irq_enable_vote_callback(struct votable *votable,
				void *data, int enable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[INPUT_CURRENT_LIMIT_IRQ].irq ||
				!chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq)
		return 0;

	if (enable) {
		enable_irq(chg->irq_info[INPUT_CURRENT_LIMIT_IRQ].irq);
#ifdef CONFIG_HTC_BATT /* Not enable HDC irq to fix irq storm with QC3 */
		if (!is_gc_charger_vote_usb_irq_enable)
#endif
		enable_irq(chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	} else {
		disable_irq(chg->irq_info[INPUT_CURRENT_LIMIT_IRQ].irq);
		disable_irq(chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	}

	return 0;
}

/*******************
 * VCONN REGULATOR *
 * *****************/

#define MAX_OTG_SS_TRIES 2
static int _smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	u8 otg_stat, stat4;
	int rc = 0, i;

	if (!chg->external_vconn) {
		/*
		 * Hardware based OTG soft start should complete within 1ms, so
		 * wait for 2ms in the worst case.
		 */
		for (i = 0; i < MAX_OTG_SS_TRIES; ++i) {
			usleep_range(1000, 1100);
			rc = smblib_read(chg, OTG_STATUS_REG, &otg_stat);
			if (rc < 0) {
				smblib_err(chg, "Couldn't read OTG status rc=%d\n",
									rc);
				return rc;
			}

			if (otg_stat & BOOST_SOFTSTART_DONE_BIT)
				break;
		}

		if (!(otg_stat & BOOST_SOFTSTART_DONE_BIT)) {
			smblib_err(chg, "Couldn't enable VCONN; OTG soft start failed\n");
			return -EAGAIN;
		}
	}

	/*
	 * VCONN_EN_ORIENTATION is overloaded with overriding the CC pin used
	 * for Vconn, and it should be set with reverse polarity of CC_OUT.
	 */
	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat4);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_OTG, "enabling VCONN\n");
	if (chg->vconn_sel == 1 ) {
		stat4 = 0;
	} else if (chg->vconn_sel == 2) {
		stat4 = VCONN_EN_ORIENTATION_BIT;
	} else if (chg->vconn_sel == 0) {
		stat4 = stat4 & CC_ORIENTATION_BIT ? 0 : VCONN_EN_ORIENTATION_BIT;
	}

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT | VCONN_EN_ORIENTATION_BIT,
				 VCONN_EN_VALUE_BIT | stat4);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable vconn setting rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (chg->vconn_en)
		goto unlock;

	rc = _smblib_vconn_regulator_enable(rdev);
	if (rc >= 0)
		chg->vconn_en = true;

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

static int _smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	smblib_dbg(chg, PR_OTG, "disabling VCONN\n");
	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable vconn regulator rc=%d\n", rc);

	return rc;
}

int smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (!chg->vconn_en)
		goto unlock;

	rc = _smblib_vconn_regulator_disable(rdev);
	if (rc >= 0)
		chg->vconn_en = false;

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&chg->otg_oc_lock);
	ret = chg->vconn_en;
	mutex_unlock(&chg->otg_oc_lock);
	return ret;
}

/*****************
 * OTG REGULATOR *
 *****************/
#define MAX_RETRY		35
#define MIN_DELAY_US		2000
#define MAX_DELAY_US		9000
static int _smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc, retry_count = 0, min_delay = MIN_DELAY_US;
	u8 stat;

	smblib_dbg(chg, PR_OTG, "halt 1 in 8 mode\n");
	rc = smblib_masked_write(chg, OTG_ENG_OTG_CFG_REG,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set OTG_ENG_OTG_CFG_REG rc=%d\n",
			rc);
		return rc;
	}

	smblib_dbg(chg, PR_OTG, "enabling OTG\n");
	rc = smblib_write(chg, CMD_OTG_REG, OTG_EN_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable OTG regulator rc=%d\n", rc);
		return rc;
	}

	if (chg->wa_flags & OTG_WA) {
		/* check for softstart */
		do {
			usleep_range(min_delay, min_delay + 100);
			rc = smblib_read(chg, OTG_STATUS_REG, &stat);
			if (rc < 0) {
				smblib_err(chg,
					"Couldn't read OTG status rc=%d\n",
					rc);
				goto out;
			}

			if (stat & BOOST_SOFTSTART_DONE_BIT) {
				rc = smblib_set_charge_param(chg,
					&chg->param.otg_cl, chg->otg_cl_ua);
				if (rc < 0)
					smblib_err(chg,
						"Couldn't set otg limit\n");
				break;
			}

			/* increase the delay for following iterations */
			if (retry_count > 5)
				min_delay = MAX_DELAY_US;
		} while (retry_count++ < MAX_RETRY);

		if (retry_count >= MAX_RETRY) {
			smblib_dbg(chg, PR_OTG, "Boost Softstart not done\n");
			goto out;
		}
	}

	return 0;
out:
	/* disable OTG if softstart failed */
	smblib_write(chg, CMD_OTG_REG, 0);
	return rc;
}

int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (chg->otg_en)
		goto unlock;

	rc = _smblib_vbus_regulator_enable(rdev);
	if (rc >= 0)
		chg->otg_en = true;

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

static int _smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	if (!chg->external_vconn && chg->vconn_en) {
		smblib_dbg(chg, PR_OTG, "Killing VCONN before disabling OTG\n");
		rc = _smblib_vconn_regulator_disable(rdev);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable VCONN rc=%d\n", rc);
	}

	if (chg->wa_flags & OTG_WA) {
		/* set OTG current limit to minimum value */
		rc = smblib_set_charge_param(chg, &chg->param.otg_cl,
						chg->param.otg_cl.min_u);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't set otg current limit rc=%d\n", rc);
			return rc;
		}
	}

	smblib_dbg(chg, PR_OTG, "disabling OTG\n");
	rc = smblib_write(chg, CMD_OTG_REG, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable OTG regulator rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_OTG, "start 1 in 8 mode\n");
	rc = smblib_masked_write(chg, OTG_ENG_OTG_CFG_REG,
				 ENG_BUCKBOOST_HALT1_8_MODE_BIT, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set OTG_ENG_OTG_CFG_REG rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_vbus_disable(struct smb_charger *chg)
{
	int rc = 0;

	if (chg->otg_en) {
		rc = _smblib_vbus_regulator_disable(chg->vbus_vreg->rdev);
		if (rc >= 0)
			chg->otg_en = false;
	}

	return rc;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);
	if (!chg->otg_en)
		goto unlock;

	rc = smblib_vbus_disable(chg);

unlock:
	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&chg->otg_oc_lock);
	ret = chg->otg_en;
	mutex_unlock(&chg->otg_oc_lock);
	return ret;
}

/********************
 * BATT PSY GETTERS *
 ********************/
#ifdef CONFIG_HTC_BATT
int smblib_get_usb_in_isen_adc(struct smb_charger *chg, struct qpnp_vadc_chip *vadc_usb_in_isen,
					unsigned int vadc_usb_in_isen_channel ,union power_supply_propval *val)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	val->intval = -1;

	if(vadc_usb_in_isen_channel == 0)
		return -EINVAL;

	if (IS_ERR(vadc_usb_in_isen)) {
		vadc_usb_in_isen = qpnp_get_vadc(chg->dev, "usb_in_isen");
	}

	if (IS_ERR(vadc_usb_in_isen)) {
		return -EINVAL;
	}

	if(vadc_usb_in_isen) {
		rc = qpnp_vadc_read(vadc_usb_in_isen, vadc_usb_in_isen_channel, &results);
		if (rc) {
			return -EINVAL;
		} else {
			val->intval =  (int)results.physical;
		}
	}
	return rc;
}

int smblib_get_usb_in_isen_current_ma(struct smb_charger *chg, unsigned int iusb_rsen,
						unsigned int iusb_multiplier, struct qpnp_vadc_chip *vadc_usb_in_isen,
                                        unsigned int vadc_usb_in_isen_channel, union power_supply_propval *val)
{
	int vsen = 0;
	union power_supply_propval ret = {0, };
	val->intval = -100;
	if (iusb_rsen)
	{
		smblib_get_usb_in_isen_adc(chg, vadc_usb_in_isen, vadc_usb_in_isen_channel, &ret);
		vsen = ret.intval;
		if (vsen >= 0) {
			val->intval = (vsen * iusb_multiplier / iusb_rsen) / 1000;
		} else {
			val->intval = -1;
		}
	} else {
		val->intval = -1;
	}
	return 0;
}
#endif

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	val->intval
		= (get_client_vote(chg->usb_icl_votable, USER_VOTER) == 0)
		 && get_client_vote(chg->dc_suspend_votable, USER_VOTER);
	return 0;
}

int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATIF_INT_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = !(stat & (BAT_THERM_OR_ID_MISSING_RT_STS_BIT
					| BAT_TERMINAL_MISSING_RT_STS_BIT));

	return rc;
}

int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (chg->fake_capacity >= 0) {
		val->intval = chg->fake_capacity;
		return 0;
	}

	if (chg->bms_psy)
		rc = power_supply_get_property(chg->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, val);
#ifdef CONFIG_HTC_BATT
			htc_battery_info_update(POWER_SUPPLY_PROP_CAPACITY, val->intval);
			val->intval = htc_battery_level_adjust();
#endif //CONFIG_HTC_BATT

	return rc;
}

int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	bool usb_online, dc_online;
	u8 stat;
	int rc;
#ifdef CONFIG_HTC_BATT
	int level = htc_battery_level_adjust();
#endif //CONFIG_HTC_BATT

	rc = smblib_get_prop_usb_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb online property rc=%d\n",
			rc);
		return rc;
	}
	usb_online = (bool)pval.intval;

	rc = smblib_get_prop_dc_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get dc online property rc=%d\n",
			rc);
		return rc;
	}
	dc_online = (bool)pval.intval;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (!usb_online && !dc_online) {
		switch (stat) {
		case TERMINATE_CHARGE:
		case INHIBIT_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		return rc;
	}

	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FAST_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
#ifdef CONFIG_HTC_BATT
		if (level == 100)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
#else
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
#endif //CONFIG_HTC_BATT
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
#ifdef CONFIG_HTC_BATT
		if (level == 100)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
#else
		val->intval = POWER_SUPPLY_STATUS_FULL;
#endif //CONFIG_HTC_BATT
		break;
	case DISABLE_CHARGE:
#ifdef CONFIG_HTC_BATT
		if (htc_battery_get_discharging_reason())
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
#else
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
#endif //CONFIG_HTC_BATT
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

#ifdef CONFIG_HTC_BATT
	if(get_client_vote_locked(chg->apsd_disable_votable, PD_HARD_RESET_VOTER) && usb_online){
		smblib_err(chg, "APSD_DISABLED, force report charging\n");
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
	}
#endif // CONFIG_HTC_BATT

	return 0;
}

int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case FAST_CHARGE:
	case FULLON_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TAPER;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval;
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_2 = 0x%02x\n",
		   stat);

	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT) {
		rc = smblib_get_prop_batt_voltage_now(chg, &pval);
		if (!rc) {
			/*
			 * If Vbatt is within 40mV above Vfloat, then don't
			 * treat it as overvoltage.
			 */
			if (pval.intval >=
				get_effective_result(chg->fv_votable) + 40000) {
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
				smblib_err(chg, "battery over-voltage\n");
				goto done;
			}
		}
	}

	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_LIMIT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_WARM;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

done:
	return rc;
}

int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->system_temp_level;
	return 0;
}

int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, AICL_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n", rc);
		return rc;
	}
	val->intval = (stat & SOFT_ILIMIT_BIT) || chg->is_hdc;
	return 0;
}

int smblib_get_prop_batt_voltage_now(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
	return rc;
}

int smblib_get_prop_batt_current_now(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_CURRENT_NOW, val);
	return rc;
}

int smblib_get_prop_batt_temp(struct smb_charger *chg,
			      union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy,
				       POWER_SUPPLY_PROP_TEMP, val);

#ifdef CONFIG_HTC_BATT
	if (get_kernel_flag() & KERNEL_FLAG_TEST_PWR_SUPPLY)
		val->intval = POWER_MONITOR_BATT_TEMP;
	if (val->intval > 650 && (get_kernel_flag() & KERNEL_FLAG_KEEP_CHARG_ON))
		val->intval = 650;
#endif //CONFIG_HTC_BATT

	return rc;
}

int smblib_get_prop_step_chg_step(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	if (!chg->step_chg_enabled) {
		val->intval = -1;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	val->intval = (stat & STEP_CHARGING_STATUS_MASK) >>
				STEP_CHARGING_STATUS_SHIFT;

	return rc;
}

int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	val->intval = (stat == TERMINATE_CHARGE);
	return 0;
}

#ifdef CONFIG_HTC_BATT
int smblib_get_ext_otg_chg_control(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc = 0;
	u8 stat = 0;

	rc = smblib_read(chg, USBIN_ADAPTER_ALLOW_CFG_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_ADAPTER_ALLOW_CFG_REG rc=%d\n",
			rc);
		return rc;
	}

	pr_info("USBIN_ADAPTER_ALLOW_CFG_REG = 0x%02x\n",stat);

	stat = stat & USBIN_ADAPTER_ALLOW_MASK;
	val->intval = !(stat == USBIN_ADAPTER_ALLOW_12V);

	return 0;
}
#endif // CONFIG_HTC_BATT

/***********************
 * BATTERY PSY SETTERS *
 ***********************/

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;

	/* vote 0mA when suspended */
	rc = vote(chg->usb_icl_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_capacity = val->intval;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels)
		return -EINVAL;

	chg->system_temp_level = val->intval;
	if (chg->system_temp_level == chg->thermal_levels)
		return vote(chg->chg_disable_votable,
			THERMAL_DAEMON_VOTER, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON_VOTER, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, true,
			chg->thermal_mitigation[chg->system_temp_level]);
	return 0;
}

int smblib_rerun_aicl(struct smb_charger *chg)
{
	int rc, settled_icl_ua;
	u8 stat;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
								rc);
		return rc;
	}

	/* USB is suspended so skip re-running AICL */
	if (stat & USBIN_SUSPEND_STS_BIT)
		return rc;

	smblib_dbg(chg, PR_MISC, "re-running AICL\n");
	switch (chg->smb_version) {
	case PMI8998_SUBTYPE:
		rc = smblib_get_charge_param(chg, &chg->param.icl_stat,
							&settled_icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
			return rc;
		}

		vote(chg->usb_icl_votable, AICL_RERUN_VOTER, true,
				max(settled_icl_ua - chg->param.usb_icl.step_u,
				chg->param.usb_icl.step_u));
		vote(chg->usb_icl_votable, AICL_RERUN_VOTER, false, 0);
		break;
	case PM660_SUBTYPE:
		/*
		 * Use restart_AICL instead of trigger_AICL as it runs the
		 * complete AICL instead of starting from the last settled
		 * value.
		 */
		rc = smblib_masked_write(chg, CMD_HVDCP_2_REG,
					RESTART_AICL_BIT, RESTART_AICL_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
									rc);
		break;
	default:
		smblib_dbg(chg, PR_PARALLEL, "unknown SMB chip %d\n",
				chg->smb_version);
		return -EINVAL;
	}

	return 0;
}

static int smblib_dp_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 increment */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_INCREMENT_BIT,
			SINGLE_INCREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

static int smblib_dm_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 decrement */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_DECREMENT_BIT,
			SINGLE_DECREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

int smblib_dp_dm(struct smb_charger *chg, int val)
{
	int target_icl_ua, rc = 0;

	switch (val) {
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		rc = smblib_dp_pulse(chg);
		if (!rc)
			chg->pulse_cnt++;
		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DP_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		rc = smblib_dm_pulse(chg);
		if (!rc && chg->pulse_cnt)
			chg->pulse_cnt--;
		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DM_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		chg->usb_icl_delta_ua -= 100000;
		target_icl_ua = get_effective_result(chg->usb_icl_votable);
		vote(chg->usb_icl_votable, SW_QC3_VOTER, true,
				target_icl_ua + chg->usb_icl_delta_ua);
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
	default:
		break;
	}

	return rc;
}

#ifdef CONFIG_HTC_BATT
int smblib_set_ext_otg_chg_control(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_masked_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG, USBIN_ADAPTER_ALLOW_MASK,
			val->intval ? USBIN_ADAPTER_ALLOW_5V_TO_9V : USBIN_ADAPTER_ALLOW_12V);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to USBIN_ADAPTER_ALLOW_MASK rc=%d\n",
			(bool)val->intval ? "5V_TO_9V" : "ALLOW_12V", rc);


	pr_info("[OTG] PMI8998 charger is %s\n", (bool)val->intval ? "enable" : "disable");

	return rc;
}
#endif // CONFIG_HTC_BATT

/*******************
 * DC PSY GETTERS *
 *******************/

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DCIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_dc_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote(chg->dc_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_DCIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);

	return rc;
}

int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_effective_result_locked(chg->dc_icl_votable);
	return 0;
}

/*******************
 * DC PSY SETTERS *
 * *****************/

int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	rc = vote(chg->dc_icl_votable, USER_VOTER, true, val->intval);
	return rc;
}

/*******************
 * USB PSY GETTERS *
 *******************/

int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_usb_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote(chg->usb_icl_votable, USER_VOTER) == 0) {
		val->intval = false;
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_USBIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);
	return rc;
}

int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
#ifdef CONFIG_HTC_BATT
#else
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;
#endif //CONFIG_HTC_BATT

	if (!chg->iio.usbin_v_chan ||
		PTR_ERR(chg->iio.usbin_v_chan) == -EPROBE_DEFER)
		chg->iio.usbin_v_chan = iio_channel_get(chg->dev, "usbin_v");

	if (IS_ERR(chg->iio.usbin_v_chan))
		return PTR_ERR(chg->iio.usbin_v_chan);

	return iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
}

int smblib_get_prop_pd_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_client_vote_locked(chg->usb_icl_votable, PD_VOTER);
	return 0;
}

int smblib_get_prop_usb_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	val->intval = get_client_vote_locked(chg->usb_icl_votable,
			USB_PSY_VOTER);
	return 0;
}

int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_get_prop_usb_present(chg, val);
	if (rc < 0 || !val->intval)
		return rc;

	if (!chg->iio.usbin_i_chan ||
		PTR_ERR(chg->iio.usbin_i_chan) == -EPROBE_DEFER)
		chg->iio.usbin_i_chan = iio_channel_get(chg->dev, "usbin_i");

	if (IS_ERR(chg->iio.usbin_i_chan))
		return PTR_ERR(chg->iio.usbin_i_chan);

	return iio_read_channel_processed(chg->iio.usbin_i_chan, &val->intval);
}

int smblib_get_prop_otg_state(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;
	u8 stat2;

	rc = smblib_read(chg, CMD_OTG_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CMD_OTG_REG rc=%d\n", rc);
		return rc;
	}

	rc = smblib_read(chg, OTG_STATUS_REG, &stat2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read OTG_STATUS_REG rc=%d\n", rc);
		return rc;
	}
	if ((stat2 & BOOST_SOFTSTART_DONE_BIT) && stat)
		val->intval = 1;
	else
		val->intval = 0;

	return rc;
}

int smblib_set_prop_otg_state(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc = 0;

	mutex_lock(&chg->otg_oc_lock);

	rc = _smblib_vbus_regulator_disable(chg->vbus_vreg->rdev);
	if (rc >= 0)
		chg->otg_en = false;

	mutex_unlock(&chg->otg_oc_lock);
	return rc;
}

int smblib_get_prop_charger_temp(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	int rc;

#ifdef CONFIG_HTC_BATT
        rc = smblib_get_prop_usb_present(chg, val);
        if (rc < 0 || !val->intval)
                return rc;
#endif //CONFIG_HTC_BATT


	if (!chg->iio.temp_chan ||
		PTR_ERR(chg->iio.temp_chan) == -EPROBE_DEFER)
		chg->iio.temp_chan = iio_channel_get(chg->dev, "charger_temp");

	if (IS_ERR(chg->iio.temp_chan))
		return PTR_ERR(chg->iio.temp_chan);

	rc = iio_read_channel_processed(chg->iio.temp_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_charger_temp_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.temp_max_chan ||
		PTR_ERR(chg->iio.temp_max_chan) == -EPROBE_DEFER)
		chg->iio.temp_max_chan = iio_channel_get(chg->dev,
							 "charger_temp_max");
	if (IS_ERR(chg->iio.temp_max_chan))
		return PTR_ERR(chg->iio.temp_max_chan);

	rc = iio_read_channel_processed(chg->iio.temp_max_chan, &val->intval);
	val->intval /= 100;
	return rc;
}

int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
					 union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n",
		   stat);

	if (stat & CC_ATTACHED_BIT)
		val->intval = (bool)(stat & CC_ORIENTATION_BIT) + 1;
	else
		val->intval = 0;

	return rc;
}

static const char * const smblib_typec_mode_name[] = {
	[POWER_SUPPLY_TYPEC_NONE]		  = "NONE",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT]	  = "SOURCE_DEFAULT",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM]	  = "SOURCE_MEDIUM",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH]	  = "SOURCE_HIGH",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT]	  = "NON_COMPLIANT",
	[POWER_SUPPLY_TYPEC_SINK]		  = "SINK",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE]   = "SINK_POWERED_CABLE",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY] = "SINK_DEBUG_ACCESSORY",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER]   = "SINK_AUDIO_ADAPTER",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY]   = "POWERED_CABLE_ONLY",
};

static int smblib_get_prop_ufp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_1 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_1 = 0x%02x\n", stat);

	switch (stat) {
	case 0:
		return POWER_SUPPLY_TYPEC_NONE;
	case UFP_TYPEC_RDSTD_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	case UFP_TYPEC_RD1P5_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case UFP_TYPEC_RD3P0_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NON_COMPLIANT;
}

static int smblib_get_prop_dfp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_2 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_2 = 0x%02x\n", stat);

	switch (stat & DFP_TYPEC_MASK) {
	case DFP_RA_RA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
	case DFP_RD_RD_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	case DFP_RD_RA_VCONN_BIT:
		return POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE;
	case DFP_RD_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_SINK;
	case DFP_RA_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

int smblib_get_prop_typec_mode(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		val->intval = POWER_SUPPLY_TYPEC_NONE;
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat);

	if (!(stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT)) {
		val->intval = POWER_SUPPLY_TYPEC_NONE;
		return rc;
	}

	if (stat & UFP_DFP_MODE_STATUS_BIT)
		val->intval = smblib_get_prop_dfp_mode(chg);
	else
		val->intval = smblib_get_prop_ufp_mode(chg);

	return rc;
}

int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc = 0;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL = 0x%02x\n",
		   ctrl);

	if (ctrl & TYPEC_DISABLE_CMD_BIT) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return rc;
	}

	switch (ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT)) {
	case 0:
		val->intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		break;
	case DFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
		break;
	case UFP_EN_CMD_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SINK;
		break;
	default:
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		smblib_err(chg, "unsupported power role 0x%02lx\n",
			ctrl & (DFP_EN_CMD_BIT | UFP_EN_CMD_BIT));
		return -EINVAL;
	}

	return rc;
}

int smblib_get_prop_pd_allowed(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = get_effective_result(chg->pd_allowed_votable);
	return 0;
}

int smblib_get_prop_input_current_settled(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

#define HVDCP3_STEP_UV	200000
int smblib_get_prop_input_voltage_settled(struct smb_charger *chg,
						union power_supply_propval *val)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);
	int rc, pulses;
	u8 stat;

	val->intval = MICRO_5V;
	if (apsd_result == NULL) {
		smblib_err(chg, "APSD result is NULL\n");
		return 0;
	}

	switch (apsd_result->pst) {
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return 0;
		}
		pulses = (stat & QC_PULSE_COUNT_MASK);
		val->intval = MICRO_5V + HVDCP3_STEP_UV * pulses;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc;
	u8 ctrl;

	rc = smblib_read(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG rc=%d\n",
			rc);
		return rc;
	}
	val->intval = ctrl & EXIT_SNK_BASED_ON_CC_BIT;
	return 0;
}

int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	/*
	 * hvdcp timeout voter is the last one to allow pd. Use its vote
	 * to indicate start of pe engine
	 */
	val->intval
		= !get_client_vote_locked(chg->pd_disallowed_votable_indirect,
			HVDCP_TIMEOUT_VOTER);
	return 0;
}

int smblib_get_prop_die_health(struct smb_charger *chg,
						union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TEMP_RANGE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TEMP_RANGE_STATUS_REG rc=%d\n",
									rc);
		return rc;
	}

	/* TEMP_RANGE bits are mutually exclusive */
	switch (stat & TEMP_RANGE_MASK) {
	case TEMP_BELOW_RANGE_BIT:
		val->intval = POWER_SUPPLY_HEALTH_COOL;
		break;
	case TEMP_WITHIN_RANGE_BIT:
		val->intval = POWER_SUPPLY_HEALTH_WARM;
		break;
	case TEMP_ABOVE_RANGE_BIT:
		val->intval = POWER_SUPPLY_HEALTH_HOT;
		break;
	case ALERT_LEVEL_BIT:
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		break;
	default:
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	return 0;
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (chg->pd_active)
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, val->intval);
	else
		rc = -EPERM;

	return rc;
}

int smblib_set_prop_usb_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc = 0;

	if (!chg->pd_active) {
		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
				true, val->intval);
	} else if (chg->system_suspend_supported) {
		if (val->intval <= USBIN_25MA)
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, true, val->intval);
		else
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, false, 0);
	}
	return rc;
}

int smblib_set_prop_boost_current(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_boost,
				val->intval <= chg->boost_threshold_ua ?
				chg->chg_freq.freq_below_otg_threshold :
				chg->chg_freq.freq_above_otg_threshold);
	if (rc < 0) {
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);
		return rc;
	}

	chg->boost_current_ua = val->intval;
	return rc;
}

int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				     const union power_supply_propval *val)
{
	int rc = 0;
	u8 power_role;

	switch (val->intval) {
	case POWER_SUPPLY_TYPEC_PR_NONE:
		power_role = TYPEC_DISABLE_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_DUAL:
		power_role = 0;
		break;
	case POWER_SUPPLY_TYPEC_PR_SINK:
		power_role = UFP_EN_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_SOURCE:
		power_role = DFP_EN_CMD_BIT;
		break;
	default:
		smblib_err(chg, "power role %d not supported\n", val->intval);
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, power_role);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			power_role, rc);
		return rc;
	}

	return rc;
}

int smblib_set_prop_usb_voltage_min(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, min_uv;

	min_uv = min(val->intval, chg->voltage_max_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, min_uv,
					       chg->voltage_max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid max voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER,
		     min_uv > MICRO_5V, 0);

	chg->voltage_min_uv = min_uv;
	return rc;
}

int smblib_set_prop_usb_voltage_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, max_uv;

#ifdef CONFIG_HTC_BATT
	if (val->intval > MICRO_9V) {
		smblib_err(chg, "invalid min voltage %duV\n",
			val->intval);
		return 0;
	}
#endif //CONFIG_HTC_BATT

	max_uv = max(val->intval, chg->voltage_min_uv);
	rc = smblib_set_usb_pd_allowed_voltage(chg, chg->voltage_min_uv,
					       max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid min voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_max_uv = max_uv;
	rc = smblib_rerun_aicl(chg);
	if (rc < 0)
		smblib_err(chg, "Couldn't re-run AICL rc=%d\n", rc);

	return rc;
}

int smblib_set_prop_pd_active(struct smb_charger *chg,
			      const union power_supply_propval *val)
{
	int rc;
	u8 stat = 0;
	bool cc_debounced;
	bool orientation;
	bool pd_active = val->intval;

	if (!get_effective_result(chg->pd_allowed_votable)) {
		smblib_err(chg, "PD is not allowed\n");
		return -EINVAL;
	}

	vote(chg->apsd_disable_votable, PD_VOTER, pd_active, 0);
	vote(chg->pd_allowed_votable, PD_VOTER, pd_active, 0);
	vote(chg->usb_irq_enable_votable, PD_VOTER, pd_active, 0);

	/*
	 * VCONN_EN_ORIENTATION_BIT controls whether to use CC1 or CC2 line
	 * when TYPEC_SPARE_CFG_BIT (CC pin selection s/w override) is set
	 * or when VCONN_EN_VALUE_BIT is set.
	 */
	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}

	if (pd_active) {
		orientation = stat & CC_ORIENTATION_BIT;
		rc = smblib_masked_write(chg,
				TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				VCONN_EN_ORIENTATION_BIT,
				orientation ? 0 : VCONN_EN_ORIENTATION_BIT);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't enable vconn on CC line rc=%d\n", rc);
			return rc;
		}
		/*
		 * Enforce 500mA for PD until the real vote comes in later.
		 * It is guaranteed that pd_active is set prior to
		 * pd_current_max
		 */
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_500MA);
		if (rc < 0) {
			smblib_err(chg, "Couldn't vote for USB ICL rc=%d\n",
					rc);
			return rc;
		}

		/* clear USB ICL vote for DCP_VOTER */
		rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't un-vote DCP from USB ICL rc=%d\n",
				rc);

		/* clear USB ICL vote for PL_USBIN_USBIN_VOTER */
		rc = vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
		if (rc < 0)
			smblib_err(chg,
					"Couldn't un-vote PL_USBIN_USBIN from USB ICL rc=%d\n",
					rc);

		/* remove USB_PSY_VOTER */
		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't unvote USB_PSY rc=%d\n", rc);
			return rc;
		}

		/* pd active set, parallel charger can be enabled now */
		rc = vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER,
				false, 0);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't unvote PL_DELAY_HVDCP_VOTER rc=%d\n",
				rc);
			return rc;
		}
	}

	/* CC pin selection s/w override in PD session; h/w otherwise. */
	rc = smblib_masked_write(chg, TAPER_TIMER_SEL_CFG_REG,
				 TYPEC_SPARE_CFG_BIT,
				 pd_active ? TYPEC_SPARE_CFG_BIT : 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't change cc_out ctrl to %s rc=%d\n",
			pd_active ? "SW" : "HW", rc);
		return rc;
	}

	cc_debounced = (bool)(stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT);
	if (!pd_active && cc_debounced)
		try_rerun_apsd_for_hvdcp(chg);

	chg->pd_active = pd_active;
	smblib_update_usb_type(chg);
	power_supply_changed(chg->usb_psy);

	return rc;
}

int smblib_set_prop_ship_mode(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "Set ship mode: %d!!\n", !!val->intval);

	rc = smblib_masked_write(chg, SHIP_MODE_REG, SHIP_MODE_EN_BIT,
			!!val->intval ? SHIP_MODE_EN_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't %s ship mode, rc=%d\n",
				!!val->intval ? "enable" : "disable", rc);

	return rc;
}

int smblib_reg_block_update(struct smb_charger *chg,
				struct reg_info *entry)
{
	int rc = 0;

	while (entry && entry->reg) {
		rc = smblib_read(chg, entry->reg, &entry->bak);
		if (rc < 0) {
			dev_err(chg->dev, "Error in reading %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry->bak &= entry->mask;

		rc = smblib_masked_write(chg, entry->reg,
					 entry->mask, entry->val);
		if (rc < 0) {
			dev_err(chg->dev, "Error in writing %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry++;
	}

	return rc;
}

int smblib_reg_block_restore(struct smb_charger *chg,
				struct reg_info *entry)
{
	int rc = 0;

	while (entry && entry->reg) {
		rc = smblib_masked_write(chg, entry->reg,
					 entry->mask, entry->bak);
		if (rc < 0) {
			dev_err(chg->dev, "Error in writing %s rc=%d\n",
				entry->desc, rc);
			break;
		}
		entry++;
	}

	return rc;
}

static struct reg_info cc2_detach_settings[] = {
	{
		.reg	= TYPE_C_CFG_REG,
		.mask	= APSD_START_ON_CC_BIT,
		.val	= 0,
		.desc	= "TYPE_C_CFG_REG",
	},
	{
		.reg	= TYPE_C_CFG_2_REG,
		.mask	= TYPE_C_UFP_MODE_BIT | EN_TRY_SOURCE_MODE_BIT,
		.val	= TYPE_C_UFP_MODE_BIT,
		.desc	= "TYPE_C_CFG_2_REG",
	},
	{
		.reg	= TYPE_C_CFG_3_REG,
		.mask	= EN_TRYSINK_MODE_BIT,
		.val	= 0,
		.desc	= "TYPE_C_CFG_3_REG",
	},
	{
		.reg	= TAPER_TIMER_SEL_CFG_REG,
		.mask	= TYPEC_SPARE_CFG_BIT,
		.val	= TYPEC_SPARE_CFG_BIT,
		.desc	= "TAPER_TIMER_SEL_CFG_REG",
	},
	{
		.reg	= TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
		.mask	= VCONN_EN_ORIENTATION_BIT,
		.val	= 0,
		.desc	= "TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG",
	},
	{
		.reg	= MISC_CFG_REG,
		.mask	= TCC_DEBOUNCE_20MS_BIT,
		.val	= TCC_DEBOUNCE_20MS_BIT,
		.desc	= "Tccdebounce time"
	},
	{
	},
};

static int smblib_cc2_sink_removal_enter(struct smb_charger *chg)
{
	int rc = 0;
	union power_supply_propval cc2_val = {0, };

	if ((chg->wa_flags & TYPEC_CC2_REMOVAL_WA_BIT) == 0)
		return rc;

	if (chg->cc2_sink_detach_flag != CC2_SINK_NONE)
		return rc;

	rc = smblib_get_prop_typec_cc_orientation(chg, &cc2_val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get cc orientation rc=%d\n", rc);
		return rc;
	}
	if (cc2_val.intval == 1)
		return rc;

	rc = smblib_get_prop_typec_mode(chg, &cc2_val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get prop typec mode rc=%d\n", rc);
		return rc;
	}

	switch (cc2_val.intval) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		smblib_reg_block_update(chg, cc2_detach_settings);
		chg->cc2_sink_detach_flag = CC2_SINK_STD;
		schedule_work(&chg->rdstd_cc2_detach_work);
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		chg->cc2_sink_detach_flag = CC2_SINK_MEDIUM_HIGH;
		break;
	default:
		break;
	}

	return rc;
}

static int smblib_cc2_sink_removal_exit(struct smb_charger *chg)
{
	int rc = 0;

	if ((chg->wa_flags & TYPEC_CC2_REMOVAL_WA_BIT) == 0)
		return rc;

	if (chg->cc2_sink_detach_flag == CC2_SINK_STD) {
		cancel_work_sync(&chg->rdstd_cc2_detach_work);
		smblib_reg_block_restore(chg, cc2_detach_settings);
	}

	chg->cc2_sink_detach_flag = CC2_SINK_NONE;

	return rc;
}

int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 EXIT_SNK_BASED_ON_CC_BIT,
				 (val->intval) ? EXIT_SNK_BASED_ON_CC_BIT : 0);
	if (rc < 0) {
		smblib_err(chg, "Could not set EXIT_SNK_BASED_ON_CC rc=%d\n",
				rc);
		return rc;
	}

	vote(chg->apsd_disable_votable, PD_HARD_RESET_VOTER, val->intval, 0);

	if (val->intval)
		rc = smblib_cc2_sink_removal_enter(chg);
	else
		rc = smblib_cc2_sink_removal_exit(chg);

	if (rc < 0) {
		smblib_err(chg, "Could not detect cc2 removal rc=%d\n", rc);
		return rc;
	}

	return rc;
}

/***********************
* USB MAIN PSY GETTERS *
*************************/
int smblib_get_prop_fcc_delta(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc, jeita_cc_delta_ua, step_cc_delta_ua, hw_cc_delta_ua = 0;

	rc = smblib_get_step_cc_delta(chg, &step_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get step cc delta rc=%d\n", rc);
		step_cc_delta_ua = 0;
	} else {
		hw_cc_delta_ua = step_cc_delta_ua;
	}

	rc = smblib_get_jeita_cc_delta(chg, &jeita_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc delta rc=%d\n", rc);
		jeita_cc_delta_ua = 0;
	} else if (jeita_cc_delta_ua < 0) {
		/* HW will take the min between JEITA and step charge */
		hw_cc_delta_ua = min(hw_cc_delta_ua, jeita_cc_delta_ua);
	}

	val->intval = hw_cc_delta_ua;
	return 0;
}

/***********************
* USB MAIN PSY SETTERS *
*************************/

#define SDP_CURRENT_MA			500000
#define CDP_CURRENT_MA			1500000
#define DCP_CURRENT_MA			1500000
#define HVDCP_CURRENT_MA		3000000
#define TYPEC_DEFAULT_CURRENT_MA	900000
#define TYPEC_MEDIUM_CURRENT_MA		1500000
#define TYPEC_HIGH_CURRENT_MA		3000000
static int smblib_get_charge_current(struct smb_charger *chg,
				int *total_current_ua)
{
	const struct apsd_result *apsd_result = smblib_update_usb_type(chg);
	union power_supply_propval val = {0, };
	int rc, typec_source_rd, current_ua;
	bool non_compliant;
	u8 stat5;

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat5);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return rc;
	}
	non_compliant = stat5 & TYPEC_NONCOMP_LEGACY_CABLE_STATUS_BIT;

	/* get settled ICL */
	rc = smblib_get_prop_input_current_settled(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
		return rc;
	}

	typec_source_rd = smblib_get_prop_ufp_mode(chg);

	/* QC 2.0/3.0 adapter */
	if (apsd_result->bit & (QC_3P0_BIT | QC_2P0_BIT)) {
		*total_current_ua = HVDCP_CURRENT_MA;
		return 0;
	}

	if (non_compliant) {
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_MA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = DCP_CURRENT_MA;
			break;
		default:
			current_ua = 0;
			break;
		}

		*total_current_ua = max(current_ua, val.intval);
		return 0;
	}

	switch (typec_source_rd) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_MA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = chg->default_icl_ua;
			break;
		default:
			current_ua = 0;
			break;
		}
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		current_ua = TYPEC_MEDIUM_CURRENT_MA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		current_ua = TYPEC_HIGH_CURRENT_MA;
		break;
	case POWER_SUPPLY_TYPEC_NON_COMPLIANT:
	case POWER_SUPPLY_TYPEC_NONE:
	default:
		current_ua = 0;
		break;
	}

	*total_current_ua = max(current_ua, val.intval);
	return 0;
}

int smblib_set_icl_reduction(struct smb_charger *chg, int reduction_ua)
{
	int current_ua, rc;

	if (reduction_ua == 0) {
		vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
	} else {
		/*
		 * No usb_icl voter means we are defaulting to hw chosen
		 * max limit. We need a vote from s/w to enforce the reduction.
		 */
		if (get_effective_result(chg->usb_icl_votable) == -EINVAL) {
			rc = smblib_get_charge_current(chg, &current_ua);
			if (rc < 0) {
				pr_err("Failed to get ICL rc=%d\n", rc);
				return rc;
			}
			vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, true,
					current_ua);
		}
	}

	chg->icl_reduction_ua = reduction_ua;

	return rerun_election(chg->usb_icl_votable);
}

/************************
 * PARALLEL PSY GETTERS *
 ************************/

int smblib_get_prop_slave_current_now(struct smb_charger *chg,
				      union power_supply_propval *pval)
{
	if (IS_ERR_OR_NULL(chg->iio.batt_i_chan))
		chg->iio.batt_i_chan = iio_channel_get(chg->dev, "batt_i");

	if (IS_ERR(chg->iio.batt_i_chan))
		return PTR_ERR(chg->iio.batt_i_chan);

	return iio_read_channel_processed(chg->iio.batt_i_chan, &pval->intval);
}

/**********************
 * INTERRUPT HANDLERS *
 **********************/

irqreturn_t smblib_handle_debug(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
#ifdef CONFIG_HTC_BATT
	bool is_log_print = true;

	if (strcmp(irq_data->name, "input-current-limiting") == 0)
		is_log_print = false;

	if (!is_log_print)
		smblib_dbg(chg, PR_REGISTER, "IRQ: %s\n", irq_data->name);
	else
#endif //CONFIG_HTC_BATT
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_otg_overcurrent(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;

	rc = smblib_read(chg, OTG_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read OTG_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if (chg->wa_flags & OTG_WA) {
		if (stat & OTG_OC_DIS_SW_STS_RT_STS_BIT)
			smblib_err(chg, "OTG disabled by hw\n");

		/* not handling software based hiccups for PM660 */
		return IRQ_HANDLED;
	}

	if (stat & OTG_OVERCURRENT_RT_STS_BIT)
		schedule_work(&chg->otg_oc_work);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return IRQ_HANDLED;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_state_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_step_chg_soc_update_fail(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->step_chg_enabled)
		rerun_election(chg->fcc_votable);

	return IRQ_HANDLED;
}

#define STEP_SOC_REQ_MS	3000
irqreturn_t smblib_handle_step_chg_soc_update_request(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	union power_supply_propval pval = {0, };

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (!chg->bms_psy) {
		schedule_delayed_work(&chg->step_soc_req_work,
				      msecs_to_jiffies(STEP_SOC_REQ_MS));
		return IRQ_HANDLED;
	}

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "Couldn't get batt capacity rc=%d\n", rc);
	else
		step_charge_soc_update(chg, pval.intval);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_temp_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	rerun_election(chg->fcc_votable);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_batt_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usb_psy_changed(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->usb_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usbin_uv(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct storm_watch *wdata;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	if (!chg->irq_info[SWITCH_POWER_OK_IRQ].irq_data)
		return IRQ_HANDLED;

	wdata = &chg->irq_info[SWITCH_POWER_OK_IRQ].irq_data->storm_data;
	reset_storm_count(wdata);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usb_plugin(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;
	bool vbus_rising;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	smblib_set_opt_freq_buck(chg,
		vbus_rising ? chg->chg_freq.freq_5V :
			chg->chg_freq.freq_removal);

	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
						"dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			smblib_err(chg, "Couldn't get dpdm regulator rc=%ld\n",
				PTR_ERR(chg->dpdm_reg));
			chg->dpdm_reg = NULL;
		}
	}

	if (vbus_rising) {
		if (chg->dpdm_reg && !regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg, "Couldn't enable dpdm regulator rc=%d\n",
					rc);
		}
	} else {
		if (chg->wa_flags & BOOST_BACK_WA)
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);

		if (chg->dpdm_reg && regulator_is_enabled(chg->dpdm_reg)) {
			smblib_dbg(chg, PR_MISC, "disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg, "Couldn't disable dpdm regulator rc=%d\n",
					rc);
		}

		if (chg->micro_usb_mode) {
			smblib_update_usb_type(chg);
			extcon_set_cable_state_(chg->extcon, EXTCON_USB, false);
			smblib_uusb_removal(chg);
		}
#ifdef CONFIG_HTC_BATT
			rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
					CFG_USB3P0_SEL_BIT | USB51_MODE_BIT | USBIN_MODE_CHG_BIT, 0);
			if (rc < 0)
				smblib_err(chg, "Couldn't set USBIN_ICL_OPTIONS rc=%d\n", rc);

			// Control ICL limit by APSD
			rc = smblib_masked_write(chg, USBIN_LOAD_CFG_REG,
						 ICL_OVERRIDE_AFTER_APSD_BIT, 0);
			smblib_dbg(chg, PR_INTERRUPT, "use APSD ICL setting\n");
			cancel_delayed_work_sync(&chg->chk_usb_icl_work);

			// Float charger related
			htc_notify_unknown_charger(false);
			chg->float_chk_sts = FLOAT_NONE;
			chg->float_chk_cnt = 0;
			cancel_delayed_work_sync(&chg->floating_chk_work);
			smblib_dbg(chg, PR_INTERRUPT, "CHK_FLOATING:%d,cnt=%d,\n",
					chg->float_chk_sts, chg->float_chk_cnt);

			// abnormal cc check
			chg->abnormal_cc_check_sts = false;

			// reverse boost check
			cancel_delayed_work_sync(&chg->reverse_boost_chk_work);

			// FTM WA for recover charging for none DCP charger
			if (!strcmp(htc_get_bootmode(),"ftm")) {
				smblib_err(chg, "%s: Under FTM mode, recover charging for cable out.", __func__);
				htc_ftm_disable_charger(false);
			}

			// cc pin floating cable
			g_cc_floating_cnt = 0;
			rc = smblib_masked_write(chg, TYPE_C_CFG_REG, TYPE_C_OR_U_USB_BIT, 0);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't write register, rc=%d\n", rc);
			cancel_delayed_work_sync(&chg->cc_floating_chk_work);

			// Clear CC detection result
			chg->cc_first_det_sts = CC_DET_NONE;
#endif //CONFIG_HTC_BATT
	}

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s %s\n",
		irq_data->name, vbus_rising ? "attached" : "detached");

	return IRQ_HANDLED;
}

#define USB_WEAK_INPUT_UA	1400000
#define ICL_CHANGE_DELAY_MS	1000
irqreturn_t smblib_handle_icl_change(int irq, void *data)
{
	u8 stat;
	int rc, settled_ua, delay = ICL_CHANGE_DELAY_MS;
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
#ifdef CONFIG_HTC_BATT
	static int pre_settled_ua = 0;
	smblib_dbg(chg, PR_REGISTER, "IRQ: %s\n", irq_data->name);
#endif //CONFIG_HTC_BATT

	if (chg->mode == PARALLEL_MASTER) {
		rc = smblib_read(chg, AICL_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n",
					rc);
			return IRQ_HANDLED;
		}

		rc = smblib_get_charge_param(chg, &chg->param.icl_stat,
				&settled_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
			return IRQ_HANDLED;
		}

		/* If AICL settled then schedule work now */
#ifndef CONFIG_HTC_BATT
		if ((settled_ua == get_effective_result(chg->usb_icl_votable))
				|| (stat & AICL_DONE_BIT))
#else
		if ((settled_ua == get_effective_result(chg->usb_icl_votable)) &&
				(pre_settled_ua != settled_ua))
#endif //CONFIG_HTC_BATT
			delay = 0;

#ifdef CONFIG_HTC_BATT
		pre_settled_ua = settled_ua;
#endif //CONFIG_HTC_BATT
		cancel_delayed_work_sync(&chg->icl_change_work);
		schedule_delayed_work(&chg->icl_change_work,
						msecs_to_jiffies(delay));
	}

	return IRQ_HANDLED;
}

static void smblib_handle_slow_plugin_timeout(struct smb_charger *chg,
					      bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: slow-plugin-timeout %s\n",
		   rising ? "rising" : "falling");
}

static void smblib_handle_sdp_enumeration_done(struct smb_charger *chg,
					       bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: sdp-enumeration-done %s\n",
		   rising ? "rising" : "falling");
}

#define QC3_PULSES_FOR_6V	5
#define QC3_PULSES_FOR_9V	20
#define QC3_PULSES_FOR_12V	35
static void smblib_hvdcp_adaptive_voltage_change(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	int pulses;

	power_supply_changed(chg->usb_main_psy);
	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_CHANGE_STATUS rc=%d\n", rc);
			return;
		}

		switch (stat & QC_2P0_STATUS_MASK) {
		case QC_5V_BIT:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_5V);
			break;
		case QC_9V_BIT:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_9V);
			break;
		case QC_12V_BIT:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_12V);
			break;
		default:
			smblib_set_opt_freq_buck(chg,
					chg->chg_freq.freq_removal);
			break;
		}
	}

	if (chg->usb_psy_desc.type == POWER_SUPPLY_TYPE_USB_HVDCP_3) {
		rc = smblib_read(chg, QC_PULSE_COUNT_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return;
		}
		pulses = (stat & QC_PULSE_COUNT_MASK);

		if (pulses < QC3_PULSES_FOR_6V)
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_5V);
		else if (pulses < QC3_PULSES_FOR_9V)
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_6V_8V);
		else if (pulses < QC3_PULSES_FOR_12V)
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_9V);
		else
			smblib_set_opt_freq_buck(chg,
				chg->chg_freq.freq_12V);
	}
}

/* triggers when HVDCP 3.0 authentication has finished */
static void smblib_handle_hvdcp_3p0_auth_done(struct smb_charger *chg,
					      bool rising)
{
	const struct apsd_result *apsd_result;
	int rc;

	if (!rising)
		return;

	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		/*
		 * Disable AUTH_IRQ_EN_CFG_BIT to receive adapter voltage
		 * change interrupt.
		 */
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, 0);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable QC auth setting rc=%d\n", rc);
	}

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, true, 0);

	/* the APSD done handler will set the USB supply type */
	apsd_result = smblib_get_apsd_result(chg);
	if (get_effective_result(chg->hvdcp_hw_inov_dis_votable)) {
		if (apsd_result->pst == POWER_SUPPLY_TYPE_USB_HVDCP) {
			/* force HVDCP2 to 9V if INOV is disabled */
			rc = smblib_masked_write(chg, CMD_HVDCP_2_REG,
					FORCE_9V_BIT, FORCE_9V_BIT);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't force 9V HVDCP rc=%d\n", rc);
		}
	}

	/* QC authentication done, parallel charger can be enabled now */
	vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, false, 0);

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-3p0-auth-done rising; %s detected\n",
		   apsd_result->name);
}

static void smblib_handle_hvdcp_check_timeout(struct smb_charger *chg,
					      bool rising, bool qc_charger)
{
	const struct apsd_result *apsd_result = smblib_update_usb_type(chg);

	/* Hold off PD only until hvdcp 2.0 detection timeout */
	if (rising) {
		vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
								false, 0);
		if (get_effective_result(chg->pd_disallowed_votable_indirect))
			/* could be a legacy cable, try doing hvdcp */
			try_rerun_apsd_for_hvdcp(chg);

		/* enable HDC and ICL irq for QC2/3 charger */
		if (qc_charger) {
#ifdef CONFIG_HTC_BATT /* Not enable HDC irq to fix QC3 stability issue */
			is_gc_charger_vote_usb_irq_enable = true;
#endif
			vote(chg->usb_irq_enable_votable, QC_VOTER, true, 0);
		}

		/*
		 * HVDCP detection timeout done
		 * If adapter is not QC2.0/QC3.0 - it is a plain old DCP.
		 */
		if (!qc_charger && (apsd_result->bit & DCP_CHARGER_BIT))
			/* enforce DCP ICL if specified */
			vote(chg->usb_icl_votable, DCP_VOTER,
				chg->dcp_icl_ua != -EINVAL, chg->dcp_icl_ua);
		/*
		 * If adapter is not QC2.0/QC3.0 remove vote for parallel
		 * disable.
		 * Otherwise if adapter is QC2.0/QC3.0 wait for authentication
		 * to complete.
		 */
		if (!qc_charger)
			vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER,
					false, 0);
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: smblib_handle_hvdcp_check_timeout %s\n",
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP is detected */
static void smblib_handle_hvdcp_detect_done(struct smb_charger *chg,
					    bool rising)
{
	if (!rising)
		return;

	/* the APSD done handler will set the USB supply type */
	cancel_delayed_work_sync(&chg->hvdcp_detect_work);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-detect-done %s\n",
		   rising ? "rising" : "falling");

#ifdef CONFIG_HTC_BATT
	// Control ICL limit by SW
	smblib_masked_write(chg, USBIN_LOAD_CFG_REG,
				ICL_OVERRIDE_AFTER_APSD_BIT, ICL_OVERRIDE_AFTER_APSD_BIT);
	smblib_dbg(chg, PR_INTERRUPT, "force SW ICL setting\n");
#endif //CONFIG_HTC_BATT
}

#define HVDCP_DET_MS 2500
#ifdef CONFIG_HTC_BATT
#define ICL_CHECK_DELAY_MS	10000
#define FLOAT_BOOST_CHK_DELAY_MS	1000
#define FLOAT_CHARGER_CHK_DELAY_MS	10000
#endif //CONFIG_HTC_BATT
static void smblib_handle_apsd_done(struct smb_charger *chg, bool rising)
{
	const struct apsd_result *apsd_result;
#ifdef CONFIG_HTC_BATT
	u8 apsd_result_sts = 0;
	u8 chgr_sts_8 = 0;
#endif //CONFIG_HTC_BATT

	if (!rising)
		return;

	smblib_read(chg, APSD_RESULT_STATUS_REG, &apsd_result_sts);
	smblib_read(chg, BATTERY_CHARGER_STATUS_8_REG, &chgr_sts_8);

	apsd_result = smblib_update_usb_type(chg);
	switch (apsd_result->bit) {
	case SDP_CHARGER_BIT:
	case CDP_CHARGER_BIT:
		if (chg->micro_usb_mode)
			extcon_set_cable_state_(chg->extcon, EXTCON_USB,
					true);
	case OCP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		/*
		 * if not DCP then no hvdcp timeout happens. Enable
		 * pd/parallel here.
		 */
		vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
				false, 0);
		vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, false, 0);
#ifdef CONFIG_HTC_BATT
		if (apsd_result_sts & FLOAT_CHARGER_BIT){
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true, USBIN_500MA);
			if (chg->float_chk_sts == FLOAT_CHECKING) {
				chg->float_chk_sts = FLOAT_DETECTED;
				htc_notify_unknown_charger(true);
			} else {
				if (chgr_sts_8 & PRE_TERM_BIT) {
					// schedule check for reverse boost
					if (chg->float_chk_sts == FLOAT_NONE)
						schedule_delayed_work(&chg->floating_chk_work, msecs_to_jiffies(FLOAT_BOOST_CHK_DELAY_MS));
				} else {
					chg->float_chk_sts = FLOAT_CHECKING;
					schedule_delayed_work(&chg->floating_chk_work, msecs_to_jiffies(FLOAT_CHARGER_CHK_DELAY_MS));
				}
			}
		} else {
			// Set ICL 1500mA for OCP
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true, USBIN_1500MA);
		}

		// FTM WA for disable charging for none DCP charger
		if (!strcmp(htc_get_bootmode(),"ftm")) {
			if (apsd_result_sts & SDP_CHARGER_BIT){
				smblib_err(chg, "%s: Under FTM mode, default disable charger.", __func__);
				htc_ftm_disable_charger(true);
			}
		}
#endif //CONFIG_HTC_BATT
		break;
	case DCP_CHARGER_BIT:
#ifdef CONFIG_HTC_BATT
		if (!chg->pd_active)
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true, USBIN_1500MA);
		smblib_masked_write(chg, CMD_HVDCP_2_REG, TRIGGER_AICL_BIT, TRIGGER_AICL_BIT);
		schedule_delayed_work(&chg->chk_usb_icl_work, msecs_to_jiffies(ICL_CHECK_DELAY_MS));
#endif //CONFIG_HTC_BATT
		if (chg->wa_flags & QC_CHARGER_DETECTION_WA_BIT)
			schedule_delayed_work(&chg->hvdcp_detect_work,
					      msecs_to_jiffies(HVDCP_DET_MS));
		break;
	default:
		break;
	}

#ifdef CONFIG_HTC_BATT
	rerun_election(chg->usb_icl_votable);

	// for reverse boost checking loop
	if (chgr_sts_8 & PRE_TERM_BIT) {
		if ((chg->float_chk_sts == FLOAT_BOOST_CHECKING) || (chg->float_chk_sts == FLOAT_BOOST_DETECTED))
			schedule_delayed_work(&chg->floating_chk_work, msecs_to_jiffies(FLOAT_BOOST_CHK_DELAY_MS));
	} else {
		if (apsd_result->bit != FLOAT_CHARGER_BIT){
			htc_notify_unknown_charger(false);
			chg->float_chk_sts = FLOAT_NONE;
			chg->float_chk_cnt = 0;
		}
	}
	smblib_dbg(chg, PR_INTERRUPT, "CHK_FLOATING:%d,cnt=%d,0x1308=0x%02x,0x100E=%02x\n",
					chg->float_chk_sts, chg->float_chk_cnt, apsd_result_sts, chgr_sts_8);
#endif //CONFIG_HTC_BATT

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: apsd-done rising; %s detected\n",
		   apsd_result->name);
}

irqreturn_t smblib_handle_usb_source_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc = 0;
	u8 stat;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);

	smblib_handle_apsd_done(chg,
		(bool)(stat & APSD_DTC_STATUS_DONE_BIT));

	smblib_handle_hvdcp_detect_done(chg,
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_check_timeout(chg,
		(bool)(stat & HVDCP_CHECK_TIMEOUT_BIT),
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_3p0_auth_done(chg,
		(bool)(stat & QC_AUTH_DONE_STATUS_BIT));

	smblib_handle_sdp_enumeration_done(chg,
		(bool)(stat & ENUMERATION_DONE_BIT));

	smblib_handle_slow_plugin_timeout(chg,
		(bool)(stat & SLOW_PLUGIN_TIMEOUT_BIT));

	smblib_hvdcp_adaptive_voltage_change(chg);

	power_supply_changed(chg->usb_psy);

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", stat);

	return IRQ_HANDLED;
}

static void typec_source_removal(struct smb_charger *chg)
{
	int rc;

	/* reset both usbin current and voltage votes */
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);

	cancel_delayed_work_sync(&chg->hvdcp_detect_work);

	if (chg->wa_flags & QC_AUTH_INTERRUPT_WA_BIT) {
		/* re-enable AUTH_IRQ_EN_CFG_BIT */
		rc = smblib_masked_write(chg,
				USBIN_SOURCE_CHANGE_INTRPT_ENB_REG,
				AUTH_IRQ_EN_CFG_BIT, AUTH_IRQ_EN_CFG_BIT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't enable QC auth setting rc=%d\n", rc);
	}

	/* reconfigure allowed voltage for HVDCP */
#ifdef CONFIG_HTC_BATT
	cancel_delayed_work_sync(&chg->cc_check_work);
	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_CFG_REG,
			  USBIN_ADAPTER_ALLOW_5V_TO_9V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_TO_9V rc=%d\n",
			rc);
#else
	rc = smblib_set_adapter_allowance(chg,
			USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V);
	if (rc < 0)
		smblib_err(chg, "Couldn't set USBIN_ADAPTER_ALLOW_5V_OR_9V_TO_12V rc=%d\n",
			rc);
#endif

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;

	/* clear USB ICL vote for PD_VOTER */
	rc = vote(chg->usb_icl_votable, PD_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote PD from USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for USB_PSY_VOTER */
	rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote USB_PSY from USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for DCP_VOTER */
	rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote DCP from USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for PL_USBIN_USBIN_VOTER */
	rc = vote(chg->usb_icl_votable, PL_USBIN_USBIN_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote PL_USBIN_USBIN from USB ICL rc=%d\n",
			rc);
}

static void typec_source_insertion(struct smb_charger *chg)
{
}

static void typec_sink_insertion(struct smb_charger *chg)
{
	/* when a sink is inserted we should not wait on hvdcp timeout to
	 * enable pd
	 */
	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
			false, 0);
}

static void typec_sink_removal(struct smb_charger *chg)
{
	smblib_set_charge_param(chg, &chg->param.freq_boost,
			chg->chg_freq.freq_above_otg_threshold);
	chg->boost_current_ua = 0;
}

static void smblib_handle_typec_removal(struct smb_charger *chg)
{
#ifdef CONFIG_HTC_BATT
	is_gc_charger_vote_usb_irq_enable = false;
#endif
	vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, LEGACY_CABLE_VOTER, true, 0);
	vote(chg->pd_disallowed_votable_indirect, VBUS_CC_SHORT_VOTER, true, 0);
	vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, true, 0);
	vote(chg->usb_irq_enable_votable, PD_VOTER, false, 0);
	vote(chg->usb_irq_enable_votable, QC_VOTER, false, 0);

	/* reset votes from vbus_cc_short */
	vote(chg->hvdcp_disable_votable_indirect, VBUS_CC_SHORT_VOTER,
			true, 0);
	vote(chg->hvdcp_disable_votable_indirect, PD_INACTIVE_VOTER,
			true, 0);
	/*
	 * cable could be removed during hard reset, remove its vote to
	 * disable apsd
	 */
	vote(chg->apsd_disable_votable, PD_HARD_RESET_VOTER, false, 0);

	chg->vconn_attempts = 0;
	chg->otg_attempts = 0;
	chg->pulse_cnt = 0;
	chg->usb_icl_delta_ua = 0;

	chg->usb_ever_removed = true;

	smblib_update_usb_type(chg);

	typec_source_removal(chg);
	typec_sink_removal(chg);
}

static void smblib_handle_typec_insertion(struct smb_charger *chg,
		bool sink_attached, bool legacy_cable)
{
	int rp;
	bool vbus_cc_short = false;
	bool valid_legacy_cable;
#ifdef CONFIG_HTC_BATT
	u8	reg;
#endif //CONFIG_HTC_BATT

	vote(chg->pd_disallowed_votable_indirect, CC_DETACHED_VOTER, false, 0);

	if (sink_attached) {
		typec_source_removal(chg);
		typec_sink_insertion(chg);
	} else {
		typec_source_insertion(chg);
		typec_sink_removal(chg);
	}

	valid_legacy_cable = legacy_cable &&
		(chg->usb_ever_removed || !smblib_sysok_reason_usbin(chg));
	vote(chg->pd_disallowed_votable_indirect, LEGACY_CABLE_VOTER,
			valid_legacy_cable, 0);

	if (valid_legacy_cable) {
		rp = smblib_get_prop_ufp_mode(chg);
		if (rp == POWER_SUPPLY_TYPEC_SOURCE_HIGH
				|| rp == POWER_SUPPLY_TYPEC_NON_COMPLIANT) {
			vbus_cc_short = true;
			smblib_err(chg, "Disabling PD and HVDCP, VBUS-CC shorted, rp = %d found\n",
					rp);
		}
	}

#ifdef CONFIG_HTC_BATT
	smblib_read(chg, TYPE_C_STATUS_5_REG, &reg);
	if (!(reg & TYPEC_LEGACY_CABLE_STATUS_BIT)){
		smblib_dbg(chg, PR_INTERRUPT, "0x130F = 0x%02x, prepare to rerun APSD\n", reg);
		schedule_delayed_work(&chg->cc_check_work, msecs_to_jiffies(10000));
	}
#endif //CONFIG_HTC_BATT

	vote(chg->hvdcp_disable_votable_indirect, VBUS_CC_SHORT_VOTER,
			vbus_cc_short, 0);
	vote(chg->pd_disallowed_votable_indirect, VBUS_CC_SHORT_VOTER,
			vbus_cc_short, 0);
}

static void smblib_handle_typec_debounce_done(struct smb_charger *chg,
			bool rising, bool sink_attached, bool legacy_cable)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (rising)
		smblib_handle_typec_insertion(chg, sink_attached, legacy_cable);
	else
		smblib_handle_typec_removal(chg);

	rc = smblib_get_prop_typec_mode(chg, &pval);
	if (rc < 0)
		smblib_err(chg, "Couldn't get prop typec mode rc=%d\n", rc);

	/*
	 * HW BUG - after cable is removed, medium or high rd reading
	 * falls to std. Use it for signal of typec cc detachment in
	 * software WA.
	 */
	if (chg->cc2_sink_detach_flag == CC2_SINK_MEDIUM_HIGH
		&& pval.intval == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) {

		chg->cc2_sink_detach_flag = CC2_SINK_WA_DONE;

		rc = smblib_masked_write(chg,
				TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				EXIT_SNK_BASED_ON_CC_BIT, 0);
		if (rc < 0)
			smblib_err(chg, "Couldn't get prop typec mode rc=%d\n",
				rc);
	}

	nanohub_vbus_status(pval.intval);

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: debounce-done %s; Type-C %s detected\n",
		   rising ? "rising" : "falling",
		   smblib_typec_mode_name[pval.intval]);
#ifdef CONFIG_HTC_BATT
	if (rising && (pval.intval == POWER_SUPPLY_TYPEC_NONE) && (g_cc_floating_cnt == 0)) {
		if (!delayed_work_pending(&chg->cc_floating_chk_work))
			schedule_delayed_work(&chg->cc_floating_chk_work, msecs_to_jiffies(3000));
	}

	// Avoid CC mis-detect when HVDCP change vbus
	if (chg->cc_first_det_sts == CC_DET_NONE) {
		if (pval.intval == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT)
			chg->cc_first_det_sts = CC_DET_DEFAULT;
		else if (pval.intval == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM)
			chg->cc_first_det_sts = CC_DET_MEDIUM;
		else if (pval.intval == POWER_SUPPLY_TYPEC_SOURCE_HIGH)
			chg->cc_first_det_sts = CC_DET_HIGH;
	}

	// Disable HVDCP for none 56K cable
	if (rising && (chg->cc_first_det_sts != CC_DET_DEFAULT)){
		vote(chg->hvdcp_disable_votable_indirect, VBUS_CC_SHORT_VOTER, true, 0);
	}

	if (rising && chg->cc_first_det_sts != CC_DET_DEFAULT) {
		if (pval.intval == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM) {
			vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_1500MA);
			rerun_election(chg->usb_icl_votable);
		} else if (pval.intval == POWER_SUPPLY_TYPEC_SOURCE_HIGH) {
			vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_3000MA);
			rerun_election(chg->usb_icl_votable);
		} else {
			vote(chg->usb_icl_votable, PD_VOTER, false, USBIN_3000MA);
			rerun_election(chg->usb_icl_votable);
		}
	}
#endif

}

irqreturn_t smblib_handle_usb_typec_change_for_uusb(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATUS_3_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_3 rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_3 = 0x%02x OTG=%d\n",
		stat, !!(stat & (U_USB_GND_NOVBUS_BIT | U_USB_GND_BIT)));

	extcon_set_cable_state_(chg->extcon, EXTCON_USB_HOST,
			!!(stat & (U_USB_GND_NOVBUS_BIT | U_USB_GND_BIT)));
	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_usb_typec_change(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat4, stat5;
	bool debounce_done, sink_attached, legacy_cable;

	if (chg->micro_usb_mode)
		return smblib_handle_usb_typec_change_for_uusb(chg);

	/* WA - not when PD hard_reset WIP on cc2 in sink mode */
	if (chg->cc2_sink_detach_flag == CC2_SINK_STD)
		return IRQ_HANDLED;

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat4);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat5);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	debounce_done = (bool)(stat4 & TYPEC_DEBOUNCE_DONE_STATUS_BIT);
	sink_attached = (bool)(stat4 & UFP_DFP_MODE_STATUS_BIT);
	legacy_cable = (bool)(stat5 & TYPEC_LEGACY_CABLE_STATUS_BIT);

	smblib_handle_typec_debounce_done(chg,
			debounce_done, sink_attached, legacy_cable);

	if (stat4 & TYPEC_VBUS_ERROR_STATUS_BIT)
		smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s vbus-error\n",
			irq_data->name);

	if (stat4 & TYPEC_VCONN_OVERCURR_STATUS_BIT)
		schedule_work(&chg->vconn_oc_work);

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat4);
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_5 = 0x%02x\n", stat5);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_dc_plugin(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	power_supply_changed(chg->dc_psy);
	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_high_duty_cycle(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	chg->is_hdc = true;
	schedule_delayed_work(&chg->clear_hdc_work, msecs_to_jiffies(60));

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_switcher_power_ok(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;
	u8 stat;

	if (!(chg->wa_flags & BOOST_BACK_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if ((stat & USE_USBIN_BIT) &&
			get_effective_result(chg->usb_icl_votable) < USBIN_25MA)
		return IRQ_HANDLED;

	if (stat & USE_DCIN_BIT)
		return IRQ_HANDLED;

	if (is_storming(&irq_data->storm_data)) {
		smblib_err(chg, "Reverse boost detected: voting 0mA to suspend input\n");
		vote(chg->usb_icl_votable, BOOST_BACK_VOTER, true, 0);
#ifdef CONFIG_HTC_BATT
		schedule_delayed_work(&chg->reverse_boost_chk_work,
				msecs_to_jiffies(1000));
#endif // CONFIG_HTC_BATT
	}

	return IRQ_HANDLED;
}

irqreturn_t smblib_handle_wdog_bark(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;

	rc = smblib_write(chg, BARK_BITE_WDOG_PET_REG, BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't pet the dog rc=%d\n", rc);

	return IRQ_HANDLED;
}

/***************
 * Work Queues *
 ***************/

static void smblib_hvdcp_detect_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					       hvdcp_detect_work.work);

	vote(chg->pd_disallowed_votable_indirect, HVDCP_TIMEOUT_VOTER,
				false, 0);
	if (get_effective_result(chg->pd_disallowed_votable_indirect))
		/* pd is still disabled, try hvdcp */
		try_rerun_apsd_for_hvdcp(chg);
	else
		/* notify pd now that pd is allowed */
		power_supply_changed(chg->usb_psy);
}

static void bms_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bms_update_work);

	smblib_suspend_on_debug_battery(chg);

	if (chg->batt_psy)
		power_supply_changed(chg->batt_psy);
}

static void step_soc_req_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						step_soc_req_work.work);
	union power_supply_propval pval = {0, };
	int rc;

	rc = smblib_get_prop_batt_capacity(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get batt capacity rc=%d\n", rc);
		return;
	}

	step_charge_soc_update(chg, pval.intval);
}

static void clear_hdc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						clear_hdc_work.work);

	chg->is_hdc = 0;
}

static void rdstd_cc2_detach_work(struct work_struct *work)
{
	int rc;
	u8 stat;
	struct smb_irq_data irq_data = {NULL, "cc2-removal-workaround"};
	struct smb_charger *chg = container_of(work, struct smb_charger,
						rdstd_cc2_detach_work);

	/*
	 * WA steps -
	 * 1. Enable both UFP and DFP, wait for 10ms.
	 * 2. Disable DFP, wait for 30ms.
	 * 3. Removal detected if both TYPEC_DEBOUNCE_DONE_STATUS
	 *    and TIMER_STAGE bits are gone, otherwise repeat all by
	 *    work rescheduling.
	 * Note, work will be cancelled when pd_hard_reset is 0.
	 */

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write TYPE_C_CTRL_REG rc=%d\n", rc);
		return;
	}

	usleep_range(10000, 11000);

	rc = smblib_masked_write(chg, TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 UFP_EN_CMD_BIT | DFP_EN_CMD_BIT,
				 UFP_EN_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write TYPE_C_CTRL_REG rc=%d\n", rc);
		return;
	}

	usleep_range(30000, 31000);

	rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
			rc);
		return;
	}
	if (stat & TYPEC_DEBOUNCE_DONE_STATUS_BIT)
		goto rerun;

	rc = smblib_read(chg, TYPE_C_STATUS_5_REG, &stat);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't read TYPE_C_STATUS_5_REG rc=%d\n", rc);
		return;
	}
	if (stat & TIMER_STAGE_2_BIT)
		goto rerun;

	/* Bingo, cc2 removal detected */
	smblib_reg_block_restore(chg, cc2_detach_settings);
	chg->cc2_sink_detach_flag = CC2_SINK_WA_DONE;
	irq_data.parent_data = chg;
	smblib_handle_usb_typec_change(0, &irq_data);

	return;

rerun:
	schedule_work(&chg->rdstd_cc2_detach_work);
}

#ifdef CONFIG_HTC_BATT
#define ICL_RECHK_DELAY_MS	1000
#define RE_CHECK_MAX			3
static void chk_usb_icl_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
					       chk_usb_icl_work.work);
	int aicl_result;
	u8 reg;
	static int chk_cnt = 0;

	if (chg->usb_psy_desc.type != POWER_SUPPLY_TYPE_USB_DCP)
		return;

	smblib_read(chg, AICL_STATUS_REG, &reg);

	if ((reg & AICL_DONE_BIT) || (chk_cnt >= RE_CHECK_MAX)) {
		smblib_get_charge_param(chg, &chg->param.icl_stat, &aicl_result);
		if (aicl_result == USBIN_1500MA) {
			// Start 5V/2A pre-check
			htc_5v2a_pre_chk();
			smblib_dbg(chg, PR_INTERRUPT, "AICL=%d, start pre-check\n", aicl_result/1000);
		} else if (aicl_result < USBIN_1500MA) {
			// Downgrade iusb to 1A
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true, USBIN_1000MA);
			smblib_dbg(chg, PR_INTERRUPT, "AICL=%d, downgrade 1A\n", aicl_result/1000);
		}
		chk_cnt = 0;
	} else {
		chk_cnt++;
		schedule_delayed_work(&chg->chk_usb_icl_work,
				msecs_to_jiffies(ICL_RECHK_DELAY_MS));
	}

	return;
}

static void smblib_cc_check_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
											cc_check_work.work);
	u8 reg;

	if (chg->abnormal_cc_check_sts)
		smblib_dbg(chg, PR_INTERRUPT, "cc-check done don't rerun APSD again\n");
	else {
		chg->abnormal_cc_check_sts = true;
		smblib_read(chg, TYPE_C_STATUS_5_REG, &reg);
		smblib_dbg(chg, PR_INTERRUPT, "re-check 0x130F = 0x%02x, RERUN APSD\n", reg);
		smblib_rerun_apsd(chg);
	}

	return;
}

#define FLOAT_BOOST_MAX_CNT	10
static void smblib_floating_chk_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
											floating_chk_work.work);
	u8 apsd_result_sts;
	u8 chgr_sts_8;
	smblib_read(chg, APSD_RESULT_STATUS_REG, &apsd_result_sts);
	smblib_read(chg, BATTERY_CHARGER_STATUS_8_REG, &chgr_sts_8);

	switch (chg->float_chk_sts) {
		case FLOAT_NONE:
			if (chgr_sts_8 & PRE_TERM_BIT) {
				chg->float_chk_cnt++;
			}
			chg->float_chk_sts = FLOAT_BOOST_CHECKING;
			smblib_rerun_apsd(chg);
			break;
		case FLOAT_BOOST_CHECKING:
			if (chgr_sts_8 & PRE_TERM_BIT) {
				chg->float_chk_cnt++;
				if (chg->float_chk_cnt >= FLOAT_BOOST_MAX_CNT) {
					chg->float_chk_sts = FLOAT_BOOST_DETECTED;
					htc_notify_unknown_charger(true);
					chg->float_chk_cnt = 0;
				}
			}
			smblib_rerun_apsd(chg);
			break;
		case FLOAT_CHECKING:
			chg->float_chk_cnt = 0;
			smblib_rerun_apsd(chg);
			break;
		default:
			break;
	}
	smblib_dbg(chg, PR_INTERRUPT, "CHK_FLOATING:%d,cnt=%d,0x1308=0x%02x,0x100E=%02x\n",
					chg->float_chk_sts, chg->float_chk_cnt, apsd_result_sts, chgr_sts_8);
}

static void smblib_boost_chk_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						reverse_boost_chk_work.work);
	u8 reg;
	int rc = 0;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &reg);
	if (rc) {
		smblib_err(chg, "Failed to read USBIN_STS schedule re-check, rc=%d\n", rc);
		schedule_delayed_work(&chg->reverse_boost_chk_work,
				msecs_to_jiffies(1000));
		return;
	}

	if (reg & USBIN_PLUGIN_RT_STS_BIT){
		smblib_dbg(chg, PR_INTERRUPT, "USBIN_STS = 0x%02x, Vbus exist after reverse boost detected: resume input\n", reg);
		vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
	}

	return;
}

static void smblib_cc_floating_chk_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						cc_floating_chk_work.work);
	int rc = 0, present = 0, type = 0;
	union power_supply_propval ret = {0, };
	u8 reg;

	smblib_dbg(chg, PR_INTERRUPT, "[CC FLOAT]Start CC pin floating detection, cnt=%d\n", g_cc_floating_cnt);

	if (g_cc_floating_cnt < 5) {
		rc = power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_PRESENT, &ret);
		if (rc)
			smblib_err(chg, "Failed to read chg present rc=%d\n", rc);
		else
			present = ret.intval;
		rc = power_supply_get_property(chg->usb_psy, POWER_SUPPLY_PROP_TYPE, &ret);
		if (rc)
			smblib_err(chg, "Failed to read chg type rc=%d\n", rc);
		else
			type = ret.intval;

		if (present && type == 0) {
			g_cc_floating_cnt += 1;
			rc = smblib_masked_write(chg, TYPE_C_CFG_REG, TYPE_C_OR_U_USB_BIT, 0);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't write register, rc=%d\n", rc);
			msleep(50);
			rc = smblib_masked_write(chg, TYPE_C_CFG_REG, TYPE_C_OR_U_USB_BIT, TYPE_C_OR_U_USB_BIT);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't write register, rc=%d\n", rc);
			else {
				msleep(50);
				smblib_rerun_apsd(chg);
			}
			schedule_delayed_work(&chg->cc_floating_chk_work, msecs_to_jiffies(3000));
		} else if (present > 0)
			g_cc_floating_cnt = 5;
		else {
			g_cc_floating_cnt = 0;
			rc = smblib_read(chg, TYPE_C_CFG_REG, &reg);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't read register, rc=%d\n", rc);
			else if (reg & TYPE_C_OR_U_USB_BIT) {
				rc = smblib_masked_write(chg, TYPE_C_CFG_REG, TYPE_C_OR_U_USB_BIT, 0);
				if (rc < 0)
					dev_err(chg->dev, "Couldn't write register, rc=%d\n", rc);
			}
		}

		smblib_dbg(chg, PR_INTERRUPT, "[CC FLOAT]chg_present=%d, chg_type=%d, cnt=%d.\n",
				present, type, g_cc_floating_cnt);
	}
}

#define ICL_1500_MA_MAX_THRES	1700000
#define ICL_1500_MA_MIN_THRES	1400000
#define ICL_2000_MA_MAX_THRES	2200000
#define ICL_2500_MA_MAX_THRES	2700000
static void smblib_type_c_aicl_chk_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						type_c_aicl_chk_work.work);
	int rc = 0;
	union power_supply_propval val = {0, };

	rc = smblib_get_prop_input_current_settled(chg, &val);
	if (rc) {
		smblib_err(chg, "Failed to read AICL result, rc=%d\n", rc);
		return;
	}

	if (val.intval < ICL_1500_MA_MIN_THRES) {
		smblib_dbg(chg, PR_INTERRUPT, "[TYPEC]AICL=%d, DOWNGRADE 1A.\n",
				val.intval/1000);
		vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_1000MA);
	} else if (val.intval < ICL_1500_MA_MAX_THRES) {
		smblib_dbg(chg, PR_INTERRUPT, "[TYPEC]AICL=%d, DOWNGRADE 1.5A.\n",
				val.intval/1000);
		vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_1500MA);
	} else if (val.intval < ICL_2000_MA_MAX_THRES) {
		smblib_dbg(chg, PR_INTERRUPT, "[TYPEC]AICL=%d, DOWNGRADE 2A.\n",
				val.intval/1000);
		vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_2000MA);
	} else if (val.intval < ICL_2500_MA_MAX_THRES) {
		smblib_dbg(chg, PR_INTERRUPT, "[TYPEC]AICL=%d, DOWNGRADE 2.5A.\n",
				val.intval/1000);
		vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_2500MA);
	}

	return;
}

/***************
 * HTC Battery *
 ***************/

//static struct smb2 *the_chip;
int smblib_get_bat_fet_off(struct smb_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't read BAT_2_SYS_FET_DIS_BIT rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & BAT_2_SYS_FET_DIS_BIT;

	return rc;
}

int smblib_set_bat_fet_off(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, BAT_2_SYS_FET_DIS_BIT,
			suspend ? BAT_2_SYS_FET_DIS_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't write %s to BAT_2_SYS_FET_DIS_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

void smblib_check_batt_enable(struct smb_charger *chg,
					union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	smblib_get_bat_fet_off(chg, &pval.intval);
	val->intval = !pval.intval;
}

void smblib_check_chg_enable(struct smb_charger *chg,
					union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	smblib_get_usb_suspend(chg, &pval.intval);
	val->intval = !pval.intval;
}

static int smblib_usb_conn_voltage_map_temp(int input, int *output)
{
        int i = 0;
        const struct qpnp_vadc_map_pt *pts;
        int tablesize;

        pts  = usb_conn_temp_adc_map;
        if (pts == NULL)
                return -EINVAL;

        tablesize = ARRAY_SIZE(usb_conn_temp_adc_map);

        while (i < tablesize) {
                if ((pts[i].y < input)) {
                        /* table entry is less than measured
                                value and table is descending, stop */
                        break;
                } else {
                        i++;
                }
        }

        if (i == 0) {
                *output = pts[0].x;
        } else if (i == tablesize) {
                *output = pts[tablesize-1].x;
        } else {
                /* result is between search_index and search_index-1 */
                /* interpolate linearly */
                *output = (( ((pts[i].x - pts[i-1].x)*
                        (input - pts[i-1].y))/
                        (pts[i].y - pts[i-1].y))+
                        pts[i-1].x);
        }

        return 0;
}

int smblib_get_usb_conn_adc(struct smb_charger *chg, struct qpnp_vadc_chip *vadc_dev,
					unsigned int vadc_channel)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if(vadc_dev == 0)
		return -EINVAL;

	if (IS_ERR(vadc_dev)) {
		vadc_dev = qpnp_get_vadc(chg->dev, "usb_conn_temp");
	}

	if (IS_ERR(vadc_dev)) {
		dev_err(chg->dev,"Failed to get vadc_usb_conn_temp %ld\n", PTR_ERR(vadc_dev));
		return -EINVAL;
	}

	if(vadc_dev) {
		rc = qpnp_vadc_read(vadc_dev, vadc_channel, &results);
		if (rc) {
			dev_err(chg->dev,"Unable to read USB_CONN_TEMP rc=%d\n", rc);
			return -EINVAL;
		} else {
			return (int)results.physical;
		}
	}

	return -EINVAL;
}

int smblib_get_usb_conn_temp(struct smb_charger *chg, struct qpnp_vadc_chip *vadc_dev,
					unsigned int vadc_channel, union power_supply_propval *val)
{
	int rc = 0;
	int adc_val[3], usb_conn_adc, usb_conn_temp;
	val->intval = 222;

	adc_val[0] =  smblib_get_usb_conn_adc(chg, vadc_dev, vadc_channel);
	if (adc_val[0] < 0)
		return -EINVAL;

	usleep_range(5000, 5100);

	adc_val[1] =  smblib_get_usb_conn_adc(chg, vadc_dev, vadc_channel);
	if (adc_val[1] < 0)
		return -EINVAL;

	usleep_range(5000, 5100);

	adc_val[2] =  smblib_get_usb_conn_adc(chg, vadc_dev, vadc_channel);
	if (adc_val[2] < 0)
		return -EINVAL;

	usb_conn_adc = (adc_val[0] + adc_val[1] + adc_val[2]) / 3000;
	rc = smblib_usb_conn_voltage_map_temp(usb_conn_adc, &usb_conn_temp);
	if(rc)
		return -EINVAL;

	val->intval = usb_conn_temp;

	return rc;
}

#endif //CONFIG_HTC_BATT

static void smblib_otg_oc_exit(struct smb_charger *chg, bool success)
{
	int rc;

	chg->otg_attempts = 0;
	if (!success) {
		smblib_err(chg, "OTG soft start failed\n");
		chg->otg_en = false;
	}

	smblib_dbg(chg, PR_OTG, "enabling VBUS < 1V check\n");
	rc = smblib_masked_write(chg, OTG_CFG_REG,
					QUICKSTART_OTG_FASTROLESWAP_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable VBUS < 1V check rc=%d\n", rc);

	if (!chg->external_vconn && chg->vconn_en) {
		chg->vconn_attempts = 0;
		if (success) {
			rc = _smblib_vconn_regulator_enable(
							chg->vconn_vreg->rdev);
			if (rc < 0)
				smblib_err(chg, "Couldn't enable VCONN rc=%d\n",
									rc);
		} else {
			chg->vconn_en = false;
		}
	}
}

#define MAX_OC_FALLING_TRIES 10
static void smblib_otg_oc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
								otg_oc_work);
	int rc, i;
	u8 stat;

	if (!chg->vbus_vreg || !chg->vbus_vreg->rdev)
		return;

	smblib_err(chg, "over-current detected on VBUS\n");
	mutex_lock(&chg->otg_oc_lock);
	if (!chg->otg_en)
		goto unlock;

	smblib_dbg(chg, PR_OTG, "disabling VBUS < 1V check\n");
	smblib_masked_write(chg, OTG_CFG_REG,
					QUICKSTART_OTG_FASTROLESWAP_BIT,
					QUICKSTART_OTG_FASTROLESWAP_BIT);

	/*
	 * If 500ms has passed and another over-current interrupt has not
	 * triggered then it is likely that the software based soft start was
	 * successful and the VBUS < 1V restriction should be re-enabled.
	 */
	schedule_delayed_work(&chg->otg_ss_done_work, msecs_to_jiffies(500));

	rc = _smblib_vbus_regulator_disable(chg->vbus_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable VBUS rc=%d\n", rc);
		goto unlock;
	}

	if (++chg->otg_attempts > OTG_MAX_ATTEMPTS) {
		cancel_delayed_work_sync(&chg->otg_ss_done_work);
		smblib_err(chg, "OTG failed to enable after %d attempts\n",
			   chg->otg_attempts - 1);
		smblib_otg_oc_exit(chg, false);
		goto unlock;
	}

	/*
	 * The real time status should go low within 10ms. Poll every 1-2ms to
	 * minimize the delay when re-enabling OTG.
	 */
	for (i = 0; i < MAX_OC_FALLING_TRIES; ++i) {
		usleep_range(1000, 2000);
		rc = smblib_read(chg, OTG_BASE + INT_RT_STS_OFFSET, &stat);
		if (rc >= 0 && !(stat & OTG_OVERCURRENT_RT_STS_BIT))
			break;
	}

	if (i >= MAX_OC_FALLING_TRIES) {
		cancel_delayed_work_sync(&chg->otg_ss_done_work);
		smblib_err(chg, "OTG OC did not fall after %dms\n",
						2 * MAX_OC_FALLING_TRIES);
		smblib_otg_oc_exit(chg, false);
		goto unlock;
	}

	smblib_dbg(chg, PR_OTG, "OTG OC fell after %dms\n", 2 * i + 1);
	rc = _smblib_vbus_regulator_enable(chg->vbus_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable VBUS rc=%d\n", rc);
		goto unlock;
	}

unlock:
	mutex_unlock(&chg->otg_oc_lock);
}

static void smblib_vconn_oc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
								vconn_oc_work);
	int rc, i;
	u8 stat;

	smblib_err(chg, "over-current detected on VCONN\n");
	if (!chg->vconn_vreg || !chg->vconn_vreg->rdev)
		return;

	mutex_lock(&chg->otg_oc_lock);
	rc = _smblib_vconn_regulator_disable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable VCONN rc=%d\n", rc);
		goto unlock;
	}

	if (++chg->vconn_attempts > VCONN_MAX_ATTEMPTS) {
		smblib_err(chg, "VCONN failed to enable after %d attempts\n",
			   chg->otg_attempts - 1);
		chg->vconn_en = false;
		chg->vconn_attempts = 0;
		goto unlock;
	}

	/*
	 * The real time status should go low within 10ms. Poll every 1-2ms to
	 * minimize the delay when re-enabling OTG.
	 */
	for (i = 0; i < MAX_OC_FALLING_TRIES; ++i) {
#ifdef CONFIG_HTC_USB
		usleep_range(500, 550);
#else
		usleep_range(1000, 2000);
#endif
		rc = smblib_read(chg, TYPE_C_STATUS_4_REG, &stat);
		if (rc >= 0 && !(stat & TYPEC_VCONN_OVERCURR_STATUS_BIT))
			break;
	}

	if (i >= MAX_OC_FALLING_TRIES) {
		smblib_err(chg, "VCONN OC did not fall after %dms\n",
						2 * MAX_OC_FALLING_TRIES);
		chg->vconn_en = false;
		chg->vconn_attempts = 0;
		goto unlock;
	}

	smblib_dbg(chg, PR_OTG, "VCONN OC fell after %dms\n", 2 * i + 1);
	if (++chg->vconn_attempts > VCONN_MAX_ATTEMPTS) {
		smblib_err(chg, "VCONN failed to enable after %d attempts\n",
			   chg->vconn_attempts - 1);
		chg->vconn_en = false;
		goto unlock;
	}

	rc = _smblib_vconn_regulator_enable(chg->vconn_vreg->rdev);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable VCONN rc=%d\n", rc);
		goto unlock;
	}

unlock:
	mutex_unlock(&chg->otg_oc_lock);
}

static void smblib_otg_ss_done_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							otg_ss_done_work.work);
	int rc;
	bool success = false;
	u8 stat;

	mutex_lock(&chg->otg_oc_lock);
	rc = smblib_read(chg, OTG_STATUS_REG, &stat);
	if (rc < 0)
		smblib_err(chg, "Couldn't read OTG status rc=%d\n", rc);
	else if (stat & BOOST_SOFTSTART_DONE_BIT)
		success = true;

	smblib_otg_oc_exit(chg, success);
	mutex_unlock(&chg->otg_oc_lock);
}

static void smblib_icl_change_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							icl_change_work.work);
	int rc, settled_ua;

	rc = smblib_get_charge_param(chg, &chg->param.icl_stat, &settled_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
		return;
	}

	power_supply_changed(chg->usb_main_psy);
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER,
				settled_ua >= USB_WEAK_INPUT_UA, 0);

	smblib_dbg(chg, PR_INTERRUPT, "icl_settled=%d\n", settled_ua);
}

static int smblib_create_votables(struct smb_charger *chg)
{
	int rc = 0;

	chg->fcc_votable = find_votable("FCC");
	if (!chg->fcc_votable) {
		rc = -EPROBE_DEFER;
		return rc;
	}

	chg->fv_votable = find_votable("FV");
	if (!chg->fv_votable) {
		rc = -EPROBE_DEFER;
		return rc;
	}

	chg->pl_disable_votable = find_votable("PL_DISABLE");
	if (!chg->pl_disable_votable) {
		rc = -EPROBE_DEFER;
		return rc;
	}
	vote(chg->pl_disable_votable, PL_INDIRECT_VOTER, true, 0);
	vote(chg->pl_disable_votable, PL_DELAY_HVDCP_VOTER, true, 0);

	chg->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					smblib_dc_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->dc_suspend_votable)) {
		rc = PTR_ERR(chg->dc_suspend_votable);
		return rc;
	}

	chg->usb_icl_votable = create_votable("USB_ICL", VOTE_MIN,
					smblib_usb_icl_vote_callback,
					chg);
	if (IS_ERR(chg->usb_icl_votable)) {
		rc = PTR_ERR(chg->usb_icl_votable);
		return rc;
	}

	chg->dc_icl_votable = create_votable("DC_ICL", VOTE_MIN,
					smblib_dc_icl_vote_callback,
					chg);
	if (IS_ERR(chg->dc_icl_votable)) {
		rc = PTR_ERR(chg->dc_icl_votable);
		return rc;
	}

	chg->pd_disallowed_votable_indirect
		= create_votable("PD_DISALLOWED_INDIRECT", VOTE_SET_ANY,
			smblib_pd_disallowed_votable_indirect_callback, chg);
	if (IS_ERR(chg->pd_disallowed_votable_indirect)) {
		rc = PTR_ERR(chg->pd_disallowed_votable_indirect);
		return rc;
	}

	chg->pd_allowed_votable = create_votable("PD_ALLOWED",
					VOTE_SET_ANY, NULL, NULL);
	if (IS_ERR(chg->pd_allowed_votable)) {
		rc = PTR_ERR(chg->pd_allowed_votable);
		return rc;
	}

	chg->awake_votable = create_votable("AWAKE", VOTE_SET_ANY,
					smblib_awake_vote_callback,
					chg);
	if (IS_ERR(chg->awake_votable)) {
		rc = PTR_ERR(chg->awake_votable);
		return rc;
	}

	chg->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					smblib_chg_disable_vote_callback,
					chg);
	if (IS_ERR(chg->chg_disable_votable)) {
		rc = PTR_ERR(chg->chg_disable_votable);
		return rc;
	}

	chg->pl_enable_votable_indirect = create_votable("PL_ENABLE_INDIRECT",
					VOTE_SET_ANY,
					smblib_pl_enable_indirect_vote_callback,
					chg);
	if (IS_ERR(chg->pl_enable_votable_indirect)) {
		rc = PTR_ERR(chg->pl_enable_votable_indirect);
		return rc;
	}

	chg->hvdcp_disable_votable_indirect = create_votable(
				"HVDCP_DISABLE_INDIRECT",
				VOTE_SET_ANY,
				smblib_hvdcp_disable_indirect_vote_callback,
				chg);
	if (IS_ERR(chg->hvdcp_disable_votable_indirect)) {
		rc = PTR_ERR(chg->hvdcp_disable_votable_indirect);
		return rc;
	}

	chg->hvdcp_enable_votable = create_votable("HVDCP_ENABLE",
					VOTE_SET_ANY,
					smblib_hvdcp_enable_vote_callback,
					chg);
	if (IS_ERR(chg->hvdcp_enable_votable)) {
		rc = PTR_ERR(chg->hvdcp_enable_votable);
		return rc;
	}

	chg->apsd_disable_votable = create_votable("APSD_DISABLE",
					VOTE_SET_ANY,
					smblib_apsd_disable_vote_callback,
					chg);
	if (IS_ERR(chg->apsd_disable_votable)) {
		rc = PTR_ERR(chg->apsd_disable_votable);
		return rc;
	}

	chg->hvdcp_hw_inov_dis_votable = create_votable("HVDCP_HW_INOV_DIS",
					VOTE_SET_ANY,
					smblib_hvdcp_hw_inov_dis_vote_callback,
					chg);
	if (IS_ERR(chg->hvdcp_hw_inov_dis_votable)) {
		rc = PTR_ERR(chg->hvdcp_hw_inov_dis_votable);
		return rc;
	}

	chg->usb_irq_enable_votable = create_votable("USB_IRQ_DISABLE",
					VOTE_SET_ANY,
					smblib_usb_irq_enable_vote_callback,
					chg);
	if (IS_ERR(chg->usb_irq_enable_votable)) {
		rc = PTR_ERR(chg->usb_irq_enable_votable);
		return rc;
	}

	return rc;
}

static void smblib_destroy_votables(struct smb_charger *chg)
{
	if (chg->dc_suspend_votable)
		destroy_votable(chg->dc_suspend_votable);
	if (chg->usb_icl_votable)
		destroy_votable(chg->usb_icl_votable);
	if (chg->dc_icl_votable)
		destroy_votable(chg->dc_icl_votable);
	if (chg->pd_disallowed_votable_indirect)
		destroy_votable(chg->pd_disallowed_votable_indirect);
	if (chg->pd_allowed_votable)
		destroy_votable(chg->pd_allowed_votable);
	if (chg->awake_votable)
		destroy_votable(chg->awake_votable);
	if (chg->chg_disable_votable)
		destroy_votable(chg->chg_disable_votable);
	if (chg->pl_enable_votable_indirect)
		destroy_votable(chg->pl_enable_votable_indirect);
	if (chg->apsd_disable_votable)
		destroy_votable(chg->apsd_disable_votable);
	if (chg->hvdcp_hw_inov_dis_votable)
		destroy_votable(chg->hvdcp_hw_inov_dis_votable);
}

static void smblib_iio_deinit(struct smb_charger *chg)
{
	if (!IS_ERR_OR_NULL(chg->iio.temp_chan))
		iio_channel_release(chg->iio.temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.temp_max_chan))
		iio_channel_release(chg->iio.temp_max_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_i_chan))
		iio_channel_release(chg->iio.usbin_i_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_v_chan))
		iio_channel_release(chg->iio.usbin_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.batt_i_chan))
		iio_channel_release(chg->iio.batt_i_chan);
}

int smblib_init(struct smb_charger *chg)
{
	int rc = 0;

	mutex_init(&chg->write_lock);
	mutex_init(&chg->otg_oc_lock);
	INIT_WORK(&chg->bms_update_work, bms_update_work);
	INIT_WORK(&chg->rdstd_cc2_detach_work, rdstd_cc2_detach_work);
	INIT_DELAYED_WORK(&chg->hvdcp_detect_work, smblib_hvdcp_detect_work);
	INIT_DELAYED_WORK(&chg->step_soc_req_work, step_soc_req_work);
	INIT_DELAYED_WORK(&chg->clear_hdc_work, clear_hdc_work);
#ifdef CONFIG_HTC_BATT
	INIT_DELAYED_WORK(&chg->chk_usb_icl_work, chk_usb_icl_work);
	INIT_DELAYED_WORK(&chg->cc_check_work, smblib_cc_check_work);
	INIT_DELAYED_WORK(&chg->floating_chk_work, smblib_floating_chk_work);
	INIT_DELAYED_WORK(&chg->reverse_boost_chk_work, smblib_boost_chk_work);
	INIT_DELAYED_WORK(&chg->cc_floating_chk_work, smblib_cc_floating_chk_work);
	INIT_DELAYED_WORK(&chg->type_c_aicl_chk_work, smblib_type_c_aicl_chk_work);
	chg->float_chk_sts = FLOAT_NONE;
	chg->float_chk_cnt = 0;
	chg->abnormal_cc_check_sts = false;
	chg->cc_first_det_sts = CC_DET_NONE;
#endif //CONFIG_HTC_BATT
	INIT_WORK(&chg->otg_oc_work, smblib_otg_oc_work);
	INIT_WORK(&chg->vconn_oc_work, smblib_vconn_oc_work);
	INIT_DELAYED_WORK(&chg->otg_ss_done_work, smblib_otg_ss_done_work);
	INIT_DELAYED_WORK(&chg->icl_change_work, smblib_icl_change_work);
	chg->fake_capacity = -EINVAL;

	switch (chg->mode) {
	case PARALLEL_MASTER:
		chg->qnovo_fcc_ua = -EINVAL;
		chg->qnovo_fv_uv = -EINVAL;
		rc = smblib_create_votables(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't create votables rc=%d\n",
				rc);
			return rc;
		}

		rc = smblib_register_notifier(chg);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't register notifier rc=%d\n", rc);
			return rc;
		}

		chg->ext_power = devm_regulator_get(chg->dev, "ext_power");
		if (IS_ERR(chg->ext_power)) {
			rc = PTR_ERR(chg->ext_power);
			smblib_err(chg, "Couldn't get external TPS61099 regulator, rc=%d\n", rc);
		}

		chg->bms_psy = power_supply_get_by_name("bms");
		chg->pl.psy = power_supply_get_by_name("parallel");
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	return rc;
}

int smblib_deinit(struct smb_charger *chg)
{
	switch (chg->mode) {
	case PARALLEL_MASTER:
		power_supply_unreg_notifier(&chg->nb);
		smblib_destroy_votables(chg);
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	smblib_iio_deinit(chg);

	return 0;
}
