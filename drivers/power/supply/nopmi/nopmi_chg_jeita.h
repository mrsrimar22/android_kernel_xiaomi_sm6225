/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * nopmi_chg_jeita - Software JEITA charging control sub-module
 */

#ifndef __NOPMI_CHG_JEITA_H__
#define __NOPMI_CHG_JEITA_H__

#include "nopmi_chg_common.h"
#include "bq2589x_charger.h"

struct nopmi_chg;

#define JEITA_CV_ABOVE_T4	4240
#define JEITA_CV_T3_TO_T4	4240
#define JEITA_CV_T2_TO_T3	4350
#define JEITA_CV_T1P5_TO_T2	4240
#define JEITA_CV_T1_TO_T1P5	4040
#define JEITA_CV_T0_TO_T1	4040
#define JEITA_CV_TN1_TO_T0	4040
#define JEITA_CV_BELOW_T0	4040
#define JEITA_CV_NORMAL		4450	/* T2_TO_T3 normal charge voltage */
#define JEITA_T4_THRES			50
#define JEITA_T4_THRES_MINUS_X		47
#define JEITA_T3_THRES			45
#define JEITA_T3_THRES_MINUS_X		39
#define JEITA_T2_THRES			12
#define JEITA_T2_THRES_PLUS_X		16
#define JEITA_T1P5_THRES		6
#define JEITA_T1P5_THRES_PLUS_X		11
#define JEITA_T1_THRES			3
#define JEITA_T1_THRES_PLUS_X		5
#define JEITA_T0_THRES			0
#define JEITA_T0_THRES_PLUS_X		2
#define JEITA_TN1_THRES			(-5)
#define JEITA_TN1_THRES_PLUS_X		(-3)
#define JEITA_NEG10_THRES		(-10)
#define JEITA_FCC_TN1_TO_T0	442
#define JEITA_FCC_T0_TO_T1	884
#define JEITA_FCC_T1_TO_T1P5	2210
#define JEITA_FCC_T1P5_TO_T2	3536
#define JEITA_FCC_T2_TO_T3	4000	/* also used as default jeita_current_limit */
#define JEITA_FCC_T3_TO_T4	2210

#define JEITA_WORK_DELAY_MS	2000
#define JEITA_VOTER		"JEITA_VOTER"
#define JEITA_CHG_VOTER		"JEITA_CHG_VOTER"

enum sw_jeita_state {
	TEMP_BELOW_T0 = 0,
	TEMP_TN1_TO_T0,
	TEMP_T0_TO_T1,
	TEMP_T1_TO_T1P5,
	TEMP_T1P5_TO_T2,
	TEMP_T2_TO_T3,
	TEMP_T3_TO_T4,
	TEMP_ABOVE_T4,
};

enum charge_mode {
	CHG_MODE_BUCK_OFF = 0,
	CHG_MODE_CHARGING_OFF,
	CHG_MODE_CHARGING,
	CHG_MODE_MAX,
};

struct sw_jeita_data {
	enum sw_jeita_state	sm;
	enum sw_jeita_state	pre_sm;
	int			cv;
	int			pre_cv;
	int			term_curr;
	bool			charging;
	bool			can_recharging;
	bool			error_recovery_flag;
};

struct jeita_config {
	bool	enable_sw_jeita;

	int	cv_above_t4;
	int	cv_t3_to_t4;
	int	cv_t2_to_t3;
	int	cv_t1p5_to_t2;
	int	cv_t1_to_t1p5;
	int	cv_t0_to_t1;
	int	cv_tn1_to_t0;
	int	cv_below_t0;
	int	cv_normal;

	int	t4;
	int	t4_minus_x;
	int	t3;
	int	t3_minus_x;
	int	t2;
	int	t2_plus_x;
	int	t1p5;
	int	t1p5_plus_x;
	int	t1;
	int	t1_plus_x;
	int	t0;
	int	t0_plus_x;
	int	tn1;
	int	tn1_plus_x;
	int	tneg10;

	int	fcc_t3_to_t4;
	int	fcc_t2_to_t3;
	int	fcc_t1p5_to_t2;
	int	fcc_t1_to_t1p5;
	int	fcc_t0_to_t1;
	int	fcc_tn1_to_t0;
};

struct nopmi_chg_jeita_st {
	struct nopmi_chg	*parent;
	struct jeita_config	cfg;
	struct sw_jeita_data	sw;
	struct delayed_work	work;

	bool	sw_jeita_start;
	bool	usb_present;

	int	battery_temp;
	int	battery_id;
	int	current_limit;
	int	fast_charge_mode;
};

int nopmi_chg_jeita_init(struct nopmi_chg *chg);
int nopmi_chg_jeita_deinit(struct nopmi_chg *chg);
void start_nopmi_chg_jeita_workfunc(struct nopmi_chg *chg);
void stop_nopmi_chg_jeita_workfunc(struct nopmi_chg *chg);

#endif /* __NOPMI_CHG_JEITA_H__ */
