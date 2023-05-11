/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __BQ2589X_CHARGER_HEADER__
#define __BQ2589X_CHARGER_HEADER__

#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/srcu.h>
#include <linux/pmic-voter.h>
#include <linux/ratelimit.h>
#include <linux/regulator/consumer.h>
#include "inc/tcpci.h"
#include "nopmi_chg_common.h"

#define BQ2589X_DEV_NAME	"bq2589x"

#define BQ_RL_SLOTS	16
#define BQ_RL_INTERVAL	(5 * HZ)
#define BQ_RL_BURST	1

#ifdef __COUNTER__
#define _BQ_ID()	__COUNTER__
#else
#define _BQ_ID()	__LINE__
#endif

enum bq_charger_event {
	BQ_CHG_EVENT_START_MONITOR = 0,
	BQ_CHG_EVENT_STOP_MONITOR,
	BQ_CHG_EVENT_CHARGE_DONE,
	BQ_CHG_EVENT_CHARGE_NOT_DONE,
};

enum bq2589x_vbus_type {
	BQ2589X_VBUS_NONE,
	BQ2589X_VBUS_USB_SDP,
	BQ2589X_VBUS_USB_CDP,
	BQ2589X_VBUS_USB_DCP,
	BQ2589X_VBUS_USB_HVDCP,
	BQ2589X_VBUS_UNKNOWN,
	BQ2589X_VBUS_NONSTAND,
	BQ2589X_VBUS_OTG,
	BQ2589X_VBUS_TYPE_NUM,
};

enum bq2589x_part_no {
	SYV690		= 0x01,
	BQ25890		= 0x03,
	BQ25892		= 0x00,
	BQ25895		= 0x07,
	SC89890H	= 0x04,
};

enum bq2589x_status_bits {
	BQ2589X_STAT_PLUGIN_BIT = 0,
	BQ2589X_STAT_PG_BIT,
	BQ2589X_STAT_CHARGE_ENABLE_BIT,
	BQ2589X_STAT_FAULT_BIT,
	BQ2589X_STAT_EXIST_BIT,
};

#define BQ2589X_MAX_ICL			2950
#define BQ2589X_MAX_FCC			5950

struct bq2589x_config {
	bool	enable_auto_dpdm;
	int	charge_voltage;
	int	charge_current;
	int	charge_current_3500;
	int	charge_current_1500;
	int	charge_current_1000;
	int	charge_current_500;
	int	input_current_2000;

	bool	enable_term;
	int	term_current;

	bool	enable_ico;
	bool	use_absolute_vindpm;
};

struct pe_ctrl {
	bool	enable;
	bool	tune_up_volt;
	bool	tune_down_volt;
	bool	tune_done;
	bool	tune_fail;
	int	tune_count;
	int	target_volt;
	int	high_volt_level;
	int	low_volt_level;
	int	vbat_min_volt;
};

struct bq2589x {
	struct device		*dev;
	struct device		class_dev;
	const char		*name;

	struct i2c_client	*client;
	struct regmap		*regmap;
	enum bq2589x_part_no	part_no;
	enum bq2589x_vbus_type	vbus_type;

	struct srcu_notifier_head	evt_notifier;

	struct tcpc_device	*tcpc_dev;
	struct notifier_block	pd_nb;

	struct gpio_desc	*irq_gpiod;

	int			revision;
	unsigned long		status;
	int			vbus_volt;
	int			vbat_volt;
	int			chg_current;
	int			rsoc;
	int			pd_active;
	bool			enabled;

	struct mutex		usb_switch_lock; /* USB switch protection */

	int			usb_changed_last_chg_type;
	int			usb_changed_no_change_count;
	int			usb_changed_wait;
	bool			usb_changed_last_valid;
	unsigned long		usb_changed_last_jiffies;
	bool			ico_issued;
	bool			pumpx_cmd_issued;
	bool			ico_pending;
	bool			pe_check_pending;
	bool			tune_pending;
	bool			usb_changed_pending;
	int			monitor_wait;
	int			pe_check_wait;
	int			pe_tune_wait;
	int			ico_wait;

	struct bq2589x_config	cfg;
	struct pe_ctrl		pe;

	struct work_struct	adapter_in_work;
	struct work_struct	adapter_out_work;
	struct work_struct	start_charging_work;
	struct delayed_work	poll_work;
	struct delayed_work	charger_work;
	struct delayed_work	time_delay_work;

	struct power_supply_desc	wall;
	struct power_supply		*batt_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*wall_psy;
	struct power_supply		*bms_psy;
	struct power_supply_config	usb_cfg;
	struct power_supply_config	wall_cfg;

	struct votable		*fcc_votable;
	struct votable		*fv_votable;
	struct votable		*usb_icl_votable;
	struct votable		*chg_dis_votable;

	enum power_supply_type	chg_type;
	bool			chg_online;

	int			irq_gpio;
	bool			usb_switch_flag;

	int			irq;

	struct wakeup_source	*bq_ws;
	struct ratelimit_state	rl_slots[BQ_RL_SLOTS];

	bool			vbus_on;
	bool			otg_attached;

	struct workqueue_struct	*event_wq;
	struct workqueue_struct	*bg_wq;
	struct notifier_block	late_sync_nb;
};

extern struct class *bq2589x_class;

struct bq2589x *bbc_dev_get_by_name(const char *name);
void bbc_dev_put(struct bq2589x *bq);
int bq_charger_register_notifier(struct bq2589x *bq, struct notifier_block *nb);
int bq_charger_unregister_notifier(struct bq2589x *bq, struct notifier_block *nb);

#endif
