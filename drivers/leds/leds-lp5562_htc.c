/* driver/leds/leds-lp5562_htc.c
 *
 * Copyright (C) 2010 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
/*#include <linux/android_alarm.h>*/
#include <linux/leds.h>
#include <linux/leds-lp5562_htc.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
/*include <mach/ADP5585_ioextender.h>*/
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>

#if 1
#include <linux/stacktrace.h>
#endif

#define CONFIG_LEDS_QPNP_BUTTON_BLINK
#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
#include <linux/alarmtimer.h>
#include <linux/notification/notification.h>
#include <linux/uci/uci.h>
#endif

#ifdef CONFIG_HZ_300
#define JIFFY_MUL 3
#else
#define JIFFY_MUL 1
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif


#define LP5562_MAX_LEDS			9	/* Maximum number of LEDs */
#define LED_DEBUG				1
#if LED_DEBUG
#define D(x...) printk(KERN_DEBUG "[LED]" x)
#define I(x...) printk(KERN_INFO "[LED]" x)
#else
#define D(x...)
#define I(x...)
#endif

#define MAIN_TOUCH_SOLUTION 2
#define SEC_TOUCH_SOLUTION 1


#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK

bool vk_present = false;

// bln_on_screenoff : this is used to follow register_haptic based notif events... when
// ...AmbientDisplay wakes device (BLN is not possible), but after screen off, BLN is triggered based on this
// ... in the fb notifier BLANK event part. register_haptic sets it 1, register_input sets it 0 because that will be a user wake...
// ... also wake_by_user in UNBLANK notification will set it 0:
static int bln_on_screenoff = 0;
// wake_by_user: used for AmbientDisplay detection:
// this if 1, then device is waken by user. Otherwise no Input device was triggered, so we can deduce that it's an AmibentDisplay wake
// and therefore if this is 0, Flashlight notification can be triggered, and bln_on_screenoff also can be stored, so that BLN can be
// triggered later when screen is self BLANKing / screen off....
static int wake_by_user = 1;

int input_is_wake_by_user(void)
{
	return wake_by_user;
}
EXPORT_SYMBOL(input_is_wake_by_user);

#define BUTTON_BLINK_SPEED_MAX		2
#define BUTTON_BLINK_SPEED_DEFAULT	1

#define BUTTON_BLINK_NUMBER_MAX		50
#define BUTTON_BLINK_NUMBER_DEFAULT	6

#define RGB_PATTERN_NORMAL 0
#define RGB_PATTERN_ONEPLUS5 1
#define RGB_PATTERN_TRIPLE 2
#define RGB_PATTERN_TRIPLE_UP 3
#define RGB_PATTERN_TRIPLE_DOWN 4


static DEFINE_MUTEX(blinkstopworklock);
static struct alarm blinkstopfunc_rtc;
static struct alarm vibrate_rtc;
static struct alarm double_double_vol_rtc;
static struct alarm flash_stop_rtc;
static int probe_finished = 0;

static int bln_switch = 1; // 0 - off / 1 - on
static int bln_no_charger_switch = 1; // 0 - only do BLN when on charger / 1 - BLN when not on charger
static int bln_number = BUTTON_BLINK_NUMBER_DEFAULT; // infinite = 0 - number of max button blinks when not on charger
static int bln_speed = BUTTON_BLINK_SPEED_DEFAULT;
static int bln_dim_blink = 0; // continue quarter strength blinking after bln_number passed blinking
static int bln_dim_number = 0;//BUTTON_BLINK_NUMBER_DEFAULT * 2; // infinite = 0 - number of max dim button blinks when not on charger

static int full_or_dim = 1;

static int lights_down_divider = 1;
static int rgb_coeff_divider = 1; // value between 1 - 20
static int bln_coeff_divider = 6; // value between 1 - 21 ... on sysfs 0-20

static int pulse_rgb_blink = 1;  // 0 - normal stock blinking / 1 - pulsating
static int pulse_rgb_pattern =  RGB_PATTERN_NORMAL;
static int pulse_rgb_blink_on_charger = 0; // 0 - not blink on charger, 1 - do blink
static int pulse_rgb_blink_on_charger_red_limit = 95; // 0-100 percentage where color from red switches to green

static int colored_charge_level = 1; // if set to 1, colored charge level handling is enabled, 0 - not

static int get_bln_switch(void) {
	if (!vk_present) return 0;
	return uci_get_user_property_int_mm("bln", bln_switch, 0, 1);
}
static int get_bln_no_charger_switch(void) {
	return uci_get_user_property_int_mm("bln_no_charger_switch", bln_no_charger_switch, 0, 1);
}
static int get_bln_number(void) {
	return uci_get_user_property_int_mm("bln_number", bln_number, 0, 50);
}
static int get_bln_light_level(void) {
	return uci_get_user_property_int_mm("bln_light_level", bln_coeff_divider-1, 0, 20)+1;
}
static int get_bln_speed(void) {
	return uci_get_user_property_int_mm("bln_speed", bln_speed, 0, 2);
}
static int get_bln_dim_blink(void) {
	return uci_get_user_property_int_mm("bln_dim_blink", bln_dim_blink, 0, 1);
}
static int get_bln_dim_number(void) {
	return uci_get_user_property_int_mm("bln_dim_number", bln_dim_number, 0, 50);
}
static int get_bln_rgb_blink_light_level(void) {
	return uci_get_user_property_int_mm("bln_rgb_light_level", rgb_coeff_divider-1, 0, 20)+1;
}
static int get_bln_pulse_rgb_blink(void) {
	return uci_get_user_property_int_mm("bln_rgb_pulse", pulse_rgb_blink, 0, 1);
}
static int get_bln_pulse_rgb_pattern(void) {
	return uci_get_user_property_int_mm("bln_rgb_pulse_pattern", pulse_rgb_pattern, 0, 4);
}
static int get_bln_rgb_batt_colored(void) {
	return uci_get_user_property_int_mm("bln_rgb_batt_colored", colored_charge_level, 0, 1);
}
static int get_bln_pulse_rgb_blink_on_charger(void) {
	return uci_get_user_property_int_mm("bln_rgb_pulse_blink_on_charger", pulse_rgb_blink_on_charger, 0, 1);
}
static int get_bln_pulse_rgb_blink_on_charger_red_limit(void) {
	return uci_get_user_property_int_mm("bln_rgb_pulse_blink_on_charger_red_limit", pulse_rgb_blink_on_charger_red_limit, 0, 100);
}


static int pattern_brightness[5][6] = {
					{0x32,0x90,0xc8,0x90,0x62,0x22}, // normal
					{0x12,0x36,0x74,0x50,0x30,0x12}, // oneplus5
					{0xc8,0x10,0xc8,0x10,0xc8,0x00}, // triple
					{0x10,0x00,0x38,0x00,0xd8,0x00}, // triple up
					{0xd8,0x00,0x38,0x00,0x10,0x00}, // triple down
				    };
static int pattern_time[5][6] = {
					{0x44,0x44,0x55,0x44,0x44,0x44}, // normal
					{0x53,0x53,0x55,0x53,0x53,0x53}, // oneplus5
					{0x44,0x44,0x44,0x44,0x44,0x55}, // triple
					{0x50,0x50,0x50,0x50,0x50,0x55}, // triple up
					{0x50,0x50,0x50,0x50,0x50,0x55}, // triple down
				    };
static int pattern_time_button[5][6] = {
					{0x44,0x44,0x54,0x44,0x44,0x44}, // normal
					{0x53,0x53,0x54,0x53,0x53,0x53}, // oneplus5
					{0x44,0x44,0x54,0x44,0x44,0x44}, // triple
					{0x50,0x50,0x54,0x50,0x50,0x50}, // triple up
					{0x50,0x50,0x54,0x50,0x50,0x50}, // triple down
				    };

static int charging = 0;
static int charging_notification_occured_for_rgb = 0;
static struct work_struct vk_blink_work;
static struct work_struct vk_unblink_work;

static int charge_level = 0; // information from HTC battery driver
static int supposedly_charging = 0; // information from led call (multicolor work)
static int first_level_registered = 0; // when register_charge_level first called set to 1
static int short_vib_notif = 0; // if short vibration pattern this will be 1
static struct lp5562_led *g_led_led_data_bln;

static int last_rgb_notification_charge_level = -1;
static void clear_rgb_notification_on_charger(const char* caller) {
	pr_info("%s LED %s",__func__,caller);
	last_rgb_notification_charge_level = -1;
}
static void clear_charging_notification_occured_for_rgb(const char* caller) {
	pr_info("%s LED %s",__func__,caller);
	charging_notification_occured_for_rgb = 0;
}

#endif
#ifdef CONFIG_FB
	static int screen_on = 1;
	static int screen_on_early = 1;
	static unsigned long screen_off_jiffies = 0;
	struct notifier_block *fb_notifier_led;
#endif

static int led_rw_delay, chip_enable, rgb_enable, vk_enable;
static int current_time;
static struct i2c_client *private_lp5562_client = NULL;
static struct mutex	led_mutex;
static struct mutex	vk_led_mutex;
static struct mutex	operation_mutex;
static struct mutex	enable_mutex;
static struct workqueue_struct *g_led_work_queue;
static struct workqueue_struct *g_vk_work_queue;
static struct workqueue_struct *g_vib_work_queue;
static uint32_t ModeRGB;
static int VK_brightness;
static int last_pwm;
static uint8_t virtual_key_led_ignore_flag = 0;
static uint16_t g_led_touch_solution = MAIN_TOUCH_SOLUTION;
static u8 color_table[20] = {0};
static int use_color_table = 0;
static u8 current_table[20] = {0};
static int use_current_table = 0;
static int table_level_num = 0;
static uint32_t charging_flag = 0; /* cei mfg test only */
#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
static int display_flag = 0;
module_param(display_flag, int, 0660); /* mfg test only */
static struct lp5562_led *g_led_led_data;
#endif
static struct led_i2c_platform_data *plat_data;

module_param(virtual_key_led_ignore_flag, byte, S_IRUSR | S_IWUSR);

#define Mode_Mask (0xff << 24)
#define Red_Mask (0xff << 16)
#define Green_Mask (0xff << 8)
#define Blue_Mask 0xff

#define RG_Mask 0x3C
#define VK_Mask 0x03

#define VK_LED_FADE_LEVEL	16
#define VL_LED_FADE_TIME	125
#define VK_LED_SLEEP_TIME	115

static int gCurrent_param = 95;
static int gVK_Current_param = 95;

module_param(gCurrent_param, int, 0600);
module_param(gVK_Current_param, int, 0600);

#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
extern bool htc_check_panel_connection(void);
#endif

struct lp5562_led {
	int			id;
	u8			chan_nr;
	u8			led_current;
	u8			max_current;
	struct led_classdev	cdev;
	struct mutex 		led_data_mutex;
/*	struct alarm 		led_alarm;*/
	struct work_struct 	led_work;
	struct work_struct 	led_work_multicolor;
	uint8_t 		Mode;
	uint8_t			Red;
	uint8_t 		Green;
	uint8_t 		Blue;
	int			VK_brightness;
	struct delayed_work	blink_delayed_work;
	struct wake_lock        led_wake_lock;
};

struct lp5562_chip {
	struct led_i2c_platform_data *pdata;
	struct mutex		led_i2c_rw_mutex; /* Serialize control */
	struct i2c_client	*client;
	struct lp5562_led	leds[LP5562_MAX_LEDS];
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_init;
};

typedef enum {
	INDICATOR_LED_ID = 0,
	VIRTUAL_KEY_LED_ID = 1,
} LED_ID;

static int lp5562_parse_dt(struct device *, struct led_i2c_platform_data *);

static char *hex2string(uint8_t *data, int len)
{
	static char buf[LED_I2C_WRITE_BLOCK_SIZE*4];
	int i;

	i = LED_I2C_WRITE_BLOCK_SIZE -1;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		snprintf(buf + i * 4, sizeof(buf),"[%02X]", data[i]);

	return buf;
}

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
		uint8_t *data, int length)
{
	int retry;
	uint8_t buf[LED_I2C_WRITE_BLOCK_SIZE];
	int i;
	struct lp5562_chip *cdata;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n",
			addr, hex2string(data, length));

	cdata = i2c_get_clientdata(client);
	if (length + 1 > LED_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "[LED] i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	mutex_lock(&cdata->led_i2c_rw_mutex);
	msleep(1);
	for (retry = 0; retry < I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(led_rw_delay);
	}
	if (retry >= I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "[LED] i2c_write_block retry over %d times\n",
				I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&cdata->led_i2c_rw_mutex);
		return -EIO;
	}
	mutex_unlock(&cdata->led_i2c_rw_mutex);

	return 0;
}


static int I2C_RxData_2(char *rxData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msgs[] = {
		{
			.addr = private_lp5562_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = private_lp5562_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_WRITE_RETRY_TIMES; loop_i++) {
		if (i2c_transfer(private_lp5562_client->adapter, msgs, 2) > 0)
			break;
		msleep(10);
	}

	if (loop_i >= I2C_WRITE_RETRY_TIMES) {
		printk(KERN_ERR "[LED] %s retry over %d times\n",
				__func__, I2C_WRITE_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_read_block(struct i2c_client *client,
		uint8_t cmd, uint8_t *pdata, int length)
{
	char buffer[3] = {0};
	int ret = 0, i;

	if (pdata == NULL)
		return -EFAULT;

	if (length > 2) {
		pr_err("[LED]%s: length %d> 2: \n", __func__, length);
		return ret;
	}
	buffer[0] = cmd;
	ret = I2C_RxData_2(buffer, length);
	if (ret < 0) {
		pr_err("[LED]%s: I2C_RxData fail \n", __func__);
		return ret;
	}

	for (i = 0; i < length; i++) {
		*(pdata+i) = buffer[i];
	}
	return ret;
}

static uint8_t latest_set_enable_register = 0;

static int write_enable_register(struct i2c_client *client, uint8_t data, uint8_t vk_led)
{
	int ret = 0;
	uint8_t temp = 0, current_data = 0;
#if 1
	int locked = 0;
	int read_retry = 0;
	if (mutex_trylock(&enable_mutex)) {
		locked = 1;
	} else {
		pr_info("%s ALARM, CONCURRENT ENABLE \n",__func__);
	}
#endif

	if(!rgb_enable && !vk_enable && data == 0) {
		I("write_enable_register disable\n");
		ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
		latest_set_enable_register = data;
#if 1
		if (locked) mutex_unlock(&enable_mutex);
#endif
		return ret;
	}
	else
		temp = 0x40;
#if 1
	ret = 1;
	while (ret!=0) {
#endif
	ret = i2c_read_block(client, ENABLE_REGISTER, &current_data, 1);
#if 1
		if (ret) {
			I(" ### write_enable_register read error\n");
			read_retry++;
			if (read_retry>2) {
				current_data = latest_set_enable_register;
				break;
			}
			msleep(5);
		}
	}
#endif
	if(vk_led)
		temp |= (current_data & RG_Mask) | (data & VK_Mask);
	else
		temp |= (data & RG_Mask) | (current_data & VK_Mask);

	ret = i2c_write_block(client, ENABLE_REGISTER, &temp, 1);
	latest_set_enable_register = temp;
#if 1
	if (locked) mutex_unlock(&enable_mutex);
#endif
	return ret;
}

static uint8_t latest_set_operation_register = 0;

static int write_operation_register(struct i2c_client *client, uint8_t data, uint8_t vk_led)
{
	int ret = 0;
	uint8_t temp = 0, current_data = 0;
#if 1
	int locked = 0;
	int read_retry = 0;
	if (mutex_trylock(&operation_mutex)) {
		locked = 1;
	} else {
		pr_info("%s ALARM, CONCURRENT OPERATION \n",__func__);
	}
#endif
#if 1
	ret = 1;
	while (ret!=0) {
#endif
	ret = i2c_read_block(client, OPRATION_REGISTER, &current_data, 1);
#if 1
		if (ret) {
			I(" ### write_opearation_register read error\n");
			read_retry++;
			if (read_retry>2) {
				current_data = latest_set_operation_register;
				break;
			}
			msleep(5);
		}
	}
#endif

	if(vk_led)
		temp |= (current_data & RG_Mask) | (data & VK_Mask);
	else
		temp |= (data & RG_Mask) | (current_data & VK_Mask);

	ret = i2c_write_block(client, OPRATION_REGISTER, &temp, 1);
	latest_set_operation_register = temp;
#if 1
	if (locked) mutex_unlock(&operation_mutex);
#endif
	return ret;
}

static int write_vk_led_program(struct i2c_client *client)
{
	int ret = 0, reg_index = 0;
	uint8_t data, step_time, target_pwm;
	uint8_t command_data[32] = {0};

	data = 0x01;
	ret = write_operation_register(client, data, 1);


	/*=== Increase 16 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 1 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 32 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 2 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 48 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 3 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 64 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 4 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 80 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 5 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 96 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 6 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 112 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 7 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Increase 128 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 8 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++] = 0x3F & step_time;
	command_data[reg_index++]  = 0x7F & target_pwm;

	/*=== Decrease 16 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 1 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 32 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 2 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 48 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 3 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 64 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 4 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 80 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 5 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 96 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 6 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 112 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 7 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	/*=== Decrease 128 PWM in VL_LED_FADE_TIMEms ===*/
	target_pwm = VK_LED_FADE_LEVEL * 8 - 1;
	step_time = (uint8_t)(VL_LED_FADE_TIME * 100 / 49 / target_pwm); //need VL_LED_FADE_TIME ms, 0.49ms for each step
	command_data[reg_index++]  = 0x3F & step_time;
	command_data[reg_index++]  = 0x80 | (0x7F & target_pwm);

	ret = i2c_write_block(client, CMD_ENG_3_BASE, command_data, 32);

	data = 0x02;
	ret = write_operation_register(client, data, 1);

	return ret;
}

static void lp5562_led_enable(struct i2c_client *client, int blink_enable)
{
	int ret = 0;
	uint8_t data;

	char data1[1] = {0};

	I(" %s +++\n" , __func__);

	if (chip_enable) {
		I(" %s return, chip already enable\n" , __func__);
		return;
	}
#if 0 // LED enable pin keep high to avoid SRAM abnormal
	/* === led pin enable ===*/
	if (plat_data->ena_gpio) {
		ret = gpio_direction_output(plat_data->ena_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
			gpio_free(plat_data->ena_gpio);
		}
	} /*else if (pdata->ena_gpio_io_ext) {
	    ret = ioext_gpio_set_value(pdata->ena_gpio_io_ext, 1);
	    if (ret < 0) {
	    pr_err("[LED] %s: io_extender high failed %d\n", __func__, ret);
	    gpio_free(pdata->ena_gpio);
	    }
	    }*/

	msleep(1);
#endif

#if 1
	if (blink_enable) {
		/* === reset ===*/
		data = 0xFF;
		ret = i2c_write_block(client, RESET_CONTROL, &data, 1);
		msleep(20);
		ret = i2c_read_block(client, R_CURRENT_CONTROL, data1, 1);
		if (data1[0] != 0xaf) {
			I(" %s reset not ready %x\n" , __func__, data1[0]);
		}
	}
#endif
	chip_enable = 1;
	mutex_lock(&led_mutex);
	/* === enable CHIP_EN === */
	data = 0x40;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	/* === configuration control in power save mode=== */
	data = 0x29;
	ret = i2c_write_block(client, CONFIG_REGISTER, &data, 1);
	/* ===  Init LED mapping === */
	data = 0xDB;
	ret = i2c_write_block(client, LED_MAP_CONTROL, &data, 1);
	ret = i2c_read_block(client, LED_MAP_CONTROL, &data, 1);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5562_led_disable(struct i2c_client *client)
{
	int ret = 0;
	uint8_t data;
	I(" %s +++\n" , __func__);

	if (!chip_enable) {
		I(" %s return, chip already disable\n" , __func__);
		return;
	}

	data = 0x00;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	latest_set_enable_register = data;
#if 0 // LED enable pin keep high to avoid SRAM abnormal
	if (plat_data->ena_gpio) {
		ret = gpio_direction_output(plat_data->ena_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
			gpio_free(plat_data->ena_gpio);
		}
	}
#endif

	chip_enable = 0;

	I(" %s ---\n" , __func__);
}

static void lp5562_red_long_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret, reg_index = 0;

	I(" %s +++\n" , __func__);
	mutex_lock(&led_mutex);
#if 1
	clear_rgb_notification_on_charger(__func__);
#endif
	data = 0x10;
	ret = write_operation_register(client, data, 0);
	udelay(200);

	reg_index = 0;
	/* === set pwm to 200 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0xc8;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === wait 0.999s === */
	data = 0x7f;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === set pwm to 0 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === wait 0.999s === */
	data = 0x7f;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === clear register === */
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);

	/* === run program === */

	data = 0x20;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = 0x60;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
#if 1
static uint8_t *lp5562_get_ramp_program(uint8_t *data, int prescale, int step_time, int increment, int steps, int dimness) {
	data[0] = 0x00;
	data[1] = 0x00;
	if (prescale) {
		data[0] += 0x40;
	}
	step_time = step_time * dimness;
	if (prescale) {
		if (step_time > 0x3f) {
			step_time /= 30;
			data[0] = 0x00; // don't prescale for that much long step_time
		}
	}
	if (step_time > 0x3f) step_time = 0x3f;
	data[0] += step_time;
	if (!increment)  {
		data[1] += 0x80;
	}
	steps = steps / dimness;
	if (steps > 0x7f) steps = 0x7f;
	data[1] += steps;
	return data;
}
#endif

// smart automation
static int smart_get_pulse_dimming(void) {
	int ret = lights_down_divider;
	int level = smart_get_notification_level(NOTIF_PULSE_LIGHT);
	if (level==NOTIF_DIM) {
		ret = 16; // should DIM!
	}
	pr_info("%s smart_notif =========== level: %d  pulse_dimming %d \n",__func__, level, ret);
	return ret;
}
static int smart_get_button_dimming(void) {
	int ret = lights_down_divider;
	int level = smart_get_notification_level(NOTIF_BUTTON_LIGHT);
	if (level==NOTIF_DIM) {
		ret = 16; // should DIM!
	}
	pr_info("%s smart_notif =========== level: %d  buttonlight_dimming %d \n",__func__, level, ret);
	return ret;
}
static int smart_get_bln_no_charger_switch(void) {
	int ret = get_bln_no_charger_switch();
	int level = smart_get_notification_level(NOTIF_BUTTON_LIGHT);
	if (level==NOTIF_TRIM) {
		ret = 0; // should not blink if not on charger
	}
	pr_info("%s smart_notif =========== level: %d  bln_no_charger_switch %d \n",__func__, level, ret);
	return ret;
}

// ----------------

/* BLN - VK blink codes */
static int vk_led_step(uint8_t command_data[], int reg_index, uint8_t brightness, uint8_t time, int set_brightness)
{
	if (set_brightness) {
		/*=== Set PWM to brightness ===*/
		command_data[reg_index++] = 0x40;
		command_data[reg_index++]  = brightness / ((uci_get_user_property_int_mm("bln_light_level", bln_coeff_divider, 0, 20)+1) * smart_get_button_dimming());
	}

	/*=== wait for time ===*/
	command_data[reg_index++] = time;
	command_data[reg_index++]  = 0x00;
	return reg_index;
}

static int virtual_key_led_change_pwm(struct i2c_client *client, int pwm_diff);

static int vk_led_blink = 0;

static uint8_t bln_get_sleep_time(int buttons) {
		uint8_t data = 0x00;
		switch (get_bln_speed()) {
			case 2:
			data = 0x44;
			break;
			case 1:
			data = get_bln_pulse_rgb_pattern()==RGB_PATTERN_TRIPLE_UP || get_bln_pulse_rgb_pattern()==RGB_PATTERN_TRIPLE_DOWN ? 44 : (buttons?0x7c:0x7c);
			break;
			case 0:
			data = get_bln_pulse_rgb_pattern()==RGB_PATTERN_TRIPLE_UP || get_bln_pulse_rgb_pattern()==RGB_PATTERN_TRIPLE_DOWN ? (buttons?0x7d:0x7c) : (buttons?0x7c:0x7c);
			break;
		}
		return data;
}

static int bln_get_alarm_time(void) {
		int data = 0x00;
		switch (get_bln_speed()) {
			case 2:
			data = 1500;
			break;
			case 1:
			data = 2300;
			break;
			case 0:
			data = 3300;
			break;
		}
		return data;
}

static unsigned long VK_OFF_TIME = 50 * JIFFY_MUL;
static int vk_off_time_passed = 0;
/*
*	check if screen is off, and enough time passed for vk led off process... return 1 if all off.
*/
static int vk_screen_is_off(void)
{
	unsigned int diff_jiffies = 0;
	if (screen_on) return 0;
	diff_jiffies = jiffies - screen_off_jiffies;
	I("jiffies diff passed : %u\n", diff_jiffies);
	if (!vk_off_time_passed && diff_jiffies < VK_OFF_TIME) return 0;
	vk_off_time_passed = 1;
	I("vk_screen_is_off 1\n");
	return 1;
}

static void virtual_key_led_blink(int onoff, int dim)
{
	struct i2c_client *client = private_lp5562_client;
	int ret = 0, reg_index = 0;
	int dim_division = (onoff?0:dim)?8:1;
	int dimming = (dim_division * (get_bln_light_level()) * smart_get_button_dimming());
	int step_time = get_bln_pulse_rgb_pattern() == RGB_PATTERN_ONEPLUS5 ? 7:3;
	uint8_t data;
	uint8_t ramp_data[2] = {0x00,0x00};
	uint8_t command_data[50] = {0};

	if(!client)
		return;

	I("virtual_key_led_blink +++, onoff = %d\n", onoff);
	if (!vk_present) return;

	if((onoff || dim) && (vk_screen_is_off() || (!screen_on && bln_on_screenoff))) {
		vk_led_blink = 1;

		lp5562_led_enable(client, 1);

		data = 0;
		ret = i2c_write_block(client, ENG_3_PC_CONTROL, &data, 1);

		data = 0x01;
		ret = write_operation_register(client, data, 1);

		if (short_vib_notif) {
			short_vib_notif = 0; // reset, so if next time vibration is off, it doesn't accidentally trigger a double blink short notfi...
			if (get_bln_pulse_rgb_pattern()>RGB_PATTERN_ONEPLUS5) {
				reg_index = vk_led_step(command_data, reg_index, 0x40 / dim_division, 0x44, 1);
				reg_index = vk_led_step(command_data, reg_index, 0xc8 / dim_division, 0x44, 1);
				reg_index = vk_led_step(command_data, reg_index, 0x50 / dim_division, 0x4a, 1);
				reg_index = vk_led_step(command_data, reg_index, 0xc8 / dim_division, 0x4e, 1);
				reg_index = vk_led_step(command_data, reg_index, 0xb0 / dim_division, 0x44, 1);
				reg_index = vk_led_step(command_data, reg_index, 0x40 / dim_division, 0x44, 1);
			} else {
// set PWM start vaue to 5
				command_data[reg_index++] = 0x40;
				command_data[reg_index++]  = 0x05;
// Ramp up
				lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,1,108,dimming);
				command_data[reg_index++] = ramp_data[0];
				command_data[reg_index++]  = ramp_data[1];
// down quick
				lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,0,88,dimming);
				command_data[reg_index++] = ramp_data[0];
				command_data[reg_index++]  = ramp_data[1];

// up quick
				lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,1,88,dimming);
				command_data[reg_index++] = ramp_data[0];
				command_data[reg_index++]  = ramp_data[1];
// Ramp down
				lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,0,107,dimming);
				command_data[reg_index++] = ramp_data[0];
				command_data[reg_index++]  = ramp_data[1];
			}

		} else {
			if (get_bln_pulse_rgb_pattern()>RGB_PATTERN_ONEPLUS5 || dim_division != 1) {
				reg_index = vk_led_step(command_data, reg_index, 0x20 / dim_division, pattern_time_button[get_bln_pulse_rgb_pattern()][0], 1);
				reg_index = vk_led_step(command_data, reg_index, 0x70 / dim_division, pattern_time_button[get_bln_pulse_rgb_pattern()][1], 1);
				reg_index = vk_led_step(command_data, reg_index, 0xb0 / dim_division, pattern_time_button[get_bln_pulse_rgb_pattern()][2], 1);
				reg_index = vk_led_step(command_data, reg_index, 0xc8 / dim_division, pattern_time_button[get_bln_pulse_rgb_pattern()][3], 1);
				reg_index = vk_led_step(command_data, reg_index, 0xb0 / dim_division, pattern_time_button[get_bln_pulse_rgb_pattern()][4], 1);
				reg_index = vk_led_step(command_data, reg_index, 0x40 / dim_division, pattern_time_button[get_bln_pulse_rgb_pattern()][5], 1);
			} else {
// set PWM start vaue to 5
				command_data[reg_index++] = 0x40;
				command_data[reg_index++]  = 0x05;
// Ramp command will be: 0100 0010 0000 0100b
//= 4240h
				lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,1,108,dimming);
				command_data[reg_index++] = ramp_data[0];
				command_data[reg_index++]  = ramp_data[1];

				lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,1,88,dimming);
				command_data[reg_index++] = ramp_data[0];
				command_data[reg_index++]  = ramp_data[1];
// Ramp down 42c0h
				lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,0,107,dimming);
				command_data[reg_index++] = ramp_data[0];
				command_data[reg_index++]  = ramp_data[1];

				lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,0,88,dimming);
				command_data[reg_index++] = ramp_data[0];
				command_data[reg_index++]  = ramp_data[1];
			}
		}

		data = bln_get_sleep_time(1);
		reg_index = vk_led_step(command_data, reg_index, 0x00, data, 1);
		reg_index = vk_led_step(command_data, reg_index, 0x00, 0x7f, 0);
		if (get_bln_speed()==0) {
			reg_index = vk_led_step(command_data, reg_index, 0x00, 0x7f, 0);
		}

		/* === clear register === */
//		command_data[reg_index++] = 0x00;
//		command_data[reg_index++]  = 0x00;

		ret = i2c_write_block(client, CMD_ENG_3_BASE, command_data, reg_index);

		data = 0x02;
		ret = write_operation_register(client, data, 1);
		data = 0x42;
		ret = write_enable_register(client, data, 1);
	} else if (vk_led_blink && vk_screen_is_off()) {
#if 1
		lp5562_led_enable(client, 1);

		// start empty program to set to brightness 0
		data = 0;
		ret = i2c_write_block(client, ENG_3_PC_CONTROL, &data, 1);

		data = 0x01;
		ret = write_operation_register(client, data, 1);

		reg_index = vk_led_step(command_data, reg_index, 0x00, 0x7d, 1);

		/* === clear register === */
		command_data[reg_index++] = 0x00;
		command_data[reg_index++]  = 0x00;

		ret = i2c_write_block(client, CMD_ENG_3_BASE, command_data, reg_index);

		data = 0x02;
		ret = write_operation_register(client, data, 1);
		data = 0x42;
		ret = write_enable_register(client, data, 1);

		msleep(5);

		// .. and finally disable the vk led...
		data = 0x00; // off
		ret += write_enable_register(client,data,1); // vk off...
		if (ret) {
			I("failed to disable vk led 1\n");
		}
		msleep(5);
		data = 0x00; // off
		ret += write_enable_register(client,data,1); // vk off...
		if (ret) {
			I("failed to disable vk led 2\n");
		}
		vk_led_blink = 0;

#endif
	}
}

extern void flash_blink(bool haptic);
extern void flash_stop_blink(void);
extern void kernel_ambient_display(void);
extern int is_kernel_ambient_display(void);
extern void stop_kernel_ambient_display(bool interrupt_ongoing);

void register_input_event(const char* caller);
void register_input_event_early(void);

static int blink_running = 0;
static int vary1 = 1;
static void vk_unblink_work_func(struct work_struct *work)
{
	I(" %s +++\n" , __func__);
	if (full_or_dim == 1 && get_bln_dim_blink()==1 && get_bln_dim_number() > 0) { // won't keep dim blink?
		if (!mutex_is_locked(&blinkstopworklock)) {
			int sleeptime = bln_get_alarm_time() * (vary1?(vary1==2?get_bln_dim_number()+2:get_bln_dim_number()):(get_bln_dim_number()-2));

			ktime_t wakeup_time;
			ktime_t curr_time = { .tv64 = 0 };
			wakeup_time = ktime_add_us(curr_time,
				(sleeptime * 1000LL)); // msec to usec

#if 0 
// test code
			vary1 = vary1 + 1;
			if (vary1>2) vary1 = 0;
#endif

			alarm_cancel(&blinkstopfunc_rtc); // stop pending alarm...
			alarm_start_relative(&blinkstopfunc_rtc, wakeup_time); // start new...

			I("%s: Current Time tv_sec: %ld, Alarm set to tv_sec: %ld\n",
				__func__,
				ktime_to_timeval(curr_time).tv_sec,
				ktime_to_timeval(wakeup_time).tv_sec);
		}
	}
	blink_running = get_bln_dim_blink()&&full_or_dim;
	virtual_key_led_blink(0,get_bln_dim_blink()&&full_or_dim);

	full_or_dim = 0;
}

static enum alarmtimer_restart blinkstop_rtc_callback(struct alarm *al, ktime_t now)
{
	I("%s step trylock\n",__func__);
	if (!mutex_trylock(&blinkstopworklock)) {
		I("%s step trylock failed, return \n",__func__);
		return ALARMTIMER_NORESTART;
	}
	I("%s step trylock success \n",__func__);

	I("%s step unblink? .... blinking %d && !screen_on  %d \n",__func__, vk_led_blink, !screen_on);
	if (vk_led_blink && !screen_on) {
		I("%s step unblink! 0\n",__func__);
		queue_work(g_vk_work_queue, &vk_unblink_work);
	}
	mutex_unlock(&blinkstopworklock);

	return ALARMTIMER_NORESTART;
}


static int vary = 1;
static void vk_blink_work_func(struct work_struct *work)
{
	int uci_bln_number = get_bln_number();
	I(" %s +++\n" , __func__);
	if (screen_on) return;
	full_or_dim = 1;
	if (!blink_running) {
		blink_running = 1;
		virtual_key_led_blink(1,get_bln_dim_blink());
	}
	if (uci_bln_number > 0 && !charging) { // if blink number is not infinite and is not charging, schedule CANCEL work
		if (!mutex_is_locked(&blinkstopworklock)) {
			int sleeptime = bln_get_alarm_time() * (vary?(vary==2?uci_bln_number+2:uci_bln_number):(uci_bln_number-2));

			ktime_t wakeup_time;
			ktime_t curr_time = { .tv64 = 0 };
			wakeup_time = ktime_add_us(curr_time,
				(sleeptime * 1000LL)); // msec to usec

#if 0 
// test code
			vary = vary + 1;
			if (vary>2) vary = 0;
#endif

			alarm_cancel(&blinkstopfunc_rtc); // stop pending alarm...
			alarm_start_relative(&blinkstopfunc_rtc, wakeup_time); // start new...

			I("%s: Current Time tv_sec: %ld, Alarm set to tv_sec: %ld\n",
				__func__,
				ktime_to_timeval(curr_time).tv_sec,
				ktime_to_timeval(wakeup_time).tv_sec);
		}
	}
	I(" %s ---\n" , __func__);
}

static int last_charge_state = 0;

// register charging: this symbol function will register charging events from anywhere in kernel calls
// e.g. from USB driver
void register_charging(int on)
{
	bool input_event = false;
	I("%s %d\n",__func__,on);
	if (on!=charging) {
		input_event = true;
	}
	charging = on>0?1:0;
	// if going into no-charge mode, overwrite last charge state, 
	// ...so next time charging starts multicolored led will be set one time
	if (!charging) {
		last_charge_state = 0;
		clear_charging_notification_occured_for_rgb(__func__); // if user wakes phone clear this flag, otherwise it will keep blinking, even when user dismisses a notif
	}
	if (input_event) {
		register_input_event_early();
		stop_kernel_ambient_display(true);
	}
}
EXPORT_SYMBOL(register_charging);

int input_is_charging(void) {
	return !!charging;
}
EXPORT_SYMBOL(input_is_charging);


DEFINE_SPINLOCK(led_data_lock);

static void led_set_multicolor(int onoff, int red, int green){
	I(" %s , set display_flag = %d red %d green %d \n" , __func__, onoff, red, green);
	clear_rgb_notification_on_charger(__func__);
	if(onoff){
		wake_lock_timeout(&(g_led_led_data_bln->led_wake_lock), 2*HZ);
		spin_lock(&led_data_lock);
		g_led_led_data_bln->Mode = 1;
		g_led_led_data_bln->Red = red / (get_bln_rgb_blink_light_level()*smart_get_pulse_dimming());
		g_led_led_data_bln->Green = green / (get_bln_rgb_blink_light_level()*smart_get_pulse_dimming());
		g_led_led_data_bln->Blue = 0;
		queue_work(g_led_work_queue, &g_led_led_data_bln->led_work_multicolor);
		spin_unlock(&led_data_lock);
	}else {
		spin_lock(&led_data_lock);
		g_led_led_data_bln->Mode = 0;
		g_led_led_data_bln->Red = 0;
		g_led_led_data_bln->Green = 0;
		g_led_led_data_bln->Blue = 0;
		queue_work(g_led_work_queue, &g_led_led_data_bln->led_work_multicolor);
		spin_unlock(&led_data_lock);
	}
}


// depending on red variable different command base is used to write i2c...
static int color_blink_step_by_color(struct i2c_client *client, uint8_t brightness, uint8_t time, int reg_index, int red)
{
	uint8_t data = 0x00;
	int ret = 0;
	/* # === set pwm brightness === */
	data = 0x40;
	ret = i2c_write_block(client, (red?CMD_ENG_1_BASE:CMD_ENG_2_BASE) + reg_index++, &data, 1);
	data = brightness / (get_bln_rgb_blink_light_level()*smart_get_pulse_dimming());
	ret = i2c_write_block(client, (red?CMD_ENG_1_BASE:CMD_ENG_2_BASE) + reg_index++, &data, 1);
	/* === wait time === */
	data = time;
	ret = i2c_write_block(client, (red?CMD_ENG_1_BASE:CMD_ENG_2_BASE) + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, (red?CMD_ENG_1_BASE:CMD_ENG_2_BASE) + reg_index++, &data, 1);
	return reg_index;

}

// will start a gentle blinking of green, to signal something like full battery in charge mode...
static void led_multicolor_short_transition(void)
{
	struct i2c_client *client = private_lp5562_client;
	uint8_t data = 0x00;
	int ret, reg_index = 0;
	uint8_t mode = 0x00;

	if (!client) return;

	I(" %s +++ \n" , __func__);
	mutex_lock(&led_mutex);
#if 1
	clear_rgb_notification_on_charger(__func__);
#endif

	if (1) // green mode
		mode |= (3 << 2);

	data = mode & 0x15;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (u8)0;
	ret = i2c_write_block(client, ENG_1_PC_CONTROL, &data, 1);
	udelay(200);
	ret = i2c_write_block(client, ENG_2_PC_CONTROL, &data, 1);

		reg_index = 0;
		/* # === set green blink === */
		reg_index = color_blink_step_by_color(client, 0x22, 0x4c, reg_index,0);
		reg_index = color_blink_step_by_color(client, 0x42, 0x4c, reg_index,0);
		reg_index = color_blink_step_by_color(client, 0xa0, 0x4c, reg_index,0);
		reg_index = color_blink_step_by_color(client, 0xc8, 0x4c, reg_index,0);
		reg_index = color_blink_step_by_color(client, 0xb0, 0x4c, reg_index,0);
		reg_index = color_blink_step_by_color(client, 0x82, 0x4c, reg_index,0);
		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);

	/* === run program === */
	data = mode & 0x2a;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (mode & 0x2a)|0x40;
	ret = write_enable_register(client, data, 0);
	udelay(150);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}
static void rgb_charge_short_transition(struct work_struct * rgb_charge_short_transition_work)
{
	msleep(1000);
	led_multicolor_short_transition();
}
static DECLARE_WORK(rgb_charge_short_transition_work, rgb_charge_short_transition);

static void rgb_charge_short_transition_async(void) {
	queue_work(g_led_work_queue,&rgb_charge_short_transition_work);
}


static void led_multicolor_charge_notification(void) {
	uint8_t ramp_data[2] = {0x00,0x00};
	int dimming = (get_bln_rgb_blink_light_level()*smart_get_pulse_dimming());

	int speed = charge_level;

	int red = charge_level < get_bln_pulse_rgb_blink_on_charger_red_limit();
	int reg_base = CMD_ENG_2_BASE;


	struct i2c_client *client = private_lp5562_client;
	uint8_t data = 0x00;
	uint8_t green = 0xff;
	int ret, reg_index = 0;
	uint8_t mode = 0x00;
	I(" %s +++ \n" , __func__);

	if (!client) return;
	I(" %s last rg charge level %d charge level %d\n",__func__,last_rgb_notification_charge_level, charge_level);

	if (last_rgb_notification_charge_level == charge_level) return;
	last_rgb_notification_charge_level = charge_level;

	if (red) {
		reg_base = CMD_ENG_1_BASE;
	}

	mutex_lock(&led_mutex);

	if (!red) // green mode
		mode |= (3 << 2);
	else  // red mode
		mode |= (3 << 4);

	data = mode & 0x15;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (u8)0;
	ret = i2c_write_block(client, ENG_1_PC_CONTROL, &data, 1);
	udelay(200);
	ret = i2c_write_block(client, ENG_2_PC_CONTROL, &data, 1);


	// green
	{
	if (!get_bln_pulse_rgb_blink()) {
		green = green / (get_bln_rgb_blink_light_level()*smart_get_pulse_dimming());
		/* === set green pwm === */
		data = 0x40;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);

		ret = i2c_write_block(client, reg_base + reg_index++, &green, 1);
		/* === wait 0.064s === */
		data = 0x44;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		/* === set pwm to 0 === */
		data = 0x40;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		/* === wait 0.935s === */
		data = 0x7c;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		/* === wait 0.999s === */
		data = 0x7f;
		data = 0x6f;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
	} else {
		I(" %s BLINK +++ green:%d\n" , __func__, green);
		/* # === set green blink === */
		if (get_bln_pulse_rgb_pattern()>RGB_PATTERN_ONEPLUS5) {
			reg_index = color_blink_step_by_color(client, pattern_brightness[get_bln_pulse_rgb_pattern()][0], pattern_time[get_bln_pulse_rgb_pattern()][0], reg_index,red);
			reg_index = color_blink_step_by_color(client, pattern_brightness[get_bln_pulse_rgb_pattern()][1], pattern_time[get_bln_pulse_rgb_pattern()][1], reg_index,red);
			reg_index = color_blink_step_by_color(client, pattern_brightness[get_bln_pulse_rgb_pattern()][2], pattern_time[get_bln_pulse_rgb_pattern()][2], reg_index,red);
			reg_index = color_blink_step_by_color(client, pattern_brightness[get_bln_pulse_rgb_pattern()][3], pattern_time[get_bln_pulse_rgb_pattern()][3], reg_index,red);
			reg_index = color_blink_step_by_color(client, pattern_brightness[get_bln_pulse_rgb_pattern()][4], pattern_time[get_bln_pulse_rgb_pattern()][4], reg_index,red);
			reg_index = color_blink_step_by_color(client, pattern_brightness[get_bln_pulse_rgb_pattern()][5], pattern_time[get_bln_pulse_rgb_pattern()][5], reg_index,red);
		} else {
			int step_time = ((get_bln_pulse_rgb_pattern() == RGB_PATTERN_ONEPLUS5 ? 7:3)*5)/(speed/10);
// set PWM start vaue to 5
			data = 0x40;
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
			data = 0x05;
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
// Ramp command will be: 0100 0010 0000 0100b
//= 4240h
			lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,1,112,dimming);
			data = ramp_data[0]; // prescale 01000000b + 0x08 step time
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
			data = ramp_data[1]; // increment 00000000b + 0x70 steps
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);

			lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,1,88,dimming);
			data = ramp_data[0];
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
			data = ramp_data[1]; // increment 00000000b + 0x58 steps
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
// Ramp down 42c0h

			lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,0,112,dimming);
			data = ramp_data[0]; // prescale 01000000b + 0x08 step time
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
			data = ramp_data[1]; // increment 00000000b + 0x70 steps
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);

			lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,0,88,dimming);
			data = ramp_data[0];
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
			data = ramp_data[1]; // increment 00000000b + 0x58 steps
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		}
		data = bln_get_sleep_time(0);
		reg_index = color_blink_step_by_color(client, 0x00, data, reg_index,red);//0x7c, reg_index);
		/* === wait 0.999s === */
		data = 0x7f;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
#if 0
		if (get_bln_speed() == 0) {
			/* === wait 0.999s === */
			data = 0x7f;
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
			data = 0x00;
			ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		}
#endif
		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
		ret = i2c_write_block(client, reg_base + reg_index++, &data, 1);
	}
	}


	/* === run program === */
	data = mode & 0x2a;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (mode & 0x2a)|0x40;
	ret = write_enable_register(client, data, 0);
	udelay(150);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}
static void rgb_blink_on_charge(struct work_struct * rgb_blink_on_charge_work)
{
	//msleep(1000);
        led_multicolor_charge_notification();
}
static DECLARE_WORK(rgb_blink_on_charge_work, rgb_blink_on_charge);

static void rgb_blink_on_charge_async(void) {
	queue_work(g_led_work_queue,&rgb_blink_on_charge_work);
}

/*
* set RG led to appropriate red/green ratio for the current battery level...
* will set multicolor RG led, or at 100% a gently blinking green led.
* Will only do the operations if level changed or just started to charge...
*/
static void led_multi_color_charge_level(int level, bool force) {

	static int last_level = 0;
	int level_div = level / 5;
	int level_round = level_div * 5; // rounding by 5;
	int us_level = (level_round * 235)/100;
	int red_coeff = 255 - (us_level); // red be a bit more always, except on FULL charge ( 255 - 220 -> Min = 25, except on full where it is 1 )
	int green_coeff = 235 - red_coeff; // green be a bit less always, except on FULL charge ( Green max is 220, min 1 - except on full where its 255)

	I(" %s charging notif occured for rgb: %d screen_on_early %d\n",__func__,charging_notification_occured_for_rgb, screen_on_early);

	if (charging_notification_occured_for_rgb && !screen_on_early) {
		// instead of charge color, call notification blinking for RGB led
		rgb_blink_on_charge_async();
		return;
	}

	if (!force && !(get_bln_rgb_batt_colored() && supposedly_charging)) return;

	// check if we're changing from no charge state to charge led and that last received battery level differs from current...
	if (!force && last_charge_state == supposedly_charging && last_level == level) return; // no change, return

	// store new values into last_ variables
	last_charge_state = 1;
	last_level = level;

	if (green_coeff < 1) green_coeff = 10;

	if (level<5) { // under 5, always full RED but low light for red
		red_coeff = 80;
		green_coeff = 1;
	} else
	if (level<15) { // under 15, always full RED but lower light for red
		red_coeff = 160;
		green_coeff = 3;
	} else
	if (level<20) { // under 20, always full RED full light for red
		red_coeff = 255;
		green_coeff = 7;
	}

	if (level == 100) { // at 100, always full GREEN
		red_coeff = 1;
		green_coeff = 255;
		I("%s color transition at full strength: red %d green %d \n",__func__, red_coeff, green_coeff);
	}

	if (!get_bln_rgb_batt_colored()) {
		if (level == 100) {
			red_coeff = 1;
			green_coeff = 255;
		} else {
			red_coeff = 255;
			green_coeff = 1;
		}
		led_set_multicolor(1, red_coeff, green_coeff);
	} else {
		led_set_multicolor(1, red_coeff, green_coeff);
		if (level == 100) { // at 100, start fading work too for fancy noticeable look
			rgb_charge_short_transition_async();
		} 
	}
}

void register_charge_level(int level)
{
	I("%s %d\n",__func__,level);
	charge_level = level;
	if (get_bln_rgb_batt_colored() && charging && supposedly_charging) {
		// TRIGGER COLOR CHANGE of led
		I("%s triggering color change to multicolor led %d\n",__func__,level);
		led_multi_color_charge_level(level, false);
	}
	first_level_registered = 1;
}
EXPORT_SYMBOL(register_charge_level);

// handling haptic notifications if enabled to register notifications even when RGB led is already blinking, or on charger
static unsigned long last_haptic_jiffies = 0;
static int last_value = 0;
static unsigned long MAX_DIFF = 200 * JIFFY_MUL;
static unsigned long MAX_DIFF_LONG_VIB = 310 * JIFFY_MUL;

#define FINGERPRINT_VIB_TIME_EXCEPTION 40
#define ALARM_VIB_TIME_EXCEPTION 500
#define CALL_VIB_TIME_EXCEPTION 1000
#define SQUEEZE_VIB_TIME_EXCEPTION 15
#define SQUEEZE_VIB_TIME_EXCEPTION_BASE_2 20
#define DOUBLETAP_VIB_TIME_EXCEPTION 25


#define STACK_LENGTH 10

//extern void register_squeeze_wake(int nanohub_flag, int vibrator_flag, unsigned long timestamp, int init_event_flag);
extern void register_squeeze(unsigned long timestamp, int vibration);

// callback to register fingerprint vibration
extern int register_fp_vibration(void);

// vib notification reminder
extern void set_vib_notification_reminder(int value);
extern int get_vib_notification_reminder(void);
extern void set_vib_notification_slowness(int value);
extern int get_vib_notification_slowness(void);
extern void set_vib_notification_length(int value);
extern int get_vib_notification_length(void);

static enum alarmtimer_restart flash_stop_rtc_callback(struct alarm *al, ktime_t now)
{
	flash_stop_blink();
	return ALARMTIMER_NORESTART;
}

long get_global_mseconds(void) {
	struct timespec ts;
	long ret = 0;
	getnstimeofday(&ts);
	ret = (ts.tv_sec*1000LL) + ((ts.tv_nsec)/(1000LL*1000LL));
	//I("%s time = %ld",__func__,ret);
	return ret;
}

static long last_input_event = 0;
void set_last_input_event(const char * caller) {
	//I("%s caller %s",__func__,caller);
	last_input_event = get_global_mseconds();
}
void register_input_event_early(void) {
	wake_by_user = 1;
	if (charging_notification_occured_for_rgb) {
		if (screen_on && charging && get_bln_pulse_rgb_blink_on_charger() && wake_by_user) {
			clear_charging_notification_occured_for_rgb(__func__); // if user wakes phone clear this flag, otherwise it will keep blinking, even when user dismisses a notif
			// start charge color, to cancel RGB blinking
			led_multi_color_charge_level(charge_level, true);
		}
	}
	set_last_input_event(__func__);
	if (screen_on_early) {
		// user is inputing phone, no haptic blinking should trigger BLN when screen off
		bln_on_screenoff = 0;
	}
	smart_set_last_user_activity_time();
}
void register_input_event(const char * caller) {
//	pr_info("%s kad self wake: blocking event\n",__func__);
	wake_by_user = 1;
	if (charging_notification_occured_for_rgb) {
		if (charging && get_bln_pulse_rgb_blink_on_charger() && wake_by_user) {
			clear_charging_notification_occured_for_rgb(__func__); // if user wakes phone clear this flag, otherwise it will keep blinking, even when user dismisses a notif
			// start charge color, to cancel RGB blinking
			led_multi_color_charge_level(charge_level, true);
		}
	}
	//I("%s caller %s",__func__,caller);
	set_last_input_event(__func__);
	if (screen_on_early) {

		ktime_t wakeup_time;
		ktime_t curr_time = { .tv64 = 0 };
		wakeup_time = ktime_add_us(curr_time,
			(1LL * 1000LL)); // msec to usec
		// user must have exited Ambient display by pressing power/touchscreen etc, stop flash blinking!
		if (probe_finished) {
			alarm_cancel(&flash_stop_rtc); // stop pending alarm...
			alarm_start_relative(&flash_stop_rtc, wakeup_time); // start new...
		}

		// user is inputing phone, no haptic blinking should trigger BLN when screen off
		bln_on_screenoff = 0;
//		pr_info("%s kad bln_on_screenoff %d\n", __func__, bln_on_screenoff);
	}
	smart_set_last_user_activity_time();
}
EXPORT_SYMBOL(register_input_event);

/**
* is bln to be started, based on switches like BLN, bln no charger, smart button dimming and charge state
*/
static bool should_start_bln(void) {
    if (!vk_present) return false;
    return !vk_led_blink && get_bln_switch() && ((smart_get_bln_no_charger_switch() && !charging && smart_get_button_dimming()==1) || charging);
}
static bool should_start_pulse_blink_on_charger(void) {
	// only start rg blinking if screen is off! otherwise it will get stuck even when dismissing notifs, as that won't stop through RGB led state change
	int ret = vk_screen_is_off() && get_bln_pulse_rgb_blink_on_charger() && charging;
	pr_info("LED %s ret = %d\n",__func__,ret);
	return ret;
}

static int last_notification_number = 0;
// register sys uci listener
void uci_sys_listener(void) {
	pr_info("%s uci sys parse happened...\n",__func__);
	{
		int ringing = uci_get_sys_property_int_mm("ringing", 0, 0, 1);
		pr_info("%s uci sys ringing %d\n",__func__,ringing);
		if (ringing) {
			register_input_event(__func__);
			stop_kernel_ambient_display(true);
		}
	}
	{
		int notifications = uci_get_sys_property_int("notifications",0);
		if (notifications != -EINVAL) {
		if (notifications>last_notification_number) {
			if (should_start_bln()) {
				// store haptic blinking, so if ambient display blocks the bln, later in BLANK screen off, still it can be triggered
				bln_on_screenoff = 1;
				pr_info("%s kad bln_on_screenoff %d\n", __func__, bln_on_screenoff);
				if (!is_kernel_ambient_display() && !screen_on) queue_work(g_vk_work_queue, &vk_blink_work);
			}
			if (should_start_pulse_blink_on_charger()) {
				charging_notification_occured_for_rgb = 1;
				rgb_blink_on_charge_async(); // call charging level based speedy pulse blink
			}
			// call flash blink for flashlight notif if lights_down mode (>1) is not active...
			if ((vk_screen_is_off() || !wake_by_user) && (lights_down_divider==1)) { // only flash if lights_down mode is not active...screen is off or ambient wake
				flash_blink(false);
			}
			kernel_ambient_display();
		}
		last_notification_number = notifications;
		}
        }
}

// if a single haptic vibration was last time interpreted as notif set this true
// .. based on this next such exact vibration length will be skipped to be interpreted as notif avoiding doubles...
static bool single_vibration_interpreted_as_notif = false;

int register_haptic(int value)
{
	bool vib_notif_pattern_detected = false;
	unsigned long stack_entries[STACK_LENGTH];
	struct stack_trace trace = {
		.nr_entries = 0,
		.entries = &stack_entries[0],

		.max_entries = STACK_LENGTH,

		/* How many "lower entries" to skip. */
		.skip = 0
	};
	unsigned int diff_jiffies = jiffies - last_haptic_jiffies;
	last_haptic_jiffies = jiffies;
	I("%s %d - jiffies diff %u \n",__func__,value, diff_jiffies);

//	if this exceptional time is used, it means, fingerprint scanner vibrated with proxomity sensor detection on
//	and with unregistered finger, so no wake event. In this case, don't start blinking, not a notif, just return
//	same with squeeze vibration time value.

	save_stack_trace(&trace);
	//WARN_ON(1);

	if (value == DOUBLETAP_VIB_TIME_EXCEPTION) {
		register_input_event(__func__);
		return value;
	}
	if (value == CALL_VIB_TIME_EXCEPTION || value == ALARM_VIB_TIME_EXCEPTION) {
		register_input_event(__func__);
		stop_kernel_ambient_display(true);
		return value;
	}
	if (value == FINGERPRINT_VIB_TIME_EXCEPTION) {
		int vib_strength = register_fp_vibration();
		//register_input_event(); // do not register here as long as it's not sure that it actually will wake the device or is just a nonwaking wrong FP scan double vib feedback
		if (vib_strength > 0 && vib_strength<=FINGERPRINT_VIB_TIME_EXCEPTION*2) return vib_strength/2; else return vib_strength>0?FINGERPRINT_VIB_TIME_EXCEPTION:0;
	}
	if (value == SQUEEZE_VIB_TIME_EXCEPTION || value == SQUEEZE_VIB_TIME_EXCEPTION_BASE_2) {
		last_value = value;
//		register_squeeze_wake(0,1,jiffies,0);
		register_squeeze(jiffies,1);
		return value;
	}

	if (screen_on && wake_by_user) return value;

	// checking repetition of same length vibs
	if (last_value == value) {
		if ((value > 500 && diff_jiffies < MAX_DIFF_LONG_VIB) || diff_jiffies < MAX_DIFF) {
			if (single_vibration_interpreted_as_notif) {
				single_vibration_interpreted_as_notif = false;
			} else {
				vib_notif_pattern_detected = true;
				if (value <= 200) {
					short_vib_notif = 1;
				} else {
					short_vib_notif = 0;
				}
			}
		} else {
			// if same length but time diff too long, check for single vibration patter over a 100 msec length...
			if (value>=100) {
				single_vibration_interpreted_as_notif = true;
				vib_notif_pattern_detected = true;
			} else {
				single_vibration_interpreted_as_notif = false;
			}
		}
	} else {
		// checking first time vib length, if over a length, interpret as notif
		if (value>=100) {
			single_vibration_interpreted_as_notif = true;
			vib_notif_pattern_detected = true;
		} else {
			single_vibration_interpreted_as_notif = false;
		}
	}

	if (vib_notif_pattern_detected) {
		if (should_start_bln()) {
			// store haptic blinking, so if ambient display blocks the bln, later in BLANK screen off, still it can be triggered
			bln_on_screenoff = 1;
			pr_info("%s kad bln_on_screenoff %d\n", __func__, bln_on_screenoff);
			if (!is_kernel_ambient_display() && !screen_on) queue_work(g_vk_work_queue, &vk_blink_work);
		}
		if (should_start_pulse_blink_on_charger()) {
			charging_notification_occured_for_rgb = 1;
			rgb_blink_on_charge_async(); // call charging level based speedy pulse blink
		}
		// call flash blink for flashlight notif if lights_down mode (>1) is not active...
		if (lights_down_divider==1) {
			flash_blink(true);
		}
		kernel_ambient_display();
	}
	last_value = value;
	return value;
}

EXPORT_SYMBOL(register_haptic);

extern void set_vibrate(int value);
extern void set_suspend_booster(int value);
extern void override_kad_on(int on);

static enum alarmtimer_restart vibrate_rtc_callback(struct alarm *al, ktime_t now)
{
	if (lights_down_divider == 1) { 
		set_vibrate(111);
	} else {
		set_vibrate(234);
	}
	return ALARMTIMER_NORESTART;
}

#define VIB_LIGHT_MODE_HIGH_LIGHT_VIB_REMINDER 3
#define VIB_LIGHT_MODE_HIGH_LIGHT_VIB 2
#define VIB_LIGHT_MODE_LOW_LIGHT 1
#define VIB_LIGHT_MODE_LOW_LIGHT_VIB 0
#define VIB_LIGHT_MODE_LOW_LIGHT_VIB_KAD -1
static int vib_light_mode = VIB_LIGHT_MODE_HIGH_LIGHT_VIB;

static int double_double_vol_check_running = 0;
static enum alarmtimer_restart double_double_vol_rtc_callback(struct alarm *al, ktime_t now)
{
	double_double_vol_check_running = 0;
	// single press happened...
	if (lights_down_divider==1 && !get_vib_notification_reminder()) {
		vib_light_mode = VIB_LIGHT_MODE_LOW_LIGHT;
		lights_down_divider = 16; set_vibrate(451);
	} else {
		vib_light_mode = VIB_LIGHT_MODE_HIGH_LIGHT_VIB;
		lights_down_divider = 1;
		set_vibrate(101);
	}
	pr_info("%s override kad on 1",__func__);
	override_kad_on(0);
	set_vib_notification_reminder(0);
	stop_kernel_ambient_display(true);
	return ALARMTIMER_NORESTART;
}

static bool uci_is_double_volume_gesture(void) {
	int ret = uci_get_user_property_int_mm("double_volume_gesture",0,0,1);
	return !!ret;
}

void register_double_volume_key_press(int long_press) {
	if (!uci_is_double_volume_gesture()) {
		override_kad_on(0);
		set_suspend_booster(0);
		lights_down_divider = 1;
		return;
	}
	if (screen_on) return;
	pr_info("%s gpio -> lights down divider - %d\n",__func__,lights_down_divider);
	if (long_press) {
		// long press
		ktime_t wakeup_time;
		ktime_t curr_time = { .tv64 = 0 };
		// always switch on low light mode, very long vibration signal
		set_suspend_booster(1); // suspend vibration boosting
		if (long_press == 2) {
			vib_light_mode = VIB_LIGHT_MODE_LOW_LIGHT_VIB_KAD;
			override_kad_on(1);
			set_vibrate(431);
		} else {
			vib_light_mode = VIB_LIGHT_MODE_LOW_LIGHT_VIB;
			override_kad_on(0);
			set_vibrate(231);
		}
		wakeup_time = ktime_add_us(curr_time,
			(500 * 1000LL)); // msec to usec
		lights_down_divider = 16;
		alarm_cancel(&vibrate_rtc); // stop pending alarm...
		alarm_start_relative(&vibrate_rtc, wakeup_time); // start new...
		stop_kernel_ambient_display(true);
	} else {
		set_suspend_booster(0); // single or double short press means enable boosting again...
		if (!double_double_vol_check_running) {
			// start checking for double press..
			ktime_t wakeup_time;
			ktime_t curr_time = { .tv64 = 0 };
			wakeup_time = ktime_add_us(curr_time,
				(400 * 1000LL)); // msec to usec
			// start double double check alarm timer... if it is not canceled, then it will switch between low/full light modes...
			double_double_vol_check_running = 1;
			alarm_cancel(&double_double_vol_rtc); // stop pending alarm...
			alarm_start_relative(&double_double_vol_rtc, wakeup_time); // start new...
		} else {
			// double press happened...
			ktime_t wakeup_time;
			ktime_t curr_time = { .tv64 = 0 };
			wakeup_time = ktime_add_us(curr_time,
				(200 * 1000LL)); // msec to usec
			// double press on time...switch full light mode and vib notification reminder on
			alarm_cancel(&double_double_vol_rtc); // stop pending alarm...
			double_double_vol_check_running = 0;
			if (vib_light_mode != VIB_LIGHT_MODE_HIGH_LIGHT_VIB_REMINDER) {
				set_vib_notification_reminder(1);
				lights_down_divider = 1;
				pr_info("%s override kad on 1",__func__);
				override_kad_on(1);
			}
			vib_light_mode = VIB_LIGHT_MODE_HIGH_LIGHT_VIB_REMINDER;
			set_vibrate(110);
			alarm_cancel(&vibrate_rtc); // stop pending alarm...
			alarm_start_relative(&vibrate_rtc, wakeup_time); // start new...vibrate a second one...
			stop_kernel_ambient_display(true);
		}
	}
}
EXPORT_SYMBOL(register_double_volume_key_press);



/* BLN - color blink codes */
static int color_blink_step(struct i2c_client *client, uint8_t brightness, uint8_t time, int reg_index)
{
	uint8_t data = 0x00;
	int ret = 0;
	/* # === set pwm brightness === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = brightness / (get_bln_rgb_blink_light_level()*smart_get_pulse_dimming());
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === wait time === */
	data = time;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	return reg_index;

}
#endif

static void lp5562_color_blink(struct i2c_client *client, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t data = 0x00;
	uint8_t ramp_data[2] = {0x00,0x00};
	int dimming = (get_bln_rgb_blink_light_level()*smart_get_pulse_dimming());

	int ret, reg_index = 0;
	uint8_t mode = 0x00;
	I(" %s +++ red:%d, green:%d, blue:%d\n" , __func__, red, green, blue);
	mutex_lock(&led_mutex);
#if 1
	clear_rgb_notification_on_charger(__func__);
#endif
	if (red)
		mode |= (3 << 4);
	if (green)
		mode |= (3 << 2);
	if (blue)
		mode |= 3;
	data = mode & 0x15;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (u8)0;
	ret = i2c_write_block(client, ENG_1_PC_CONTROL, &data, 1);
	udelay(200);
	ret = i2c_write_block(client, ENG_2_PC_CONTROL, &data, 1);
#ifdef LP5562_BLUE_LED
	udelay(200);
	ret = i2c_write_block(client, ENG_3_PC_CONTROL, &data, 1);
#endif

	if (red) {
		reg_index = 0;
		/* === set red pwm === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);

		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &red, 1);
		/* === wait 0.064s === */
		data = 0x44;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		/* === set pwm to 0 === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		/* === wait 0.935s === */
		data = 0x7c;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		/* === wait 0.999s === */
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	}
	if (green) {
		reg_index = 0;
#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
		// BLN
		// sysfs configuation bln_no_charger_switch == 1 -> always blink even if not on charger
		if (get_bln_switch() && ((smart_get_bln_no_charger_switch() && smart_get_button_dimming()==1)||charging)) {
			if (!wake_by_user || vk_screen_is_off()) {
				// store haptic blinking, so if ambient display blocks the bln, later in BLANK screen off, still it can be triggered
				bln_on_screenoff = 1;
				pr_info("%s kad bln_on_screenoff %d\n", __func__, bln_on_screenoff);
			}
			if (vk_screen_is_off() && should_start_bln()) {
				if (!is_kernel_ambient_display()) queue_work(g_vk_work_queue, &vk_blink_work);
			}
		}
		if ((vk_screen_is_off() || !wake_by_user) && (lights_down_divider==1)) { // only flash if lights_down mode is not active...
			flash_blink(false);
		}
		kernel_ambient_display();
		if (!get_bln_pulse_rgb_blink()) {
		green = green / (get_bln_rgb_blink_light_level()*smart_get_pulse_dimming());
#endif
		/* === set green pwm === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);

		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &green, 1);
		/* === wait 0.064s === */
		data = 0x44;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === set pwm to 0 === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === wait 0.935s === */
		data = 0x7c;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === wait 0.999s === */
		data = 0x7f;
		data = 0x6f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
		} else {
		I(" %s BLINK +++ red:%d, green:%d, blue:%d\n" , __func__, red, green, blue);
		/* # === set green blink === */
		if (get_bln_pulse_rgb_pattern()>RGB_PATTERN_ONEPLUS5) {
			reg_index = color_blink_step(client, pattern_brightness[get_bln_pulse_rgb_pattern()][0], pattern_time[get_bln_pulse_rgb_pattern()][0], reg_index);
			reg_index = color_blink_step(client, pattern_brightness[get_bln_pulse_rgb_pattern()][1], pattern_time[get_bln_pulse_rgb_pattern()][1], reg_index);
			reg_index = color_blink_step(client, pattern_brightness[get_bln_pulse_rgb_pattern()][2], pattern_time[get_bln_pulse_rgb_pattern()][2], reg_index);
			reg_index = color_blink_step(client, pattern_brightness[get_bln_pulse_rgb_pattern()][3], pattern_time[get_bln_pulse_rgb_pattern()][3], reg_index);
			reg_index = color_blink_step(client, pattern_brightness[get_bln_pulse_rgb_pattern()][4], pattern_time[get_bln_pulse_rgb_pattern()][4], reg_index);
			reg_index = color_blink_step(client, pattern_brightness[get_bln_pulse_rgb_pattern()][5], pattern_time[get_bln_pulse_rgb_pattern()][5], reg_index);
		} else {
			int step_time = get_bln_pulse_rgb_pattern() == RGB_PATTERN_ONEPLUS5 ? 7:3;
// set PWM start vaue to 5
			data = 0x40;
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
			data = 0x05;
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
// Ramp command will be: 0100 0010 0000 0100b
//= 4240h
			lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,1,112,dimming);
			data = ramp_data[0]; // prescale 01000000b + 0x08 step time
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
			data = ramp_data[1]; // increment 00000000b + 0x70 steps
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);

			lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,1,88,dimming);
			data = ramp_data[0];
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
			data = ramp_data[1]; // increment 00000000b + 0x58 steps
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
// Ramp down 42c0h

			lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,0,112,dimming);
			data = ramp_data[0]; // prescale 01000000b + 0x08 step time
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
			data = ramp_data[1]; // increment 00000000b + 0x70 steps
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);

			lp5562_get_ramp_program(&(ramp_data[0]),0,step_time,0,88,dimming);
			data = ramp_data[0];
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
			data = ramp_data[1]; // increment 00000000b + 0x58 steps
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		}
		data = bln_get_sleep_time(0);
		reg_index = color_blink_step(client, 0x00, data, reg_index);//0x7c, reg_index);
		/* === wait 0.999s === */
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		if (get_bln_speed() == 0) {
			/* === wait 0.999s === */
			data = 0x7f;
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
			data = 0x00;
			ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		}
		}
#endif
	}
#ifdef LP5562_BLUE_LED
	if (blue) {
		reg_index = 0;
		/* === set blue pwm === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);

		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &blue, 1);
		/* === wait 0.064s === */
		data = 0x44;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		/* === set pwm to 0 === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		/* === wait 0.935s === */
		data = 0x7c;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		/* === wait 0.999s === */
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_3_BASE + reg_index++, &data, 1);
	}
#endif
	/* === run program === */
	data = mode & 0x2a;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (mode & 0x2a)|0x40;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5562_dual_color_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret, reg_index = 0;

	I(" %s +++\n" , __func__);
	mutex_lock(&led_mutex);
#if 1
	clear_rgb_notification_on_charger(__func__);
#endif
	data = 0x14;
	ret = write_operation_register(client, data, 0);
	udelay(200);

	reg_index = 0;
	/* === set pwm to 200 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0xc8;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === wait 0.064s === */
	data = 0x44;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === set pwm to 0 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === wait 0.25s === */
	data = 0x50;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === trigger sg, wg === */
	data = 0xe1;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	data = 0x04;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	/* === clear register === */
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	ret = i2c_write_block(client, CMD_ENG_1_BASE + reg_index++, &data, 1);
	udelay(550);

	/* === trigger wr === */
	reg_index = 0;
	data = 0xe0;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x80;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	udelay(550);
	/* set pwm to 200 */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0xc8;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === wait 0.064s === */
	data = 0x44;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === set pwm to 0 === */
	data = 0x40;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === wait 0.999s === */
	data = 0x7f;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === wait 0.622s === */
	data = 0x68;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === trigger sr === */
	data = 0xe0;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	data = 0x02;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	/* === clear register === */
	data = 0x00;
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	udelay(550);

	/* === run program === */

	data = 0x28;
	ret = write_operation_register(client, data, 0);
	udelay(200);

	data = 0x68;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5562_green_blink (struct i2c_client *client,  uint8_t green)
{
	uint8_t data = 0x00;
	int ret, reg_index = 0;

	I(" %s +++ \n" , __func__);
	mutex_lock(&led_mutex);
#if 1
	clear_rgb_notification_on_charger(__func__);
#endif

	data = 0x04;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = (u8)0;
	ret = i2c_write_block(client, ENG_1_PC_CONTROL, &data, 1);
	udelay(200);
	ret = i2c_write_block(client, ENG_2_PC_CONTROL, &data, 1);
#ifdef LP5562_BLUE_LED
	udelay(200);
	ret = i2c_write_block(client, ENG_3_PC_CONTROL, &data, 1);
#endif
	if (green) {
		reg_index = 0;
		/* === set green pwm === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &green, 1);
		/* === wait 0.064s === */
		data = 0x44;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		/* === set pwm to 0 === */
		data = 0x40;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		// 1 sec
		data = 0x7f;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);

		/* === clear register === */
		data = 0x00;
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
		ret = i2c_write_block(client, CMD_ENG_2_BASE + reg_index++, &data, 1);
	}

	/* === run program === */
	data = 0x08;
	ret = write_operation_register(client, data, 0);
	udelay(200);
	data = 0x08|0x40;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5562_led_off(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;
	char data1[1] = {0};

	I(" %s +++\n" , __func__);
	if (!chip_enable) {
		I(" %s return, chip already disable\n" , __func__);
		return;
	}
	ret = i2c_read_block(client, ENABLE_REGISTER, data1, 1);
	if (!data1[0]) {
		I(" %s return, chip already disable\n" , __func__);
		return;
	}

	mutex_lock(&led_mutex);
#if 1
	clear_rgb_notification_on_charger(__func__);
#endif
	/* === reset red green blue === */
	data = 0x00;
	ret = i2c_write_block(client, R_PWM_CONTROL, &data, 1);
	ret = i2c_write_block(client, G_PWM_CONTROL, &data, 1);
#ifdef LP5562_BLUE_LED
	ret = i2c_write_block(client, B_PWM_CONTROL, &data, 1);
#endif
	ret = write_operation_register(client, data, 0);
	ret = write_enable_register(client, data, 0);
	mutex_unlock(&led_mutex);

#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
	// BLN
	blink_running = 0;
	if (vk_present) {
		virtual_key_led_blink(0,0);
	}
	flash_stop_blink();
#endif

	if(!rgb_enable && !vk_enable)
		lp5562_led_disable(client);

	I(" %s ---\n" , __func__);
}

#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
static void green_blink_mfg(int onoff){
	I(" %s , set display_flag = %d\n" , __func__, onoff);
	display_flag = onoff;
	if(display_flag){
		spin_lock(&led_data_lock);
		g_led_led_data->Mode = 2;
		g_led_led_data->Red = 0;
		g_led_led_data->Green = 0xc8;
		g_led_led_data->Blue = 0;
		queue_work(g_led_work_queue, &g_led_led_data->led_work_multicolor);
		spin_unlock(&led_data_lock);
	}else {
		spin_lock(&led_data_lock);
		g_led_led_data->Mode = 0;
		g_led_led_data->Red = 0;
		g_led_led_data->Green = 0;
		g_led_led_data->Blue = 0;
		queue_work(g_led_work_queue, &g_led_led_data->led_work_multicolor);
		spin_unlock(&led_data_lock);
	}
}
#endif

static void led_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;

	I(" %s +++\n" , __func__);
	ldata = container_of(work, struct lp5562_led, led_work);
	lp5562_led_off(client);
	I(" %s ---\n" , __func__);
}

static void multicolor_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;
	int ret;
	uint8_t data = 0x00;
        char data1[1] = {0};
        int i;

	if(!client)
		return;

	ldata = container_of(work, struct lp5562_led, led_work_multicolor);
	I(" %s , Mode = %x\n" , __func__, ldata->Mode);

//	if (ldata->Mode > 1 && ldata->Mode <= 6) // TODO check this
	if (ldata->Mode > 1 && ldata->Mode <= 5)
		lp5562_led_enable(client, 1);
	else if (ldata->Mode == 1)
		lp5562_led_enable(client, 0);
	if (ldata->Red) {
		rgb_enable = 1;
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, R_CURRENT_CONTROL, &data, 1);
	} else {
		data = (u8)0;
		ret = i2c_write_block(client, R_CURRENT_CONTROL, &data, 1);
	}
	if (ldata->Green) {
		rgb_enable = 1;
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, G_CURRENT_CONTROL, &data, 1);
	} else {
		data = (u8)0;
		ret = i2c_write_block(client, G_CURRENT_CONTROL, &data, 1);
	}
#ifdef LP5562_BLUE_LED
	if (ldata->Blue) {
		rgb_enable = 1;
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
	} else {
		data = (u8)0;
		ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
	}
#endif
	if (ldata->Mode == 0 || (!ldata->Red && !ldata->Green && !ldata->Blue)) {
		rgb_enable = 0;
		lp5562_led_off(client);
	} else if (ldata->Mode == 1) {  /* === set red, green, blue direct control === */
		mutex_lock(&led_mutex);
#if 1
		clear_rgb_notification_on_charger(__func__);
#endif
		ret = i2c_write_block(client, R_PWM_CONTROL, &ldata->Red, 1);
		ret = i2c_write_block(client, G_PWM_CONTROL, &ldata->Green, 1);
#ifdef LP5562_BLUE_LED
		ret = i2c_write_block(client, B_PWM_CONTROL, &ldata->Blue, 1);
#endif
		data = 0x3f;
		ret = write_operation_register(client, data, 0);
		udelay(200);
		data = 0x40;
		ret = write_enable_register(client, data, 0);
		udelay(500);
		mutex_unlock(&led_mutex);
	} else if (ldata->Mode == 2) { /* === set short blink === */
		lp5562_color_blink(client, ldata->Red, ldata->Green, ldata->Blue);
	} else if (ldata->Mode == 3) { /* === set delayed short blink === */
		msleep(1000);
		lp5562_color_blink(client, ldata->Red, ldata->Green, ldata->Blue);
	} else if (ldata->Mode == 4 && ldata->Red && !ldata->Green && !ldata->Blue) { /* === set red long blink === */
		lp5562_red_long_blink(client);
	} else if (ldata->Mode ==5 && ldata->Red && ldata->Green && !ldata->Blue) { /* === set red green blink === */
		lp5562_dual_color_blink(client);
	} else if (ldata->Mode == 6) {
		lp5562_green_blink(client, ldata->Green);
	} else {
		for (i = 0; i <= 0x6f; i++) {
			ret = i2c_read_block(client, i, data1, 1);
			I(" %s i2c(%x) = 0x%x\n", __func__, i, data1[0]);
		}
	}
}
/*
static void led_alarm_handler(struct alarm *alarm)
{
	struct lp5562_led *ldata;

	I(" %s +++\n" , __func__);
	ldata = container_of(alarm, struct lp5562_led, led_alarm);
	queue_work(g_led_work_queue, &ldata->led_work);
	I(" %s ---\n" , __func__);
}
*/
static void led_blink_do_work(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;

	I(" %s +++\n" , __func__);
	ldata = container_of(work, struct lp5562_led, blink_delayed_work.work);
	lp5562_color_blink(client, ldata->Red, ldata->Green, ldata->Blue);
	I(" %s ---\n" , __func__);
}

static int virtual_key_led_change_pwm(struct i2c_client *client, int pwm_diff)
{
	int target_pwm_diff, ret = 0;
	uint8_t pc_addr1, pc_addr2, pwm_level, data;

	I("%s: pwm change. pwm diff %d \n", __func__, pwm_diff);

	if(pwm_diff > 0){
		target_pwm_diff = pwm_diff;
		pc_addr1 = 0;
		pc_addr2 = 0;
	}
	else if (pwm_diff < 0){
		target_pwm_diff = -pwm_diff;
		pc_addr1 = 8;
		pc_addr2 = 8;
	}
	else {
		I("%s: pwm no changed, return.\n", __func__);
		return ret;
	}
	pwm_level = target_pwm_diff / VK_LED_FADE_LEVEL;
	if(pwm_level / 2) {
		pc_addr2 += (pwm_level / 2) - 1;
		pc_addr1 = pc_addr2 + (pwm_level % 2);
	}

	ret = i2c_write_block(client, ENG_3_PC_CONTROL, &pc_addr1, 1);
	data = 0x43;
	ret = write_enable_register(client, data, 1);
	if(pwm_level / 2) {
		msleep(VK_LED_SLEEP_TIME);
		ret = i2c_write_block(client, ENG_3_PC_CONTROL, &pc_addr2, 1);
		data = 0x43;
		ret = write_enable_register(client, data, 1);
	}
	msleep(VK_LED_SLEEP_TIME);

	return ret;
}

void virtual_key_led_reset_blink(int onoff)
{
	struct i2c_client *client = private_lp5562_client;
	int ret = 0, reg_index = 0;
	int target_pwm;
	uint8_t data;
	uint8_t command_data[10] = {0};

	if(!client)
		return;

	if (!plat_data->vk_use) {
		I("No virtual key led used\n");
		return;
	}

	I("virtual_key_led_reset_blink +++, onoff = %d\n", onoff);

	if(onoff) {
		virtual_key_led_ignore_flag = 1;
		lp5562_led_enable(client, 0);

		data = 0;
		ret = i2c_write_block(client, ENG_3_PC_CONTROL, &data, 1);

		data = 0x01;
		ret = write_operation_register(client, data, 1);

		/*=== Set PWM to 0 ===*/
		command_data[reg_index++] = 0x40;
		command_data[reg_index++]  = 0x00;

		/*=== wait for 300 ms ===*/
		command_data[reg_index++] = 0x54;
		command_data[reg_index++]  = 0x00;

		/*=== Set PWM to last_pwm ===*/
		command_data[reg_index++] = 0x40;
		command_data[reg_index++]  = 0xFF;

		/*=== wait for 300 ms ===*/
		command_data[reg_index++] = 0x54;
		command_data[reg_index++]  = 0x00;

		/* === clear register === */
		command_data[reg_index++] = 0x00;
		command_data[reg_index++]  = 0x00;

		ret = i2c_write_block(client, CMD_ENG_3_BASE, command_data, 10);

		data = 0x02;
		ret = write_operation_register(client, data, 1);
		data = 0x42;
		ret = write_enable_register(client, data, 1);
	} else {
		virtual_key_led_ignore_flag = 0;
		data = 0x40;
		ret = write_enable_register(client, data, 1);
		write_vk_led_program(client);

		mutex_lock(&vk_led_mutex);
		target_pwm = VK_brightness / VK_LED_FADE_LEVEL * VK_LED_FADE_LEVEL;
		ret = virtual_key_led_change_pwm(client, (int)target_pwm);
		last_pwm = target_pwm;
		if(last_pwm)
			vk_enable = 1;
		else
			vk_enable = 0;
		mutex_unlock(&vk_led_mutex);

		if(!rgb_enable && !vk_enable)
			lp5562_led_disable(client);
	}
}

EXPORT_SYMBOL(virtual_key_led_reset_blink);

static void virtual_key_led_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;
	int ret, reg_index = 0;
	int target_pwm;
	uint8_t data, change_current = 0;

	if(!client)
		return;

	ldata = container_of(work, struct lp5562_led, led_work);
	I("%s: led work func. vk brightness: %d \n", __func__, ldata->VK_brightness);

	if(ldata->VK_brightness) {
		if(use_color_table && ldata->VK_brightness < table_level_num){
			if(use_current_table) {
				if(gVK_Current_param != current_table[ldata->VK_brightness] && ldata->VK_brightness != 0) {
					gVK_Current_param = current_table[ldata->VK_brightness];
					change_current = 1;
				}
			}
			ldata->VK_brightness = VK_brightness;
		}
		target_pwm = ldata->VK_brightness / VK_LED_FADE_LEVEL * VK_LED_FADE_LEVEL;
		if (vk_enable) {
		I(" %s virtual key already enable, change brightness\n" , __func__);

		if(change_current) {
			data = (u8)gVK_Current_param;
			ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
			ret = i2c_write_block(client, W_CURRENT_CONTROL, &data, 1);
		}
		mutex_lock(&vk_led_mutex);
		ret = virtual_key_led_change_pwm(client, (int)(target_pwm - last_pwm));
		last_pwm = target_pwm;
		mutex_unlock(&vk_led_mutex);
		return;
		}
		lp5562_led_enable(client, 1);
		data = (u8)gVK_Current_param;
		ret = i2c_write_block(client, B_CURRENT_CONTROL, &data, 1);
		ret = i2c_write_block(client, W_CURRENT_CONTROL, &data, 1);
		write_vk_led_program(client);
		vk_enable = 1;
		reg_index = 0;

		mutex_lock(&vk_led_mutex);
		ret = virtual_key_led_change_pwm(client, (int)target_pwm);
		last_pwm = target_pwm;
		mutex_unlock(&vk_led_mutex);
	}else {
		if (!vk_enable) {
			I(" %s return, virtual key already disable\n" , __func__);
			return;
		}
		mutex_lock(&vk_led_mutex);
		I("%s: led work func pwm diff %d \n", __func__, -last_pwm);
		ret = virtual_key_led_change_pwm(client, -last_pwm);
		last_pwm = 0;
		mutex_unlock(&vk_led_mutex);
		queue_delayed_work(g_led_work_queue, &ldata->blink_delayed_work, msecs_to_jiffies(VK_LED_SLEEP_TIME));
	}
}

static void led_fade_do_work(struct work_struct *work)
{
	struct i2c_client *client = private_lp5562_client;
	struct lp5562_led *ldata;

	ldata = container_of((struct delayed_work *)work, struct lp5562_led, blink_delayed_work);

	if(!ldata->VK_brightness) {
		vk_enable = 0;
		if(!rgb_enable && !vk_enable)
			lp5562_led_disable(client);
	}
}


#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK

// flash blink settings
extern void set_flash_blink_on(int value);
extern int get_flash_blink_on(void);
extern void set_flash_only_face_down(int value);
extern int get_flash_only_face_down(void);
extern void set_flash_blink_number(int value);
extern int get_flash_blink_number(void);

extern void set_flash_blink_bright(int value);
extern int get_flash_blink_bright(void);
extern void set_flash_blink_bright_number(int value);
extern int get_flash_blink_bright_number(void);

extern void set_flash_blink_wait_sec(int value);
extern int get_flash_blink_wait_sec(void);

extern void set_flash_blink_wait_inc(int value);
extern int get_flash_blink_wait_inc(void);
extern void set_flash_blink_wait_inc_max(int value);
extern int get_flash_blink_wait_inc_max(void);

extern void set_flash_haptic_mode(int value);
extern int get_flash_haptic_mode(void);
extern void set_flash_dim_mode(int value);
extern int get_flash_dim_mode(void);
extern void set_flash_dim_use_period(int value);
extern int get_flash_dim_use_period(void);
extern void set_flash_dim_period_hours(int startHour, int endHour);
extern int get_flash_dim_period_hours(int *startEndHours);


static ssize_t flash_dim_period_end_hour_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
	int r[2];
	get_flash_dim_period_hours(&r[0]);
      return snprintf(buf, PAGE_SIZE, "%d\n", r[1]);
}

static ssize_t flash_dim_period_end_hour_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;
	int r[2];
	get_flash_dim_period_hours(&r[0]);

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input <= 0 || input > 23)
            input = 6;
	r[1] = input;
	set_flash_dim_period_hours(r[0],r[1]);

      return count;
}
static DEVICE_ATTR(bln_flash_dim_period_end_hour, (S_IWUSR|S_IRUGO),
      flash_dim_period_end_hour_show, flash_dim_period_end_hour_dump);



static ssize_t flash_dim_period_start_hour_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
	int r[2];
	get_flash_dim_period_hours(&r[0]);
      return snprintf(buf, PAGE_SIZE, "%d\n", r[0]);
}

static ssize_t flash_dim_period_start_hour_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;
	int r[2];
	get_flash_dim_period_hours(&r[0]);

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input <= 0 || input > 23)
            input = 22;
	r[0] = input;
	set_flash_dim_period_hours(r[0],r[1]);

      return count;
}
static DEVICE_ATTR(bln_flash_dim_period_start_hour, (S_IWUSR|S_IRUGO),
      flash_dim_period_start_hour_show, flash_dim_period_start_hour_dump);


static ssize_t flash_dim_use_period_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_dim_use_period());
}

static ssize_t flash_dim_use_period_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 1; // 0 - use, 1 - not use period

	set_flash_dim_use_period(input);

      return count;
}
static DEVICE_ATTR(bln_flash_dim_use_period, (S_IWUSR|S_IRUGO),
      flash_dim_use_period_show, flash_dim_use_period_dump);


static ssize_t flash_dim_mode_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_dim_mode());
}

static ssize_t flash_dim_mode_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 2)
            input = 1; // 0 no dim mode, 1 half the brightness, 2 fully blanked dim

	set_flash_dim_mode(input);

      return count;
}
static DEVICE_ATTR(bln_flash_dim_mode, (S_IWUSR|S_IRUGO),
      flash_dim_mode_show, flash_dim_mode_dump);


static ssize_t flash_haptic_mode_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_haptic_mode());
}

static ssize_t flash_haptic_mode_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 1;

	set_flash_haptic_mode(input);

      return count;
}
static DEVICE_ATTR(bln_flash_haptic_mode, (S_IWUSR|S_IRUGO),
      flash_haptic_mode_show, flash_haptic_mode_dump);


static ssize_t flash_blink_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_blink_on());
}

static ssize_t flash_blink_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 1;

	set_flash_blink_on(input);

      return count;
}

static DEVICE_ATTR(bln_flash_blink, (S_IWUSR|S_IRUGO),
      flash_blink_show, flash_blink_dump);

static ssize_t flash_only_face_down_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_only_face_down());
}

static ssize_t flash_only_face_down_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 1;

	set_flash_only_face_down(input);

      return count;
}

static DEVICE_ATTR(bln_flash_only_face_down, (S_IWUSR|S_IRUGO),
      flash_only_face_down_show, flash_only_face_down_dump);

static ssize_t flash_blink_bright_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_blink_bright());
}

static ssize_t flash_blink_bright_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 1;

	set_flash_blink_bright(input);

      return count;
}

static DEVICE_ATTR(bln_flash_blink_bright, (S_IWUSR|S_IRUGO),
      flash_blink_bright_show, flash_blink_bright_dump);

static ssize_t flash_blink_bright_number_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_blink_bright_number());
}

static ssize_t flash_blink_bright_number_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 1 || input > 10)
            input = 5;

      set_flash_blink_bright_number(input);

      return count;
}

static DEVICE_ATTR(bln_flash_blink_bright_number, (S_IWUSR|S_IRUGO),
      flash_blink_bright_number_show, flash_blink_bright_number_dump);

static ssize_t flash_blink_number_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_blink_number());
}

static ssize_t flash_blink_number_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > BUTTON_BLINK_NUMBER_MAX)
            input = BUTTON_BLINK_NUMBER_DEFAULT;

      set_flash_blink_number(input);

      return count;
}

static DEVICE_ATTR(bln_flash_blink_number, (S_IWUSR|S_IRUGO),
      flash_blink_number_show, flash_blink_number_dump);


static ssize_t flash_blink_wait_inc_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_blink_wait_inc());
}

static ssize_t flash_blink_wait_inc_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 1;

	set_flash_blink_wait_inc(input);

      return count;
}

static DEVICE_ATTR(bln_flash_blink_wait_inc, (S_IWUSR|S_IRUGO),
      flash_blink_wait_inc_show, flash_blink_wait_inc_dump);


static ssize_t flash_blink_wait_inc_max_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_blink_wait_inc_max());
}

static ssize_t flash_blink_wait_inc_max_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 1 || input > 8)
            input = 4;

      set_flash_blink_wait_inc_max(input);

      return count;
}

static DEVICE_ATTR(bln_flash_blink_wait_inc_max, (S_IWUSR|S_IRUGO),
      flash_blink_wait_inc_max_show, flash_blink_wait_inc_max_dump);

static ssize_t flash_blink_wait_sec_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_flash_blink_wait_sec());
}

static ssize_t flash_blink_wait_sec_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 1 || input > 10)
            input = 2;

      set_flash_blink_wait_sec(input);

      return count;
}

static DEVICE_ATTR(bln_flash_blink_wait_sec, (S_IWUSR|S_IRUGO),
      flash_blink_wait_sec_show, flash_blink_wait_sec_dump);


// bln on/off settings
static ssize_t bln_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", bln_switch);
}

static ssize_t bln_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 0;

      bln_switch = input;

      return count;
}

static DEVICE_ATTR(bln, (S_IWUSR|S_IRUGO),
      bln_show, bln_dump);

// bln no charger on/off settings
static ssize_t bln_no_charger_switch_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", bln_no_charger_switch);
}

static ssize_t bln_no_charger_switch_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 0;

      bln_no_charger_switch = input;

      return count;
}

static DEVICE_ATTR(bln_no_charger, (S_IWUSR|S_IRUGO),
      bln_no_charger_switch_show, bln_no_charger_switch_dump);

// pulse on/off settings
static ssize_t bln_pulse_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", pulse_rgb_blink);
}

static ssize_t bln_pulse_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 0;

      pulse_rgb_blink = input;

      return count;
}

static DEVICE_ATTR(bln_rgb_pulse, (S_IWUSR|S_IRUGO),
      bln_pulse_show, bln_pulse_dump);

// charge color combination on/off
static ssize_t bln_rgb_colored_battery_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", colored_charge_level);
}

static ssize_t bln_rgb_colored_battery_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 1;

	colored_charge_level = input;

      return count;
}

static DEVICE_ATTR(bln_rgb_batt_colored, (S_IWUSR|S_IRUGO),
      bln_rgb_colored_battery_show, bln_rgb_colored_battery_dump);

// pulse_rgb_blink_on_charger
static ssize_t bln_rgb_blink_on_charger_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", pulse_rgb_blink_on_charger);
}

static ssize_t bln_rgb_blink_on_charger_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 0;

      pulse_rgb_blink_on_charger = input;

      return count;
}

static DEVICE_ATTR(bln_rgb_blink_on_charger, (S_IWUSR|S_IRUGO),
      bln_rgb_blink_on_charger_show, bln_rgb_blink_on_charger_dump);

// pulse_rgb_blink_on_charger_red_limit
static ssize_t bln_rgb_blink_on_charger_red_limit_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", pulse_rgb_blink_on_charger_red_limit);
}

static ssize_t bln_rgb_blink_on_charger_red_limit_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 100)
            input = 95;

      pulse_rgb_blink_on_charger_red_limit = input;

      return count;
}

static DEVICE_ATTR(bln_rgb_blink_on_charger_red_limit, (S_IWUSR|S_IRUGO),
      bln_rgb_blink_on_charger_red_limit_show, bln_rgb_blink_on_charger_red_limit_dump);


static ssize_t bln_number_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", bln_number);
}

static ssize_t bln_number_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;
            
      if (input < 0 || input > BUTTON_BLINK_NUMBER_MAX)
            input = BUTTON_BLINK_NUMBER_DEFAULT;
            
      bln_number = input;
      
      return count;
}

static DEVICE_ATTR(bln_number, (S_IWUSR|S_IRUGO),
      bln_number_show, bln_number_dump);


static ssize_t bln_number_max_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", BUTTON_BLINK_NUMBER_MAX);
}

static ssize_t bln_number_max_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;
      
      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;
            
      return count;
}

static DEVICE_ATTR(bln_number_max, (S_IWUSR|S_IRUGO),
      bln_number_max_show, bln_number_max_dump);


static ssize_t bln_speed_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", bln_speed);
}


static ssize_t bln_speed_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > BUTTON_BLINK_SPEED_MAX)
            input = BUTTON_BLINK_SPEED_DEFAULT;

      bln_speed = input;

      return count;
}

static DEVICE_ATTR(bln_speed, (S_IWUSR|S_IRUGO),
      bln_speed_show, bln_speed_dump);

static ssize_t bln_speed_max_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", BUTTON_BLINK_SPEED_MAX);
}

static ssize_t bln_speed_max_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > BUTTON_BLINK_SPEED_MAX)
            input = 8;

      return count;
}

static DEVICE_ATTR(bln_speed_max, (S_IWUSR|S_IRUGO),
      bln_speed_max_show, bln_speed_max_dump);

// coeff divider for notification blinking
static ssize_t bln_coeff_div_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", (get_bln_rgb_blink_light_level()<20?(get_bln_rgb_blink_light_level()-1):20));
}

static ssize_t bln_coeff_div_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 20)
            input = 0;

      if (input < 20) {
	     rgb_coeff_divider = input + 1;
      } else rgb_coeff_divider = 500;

      return count;
}

static DEVICE_ATTR(bln_rgb_blink_light_level, (S_IWUSR|S_IRUGO),
      bln_coeff_div_show, bln_coeff_div_dump);


// coeff divider for button blinking
static ssize_t bln_coeff2_div_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", (bln_coeff_divider-1));
}

static ssize_t bln_coeff2_div_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 20)
            input = 0;

      bln_coeff_divider = input + 1;

      return count;
}

static DEVICE_ATTR(bln_light_level, (S_IWUSR|S_IRUGO),
      bln_coeff2_div_show, bln_coeff2_div_dump);

// dim blink switch sysfs
static ssize_t bln_dim_blink_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", bln_dim_blink);
}

static ssize_t bln_dim_blink_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 0;

      bln_dim_blink = input;

      return count;
}

static DEVICE_ATTR(bln_dim_blink, (S_IWUSR|S_IRUGO),
      bln_dim_blink_show, bln_dim_blink_dump);


// dim blink number switch sysfs
static ssize_t bln_dim_number_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", bln_dim_number);
}
      
static ssize_t bln_dim_number_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > BUTTON_BLINK_NUMBER_MAX)
            input = 0;

      bln_dim_number = input;

      return count;
}

static DEVICE_ATTR(bln_dim_number, (S_IWUSR|S_IRUGO),
      bln_dim_number_show, bln_dim_number_dump);

static ssize_t bln_dim_number_max_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", BUTTON_BLINK_NUMBER_MAX);
}

static ssize_t bln_dim_number_max_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      return count;
}

static DEVICE_ATTR(bln_dim_number_max, (S_IWUSR|S_IRUGO),
      bln_dim_number_max_show, bln_dim_number_max_dump);



// rgb pulse pattern switch sysfs
static ssize_t bln_pulse_rgb_pattern_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", pulse_rgb_pattern);
}
      
static ssize_t bln_pulse_rgb_pattern_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > RGB_PATTERN_TRIPLE_DOWN)
            input = 0;

      pulse_rgb_pattern = input;

      return count;
}

static DEVICE_ATTR(bln_pulse_rgb_pattern, (S_IWUSR|S_IRUGO),
      bln_pulse_rgb_pattern_show, bln_pulse_rgb_pattern_dump);

static ssize_t bln_pulse_rgb_pattern_max_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", RGB_PATTERN_TRIPLE_DOWN);
}

static ssize_t bln_pulse_rgb_pattern_max_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      return count;
}

static DEVICE_ATTR(bln_pulse_rgb_pattern_max, (S_IWUSR|S_IRUGO),
      bln_pulse_rgb_pattern_max_show, bln_pulse_rgb_pattern_max_dump);

// --- vib notification reminder
static ssize_t vib_notification_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_vib_notification_reminder());
}

static ssize_t vib_notification_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 0 || input > 1)
            input = 1;

	set_vib_notification_reminder(input);

      return count;
}

static DEVICE_ATTR(bln_vib_notification, (S_IWUSR|S_IRUGO),
      vib_notification_show, vib_notification_dump);


static ssize_t vib_notification_slowness_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_vib_notification_slowness());
}

static ssize_t vib_notification_slowness_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 5 || input > 30)
            input = 15;

      set_vib_notification_slowness(input);

      return count;
}

static DEVICE_ATTR(bln_vib_notification_slowness, (S_IWUSR|S_IRUGO),
      vib_notification_slowness_show, vib_notification_slowness_dump);


static ssize_t vib_notification_length_show(struct device *dev,
            struct device_attribute *attr, char *buf)
{
      return snprintf(buf, PAGE_SIZE, "%d\n", get_vib_notification_length());
}

static ssize_t vib_notification_length_dump(struct device *dev,
            struct device_attribute *attr, const char *buf, size_t count)
{
      int ret;
      unsigned long input;

      ret = kstrtoul(buf, 0, &input);
      if (ret < 0)
            return ret;

      if (input < 1 || input > 500)
            input = 300;

	set_vib_notification_length(input);

      return count;
}

static DEVICE_ATTR(bln_vib_notification_length, (S_IWUSR|S_IRUGO),
      vib_notification_length_show, vib_notification_length_dump);


#endif

static ssize_t lp5562_charging_led_switch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", charging_flag);
}

static ssize_t lp5562_charging_led_switch_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = private_lp5562_client;
	uint8_t data = 0x00;
	int flag,ret;
	sscanf(buf, "%d", &flag);
	if(flag == 1 && flag != charging_flag) {
		charging_flag=flag;
		mutex_lock(&led_mutex);
#if 1
		clear_rgb_notification_on_charger(__func__);
#endif
		data = 0x00;
		ret = i2c_write_block(client, R_PWM_CONTROL, &data, 1);
		ret = i2c_write_block(client, G_PWM_CONTROL, &data, 1);
#ifdef LP5562_BLUE_LED
		ret = i2c_write_block(client, B_PWM_CONTROL, &data, 1);
#endif
		ret = write_operation_register(client, data, 0);
		ret = write_enable_register(client, data, 0);
		ret = gpio_direction_output(plat_data->charging_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(plat_data->charging_gpio);
			return ret;
		}
	}
	else if(flag == 0 && flag != charging_flag) {
		charging_flag=flag;
		ret = gpio_direction_output(plat_data->charging_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(plat_data->charging_gpio);
			return ret;
		}
		mutex_unlock(&led_mutex);
	}
	return count;
}

static DEVICE_ATTR(charging_led_switch, 0640, lp5562_charging_led_switch_show,
		lp5562_charging_led_switch_store);


static ssize_t lp5562_led_off_timer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", current_time);
}

static ssize_t lp5562_led_off_timer_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct lp5562_led *ldata;
	int min, sec;
	uint16_t off_timer;
/*	ktime_t interval;
	ktime_t next_alarm;*/

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);
	I(" %s , min = %d, sec = %d\n" , __func__, min, sec);
	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct lp5562_led, cdev);

	off_timer = min * 60 + sec;
	cancel_work_sync(&ldata->led_work);
	/*alarm_cancel(&ldata->led_alarm);
	if (off_timer) {
		interval = ktime_set(off_timer, 0);
		next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
		alarm_start_range(&ldata->led_alarm, next_alarm, next_alarm);
	}*/

	return count;
}

static DEVICE_ATTR(off_timer, 0644, lp5562_led_off_timer_show,
		lp5562_led_off_timer_store);

static ssize_t lp5562_led_multi_color_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", ModeRGB);
}

static ssize_t lp5562_led_multi_color_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct lp5562_led *ldata;
	uint32_t val;
#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
	if(display_flag) {
		I(" %s , display_flag = %d, return\n" , __func__, display_flag);
		return count;
	}
#endif
	sscanf(buf, "%x", &val);

	if (val > 0xFFFFFFFF)
		return -EINVAL;
	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct lp5562_led, cdev);
	wake_lock_timeout(&(ldata->led_wake_lock), 2*HZ);

	spin_lock(&led_data_lock);

	ldata->Mode = (val & Mode_Mask) >> 24;
	ldata->Red = (val & Red_Mask) >> 16;
	ldata->Green = (val & Green_Mask) >> 8;
#ifdef LP5562_BLUE_LED
	ldata->Blue = val & Blue_Mask;
#endif
	ModeRGB = val;
	I(" %s , ModeRGB = %x\n" , __func__, val);
#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
	supposedly_charging = ldata->Mode == 1 && (ldata->Red > 0 || ldata->Green > 0);
	I(" %s , RED = %d supposedly charging %d charging %d\n" , __func__, ldata->Red, supposedly_charging, charging);

	if (get_bln_rgb_batt_colored() && supposedly_charging && first_level_registered) {
		// if it's supposedly charging and first level registered from HTC battery, we can go and set charge level color mix instead of normal multicolor setting later...
		spin_unlock(&led_data_lock);
		led_multi_color_charge_level(charge_level, false);
		// and return so color is not overwritten...
		return count;
	}
	if (supposedly_charging) {
		ldata->Red = ((val/(rgb_coeff_divider*smart_get_pulse_dimming())) & Red_Mask) >> 16;
		ldata->Green = ((val/(rgb_coeff_divider*smart_get_pulse_dimming())) & Green_Mask) >> 8;
	}
#endif
	queue_work(g_led_work_queue, &ldata->led_work_multicolor);
	spin_unlock(&led_data_lock);
	return count;
}

static DEVICE_ATTR(ModeRGB, 0644, lp5562_led_multi_color_show,
		lp5562_led_multi_color_store);

enum led_brightness lp5562_led_get_brightness(struct led_classdev *led_cdev)
{
	return ModeRGB;
}

static void lp5562_led_set_brightness(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	return;
}

enum led_brightness lp5562_vk_led_get_brightness(struct led_classdev *led_cdev)
{
	return VK_brightness;
}

static void lp5562_vk_led_set_brightness(struct led_classdev *led_cdev,
					  enum led_brightness brightness)
{
	struct lp5562_led *ldata;
#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
	int uci_bln_light_level = get_bln_light_level();
	int divider = (uci_bln_light_level)*smart_get_button_dimming()>8?4:(uci_bln_light_level<3?1:(uci_bln_light_level/2));
#endif
	ldata = container_of(led_cdev, struct lp5562_led, cdev);
	ldata->VK_brightness = brightness == LED_FULL? 256 : brightness;

	if(use_color_table && brightness < table_level_num){
		I("color_table[%d] = %d, current_table[%d] = %d\n", brightness, color_table[brightness], brightness, current_table[brightness]);
		brightness = color_table[brightness];
	}

	VK_brightness = brightness == LED_FULL? 256 : brightness;
	I(" %s , VK_brightness = %u\n" , __func__, VK_brightness);
#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
	if (ldata->VK_brightness > 0) {
		ldata->VK_brightness = ldata->VK_brightness / divider;
		if (ldata->VK_brightness == 0) ldata->VK_brightness = 1;
	}
	if (VK_brightness > 0) {
		VK_brightness = VK_brightness / divider;
		if (VK_brightness < 16) VK_brightness = 16;
	}
	// if divider set to max, turn off button lights permanently
	if (uci_bln_light_level==21) {
		ldata->VK_brightness = 0;
		VK_brightness = 0;
		if (vk_present) {
			if (vk_led_blink) {
				virtual_key_led_blink(0,0);
				return;
			}
		}
	}
	I(" %s , VK_brightness after bln coeff division = %u\n" , __func__, VK_brightness);
#endif

	if(!virtual_key_led_ignore_flag)
		queue_work(g_led_work_queue, &ldata->led_work);
#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
	vk_led_blink = 0;
#endif
	return;
}

/* === read/write i2c and control enable pin for debug === */
static ssize_t lp5562_led_i2c_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	char data[1] = {0};
	int i;
	struct i2c_client *client = private_lp5562_client;

	if(!client)
		return 0;

	for (i = 0; i <= 0x6f; i++) {
		ret = i2c_read_block(client, i, data, 1);
		I(" %s i2c(%x) = 0x%x\n", __func__, i, data[0]);
	}
	return ret;
}

static ssize_t lp5562_led_i2c_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = private_lp5562_client;
	int i, ret;
	char *token[10];
	unsigned long ul_reg, ul_data = 0;
	uint8_t reg = 0, data;
	char value[1] = {0};
	struct led_i2c_platform_data *pdata;

	if(!client)
		return count;

	pdata = client->dev.platform_data;

	for (i = 0; i < 2; i++) {
		token[i] = strsep((char **)&buf, " ");
		D("%s: token[%d] = %s\n", __func__, i, token[i]);
	}
	ret = kstrtoul(token[0], 16, &ul_reg);
	ret = kstrtoul(token[1], 16, &ul_data);

	reg = ul_reg;
	data = ul_data;

	if (reg < 0x6F) {
		ret = i2c_write_block(client, reg, &data, 1);
		ret = i2c_read_block(client, reg, value, 1);
		I(" %s , ret = %d, Set REG=0x%x, data=0x%x\n" , __func__, ret, reg, data);
		ret = i2c_read_block(client, reg, value, 1);
		I(" %s , ret = %d, Get REG=0x%x, data=0x%x\n" , __func__, ret, reg, value[0]);
	}
	if (reg == 0x99) {
		if (data == 1) {
			I("%s , pull up enable pin\n", __func__);
			if (pdata->ena_gpio) {
				ret = gpio_direction_output(pdata->ena_gpio, 1);
				if (ret < 0) {
					pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
					gpio_free(pdata->ena_gpio);
				}
			}
		} else if (data == 0) {
			I("%s , pull down enable pin\n", __func__);
			if (pdata->ena_gpio) {
				ret = gpio_direction_output(pdata->ena_gpio, 1);
				if (ret < 0) {
					pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
					gpio_free(pdata->ena_gpio);
				}
			}
		}
	}
	return count;
}

static DEVICE_ATTR(i2c, 0644, lp5562_led_i2c_show, lp5562_led_i2c_store);


static int lp5562_pinctrl_init(struct lp5562_chip *cdata, struct device *dev){

	int retval, ret;

	/* Get pinctrl if target uses pinctrl */
	D("[LED]LP5562_pinctrl_init");

	cdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(cdata->pinctrl)) {
		retval = PTR_ERR(cdata->pinctrl);
		pr_err("[LED][lp5562 error]%s: Target does not use pinctrl\n", __func__);
		cdata->pinctrl = NULL;
		goto err_pinctrl_get;
	}
	cdata->gpio_state_init = pinctrl_lookup_state(cdata->pinctrl, "lp5562_init");
	if (IS_ERR_OR_NULL(cdata->gpio_state_init)) {
		pr_err("[LED][lp5562 error]%s: Cannot get pintctrl state\n", __func__);
		retval = PTR_ERR(cdata->gpio_state_init);
		cdata->pinctrl = NULL;
		return retval;
	}
	ret = pinctrl_select_state(cdata->pinctrl, cdata->gpio_state_init);
	if (ret) {
		pr_err("[LED][LP5562 error]%s: Cannot init INT gpio\n", __func__);
		return ret;
	}

	return 0;

err_pinctrl_get:
	cdata->pinctrl = NULL;
	return retval;
}

#define CG_ID_LEN 5
#define BLACK_ID 1
#define WHITE_ID 2

static void get_brightness_mapping_table(struct device_node *node)
{
	struct property *prop;
	int current_table_level_num;
	int color_ID = BLACK_ID;
	int touch_solution = g_led_touch_solution;
	const char* cmdline;
	char* temp_cmdline;

	prop = of_find_property(node, "vk-pwm-array",
			&table_level_num);
	if(!prop) {
		I("Not use color mapping table\n");
		return;
	}
    I("%s, vk-pwm-array table_level_num: %d\n", __func__, table_level_num);
	use_color_table = 1;
	memcpy(color_table, prop->value, table_level_num);

	cmdline = kstrdup(saved_command_line, GFP_KERNEL);
	if (cmdline) {
		I("Get cmdline success\n");
		temp_cmdline = strstr(cmdline, "color_ID=");
		if(temp_cmdline == NULL) {
			I("No color_ID at devices\n");
			kfree(cmdline);
		} else {
			temp_cmdline += strlen("color_ID=");
			temp_cmdline[CG_ID_LEN] = '\0';
			if(of_property_match_string(node, "vk-black-cg-id-def", temp_cmdline) >= 0) {
				color_ID = BLACK_ID;
				I("color_ID match %s, use color_ID: %d, touch_solution = %d\n", temp_cmdline, color_ID, touch_solution);
			} else if(of_property_match_string(node, "vk-white-cg-id-def", temp_cmdline) >= 0) {
				color_ID = WHITE_ID;
				I("color_ID match %s, use color_ID: %d, touch_solution = %d\n", temp_cmdline, color_ID, touch_solution);
			} else {
				I("No color_ID matched\n");
			}
			kfree(cmdline);
		}
	} else {
		I("Get cmdline failed\n");
	}

	if(color_ID == BLACK_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-black-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-black-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	} else if(color_ID == WHITE_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-white-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-white-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	}

	if(touch_solution == SEC_TOUCH_SOLUTION) {
		prop = of_find_property(node, "vk-current-array-sec",
						&current_table_level_num);
	} else {
		prop = of_find_property(node, "vk-current-array-def",
						&current_table_level_num);
	}
	if(!prop) {
		use_current_table = 0;
	} else {
		use_current_table = 1;
		memcpy(current_table, prop->value, current_table_level_num);
	}
}

#ifdef CONFIG_LEDS_SYNC_TOUCH_SOLUTION
void set_led_touch_solution(uint16_t solution)
{
	g_led_touch_solution = solution;

	if(private_lp5562_client == NULL) {
		I("%s, virtual key led probe not ready\n", __func__);
		return;
	}
	I("%s, led_touch_solution = %d\n", __func__, g_led_touch_solution);

	get_brightness_mapping_table(private_lp5562_client->dev.of_node);
}
EXPORT_SYMBOL(set_led_touch_solution);
#endif

static ssize_t led_color_ID_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct device_node *node;
	struct property *prop;
	char color_ID_name[6];
	int current_table_level_num;
	int color_ID = BLACK_ID;
	int touch_solution = g_led_touch_solution;

	if(private_lp5562_client == NULL) {
		return count;
	}

	node = private_lp5562_client->dev.of_node;

	memcpy(color_ID_name, buf, CG_ID_LEN);
	color_ID_name[CG_ID_LEN] = '\0';

    I("%s, Update color mapping talbe of color_ID: %s\n", __func__, color_ID_name);

	if(of_property_match_string(node, "vk-black-cg-id-def", color_ID_name) >= 0) {
		color_ID = BLACK_ID;
		I("color_ID match %s, use color_ID: %d, touch_solution = %d\n", color_ID_name, color_ID, touch_solution);
	} else if(of_property_match_string(node, "vk-white-cg-id-def", color_ID_name) >= 0) {
		color_ID = WHITE_ID;
		I("color_ID match %s, use color_ID: %d, touch_solution = %d\n", color_ID_name, color_ID, touch_solution);
	} else {
		I("No color_ID matched\n");
		return count;
	}

	if(color_ID == BLACK_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-black-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-black-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	} else if(color_ID == WHITE_ID) {
		if(touch_solution == SEC_TOUCH_SOLUTION) {
			prop = of_find_property(node, "vk-white-pwm-array-sec",
				&table_level_num);
		} else {
			prop = of_find_property(node, "vk-white-pwm-array-def",
				&table_level_num);
		}
		if(!prop) {
			I("Not use color_table\n");
		} else {
			memcpy(color_table, prop->value, table_level_num);
		}
	}

	if(touch_solution == SEC_TOUCH_SOLUTION) {
		prop = of_find_property(node, "vk-current-array-sec",
				&current_table_level_num);
	} else {
		prop = of_find_property(node, "vk-current-array-def",
				&current_table_level_num);
	}
	if(!prop) {
		use_current_table = 0;
	} else {
		use_current_table = 1;
		memcpy(current_table, prop->value, current_table_level_num);
	}
	return count;
}

static DEVICE_ATTR(set_color_ID, 0200, NULL, led_color_ID_store);


static int lp5562_parse_dt(struct device *dev, struct led_i2c_platform_data *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	int rc = 3;
	prop = of_find_property(dt, "lp5562,lp5562_en", NULL);
	if (prop) {
		pdata->ena_gpio = of_get_named_gpio(dt, "lp5562,lp5562_en", 0);
	}
	prop = of_find_property(dt, "lp5562,charging_en", NULL);
	if (prop) {
		pdata->charging_gpio = of_get_named_gpio(dt, "lp5562,charging_en", 0);
	}

	pdata->tp_3v3_en = 0;
	prop = of_find_property(dt, "lp5562,LED_3v3_en", NULL);
	if (prop) {
		pdata->tp_3v3_en = of_get_named_gpio(dt, "lp5562,LED_3v3_en", 0);
		rc = gpio_request(pdata->tp_3v3_en, "led_3v3");
		if(rc < 0) {
			pr_err("[LED] gpio_request failed led_3v3 gpio %d\n", rc);
		}

		rc = gpio_direction_output(pdata->tp_3v3_en, 1);
		if(rc <0) {
			pr_err("[LED] gpio_direction_output led_3v3 failed %d\n", rc);
		}
		rc = gpio_get_value(pdata->tp_3v3_en);
		printk("[LED][PARSE] %d, gpio_dir gpio_get_value= %d \n",pdata->tp_3v3_en, rc);
	}

	prop = of_find_property(dt, "lp5562,num_leds", NULL);
	if (prop) {
		of_property_read_u32(dt, "lp5562,num_leds", &pdata->num_leds);
	}
	prop = of_find_property(dt, "lp5562,current_param", NULL);
	if (prop) {
		of_property_read_u32(dt, "lp5562,current_param", &gCurrent_param);
	}
	if (prop) {
		of_property_read_u32(dt, "lp5562,vk_current_param", &gVK_Current_param);
	}
	pdata->vk_use = of_property_read_bool(dt, "lp5562,vk_use");
	vk_present = pdata->vk_use;
	return 0;
}

static int first_wake = 1;

#ifdef CONFIG_FB
static int fb_notifier_led_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    long last_input_event_diff = (get_global_mseconds() - last_input_event);

    // catch early events as well, as this helps a lot correct functioning knowing when screen is almost off/on, preventing many problems.
    // interpreting still screen ON while it's almost off and vica versa
    if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK && plat_data) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
		pr_info("%s kad self wake: wake event EARLY\n",__func__);
		// if last virtualkey wake was very close, it means that it the user powered screen on.
		// if it's ambient display, virtual key lights are not set very close to the fb blank events...
		///// ...also do not care about this if it's not aosp mode
		wake_by_user = 0;//first_wake || last_vk_wake_diff < 60; // || !aosp_mode;
		pr_info("%s fb wake_by_user %d diff %ld\n",__func__, wake_by_user, last_input_event_diff);
		screen_on_early = 1;
		screen_on = 1;
		vk_off_time_passed = 0;
		blink_running = 0;
		alarm_cancel(&blinkstopfunc_rtc); // stop pending alarm...
		I("screen on -early\n");
            break;
            
        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
//		screen_off_jiffies = jiffies;
//		screen_on = 0;
//		I("screen off -early\n");
            break;
        }
    }
    if (evdata && evdata->data && event == FB_EVENT_BLANK && plat_data) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
		screen_on = 1;
		// check if first wake or other user input directly before this notifier callback, like doubletap wake...
		wake_by_user = first_wake || last_input_event_diff < 1400; // || !aosp_mode;
		first_wake = 0; // boot up wake over...
		pr_info("%s kad self wake: wake event FULL: wake by user result %d diff %ld \n",__func__, wake_by_user, last_input_event_diff);
		vk_off_time_passed = 0;
#if 0
		pulse_rgb_pattern++;
		if (pulse_rgb_pattern > 4) pulse_rgb_pattern = 0;
#endif
		blink_running = 0;
		alarm_cancel(&blinkstopfunc_rtc); // stop pending alarm...
		if (wake_by_user) {
			bln_on_screenoff = 0;
			flash_stop_blink();
			stop_kernel_ambient_display(false);
		}
		// if still charging call charge
		if (charging_notification_occured_for_rgb && charging && get_bln_pulse_rgb_blink_on_charger() && wake_by_user) {
			clear_charging_notification_occured_for_rgb(__func__);
			// start charge color, to cancel RGB blinking
			led_multi_color_charge_level(charge_level, true);
		}
		I("screen on\n");
            break;
            
        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
		screen_off_jiffies = jiffies;
		screen_on = 0;
		screen_on_early = 0;
		I("screen off\n");
		if (bln_on_screenoff) {
			blink_running = 0;
			pr_info("%s kad bln_on_screenoff %d\n", __func__, bln_on_screenoff);
			if (should_start_bln()) {
				queue_work(g_vk_work_queue, &vk_blink_work);
			}
		} 
            break;
        }
    }
    return 0;
}
#endif



static int lp5562_led_probe(struct i2c_client *client
		, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lp5562_chip		*cdata;
	struct led_i2c_platform_data *pdata;
	int ret =0;
	int i;
	u8 check_chip_used;


	printk("[LED][PROBE] led driver probe +++\n");

	/* === init platform and client data === */
	cdata = kzalloc(sizeof(struct lp5562_chip), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		printk("[LED][PROBE_ERR] failed on allocat cdata\n");
		goto err_cdata;
	}

	i2c_set_clientdata(client, cdata);
	cdata->client = client;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) {
		ret = -ENOMEM;
		goto err_exit;
	}
	ret = lp5562_parse_dt(&client->dev, pdata);

	/*=== led pinctrl init ===*/
	if (lp5562_pinctrl_init(cdata, &client->dev) < 0) {
		pr_err("[LED] pinctrl setup failed");
	}

	led_rw_delay = 5;
	/* === led enable pin === */
	if (pdata->ena_gpio) {
		ret = gpio_request(pdata->ena_gpio, "led_enable");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed ena gpio %d\n", __func__, ret);
			goto err_request_ena_gpio;
		}
		ret = gpio_direction_output(pdata->ena_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(pdata->ena_gpio);
			goto err_request_ena_gpio;
		}
		msleep(1);
	} /*else if (pdata->ena_gpio_io_ext) {
	    ret = ioext_gpio_set_value(pdata->ena_gpio_io_ext, 1);
	    if (ret < 0) {
	    pr_err("[LED] %s: io extender high failed %d\n", __func__, ret);
	    gpio_free(pdata->ena_gpio);
	    }
	    }*/
	/* === charging led switch pin === */
	if (pdata->charging_gpio) {
		ret = gpio_request(pdata->charging_gpio, "charging_led_switch");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed charging switch %d\n", __func__, ret);
		}
	}
	/* === led trigger signal pin === */
	if (pdata->tri_gpio) {
		ret = gpio_request(pdata->tri_gpio, "led_trigger");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed led trigger %d\n", __func__, ret);
		}
		ret = gpio_direction_output(pdata->tri_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(pdata->tri_gpio);
		}
	}
	private_lp5562_client = client;

	ret = i2c_read_block(client, ENABLE_REGISTER, &check_chip_used, 1);
	if(ret < 0) {
		I("Not use LP5562 LED.\n");
		goto err_check_chip_not_used;
	}
#if 1
	mutex_init(&operation_mutex);
	mutex_init(&enable_mutex);
#endif

	g_led_work_queue = create_singlethread_workqueue("led");
#if 1
	g_vk_work_queue = g_led_work_queue; // this didn't work out at all: create_singlethread_workqueue("vk_wq"); they collide in i2c write/read!
	g_vib_work_queue = create_singlethread_workqueue("vib_wq");
#endif
	if (!g_led_work_queue) {
		ret = -10;
		pr_err("[LED] %s: create workqueue fail %d\n", __func__, ret);
		goto err_create_work_queue;
	}
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == VIRTUAL_KEY_LED_ID && pdata->vk_use) {
			I("VK probe, i = %d, num_leds = %d\n", i, pdata->num_leds);
			cdata->leds[i].cdev.name = "button-backlight";
			ret = led_classdev_register(dev, &cdata->leds[i].cdev);
			if (ret < 0) {
				dev_err(dev, "couldn't register led[%d]\n", i);
				goto err_register_button_backlight_dev;
			}

			get_brightness_mapping_table(client->dev.of_node);

			cdata->leds[i].cdev.brightness_set = lp5562_vk_led_set_brightness;
			cdata->leds[i].cdev.brightness_get = lp5562_vk_led_get_brightness;
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_set_color_ID);

			INIT_WORK(&cdata->leds[i].led_work, virtual_key_led_work_func);
#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
			INIT_WORK(&vk_blink_work, vk_blink_work_func);
			INIT_WORK(&vk_unblink_work, vk_unblink_work_func);
#endif 
			//INIT_WORK(&cdata->leds[i].led_work_multicolor, virtual_key_led_blink_work_func);
			INIT_DELAYED_WORK(&cdata->leds[i].blink_delayed_work, led_fade_do_work);
			mutex_init(&vk_led_mutex);
		} else if (i == INDICATOR_LED_ID) {
			cdata->leds[i].cdev.name = "indicator";
			ret = led_classdev_register(dev, &cdata->leds[i].cdev);
			if (ret < 0) {
				dev_err(dev, "couldn't register led[%d]\n", i);
				goto err_create_work_queue;
			}

			cdata->leds[i].cdev.brightness_set = lp5562_led_set_brightness;
			cdata->leds[i].cdev.brightness_get = lp5562_led_get_brightness;

			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_ModeRGB);
			if (ret < 0) {
				pr_err("%s: failed on create attr ModeRGB [%d]\n", __func__, i);
				goto err_register_attr_ModeRGB;
			}

			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_off_timer);
			if (ret < 0) {
				pr_err("%s: failed on create attr off_timer [%d]\n", __func__, i);
				goto err_register_attr_off_timer;
			}
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_i2c);
			if (ret < 0) {
				pr_err("%s: failed on create attr i2c [%d]\n", __func__, i);
				goto err_register_attr_i2c;
			}
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_charging_led_switch);
			if (ret < 0) {
				pr_err("%s: failed on create attr charging_led_switch [%d]\n", __func__, i);
				goto err_register_attr_charging_led_switch;
			}
#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_rgb_pulse);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_rgb_blink_on_charger);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_rgb_blink_on_charger_red_limit);
			if (pdata->vk_use) {
				ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln);
				ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_no_charger);
				ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_number);
				ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_number_max);
				ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_light_level);
				ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_dim_blink);
				ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_dim_number);
				ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_dim_number_max);
			}
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_rgb_batt_colored);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_speed);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_speed_max);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_rgb_blink_light_level);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_pulse_rgb_pattern);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_pulse_rgb_pattern_max);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_vib_notification);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_vib_notification_slowness);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_vib_notification_length);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_blink);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_only_face_down);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_blink_number);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_blink_bright);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_blink_bright_number);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_blink_wait_sec);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_blink_wait_inc);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_blink_wait_inc_max);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_haptic_mode);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_dim_mode);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_dim_use_period);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_dim_period_start_hour);
			ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_bln_flash_dim_period_end_hour);
			g_led_led_data_bln = &cdata->leds[i];
			alarm_init(&blinkstopfunc_rtc, ALARM_REALTIME,
				blinkstop_rtc_callback);
			alarm_init(&flash_stop_rtc, ALARM_REALTIME,
				flash_stop_rtc_callback);
			alarm_init(&vibrate_rtc, ALARM_REALTIME,
				vibrate_rtc_callback);
			alarm_init(&double_double_vol_rtc, ALARM_REALTIME,
				double_double_vol_rtc_callback);
			uci_add_sys_listener(uci_sys_listener);
#endif
			wake_lock_init(&cdata->leds[i].led_wake_lock, WAKE_LOCK_SUSPEND, "lp5562");
			INIT_WORK(&cdata->leds[i].led_work, led_work_func);
			INIT_WORK(&cdata->leds[i].led_work_multicolor, multicolor_work_func);
			INIT_DELAYED_WORK(&cdata->leds[i].blink_delayed_work, led_blink_do_work);
			/*alarm_init(&cdata->leds[i].led_alarm,
					ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
					led_alarm_handler);*/
#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
			g_led_led_data = &cdata->leds[i];
#endif
		}
	}
	mutex_init(&cdata->led_i2c_rw_mutex);
	mutex_init(&led_mutex);
	plat_data = pdata;
	/* Avoid led turn off when entering kernel */
#if 0
	/* === disable CHIP_EN === */
	data = 0x00;
	ret = write_enable_register(client, data, 0);
	udelay(550);
	if (pdata->ena_gpio) {
		gpio_direction_output(pdata->ena_gpio, 0);
	} /*else if (pdata->ena_gpio_io_ext) {
	    ioext_gpio_set_value(pdata->ena_gpio_io_ext, 0);
	    }*/
#endif
	printk("[LED][PROBE] led driver probe ---\n");

#ifdef CONFIG_LED_CHECK_PANEL_CONNECTED
	if(!htc_check_panel_connection()){
		if(pdata->tp_3v3_en)
			ret = gpio_direction_output(pdata->tp_3v3_en, 1);
		green_blink_mfg(1);
	}
#endif

#ifdef CONFIG_FB
	fb_notifier_led = kzalloc(sizeof(struct notifier_block), GFP_KERNEL);
	fb_notifier_led->notifier_call = fb_notifier_led_callback;
	fb_register_client(fb_notifier_led);
#endif
#if 1
	probe_finished = 1;
#endif
	return 0;

err_register_button_backlight_dev:
	wake_lock_destroy(&cdata->leds[INDICATOR_LED_ID].led_wake_lock);
err_register_attr_charging_led_switch:
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == INDICATOR_LED_ID)
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_charging_led_switch);
	}
err_register_attr_i2c:
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == INDICATOR_LED_ID)
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_i2c);
	}
err_register_attr_off_timer:
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == INDICATOR_LED_ID)
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_off_timer);
	}
err_register_attr_ModeRGB:
	for (i = 0; i < pdata->num_leds; i++) {
		if(i == INDICATOR_LED_ID)
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_ModeRGB);
	}

	led_classdev_unregister(&cdata->leds[INDICATOR_LED_ID].cdev);
err_create_work_queue:
err_check_chip_not_used:
	if (pdata->ena_gpio) {
		gpio_direction_output(pdata->ena_gpio, 0);
		gpio_free(pdata->ena_gpio);
	}
	if (pdata->charging_gpio)
		gpio_free(pdata->charging_gpio);
	if (pdata->tri_gpio)
		gpio_free(pdata->tri_gpio);
	private_lp5562_client = NULL;
err_request_ena_gpio:
	kfree(pdata);
err_exit:
	kfree(cdata);
err_cdata:
	return ret;
}

static int lp5562_led_remove(struct i2c_client *client)
{
	struct lp5562_chip *cdata;
	int i,ret;

	cdata = i2c_get_clientdata(client);

	ret = lp5562_parse_dt(&client->dev, plat_data);
	if (plat_data->ena_gpio) {
		gpio_direction_output(plat_data->ena_gpio, 0);
	} /*else if (pdata->ena_gpio_io_ext) {
	    ioext_gpio_set_value(pdata->ena_gpio_io_ext, 0);
	    }*/
	for (i = 0; i < plat_data->num_leds; i++) {
		if(i == INDICATOR_LED_ID) {
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_off_timer);
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_ModeRGB);
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_i2c);
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_charging_led_switch);
		}
		led_classdev_unregister(&cdata->leds[i].cdev);
	}
	destroy_workqueue(g_led_work_queue);
#if 1
	destroy_workqueue(g_vk_work_queue);
	destroy_workqueue(g_vib_work_queue);
#endif
	kfree(cdata);

	return 0;
}


static const struct i2c_device_id led_i2c_id[] = {
	{ "LP5562-LED", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, led_i2c_id);

static const struct of_device_id lp5562_mttable[] = {
	{ .compatible = "LP5562-LED"},
	{ },
};

static struct i2c_driver led_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "LP5562-LED",
		.of_match_table = lp5562_mttable,
	},
	.id_table = led_i2c_id,
	.probe = lp5562_led_probe,
	.remove = lp5562_led_remove,
};
module_i2c_driver(led_i2c_driver);
/*
   static int __init lp5562_led_init(void)
   {
   int ret;

   ret = i2c_add_driver(&led_i2c_driver);
   if (ret)
   return ret;
   return 0;
   }

   static void __exit lp5562_led_exit(void)
   {
   i2c_del_driver(&led_i2c_driver);
   }

   module_init(lp5562_led_init);
   module_exit(lp5562_led_exit);
 */
MODULE_AUTHOR("<ShihHao_Shiung@htc.com>, <Dirk_Chang@htc.com>");
MODULE_DESCRIPTION("LP5562 LED driver");

