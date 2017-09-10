#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define DRIVER_AUTHOR "illes pal <illespal@gmail.com>"
#define DRIVER_DESCRIPTION "fingerprint_filter driver"
#define DRIVER_VERSION "1.0"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

#define fpf_PWRKEY_DUR          60
#define FUNC_CYCLE_DUR          9
#define VIB_STRENGTH		20

extern void set_vibrate(int value);

static int fpf_switch = 2;
static struct input_dev * fpf_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static DEFINE_MUTEX(fpfuncworklock);
static struct workqueue_struct *fpf_input_wq;
static struct work_struct fpf_input_work;
static int vib_strength = VIB_STRENGTH;

// touchscreen input handler input work queue and work
static struct workqueue_struct *ts_input_wq;
static struct work_struct ts_input_work;
static struct input_dev *ts_device = NULL;


// Signal int when squeeze2peek triggered set to 1, while waiting for time passing, before the automatic screen off.
// It is used also when a second short squeeze happens, which should interrupt the process by setting this to 0, 
// and don't let auto screen off happen in squeeze_peekmode function.
static int squeeze_peek_wait = 0;


#ifdef CONFIG_FB
	// early screen on flag
	static int screen_on = 1;
	// full screen on flag
	static int screen_on_full = 1;
	struct notifier_block *fb_notifier;
#endif

// value used to signal that HOME button release event should be synced as well in home button func work if it was not interrupted.
static int do_home_button_off_too_in_work_func = 0;

static int wait_for_squeeze_power = 0;
static unsigned long last_squeeze_power_registration_jiffies = 0;

/* PowerKey work func */
static void fpf_presspwr(struct work_struct * fpf_presspwr_work) {

	unsigned int squeeze_reg_diff = 0;
	if (!mutex_trylock(&pwrkeyworklock))
                return;
	// if wait for squeeze power is on (called from squeeze code)...
	if (wait_for_squeeze_power) {
		wait_for_squeeze_power = 0;
		// if screen is on...
		if (screen_on) {
			// ...wait a bit...
			msleep(30);
			// .. check if last power resgistration through threshold reg callback happened lately... if so no need to do screen off..
			squeeze_reg_diff = jiffies - last_squeeze_power_registration_jiffies;
			pr_info("%s squeeze_reg_diff %u\n",__func__,squeeze_reg_diff);
			if (squeeze_reg_diff<4) goto exit;
		}
	}
	input_event(fpf_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(fpf_pwrdev, EV_SYN, 0, 0);
	msleep(fpf_PWRKEY_DUR);
	input_event(fpf_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(fpf_pwrdev, EV_SYN, 0, 0);
	msleep(fpf_PWRKEY_DUR/2);
	// resetting this do_home_button_off_too_in_work_func when powering down, as it causes the running HOME button func work 
	//	to trigger a HOME button release sync event to input device, resulting in an unwanted  screen on.
	do_home_button_off_too_in_work_func = 0;
	exit:
        mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(fpf_presspwr_work, fpf_presspwr);

static void fpf_vib(void) {
	// avoid using squeeze vib length 15...
	set_vibrate(vib_strength==15?14:vib_strength);
}

/* PowerKey trigger */
static void fpf_pwrtrigger(int vibration) {
	if (vibration) fpf_vib();
	schedule_work(&fpf_presspwr_work);
        return;
}


static void fpf_input_callback(struct work_struct *unused) {
	return;
}

static void fpf_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value) {
}

static int fpf_input_dev_filter(struct input_dev *dev) {
	if (strstr(dev->name, "uinput-fpc")) {
		return 0;
	} else {
		return 1;
	}
}

static int fpf_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	if (fpf_input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "fpf";

	error = input_register_handle(handle);

	error = input_open_device(handle);

	return 0;

}


// break home button func -> in the HOME button work func, where we count from a sync-surpressend first press of HOME button, this is used in external sources 
// 	to break counting of time passing. This way, HOME button press sync can be avoided, and double tap of HOME button can be translated into a POWER OFF even instead.
//	If this value is set to 1, counting will break, and work func will exit without calling INPUT device HOME sync.
//	meanwhile the other func_trigger call will still face LOCK locking, and based on the locking=true will start a Power off.
static int break_home_button_func_work = 1;

// time_count_done_in_home_button_func_work -> represents if the time counting in the HOME button work func is over, meaning that the next HOME button press in func_trigger code
//	shouldn't be interpreted as a double tap instead 
// 	just exit the func trigger call without Power off and leaving normal HOME button sync to work in the home_button work func after the time count
static int time_count_done_in_home_button_func_work = 0;

// job_done_in_home_button_func_work -> represents if we arrived inside the home button work func at the counting of time, without interruption (break_home_button still 0), thus it can be set to 1,
//	HOME button 1 sync will be done in work, and it's also signalling that when the FP device sends release event, in the filter code, HOME button 0 sync can be done.
//	The trigger func will set it to 0 always, so it shows the job was interrupted, which is important when the release of the button is done after
//		trigger job found that the LOCK is not holding anymore, and does nothing, in which case the filter call should send HOME - 0 sync to input device.
static int job_done_in_home_button_func_work = 0;

// signals if fingerprint PRESS was registered, so we can track that no multiple releases happen from FP device
static int fingerprint_pressed = 0;

// signals when the powering down of screen happens while FP is still being pressed, so filter won't turn screen on, when the button is released based on this value.
static int powering_down_with_fingerprint_still_pressed = 0;


// minimum doubletap wait latency will be: (BASE_VALUE + PERIOD) * FUNC_CYLCE_DUR -> minimum is right now (9+0) * 9 = 81msec
#define DT_WAIT_PERIOD_MAX 9
#define DT_WAIT_PERIOD_BASE_VALUE 12
#define DT_WAIT_PERIOD_DEFAULT 2
static int doubletap_wait_period = DT_WAIT_PERIOD_DEFAULT;

/* Home button work func 
	will start with trying to lock worklock
	then use vibrator to signal button press 'imitation'
	Will set break_home_button_func_work to 0, as we just started, and interruptions are signalled through this integer
	While break is not done, it will count the maximum time that is acceptable between two BUTTON presses whchi is interpreted as double press
	- If it's exiting due to Interruption (break_home_button_func_work called from another func_trigger call)
	    it won't do anything just release lock and return. The trigger call will then power down screen, as this counts as double tap
	- If it exited with counting done (break == 0) it will sync a HOME = 1 event to itself
	    - and it will signal job_done_in_home_button_func_work = 1, so when filter func receives Key released, it can Sync a HOME = 0 to input device,
		or set do_home_buttons_too -> 1, so the hom button func work job will itself send the HOME = 0 synced before exiting
*/
static void fpf_home_button_func(struct work_struct * fpf_presspwr_work) {
	int count_cycles = 0;
	if (!mutex_trylock(&fpfuncworklock)) {
		return;
	}
	break_home_button_func_work = 0;
	time_count_done_in_home_button_func_work = 0;
	fpf_vib();
	while (!break_home_button_func_work) {
		count_cycles++;
		if (count_cycles > (DT_WAIT_PERIOD_BASE_VALUE + doubletap_wait_period)) {
			break;
		}
		msleep(FUNC_CYCLE_DUR);
		pr_debug("fpf %s counting in cycle before KEY_HOME 1 synced: %d / %d cycles \n",__func__, count_cycles, DT_WAIT_PERIOD_BASE_VALUE+doubletap_wait_period);
	}
	time_count_done_in_home_button_func_work = 1;
	if (break_home_button_func_work == 0) {
		job_done_in_home_button_func_work = 1;
		pr_info("fpf %s home 1 \n",__func__);
		input_event(fpf_pwrdev, EV_KEY, KEY_HOME, 1);
		input_event(fpf_pwrdev, EV_SYN, 0, 0);
		msleep(1);
		if (do_home_button_off_too_in_work_func) {
			pr_info("fpf %s home 0 \n",__func__);
			input_event(fpf_pwrdev, EV_KEY, KEY_HOME, 0);
			input_event(fpf_pwrdev, EV_SYN, 0, 0);
			do_home_button_off_too_in_work_func = 0;
			msleep(1);
//			fpf_vib();
		}
	} 
	mutex_unlock(&fpfuncworklock);
	pr_info("fpf %s mutex unlocked \n",__func__);
	return;
}
static DECLARE_WORK(fpf_home_button_func_work, fpf_home_button_func);


/* fpf home button func trigger */
static void fpf_home_button_func_trigger(void) {
	pr_info("fpf %s time_count_done_in_home_button_func_work %d job_done_in_home_button_func_work %d\n",__func__, time_count_done_in_home_button_func_work, job_done_in_home_button_func_work);
	job_done_in_home_button_func_work = 0;
	break_home_button_func_work = 1;
	if (mutex_is_locked(&fpfuncworklock)) {
		// mutex in hold, this means the HOME button was pressed again in a short time...
		pr_info("fpf %s is locked, checkin %d time_count_done_in_home_button_func_work...", __func__, time_count_done_in_home_button_func_work);
		if (!time_count_done_in_home_button_func_work) { // and we still counting the cycles in the job, so double tap poweroff can be done...
			// double home:
			pr_info("fpf double tap home, power off\n");
			if (fingerprint_pressed == 1) { // there was no release of the fingerprint button, so go screen off with signalling that here...
				powering_down_with_fingerprint_still_pressed = 1;
			} else { 
				powering_down_with_fingerprint_still_pressed = 0; 
			}
			fpf_pwrtrigger(1);
			do_home_button_off_too_in_work_func = 0;
		}
                return;
	}
	schedule_work(&fpf_home_button_func_work);
        return;
}

/*
    filter will work on FP card events.
    if screen is not on it will work on powering it on when needed (except when Button released start (button press) was started while screen was still on: powering_down_with_fingerprint_still_pressed = 1)
    Otherwise if
	- pressed (value > 0)
		it will call home button trigger job, to handle single fp button presses or double taps.
	- if released:
		if home button work job is done already, finish with syncing HOME release to input device
		if home button work job is still running, set 'do_home_buttons_off_too' to 1, so job will sync HOME release as well
*/
static bool fpf_input_filter(struct input_handle *handle,
                                    unsigned int type, unsigned int code,
                                    int value)
{
	if (screen_on_full) squeeze_peek_wait = 0; // interrupt peek wait, touchscreen was interacted, don't turn screen off after peek time over...

	// if it's not on, don't filter anything...
	if (fpf_switch == 0) return false;

	if (type != EV_KEY)
		return false;

	if (code == KEY_WAKEUP) {
		pr_debug("fpf - wakeup %d %d \n",code,value);
	}


	if (fpf_switch == 2) {
	//standalone kernel mode. double tap means switch off
	if (value > 0) {
		if (!screen_on) {
			return false; // don't filter so pin appears
		} else {
			fingerprint_pressed = 1;
			pr_info("fpf %s starting trigger \n",__func__);
			fpf_home_button_func_trigger();
		}
		return true;
	} else {
		if (fingerprint_pressed) {
			if (!screen_on) {
				if (!powering_down_with_fingerprint_still_pressed) {
					return false; // don't filter so pin appears
				} else {
					// fingerprint release happens after a screen off that started AFTER the fingerprint was pressed. So do not wake the screen
					powering_down_with_fingerprint_still_pressed = 0;
					return false;
				}
			} else {
				// screen is on...
				// release the fingerprint_pressed variable...
				fingerprint_pressed = 0;
				// if job was all finished inside the work func, we need to call the HOME = 0 release event here, as we couldn't signal to the work to do it on it's own
				if (job_done_in_home_button_func_work) {
						pr_info("fpf %s do key_home 0 sync as job was done, but without the possible signalling for HOME 0\n",__func__);
						input_report_key(fpf_pwrdev, KEY_HOME, 0);
						input_sync(fpf_pwrdev);
				} else {
				// job is not yet finished in home button func work, let's signal it, to do the home button = 0 sync as well
					if (screen_on) {
						do_home_button_off_too_in_work_func = 1;
					} else {
						return false;
					}
				}
			}
			return true;
		} else 
		{ // let event flow through
			return false;
		}
	}
	}
	if (fpf_switch == 1) {
		// simple home button mode, user space handles behavior
		if (!screen_on) {
			return false;
		}
		if (value > 0) {
			fpf_vib();
			input_report_key(fpf_pwrdev, KEY_HOME, 1);
			input_sync(fpf_pwrdev);
		} else {
			input_report_key(fpf_pwrdev, KEY_HOME, 0);
			input_sync(fpf_pwrdev);
		}
	}
	return true;
}



// ---------------- SQUEEZE TO WAKE SLEEP: 
// wakelock method code
static int squeeze_wake = 1;
static int squeeze_sleep = 1;
static int squeeze_peek = 1;
static int squeeze_peek_halfseconds = 4;

// defines what maximum level of user setting for minimum squeeze power set on sense ui
// will set kernel-side squeeze-to-sleep/wake active. This way user can set below this
// level the squeeze power on Sense UI, and kernel-squeeze handling will turn on
static int squeeze_power_kernel_max_threshold = 1;// 0 - 9
// 112, 132, 152, 172, 192, 212, 232, 252, 272, 292
// 100-120 - 121-130...- 280-300

// int that signals if kernel should handle squeezes for squeeze to sleep/wake
static int squeeze_kernel_handled = 0;

void register_squeeze_power_threshold_change(int power) {
	int new_level = (power - 101) / 20;
	pr_info("%s squeeze call new_level power %d max level %d power %d \n",__func__,new_level,squeeze_power_kernel_max_threshold,power);
	if (new_level <= squeeze_power_kernel_max_threshold && power>=100) {
		squeeze_kernel_handled = 1;
	} else {
		squeeze_kernel_handled = 0;
	}
	last_squeeze_power_registration_jiffies = jiffies;
}
EXPORT_SYMBOL(register_squeeze_power_threshold_change);

static void squeeze_vib(void) {
	set_vibrate(35); // a bit stronger vibration to allow user releasing it
}


// ===========
// Swipe
// ===========

// sysfs parameters
static int squeeze_swipe = 1;
static int squeeze_swipe_vibration = 1;

// members...
static int squeeze_swipe_dir = 1;
int last_mt_slot = 0;
int pseudo_rnd = 0;

int swipe_step_wait_time_mul = 100; // tune this to find optimal slowness of swipe for all kinds of apps

unsigned long last_scroll_emulate_timestamp = 0;
static DEFINE_MUTEX(squeeze_swipe_lock);

#define SWIPE_ACCELERATED_TIME_LIMIT 150
int interrupt_swipe_longcount = 0;
int swipe_longcount_finished = 0;
unsigned long swipe_longcount_start = 0;
static void swipe_longcount(struct work_struct * swipe_longcount_work) {
	while (1) {
		if (interrupt_swipe_longcount) {
			interrupt_swipe_longcount = 0;
			return;
			pr_info("%s ######## squeeze call || swipe_longcount interrupted\n",__func__);
		}
		if (jiffies - swipe_longcount_start > SWIPE_ACCELERATED_TIME_LIMIT) {
			pr_info("%s ######## squeeze call || swipe_longcount VIBRATION !! \n",__func__);
			swipe_longcount_finished = 1;
			if (squeeze_swipe_vibration) {
				set_vibrate(1);
			}
			return;
		}
		msleep(1);
	}
}
static DECLARE_WORK(swipe_longcount_work, swipe_longcount);
static void swipe_longcount_trigger(void) {
	swipe_longcount_finished = 0;
	interrupt_swipe_longcount = 0;
	schedule_work(&swipe_longcount_work);
}


#if 1
#define TS_MAP_SIZE 200
static int ts_current_type[TS_MAP_SIZE], ts_current_code[TS_MAP_SIZE], ts_current_value[TS_MAP_SIZE];
static int ts_current_count = 0;
static void ts_call_input_event(void *p, int type, int code, int value)
{
	ts_current_type[ts_current_count] = type;
	ts_current_code[ts_current_count] = code;
	ts_current_value[ts_current_count] = value;
	ts_current_count = ts_current_count==(TS_MAP_SIZE-1) ? 0 : (ts_current_count+1);
	input_event(ts_device,type,code,value);
}

static int longcount_squeeze_swipe_dir_change = 0;

static int last_swipe_very_quick = 0;

/*
START
type: 3 EV_ABS
code: 57
v: 325-324 : touching screen

LOOP:

type: 3 EV_ABS
code: 53 -> X coordinate 800
type: 3 EV_ABS
code: 54 -> Y value : 1300 to 1000 --> 20 steps
type: 3 EV_ABS
code: 58 -> ? strength? 70

type: 0 code: 0 v:0 	sync
END LOOP

type: 3 EV_ABS
code: 57
v: -1 : leaving screen

type: 0 code: 0 v:0 	sync
END ALL
*/
static void ts_scroll_emulate(int down, int full) {

	int y_diff;
	int y_delta;
	int y_steps; // tune this for optimal page size of scrolling
	int rounds = 1;
	int i = 0;
	int allow_speedup_next = full?1:0;
	unsigned int last_scroll_time_diff = jiffies - last_scroll_emulate_timestamp;
	int double_swipe = 0;


	pr_info("%s ts_input ######### squeeze try_lock #########\n",__func__);
	if (!mutex_trylock(&squeeze_swipe_lock)) {
		return;
	}


	// if last scroll close enough, double round of swipe, if it's intended to be a full swipe...
	if (last_scroll_time_diff <= SWIPE_ACCELERATED_TIME_LIMIT && !swipe_longcount_finished && full) {
		pr_info("%s ts_input ###### double speed swipe ####### diff %u swipe longcount finished %d\n",__func__, last_scroll_time_diff, swipe_longcount_finished);
		rounds = 2;
		double_swipe = 1;
	}

	swipe_longcount_start = jiffies;
	swipe_longcount_trigger();

	// reset filtering store used by ts input filtering...
	ts_current_count = 0;
	for (i=0; i<TS_MAP_SIZE; i++) {
		ts_current_type[i]=100;
	}

	if (last_scroll_time_diff > 5000) { // a higher value...passed?
		if (full == 1) {
			if (longcount_squeeze_swipe_dir_change == 0) {
				// if last direction change was not done by middle long squeeze, then...
				full = -1; // long time passed, scroll only a bit to demo the direction...
			} else {
				longcount_squeeze_swipe_dir_change = 0;
			}
		}
	}
	while (--rounds>=0) {
		y_diff = down?300:0;
		y_delta = down?-2:2;
		y_steps = full>0?400:(full==0?70:60);
		pr_info("%s ts_input ######### squeeze emulation started ######### rounds %d \n",__func__, rounds);

		// speedy swipe for doubled rounds...
		if (double_swipe) {
			y_delta *= 2; // bigger delta for speed
			y_steps /= 3; // fewer steps, to not run out of screen
			swipe_step_wait_time_mul = 300 - ( ((SWIPE_ACCELERATED_TIME_LIMIT - last_scroll_time_diff)*2)/1 ); // 300 - 0
			if (swipe_step_wait_time_mul > 85) last_swipe_very_quick = 0;
			if (!last_swipe_very_quick && swipe_step_wait_time_mul < 85) last_swipe_very_quick = 1;
			if (last_swipe_very_quick && swipe_step_wait_time_mul < 85) swipe_step_wait_time_mul = (swipe_step_wait_time_mul*2)/3; // speed up on the extreme of fast value multiplier < 80, divide it
			pr_info("%s ts_input ######### squeeze emulation SPEED %d \n",__func__, swipe_step_wait_time_mul);
			if (swipe_step_wait_time_mul > 300) swipe_step_wait_time_mul = 300; // to avoid concurrency problem with last_scroll_time_diff
			if (swipe_step_wait_time_mul < 0) swipe_step_wait_time_mul = 0;


		} else {
			if (full>0) {
				swipe_step_wait_time_mul = 100;
			} else {
				swipe_step_wait_time_mul = 150;
			}
		}

		// update last scroll ts
		last_scroll_emulate_timestamp = allow_speedup_next?jiffies:0; // if small turning swipe, avoid double speed swipe on the next occasion, setting here timestamp to 0

		// TODO how to determine portrait/landscape mode? currently only portrait

		if (screen_on) {
			pr_info("fpf %s ts DOWN 1 \n",__func__);
			ts_call_input_event(ts_device, EV_ABS, ABS_MT_TRACKING_ID, --last_mt_slot);
			while (y_steps-->0) {
				ts_call_input_event(ts_device, EV_ABS, ABS_MT_POSITION_X, 800+ (pseudo_rnd++)%2);
				ts_call_input_event(ts_device, EV_ABS, ABS_MT_POSITION_Y, 1000+y_diff);
				y_diff += y_delta;
				ts_call_input_event(ts_device, EV_ABS, ABS_MT_PRESSURE, 70+ (pseudo_rnd%2));
				ts_call_input_event(ts_device, EV_SYN, 0, 0);
				usleep_range(5 * swipe_step_wait_time_mul , (5 * swipe_step_wait_time_mul) + 1);
				if (y_steps%10==0) {
					pr_info("%s ts_input squeeze emulation step = %d POS_Y = %d \n",__func__,y_steps, 1000+y_diff);
				}
			}

			pr_info("fpf %s ts DOWN 0 \n",__func__);
			ts_call_input_event(ts_device, EV_ABS, ABS_MT_TRACKING_ID, -1);
			ts_call_input_event(ts_device, EV_SYN, 0, 0);
			msleep(1);
		}
		pr_info("%s ts_input ######### squeeze emulation ended #########\n",__func__);
	}
	if (pseudo_rnd>4) pseudo_rnd = 0;
	msleep(1);
	mutex_unlock(&squeeze_swipe_lock);
	pr_info("%s ts_input ######### squeeze unlock #########\n",__func__);
}
#endif

static void squeeze_swipe_func(struct work_struct * squeeze_swipe_work) {
	ts_scroll_emulate(squeeze_swipe_dir, 1);
}
static DECLARE_WORK(squeeze_swipe_work, squeeze_swipe_func);
static void squeeze_swipe_trigger(void) {
	pr_info("%s ts_input squeeze swipe trigger is_locked...\n",__func__);
	if (mutex_is_locked(&squeeze_swipe_lock)) {
		return;
	}
	interrupt_swipe_longcount = 1;
	pr_info("%s ts_input squeeze swipe trigger is_locked NOT..scheduling work...\n",__func__);
	schedule_work(&squeeze_swipe_work);
}


static void squeeze_swipe_short_func(struct work_struct * squeeze_swipe_short_work) {
	ts_scroll_emulate(squeeze_swipe_dir, 0);
}
static DECLARE_WORK(squeeze_swipe_short_work, squeeze_swipe_short_func);
static void squeeze_swipe_short_trigger(void) {
	pr_info("%s ts_input squeeze swipe trigger is_locked...\n",__func__);
	if (mutex_is_locked(&squeeze_swipe_lock)) {
		return;
	}
	interrupt_swipe_longcount = 1;
	pr_info("%s ts_input squeeze swipe trigger is_locked NOT..scheduling work...\n",__func__);
	schedule_work(&squeeze_swipe_short_work);
}



#define MAX_SQUEEZE_TIME 35
#define MAX_SQUEEZE_TIME_LONG 69
#define MAX_NANOHUB_EVENT_TIME 4
static unsigned long longcount_start = 0;
static int interrupt_longcount = 0;
static int longcount_finished = 0;
static void squeeze_longcount(struct work_struct * squeeze_longcount_work) {
	while (1) {
		if (interrupt_longcount) {
			return;
			pr_info("%s squeeze call || longcount interrupted\n",__func__);
		}
		if (jiffies - longcount_start > MAX_SQUEEZE_TIME) {
			pr_info("%s squeeze call || longcount VIBRATION !! \n",__func__);
			longcount_finished = 1;
			squeeze_vib();
			return;
		}
		msleep(7);
	}
}
static DECLARE_WORK(squeeze_longcount_work, squeeze_longcount);
static void squeeze_longcount_trigger(void) {
	longcount_finished = 0;
	interrupt_longcount = 0;
	schedule_work(&squeeze_longcount_work);
}


static void squeeze_peekmode(struct work_struct * squeeze_peekmode_work) {
	unsigned int squeeze_reg_diff = 0;
	squeeze_peek_wait = 1;
	if (wait_for_squeeze_power) {
		// wait_for_squeeze_power = 0; not reset "wait_for..." here.. it will be done in fpf_pwrtrigger part!
		// if screen is on...
		if (screen_on) {
			// ...wait a bit...
			msleep(30);
			// .. check if last power resgistration through threshold reg callback happened lately... if so no need to do screen off..
			squeeze_reg_diff = jiffies - last_squeeze_power_registration_jiffies;
			pr_info("%s squeeze_reg_diff %u\n",__func__,squeeze_reg_diff);
			if (squeeze_reg_diff<4) return;
		}
	}
	msleep(squeeze_peek_halfseconds * 500);
	// screen still on and sqeueeze peek wait was not interrupted...
	if (screen_on && squeeze_peek_wait) fpf_pwrtrigger(0);
	squeeze_peek_wait = 0;
}
static DECLARE_WORK(squeeze_peekmode_work, squeeze_peekmode);
static void squeeze_peekmode_trigger(void) {
	schedule_work(&squeeze_peekmode_work);
}

// this callback allows registration of FP vibration, in which case peek timeout auto screen off should be canceled...
int register_fp_vibration(void) {
	// fp scanner pressed, cancel peek timeout
	squeeze_peek_wait = 0;
	return vib_strength;
}
EXPORT_SYMBOL(register_fp_vibration);

static unsigned long last_squeeze_timestamp = 0;
static unsigned long last_screen_event_timestamp = 0;
static unsigned long last_nanohub_spurious_squeeze_timestamp = 0;

#define STAGE_INIT 0
#define STAGE_FIRST_WL 1
#define STAGE_VIB 2
static int stage = STAGE_INIT;


void register_squeeze(unsigned long timestamp, int vibration) {
	// time passing since screen on/off state changed - to avoid collision of detections
	unsigned int diff = jiffies - last_screen_event_timestamp;
	// time passed since last nanohub driver based spurious squeeze wake event detection
	unsigned int nanohub_diff = jiffies - last_nanohub_spurious_squeeze_timestamp;
	pr_info("%s squeeze call ts %u diff %u nh_diff %u vibration %d\n", __func__, (unsigned int)timestamp,diff,nanohub_diff,vibration);
	if (!squeeze_kernel_handled) return;

	if (!squeeze_wake && !squeeze_sleep && !squeeze_swipe) return;
	if (!squeeze_wake && !screen_on) return;
	if (!squeeze_sleep && !squeeze_swipe && screen_on) return;

	if (!last_screen_event_timestamp) return;
	if ((!screen_on && diff < 3) || (screen_on && diff < 30)) return;

	pr_info("%s squeeze call ++ START STAGE : %d\n",__func__,stage);
	if (stage == STAGE_INIT) {
		if (!vibration) {
			stage = STAGE_FIRST_WL;
			last_squeeze_timestamp = jiffies;
		}

		// if screen is off and nanohub edge wake event detected recently, trigger power button here..
		if (!vibration && !screen_on && nanohub_diff < MAX_NANOHUB_EVENT_TIME) {
			// catching the very rare case where nanohub squeeze detection happens
			// while screen off and release event is detected without actual release,
			//  and one Wakelock event will be skipped... but through nanohub it was detected...
			pr_info("%s squeeze call -- spurious nanohub detection: power onoff endstage: %d\n",__func__,stage);
			stage = STAGE_INIT;
			last_nanohub_spurious_squeeze_timestamp = 0;
			wait_for_squeeze_power = 1; // pwr trigger should be canceled if right after squeeze happens a power setting
			// ..that would mean user is on the settings screen and calibrating.
			if (!screen_on && squeeze_peek) {
				last_screen_event_timestamp = jiffies;
				squeeze_peekmode_trigger();
			}
			if (screen_on && squeeze_peek_wait) { // checking if short squeeze happening while peeking the screen with squeeze2peek...
				last_screen_event_timestamp = jiffies;
				squeeze_peek_wait = 0; // yes, so interrupt peek sleep, screen should remain on after a second short squeeze happened still in time window of peek...
			} else {
				if (screen_on && squeeze_swipe) {
					squeeze_swipe_trigger();
				} else {
					last_screen_event_timestamp = jiffies;
					fpf_pwrtrigger(0);
				}
			}
			return;
		}
		pr_info("%s squeeze call -- END STAGE : %d\n",__func__,stage);
		return;
	}
	diff = jiffies - last_squeeze_timestamp;
	pr_info("%s squeeze call ++ squeeze diff : %u\n",__func__,diff);

	if (stage == STAGE_FIRST_WL) {
		if (vibration && diff <= 5) {

			// if screen is off and nanohub edge wake event detected recently, trigger power button here..
			if (!screen_on && nanohub_diff < MAX_NANOHUB_EVENT_TIME) {
				// catching the very rare case where nanohub squeeze detection happens
				// while screen off and release event is detected without actual release,
				//  and one Wakelock event will be skipped... but through nanohub it was detected...
				pr_info("%s squeeze call -- stage WL -- spurious nanohub detection: power onoff endstage: %d\n",__func__,stage);
				stage = STAGE_INIT;
				last_nanohub_spurious_squeeze_timestamp = 0;
				wait_for_squeeze_power = 1; // pwr trigger should be canceled if right after squeeze happens a power setting
				// ..that would mean user is on the settings screen and calibrating.
				if (!screen_on && squeeze_peek) {
					last_screen_event_timestamp = jiffies;
					squeeze_peekmode_trigger();
				}
				if (screen_on && squeeze_peek_wait) { // screen on and squeeze peek going on?
					last_screen_event_timestamp = jiffies;
					squeeze_peek_wait = 0; // interrupt peek sleep, screen should remain on after a second short squeeze while still in time window of peek...
				} else {
					if (screen_on && squeeze_swipe) {
						squeeze_swipe_trigger();
					} else {
						last_screen_event_timestamp = jiffies;
						fpf_pwrtrigger(0);
					}
				}
				return;
			}

			stage = STAGE_VIB;
			// start longcount trigger
			longcount_start = last_squeeze_timestamp;
			if (squeeze_swipe && !swipe_longcount_finished) {
				// in swipe mode first vibration should stop swipe longcount, because the start of the squeeze should stop 
				// so that while a middle long gesture goes on, it won't finish with swipe_longcount_finished == 1 output, that would prevent direction change, and go
				// into screen off, while the swipe longcount vibration was not present at starting the next squeeze...
				interrupt_swipe_longcount = 1;
			}
			squeeze_longcount_trigger();
			pr_info("%s squeeze call -- END STAGE : %d\n",__func__,stage);
			return; 
		} else {
			if (vibration) { // vibration but too late...back to init state..
				stage = STAGE_INIT; 
			} else {
				// wakelock registered -> start time counting in FIRST_WL stage again...
				last_squeeze_timestamp = jiffies;
			}
			pr_info("%s squeeze call -- END STAGE : %d\n",__func__,stage);
			return;
		}
	}
	if (stage == STAGE_VIB) {
		stage = STAGE_INIT;
		// interrupt longcount
		interrupt_longcount = 1;
		if (vibration) {
			pr_info("%s squeeze call -- exiting because vibration endstage: %d\n",__func__,stage);
			return;
		} else if ( (diff<=MAX_SQUEEZE_TIME) || (screen_on && !longcount_finished) ) {
			pr_info("%s squeeze call -- power onoff endstage: %d\n",__func__,stage);
			wait_for_squeeze_power = 1; // pwr trigger should be canceled if right after squeeze happens a power setting
			// ..that would mean user is on the settings screen and calibrating.
			// also...
			// if peek mode is on, and between long squeeze and short squeeze, peek
			if (!screen_on && squeeze_peek) {
				pr_info("%s squeeze call -- power onoff - PEEK MODE - PEEK wake: %d\n",__func__,stage);
				last_screen_event_timestamp = jiffies;
				squeeze_peekmode_trigger();
			}
			if (screen_on && squeeze_peek_wait) { // screen on and squeeze peek going on?
				last_screen_event_timestamp = jiffies;
				squeeze_peek_wait = 0; // interrupt peek sleep, screen should remain on after a second short squeeze while still in time window of peek...
			} else {
				if (screen_on && squeeze_swipe) {
					squeeze_swipe_trigger();
				} else {
					last_screen_event_timestamp = jiffies;
					fpf_pwrtrigger(0);
				}
			}
		} else if (!screen_on && diff>MAX_SQUEEZE_TIME && diff<=MAX_SQUEEZE_TIME_LONG && squeeze_peek) {
			// if peek mode is on, and between long squeeze and short squeeze, peek
			pr_info("%s squeeze call -- power onoff endstage PEEK MODE - full wake! %d\n",__func__,stage);
			last_screen_event_timestamp = jiffies;
			wait_for_squeeze_power = 1; // pwr trigger should be canceled if right after squeeze happens a power setting
			// ..that would mean user is on the settings screen and calibrating.
			fpf_pwrtrigger(0);
		} else if (screen_on && diff>MAX_SQUEEZE_TIME && diff<=MAX_SQUEEZE_TIME_LONG && squeeze_swipe) {
			if (squeeze_sleep) {
				//unsigned int last_scroll_time_diff = jiffies - last_scroll_emulate_timestamp;
				wait_for_squeeze_power = 1; // pwr trigger should be canceled if right after squeeze happens a power setting
				// ..that would mean user is on the settings screen and calibrating.
				if (!swipe_longcount_finished) {
				// if quickly squeezing before last squeeze swipe ended, turn direction, instead of power off
					// turn direction as NO squeeze sleep is set on
					longcount_squeeze_swipe_dir_change = 1;
					squeeze_swipe_dir = !squeeze_swipe_dir;
					// call a bit of scrolling to show which direction it will go (full param = 0)
					squeeze_swipe_short_trigger();
					pr_info("%s squeeze TURN SWIPE DIRECTION -- END STAGE : %d\n",__func__,stage);
					return; // exit with turning...
				}
				// if swipe mode is on, and between long squeeze and short squeeze, power off
				pr_info("%s squeeze call -- power onoff endstage SWIPE - full sleep - swipe mode middle long gesture! %d\n",__func__,stage);
				last_screen_event_timestamp = jiffies;
				fpf_pwrtrigger(0);
				return;
			} else {
				// turn direction as NO squeeze sleep is set on
				longcount_squeeze_swipe_dir_change = 1;
				squeeze_swipe_dir = !squeeze_swipe_dir;
				// call a bit of scrolling to show which direction it will go (full param = 0)
				squeeze_swipe_short_trigger();
				pr_info("%s squeeze TURN SWIPE DIRECTION -- END STAGE : %d\n",__func__,stage);
				return;
			}
		} else if (!screen_on || diff>75) { // time passed way over a normal wakelock cycle... start with second phase instead!
			stage = STAGE_FIRST_WL;
			last_squeeze_timestamp = jiffies;
		}
		pr_info("%s squeeze call -- END STAGE : %d\n",__func__,stage);
	}

}
EXPORT_SYMBOL(register_squeeze);

// ----------------- nanohub callback methods

static unsigned long last_timestamp = 0;
#define SQUEEZE_EVENT_TYPE_NANOHUB  0
#define SQUEEZE_EVENT_TYPE_NANOHUB_INIT  1
#define SQUEEZE_EVENT_TYPE_VIBRATOR  2

#define MAX_NANOHUB_DIFF_INIT_END 7
#define MIN_NANOHUB_DIFF_END_END 100

static int last_event = 0;

void register_squeeze_wake(int nanohub_flag, int vibrator_flag, unsigned long timestamp, int init_event_flag)
{
	unsigned int diff = timestamp - last_timestamp;
	int event = nanohub_flag?(init_event_flag?SQUEEZE_EVENT_TYPE_NANOHUB_INIT:SQUEEZE_EVENT_TYPE_NANOHUB):SQUEEZE_EVENT_TYPE_VIBRATOR;

	pr_info("%s squeeze wake call, nano %d vib %d ts %u diff %u init flag %d event %d last_event %d\n", __func__, nanohub_flag,vibrator_flag,(unsigned int)timestamp,diff,init_event_flag, event, last_event);
	last_timestamp = timestamp;

	if (
		!screen_on && nanohub_flag && 
		(
		    ( diff < MAX_NANOHUB_DIFF_INIT_END && event!=last_event ) || 
		    ( event == last_event && event == SQUEEZE_EVENT_TYPE_NANOHUB && diff > MIN_NANOHUB_DIFF_END_END )
		)
	) {
		// helper case, where Wakelock event miss, this nanohub detection can help
		// we store the timestamp of the case when a full squeeze was detected from
		// nanohub init event and release event with a very little timediff 
		// (diff calculated when this method was called twice, and is a small enough value).
		//
		// ... if event is a Nanohub release event following another release vent after a long period passing, it usually means
		// ... nanohub driver missed the INIT event, in that case enter this branch too.
		pr_info("%s spurious squeeze nanohub detection triggered: diff %u\n",__func__, diff);
		last_nanohub_spurious_squeeze_timestamp = timestamp;

		if (stage == STAGE_VIB) {
			pr_info("%s spurious squeeze nanohub detection triggered: STAGE_VIB - calling register_squeeze right now.\n",__func__);
			// if process is already after detecting VIB (stage_vib), call directly in,
			// in some cases this is necessary, as userspace WL can delay too much,
			// while this nanohub call happening earlier...
			last_nanohub_spurious_squeeze_timestamp = 0;
			register_squeeze(timestamp,0);
		}

	}

#if 0
// this part, if nanohub would be reliable enough, could be used again.
// Currently it's losing some events, thus this part is not used at the moment.
	if (screen_on && diff < 45 && event!=last_event) {
		if (!squeeze_sleep) return;
		pr_info("%s screen on and latest event diff small enough: pwr on\n",__func__);
		last_timestamp = 0;
		fpf_pwrtrigger(0);
		return;
	}
#endif
	last_event = event;
	pr_info("%s latest nanohub/vib event processed. diff: %u\n",__func__,diff);
}
EXPORT_SYMBOL(register_squeeze_wake);


// ==================================
// ---------------fingerprint handler
// ==================================

static void fpf_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}


static const struct input_device_id fpf_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler fpf_input_handler = {
	.filter		= fpf_input_filter,
	.event		= fpf_input_event,
	.connect	= fpf_input_connect,
	.disconnect	= fpf_input_disconnect,
	.name		= "fpf_inputreq",
	.id_table	= fpf_ids,
};


/* check stored map of ts_current_X maps for matching values */
static bool check_ts_current_map(int type, int code, int value) {
	int i = 0;
	for (i=0; i<TS_MAP_SIZE; i++) {
		if (ts_current_type[i]==type && ts_current_code[i]==code && ts_current_value[i]==value) return true;
	}
	return false;
}

// ==================================
// ------------- touch screen handler
// ==================================

static int last_x = 0, last_y = 0;
static int c_x = 0, c_y = 0;
static unsigned long last_ts_timestamp = 0;

static bool ts_input_filter(struct input_handle *handle,
                                    unsigned int type, unsigned int code,
                                    int value)
{
#if 1
	//pr_info("%s ts input filter called t %d c %d v %d\n",__func__, type,code,value);
	if (type == EV_ABS && code == ABS_MT_TRACKING_ID && value!=-1) {
		last_mt_slot = value;
	}

	if (mutex_is_locked(&squeeze_swipe_lock)) {
		// in emulated swipe...block event that is not the event matching emulation event values...
		if (!check_ts_current_map(type,code,value)) {
			pr_info("%s ts_input filtering ts input while emulated scroll! %d %d %d\n",__func__,type,code,value);
			return true;
		}
	} else 
	{
		if (code == ABS_MT_POSITION_X) {
			c_x = value;
		}
		if (code == ABS_MT_POSITION_Y) {
			c_y = value;
		}
		if (type == EV_SYN) {
			unsigned int ts_ts_diff = jiffies - last_ts_timestamp;
			if (ts_ts_diff < 2) {
				if (abs(last_x-c_x)>abs(last_y-c_y)) {
					// X direction TODO
				} else {
					// Y direction
					if (last_y>c_y) { // swiping up
						if (squeeze_swipe_dir == 0) {
							last_scroll_emulate_timestamp = 0; // direction change, make the first scroll slow by putting this timestamp 0
							squeeze_swipe_dir = 1; // SCROLL DOWN
						}
					} else if (last_y < c_y) { // swiping down
						if (squeeze_swipe_dir == 1) {
							last_scroll_emulate_timestamp = 0; // direction change, make the first scroll slow by putting this timestamp 0
							squeeze_swipe_dir = 0; // SCROLL UP
						}
					}
				}
			}
			last_ts_timestamp = jiffies;
			last_x = c_x;
			last_y = c_y;
		}
	}

#endif
	if (screen_on_full) {
		squeeze_peek_wait = 0; // interrupt peek wait, touchscreen was interacted, don't turn screen off after peek time over...
	}
	return false;
}

static void ts_input_callback(struct work_struct *unused) {
	return;
}

static void ts_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value) {
}

static int ts_input_dev_filter(struct input_dev *dev) {
	if (
		strstr(dev->name, "himax-touchscreen") ||
		strstr(dev->name, "cyttsp")
	    ) {
		// storing static ts_device for using outside this handle context as well
		if (strstr(dev->name, "cyttsp")) ts_device = dev;
		return 0;
	} else {
		return 1;
	}
}


static int ts_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	if (ts_input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "fpf_ts";


	error = input_register_handle(handle);

	error = input_open_device(handle);

	return 0;

}

static void ts_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}


static const struct input_device_id ts_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler ts_input_handler = {
	.filter		= ts_input_filter,
	.event		= ts_input_event,
	.connect	= ts_input_connect,
	.disconnect	= ts_input_disconnect,
	.name		= "ts_inputreq",
	.id_table	= ts_ids,
};

// ------------------------------------------------------

static ssize_t fpf_dt_wait_period_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", doubletap_wait_period);
}

static ssize_t fpf_dt_wait_period_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > DT_WAIT_PERIOD_MAX)
		input = DT_WAIT_PERIOD_DEFAULT;

	doubletap_wait_period = input;
	return count;
}

static DEVICE_ATTR(fpf_dt_wait_period, (S_IWUSR|S_IRUGO),
	fpf_dt_wait_period_show, fpf_dt_wait_period_dump);


static ssize_t fpf_dt_wait_period_max_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", DT_WAIT_PERIOD_MAX);
}

static ssize_t fpf_dt_wait_period_max_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;
	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;
	return count;
}

static DEVICE_ATTR(fpf_dt_wait_period_max, (S_IWUSR|S_IRUGO),
	fpf_dt_wait_period_max_show, fpf_dt_wait_period_max_dump);


static ssize_t fpf_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", fpf_switch);
}

static ssize_t fpf_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 2)
		input = 0;				

	fpf_switch = input;			
	
	return count;
}

static DEVICE_ATTR(fpf, (S_IWUSR|S_IRUGO),
	fpf_show, fpf_dump);

static ssize_t vib_strength_show(struct device *dev,
		 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", vib_strength);
}

static ssize_t vib_strength_dump(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 90) 
		input = 20;				

	vib_strength = input;			
	
	return count;
}

static DEVICE_ATTR(vib_strength, (S_IWUSR|S_IRUGO),
	vib_strength_show, vib_strength_dump);

static ssize_t squeeze_sleep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", squeeze_sleep);
}

static ssize_t squeeze_sleep_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1)
		input = 0;				

	squeeze_sleep = input;			
	
	return count;
}

static DEVICE_ATTR(squeeze_sleep, (S_IWUSR|S_IRUGO),
	squeeze_sleep_show, squeeze_sleep_dump);

static ssize_t squeeze_wake_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", squeeze_wake);
}

static ssize_t squeeze_wake_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1)
		input = 0;				

	squeeze_wake = input;			
	
	return count;
}

static DEVICE_ATTR(squeeze_wake, (S_IWUSR|S_IRUGO),
	squeeze_wake_show, squeeze_wake_dump);


static ssize_t squeeze_max_power_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", squeeze_power_kernel_max_threshold);
}

static ssize_t squeeze_max_power_level_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 9)
		input = 0;

	squeeze_power_kernel_max_threshold = input;
	
	return count;
}

static DEVICE_ATTR(squeeze_max_power_level, (S_IWUSR|S_IRUGO),
	squeeze_max_power_level_show, squeeze_max_power_level_dump);

static ssize_t squeeze_peek_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", squeeze_peek);
}

static ssize_t squeeze_peek_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1)
		input = 0;				

	squeeze_peek = input;			
	
	return count;
}

static DEVICE_ATTR(squeeze_peek, (S_IWUSR|S_IRUGO),
	squeeze_peek_show, squeeze_peek_dump);

static ssize_t squeeze_peek_halfseconds_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", squeeze_peek_halfseconds);
}

static ssize_t squeeze_peek_halfseconds_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 6)
		input = 0;				

	squeeze_peek_halfseconds = input;			
	
	return count;
}

static DEVICE_ATTR(squeeze_peek_halfseconds, (S_IWUSR|S_IRUGO),
	squeeze_peek_halfseconds_show, squeeze_peek_halfseconds_dump);



static ssize_t squeeze_swipe_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", squeeze_swipe);
}

static ssize_t squeeze_swipe_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1)
		input = 0;				

	squeeze_swipe = input;			
	
	return count;
}

static DEVICE_ATTR(squeeze_swipe, (S_IWUSR|S_IRUGO),
	squeeze_swipe_show, squeeze_swipe_dump);

static ssize_t squeeze_swipe_vibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", squeeze_swipe_vibration);
}

static ssize_t squeeze_swipe_vibration_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1)
		input = 0;				

	squeeze_swipe_vibration = input;			
	
	return count;
}

static DEVICE_ATTR(squeeze_swipe_vibration, (S_IWUSR|S_IRUGO),
	squeeze_swipe_vibration_show, squeeze_swipe_vibration_dump);


static struct kobject *fpf_kobj;




#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    // catch early events as well, as this helps a lot correct functioning knowing when screen is almost off/on, preventing many problems 
    // interpreting still screen ON while it's almost off and vica versa
    if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK && fpf_pwrdev) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
		screen_on = 1;
		pr_info("fpf screen on -early\n");
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
		screen_on = 0;
		screen_on_full = 0;
		pr_info("fpf screen off -early\n");
            break;
        }
    }
    if (evdata && evdata->data && event == FB_EVENT_BLANK && fpf_pwrdev) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
		screen_on = 1;
		screen_on_full = 1;
		last_screen_event_timestamp = jiffies;
		pr_info("fpf screen on\n");
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
		screen_on = 0;
		screen_on_full = 0;
		last_screen_event_timestamp = jiffies;
		last_scroll_emulate_timestamp = 0;
		pr_info("fpf screen off\n");
            break;
        }
    }
    return 0;
}
#endif

static int __init fpf_init(void)
{
	int rc = 0;
	pr_info("fpf - init\n");

	fpf_pwrdev = input_allocate_device();
	if (!fpf_pwrdev) {
		pr_err("Failed to allocate fpf_pwrdev\n");
		goto err_alloc_dev;
	}

	input_set_capability(fpf_pwrdev, EV_KEY, KEY_POWER);
	input_set_capability(fpf_pwrdev, EV_KEY, KEY_HOME);
	
	set_bit(EV_KEY, fpf_pwrdev->evbit);
	set_bit(KEY_HOME, fpf_pwrdev->keybit);

	fpf_pwrdev->name = "qwerty";
	fpf_pwrdev->phys = "qwerty/input0";

	rc = input_register_device(fpf_pwrdev);
	if (rc) {
		pr_err("%s: input_register_device err=%d\n", __func__, rc);
		goto err_input_dev;
	}

	// fpf handler
	fpf_input_wq = create_workqueue("fpf_iwq");
	if (!fpf_input_wq) {
		pr_err("%s: Failed to create workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&fpf_input_work, fpf_input_callback);

	rc = input_register_handler(&fpf_input_handler);
	if (rc)
		pr_err("%s: Failed to register fpf_input_handler\n", __func__);

	// ts handler
	ts_input_wq = create_workqueue("ts_iwq");
	if (!ts_input_wq) {
		pr_err("%s: Failed to create workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&ts_input_work, ts_input_callback);

	rc = input_register_handler(&ts_input_handler);
	if (rc)
		pr_err("%s: Failed to register ts_input_handler\n", __func__);


#ifdef CONFIG_FB
	fb_notifier = kzalloc(sizeof(struct notifier_block), GFP_KERNEL);;
	fb_notifier->notifier_call = fb_notifier_callback;
	fb_register_client(fb_notifier);
#endif

	fpf_kobj = kobject_create_and_add("fpf", NULL) ;
	if (fpf_kobj == NULL) {
		pr_warn("%s: fpf_kobj failed\n", __func__);
	}

	rc = sysfs_create_file(fpf_kobj, &dev_attr_fpf.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for fpf\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_squeeze_wake.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for swake\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_squeeze_peek.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for speek\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_squeeze_peek_halfseconds.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for speek_halfs\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_squeeze_sleep.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for ssleep\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_squeeze_swipe.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for sswipe\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_squeeze_swipe_vibration.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for sswipevibr\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_squeeze_max_power_level.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for squeeze max pwr level\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_fpf_dt_wait_period.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for fpf_dt_wait_period\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_fpf_dt_wait_period_max.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for fpf_dt_wait_period_max\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_vib_strength.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for vib_strength\n", __func__);

err_input_dev:
	input_free_device(fpf_pwrdev);

err_alloc_dev:
	pr_info("%s fpf done\n", __func__);

	return 0;
}

static void __exit fpf_exit(void)
{
	kobject_del(fpf_kobj);
	input_unregister_handler(&fpf_input_handler);
	destroy_workqueue(fpf_input_wq);
	input_unregister_device(fpf_pwrdev);
	input_free_device(fpf_pwrdev);

	return;
}

module_init(fpf_init);
module_exit(fpf_exit);
