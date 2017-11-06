#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>

//file operation+
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/vmalloc.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/mm.h>
//file operation-


#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/alarmtimer.h>
#include <linux/uci/uci.h>

#define DRIVER_AUTHOR "illes pal <illespal@gmail.com>"
#define DRIVER_DESCRIPTION "uci driver"
#define DRIVER_VERSION "1.0"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

#ifdef CONFIG_FB
	struct notifier_block *uci_fb_notifier;
#endif

// file operations
int uci_fwrite(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

int uci_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

void uci_fclose(struct file* file) {
    filp_close(file, NULL);
}

struct file* uci_fopen(const char* path, int flags, int rights) {
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if(IS_ERR(filp)) {
        err = PTR_ERR(filp);
    pr_err("[uci]File Open Error:%s %d\n",path, err);
        return NULL;
    }
    if(!filp->f_op){
    pr_err("[uci]File Operation Method Error!!\n");
    return NULL;
    }

    return filp;
}

#define MAX_PARAMS 100
#define MAX_STR_LEN 100
#define MAX_FILE_SIZE 1000

char *user_cfg_keys[MAX_PARAMS];
char *user_cfg_values[MAX_PARAMS];

char *sys_cfg_keys[MAX_PARAMS];
char *sys_cfg_values[MAX_PARAMS];

static int should_not_parse_next_close = 0;

DEFINE_MUTEX(cfg_rw_lock);

static int get_user_in_progress = 0;
static int get_sys_in_progress = 0;

// Parsing
int parse_uci_cfg_file(const char *file_name, bool sys) {
//	fileread(file_name);

#if 1
	struct file*fp = NULL;
	fp=uci_fopen (file_name, O_RDONLY, 0);
	if (fp==NULL) {
		pr_info("%s [uci] cannot read file %s\n",__func__,file_name);
		return -1;
	} else {
		off_t fsize;
		char *buf;
		char *line;
		char *side;
		char *dupline;
		struct inode *inode;
		int line_num = 0;
		int token_num = 0;
		int prop_num = 0;

		char *l_cfg_keys[MAX_PARAMS];
		char *l_cfg_values[MAX_PARAMS];

		inode=fp->f_inode;
		fsize=inode->i_size;
		pr_info("%s [uci] file size %d...  %s\n",__func__,(int)fsize,file_name);
		if (fsize> MAX_FILE_SIZE) { 
			pr_err("uci file too big\n"); 
			return -1;
		}

		buf=(char *) kmalloc(fsize+1,GFP_KERNEL);
		uci_read(fp,0,buf,fsize);
		buf[fsize]='\0';

		while ((line = strsep(&buf, "\n")) != NULL) {
			pr_info("%s uci %s | %d\n",__func__, line, line_num);
			token_num = 0;
			while ((side = strsep(&line, "=")) != NULL) {
				dupline = kstrdup(side, GFP_KERNEL);
				if (token_num==0) {
					l_cfg_keys[prop_num] = (char*)dupline;
					pr_info("%s uci param key = %s | prop num: %d\n",__func__, dupline, prop_num);
				} else {
					l_cfg_values[prop_num] = (char*)dupline;
					pr_info("%s uci param value = %s | prop num: %d\n",__func__, dupline, prop_num);
					prop_num++;
				}
				token_num++;
			}
			line_num++;
			if (prop_num==MAX_PARAMS-1) break;
		}
		l_cfg_keys[prop_num] = NULL;
		l_cfg_values[prop_num] = NULL;

		pr_info("%s [uci] closing file...  %s\n",__func__,file_name);

		kfree(buf);

		should_not_parse_next_close = 1;
		uci_fclose(fp);
		msleep(10);

		should_not_parse_next_close = 0;
		while(sys && get_sys_in_progress) {
			msleep(1);
		}
		while(!sys && get_user_in_progress) {
			msleep(1);
		}
		mutex_lock(&cfg_rw_lock);
		if (sys) {
			int count = 0;
			while (sys_cfg_values[count]!=NULL) {
				kfree(sys_cfg_values[count++]);
			}
			count = 0;
			while (l_cfg_values[count]!=NULL) {
				sys_cfg_values[count] = l_cfg_values[count];
				count++;
			}
			for (;count<MAX_PARAMS;count++) sys_cfg_values[count] = NULL;

			count = 0;
			while (sys_cfg_keys[count]!=NULL) {
				kfree(sys_cfg_keys[count++]);
			}
			count = 0;
			while (l_cfg_keys[count]!=NULL) {
				sys_cfg_keys[count] = l_cfg_keys[count];
				count++;
			}
			sys_cfg_keys[count] = NULL;
			for (;count<MAX_PARAMS;count++) sys_cfg_keys[count] = NULL;

		} else {
			int count = 0;
			while (user_cfg_values[count]!=NULL) {
				kfree(user_cfg_values[count++]);
			}
			count = 0;
			while (l_cfg_values[count]!=NULL) {
				user_cfg_values[count] = l_cfg_values[count];
				count++;
			}
			for (;count<MAX_PARAMS;count++) user_cfg_values[count] = NULL;

			count = 0;
			while (user_cfg_keys[count]!=NULL) {
				kfree(user_cfg_keys[count++]);
			}
			count = 0;
			while (l_cfg_keys[count]!=NULL) {
				user_cfg_keys[count] = l_cfg_keys[count];
				count++;
			}
			for (;count<MAX_PARAMS;count++) user_cfg_keys[count] = NULL;
		}
		mutex_unlock(&cfg_rw_lock);
	}
	return 0;
#endif
}

bool is_uci_path(const char *file_name) {
	if (!file_name) return false;
	if (!strcmp(file_name, UCI_USER_FILE)) return true;
	if (!strcmp(file_name, UCI_SYS_FILE)) return true;
	return false;
}
EXPORT_SYMBOL(is_uci_path);

bool is_uci_file(const char *file_name) {
	if (!file_name) return false;
	if (!strcmp(file_name, UCI_USER_FILE_END)) return true;
	if (!strcmp(file_name, UCI_SYS_FILE_END)) return true;
	return false;
}
EXPORT_SYMBOL(is_uci_file);

static bool user_cfg_parsed = false;
static bool sys_cfg_parsed = false;

static bool should_parse_user = true;
static bool should_parse_sys = true;

void parse_uci_user_cfg_file(void) {
	int rc = parse_uci_cfg_file(UCI_USER_FILE,false);
	if (!rc) { user_cfg_parsed = true; should_parse_user = false; }
}
void parse_uci_sys_cfg_file(void) {
	int rc = parse_uci_cfg_file(UCI_SYS_FILE,true);
	if (!rc) { sys_cfg_parsed = true; should_parse_sys = false; }
}


// Properties

const char* uci_get_user_property_str(const char* property, const char* default_value) {
	const char* ret = NULL;
	pr_info("%s uci get user str prop %s\n",__func__,property);
	if (user_cfg_parsed) {
		int param_count = 0;
		while(get_user_in_progress) { // mutex lock is not possible - ___might_sleep causes Ooops in the context
			udelay(1);
		}
		get_user_in_progress = 1;
		while (mutex_is_locked(&cfg_rw_lock)) {
			udelay(1);
		}
		udelay(1);
		while(1) {
			const char *key = user_cfg_keys[param_count];
			pr_info("%s uci key... %s",__func__,key);
			if (key==NULL) break;
			if (!strcmp(property,key)) {
				pr_info("%s uci key %s -> value %s",__func__,key, ret);
				ret = user_cfg_values[param_count];
				get_user_in_progress = 0;
				return  ret;
			}
			param_count++;
		}
	}
	get_user_in_progress = 0;
	return default_value;
}
EXPORT_SYMBOL(uci_get_user_property_str);

int uci_get_user_property_int(const char* property, int default_value) {
	const char* str = uci_get_user_property_str(property, 0);
	long int ret = 0;
	if (!str) return default_value;
        if (kstrtol(str, 10, &ret) < 0)
                return -EINVAL;
	return (int)ret;
}
EXPORT_SYMBOL(uci_get_user_property_int);

int uci_get_user_property_int_mm(const char* property, int default_value, int min, int max) {
	int ret = uci_get_user_property_int(property, default_value);
	if (ret<min || ret>max) ret = default_value;
	pr_info("%s uci get user prop %d\n",__func__, ret);
	return ret;
}
EXPORT_SYMBOL(uci_get_user_property_int_mm);

const char* uci_get_sys_property_str(const char* property, const char* default_value) {
	const char* ret = NULL;
	pr_info("%s uci get sys str prop %s\n",__func__,property);
	if (sys_cfg_parsed) {
		int param_count = 0;
		while(get_sys_in_progress) { // mutex lock is not possible - ___might_sleep causes Ooops in the context
			udelay(1);
		}
		get_sys_in_progress = 1;
		while (mutex_is_locked(&cfg_rw_lock)) {
			udelay(1);
		}
		udelay(1);
		while(1) {
			const char *key = sys_cfg_keys[param_count];
			if (key==NULL) break;
			if (!strcmp(property,key)) {
				ret = sys_cfg_values[param_count];
				pr_info("%s uci key %s -> value %s",__func__,key, ret);
				get_sys_in_progress = 0;
				return  ret;
			}
			param_count++;
		}
	}
	get_sys_in_progress = 0;
	return default_value;
}
EXPORT_SYMBOL(uci_get_sys_property_str);

int uci_get_sys_property_int(const char* property, int default_value) {
	const char* str = uci_get_sys_property_str(property, 0);
	long int ret = 0;
	pr_info("%s uci str = %s\n",__func__,str?str:"NULL");
	if (!str) return default_value;
        if (kstrtol(str, 10, &ret) < 0)
                return -EINVAL;
	return (int)ret;
}
EXPORT_SYMBOL(uci_get_sys_property_int);

int uci_get_sys_property_int_mm(const char* property, int default_value, int min, int max) {
	int ret = uci_get_sys_property_int(property, default_value);
	if (ret<min || ret>max) ret = default_value;
	pr_info("%s uci get sys prop %d\n",__func__, ret);
	return ret;
}
EXPORT_SYMBOL(uci_get_sys_property_int_mm);

static bool first_parse_done = 0;

static void do_reschedule(void);

static void reschedule_work_func(struct work_struct * reschedule_work)
{
	do_reschedule();
}
static DECLARE_WORK(reschedule_work, reschedule_work_func);

static void parse_work_func(struct work_struct * parse_work_func_work)
{
	pr_info("%s uci \n",__func__);
	if (should_parse_user) parse_uci_user_cfg_file();
	if (should_parse_sys) parse_uci_sys_cfg_file();
	if (!first_parse_done) {
		if (user_cfg_parsed) {
			first_parse_done = true;
		} else {
			pr_info("%s uci reschedule till read first \n",__func__);
			schedule_work(&reschedule_work);
		}
	}
}
static DECLARE_DELAYED_WORK(parse_work_func_work, parse_work_func);
static void do_reschedule(void) {
	schedule_delayed_work(&parse_work_func_work, 3 * 100);
}
// alarm timer
static struct alarm parse_user_cfg_rtc;
static enum alarmtimer_restart parse_user_cfg_rtc_callback(struct alarm *al, ktime_t now)
{
	pr_info("%s uci alarm \n",__func__);
	schedule_delayed_work(&parse_work_func_work, 15 * 100);
	return ALARMTIMER_NORESTART;
}

static void start_alarm_parse(int sec) {
	ktime_t wakeup_time;
	ktime_t curr_time = { .tv64 = 0 };
	wakeup_time = ktime_add_us(curr_time,
	    (sec * 1000LL * 1000LL)); // 40 sec to msec to usec
	alarm_cancel(&parse_user_cfg_rtc);
	alarm_start_relative(&parse_user_cfg_rtc, wakeup_time); // start new...
}

void notify_uci_file_closed(const char *file_name) {
	if (should_not_parse_next_close) {
		pr_info("%s uci skipping for now\n",__func__);
		return;
	}
	if (!strcmp(file_name, UCI_USER_FILE_END)) should_parse_user = true;
	if (!strcmp(file_name, UCI_SYS_FILE_END)) should_parse_sys = true;
	schedule_delayed_work(&parse_work_func_work,1);
}
EXPORT_SYMBOL(notify_uci_file_closed);


#ifdef CONFIG_FB
static int first_unblank = 1;

static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK ) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
		pr_info("uci screen on -early\n");
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
		pr_info("uci screen off -early\n");
            break;
        }
    }
    if (evdata && evdata->data && event == FB_EVENT_BLANK ) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
		pr_info("uci screen on\n");
		if (first_unblank) {
			start_alarm_parse(80); // start in 40 sec, user cfg parse...
			first_unblank = 0;
		}
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
		pr_info("uci screen off\n");
		// start_alarm_parse(1); // TODO vibrate on change!
            break;
        }
    }
    return 0;
}
#endif

static int __init uci_init(void)
{
	int rc = 0;
	pr_info("uci - init\n");
#ifdef CONFIG_FB
	uci_fb_notifier = kzalloc(sizeof(struct notifier_block), GFP_KERNEL);;
	uci_fb_notifier->notifier_call = fb_notifier_callback;
	fb_register_client(uci_fb_notifier);
#endif
	alarm_init(&parse_user_cfg_rtc, ALARM_REALTIME,
		parse_user_cfg_rtc_callback);

	return rc;
}

static void __exit uci_exit(void)
{
	pr_info("uci - exit\n");
}

module_init(uci_init);
module_exit(uci_exit);

