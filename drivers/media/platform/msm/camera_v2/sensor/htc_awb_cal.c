/* arch/arm/mach-msm/htc_awb_cal.c */
/* Code to extract Camera AWB calibration information from ATAG
set up by the bootloader.

Copyright (C) 2008 Google, Inc.
Author: Dmitry Shmidt <dimitrysh@google.com>

This software is licensed under the terms of the GNU General Public
License version 2, as published by the Free Software Foundation, and
may be copied, distributed, and modified under those terms.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/setup.h>

/* for outputing file to filesystem : /data/awb_calibration_data_hboot.txt */
#include <linux/fs.h>
#include <linux/syscalls.h>

#include <linux/of.h>

//#define CAM_AWB_CAL_DEBUG

/* configuration tags specific to msm */
//#define ATAG_MSM_AWB_CAL	0x59504550 /* MSM CAMERA AWB Calibration */

#define AWB_CAL_MAX_SIZE	0x4000U     /* 0x1000 = 4096 bytes  0x2000 = 8192 bytes  0x3000 = 12288 bytes 0x4000 = 16384 bytes */
#define DUAL_CAL_SIZE 4095

/* HTC_START, use dt for calibration data */
#define CALIBRATION_DATA_PATH "/calibration_data"
#define CAM_AWB_CAL_DATA "cam_awb"

/* UEFI FLOW START For SDM630 */
// for sdm630, uefi api limit page only 2048
#define UEFI_FLOW
#define UEFI_PAGE_SIZE 0x800U
#define CAM_MAIN_CAL_PAGE_1 "cam_main_cal_page_1"
#define CAM_MAIN_CAL_PAGE_2 "cam_main_cal_page_2"
#define CAM_FRONT_CAL_PAGE_1 "cam_front_cal_page_1"
#define CAM_FRONT_CAL_PAGE_2 "cam_front_cal_page_2"

enum HTC_UEFI_CAL_DATA_TYPE {
        MAIN_CAL_PAGE,
        FRONT_CAL_PAGE,
};
/* UEFI FLOW END */

struct qct_lsc_struct{
	unsigned long int	lsc_verify;
	unsigned long int	lsc_fuseid[4];
	float 			pCalcParam[17*13*4];
	unsigned long int	lsc_checksum;
};
struct gyro_struct{
	unsigned long int gyro_caBuff[1];
	unsigned long int fuse_id[4];
	unsigned long int check_sum[1];
};

struct qct_awb_lsc_struct{
	unsigned long int caBuff[8];/* AWB Calibartion */
	struct qct_lsc_struct qct_lsc_data;/* LSC Calibration */
	/* Andrew_Cheng 20120223 For Flash_Camera MB */
	unsigned long int flashcaBuff[8];  //flash_camera
	/* Andrew_Cheng 20120223 For Flash_Camera ME */
	unsigned long int aec_caBuff[9]; // AEC calibration data
	unsigned long int alight_caBuff[8]; // A-light AWB calibration data
	unsigned long int dualflashcaBuff[12];  //dual flash calibration
// HTC_START, For new AWB check
	unsigned long int awb_verify;
// HTC_END, For new AWB check
/*HTC_START for gyro offset calibration*/
    struct gyro_struct gyro_data;
/*HTC_END*/
};

static unsigned char cam_awb_ram[AWB_CAL_MAX_SIZE];

int gCAM_AWB_CAL_LEN;

/* HTC_START */
/* klocwork */
unsigned char *dummy(unsigned char *p)
{
    return p;
}
/* HTC_END */

/* HTC_START, use dt for calibration data */
unsigned char *get_cam_awb_cal( void )
{
     struct device_node *offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
     int p_size;
     unsigned char *p_data;
#ifdef CAM_AWB_CAL_DEBUG
     unsigned int i;
#endif

     p_size = 0;
     p_data = NULL;
     if (offset) {
          /* of_get_property will return address of property, and fill the length to *p_size */
          p_data = (unsigned char*) of_get_property(offset, CAM_AWB_CAL_DATA, &p_size);
#ifdef CAM_AWB_CAL_DEBUG
          if (p_data) {
		  printk("[CAM]size = %d ", p_size);
               for (i = 0; i < p_size; ++i)
                   printk("%02x ", p_data[i]);
          }
#endif
     }
        if (p_data != NULL) {
		gCAM_AWB_CAL_LEN = p_size;
            memcpy(cam_awb_ram, p_data, p_size);
        }

	return( cam_awb_ram );
}
EXPORT_SYMBOL(get_cam_awb_cal);

unsigned char *get_uefi_cam_awb_cal(int data_page)
{
     struct device_node *offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
     static unsigned char cam_cal_page_data[UEFI_PAGE_SIZE*2]; // p1+p2
     unsigned char *p1_data = NULL;
     unsigned char *p2_data = NULL;
     int p1_size = 0;
     int p2_size = 0;

     memset(cam_cal_page_data, 0, sizeof(cam_cal_page_data));

     if (offset) {
          /* of_get_property will return address of property, and fill the length to *p_size */
         switch (data_page) {
            case MAIN_CAL_PAGE:
                 p1_data = (unsigned char*) of_get_property(offset, CAM_MAIN_CAL_PAGE_1, &p1_size);
                 p2_data = (unsigned char*) of_get_property(offset, CAM_MAIN_CAL_PAGE_2, &p2_size);
                    break;
            case FRONT_CAL_PAGE:
                 p1_data = (unsigned char*) of_get_property(offset, CAM_FRONT_CAL_PAGE_1, &p1_size);
                 p2_data = (unsigned char*) of_get_property(offset, CAM_FRONT_CAL_PAGE_2, &p2_size);
                    break;
            default:
                 printk(KERN_INFO "[CAM]get_uefi_cam_awb_cal: failed, page %d\n", data_page);
                    break;
         }
     }

     if ((p1_size + p2_size) > UEFI_PAGE_SIZE * 2) {
         printk(KERN_INFO "[CAM]get_cam_awb_cal: size overflow, (p1 %d + p2 %d) > %d\n", p1_size, p2_size, UEFI_PAGE_SIZE*2);
         return cam_cal_page_data;
     }

     if (p1_data != NULL) {
         memcpy(cam_cal_page_data, p1_data, p1_size);
     }

     if (p2_data != NULL) {
         memcpy(cam_cal_page_data + UEFI_PAGE_SIZE, p2_data, p2_size);
     }

     return cam_cal_page_data;
}
EXPORT_SYMBOL(get_uefi_cam_awb_cal);
/* HTC_END */

static ssize_t awb_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char *ptr;

#ifndef UEFI_FLOW
	ptr = get_cam_awb_cal();
#else
        ptr = get_uefi_cam_awb_cal(MAIN_CAL_PAGE);
#endif

	/* fixed : workaround because of defined 8 parameters now */

	ret = sizeof(struct qct_awb_lsc_struct);/* 8*4; */
	memcpy(buf, ptr, ret);

#ifdef CAM_AWB_CAL_DEBUG
   {
	 int i, *pint;
	 printk(KERN_INFO "[CAM]awb_calibration_show():\n");
	 pint = (int *)buf;
	 for (i = 0; i < 914; i++)
	   printk(KERN_INFO "%d-%x\n", i, pint[i]);

   }
#endif

	return ret;
}

static ssize_t awb_calibration_front_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char *ptr;

#ifndef UEFI_FLOW
        ptr = get_cam_awb_cal();
#else
        ptr = get_uefi_cam_awb_cal(FRONT_CAL_PAGE);
#endif
	/* fixed : workaround because of defined 8 parameters now */

	ret = sizeof(struct qct_awb_lsc_struct);/* 8*4; */
	printk(KERN_INFO "[CAM]awb_calibration_front_show(%d)\n", ret);
#ifndef UEFI_FLOW
        memcpy(buf, ptr + 0x1000U, ret);
#else
        memcpy(buf, ptr, ret);
#endif


#ifdef CAM_AWB_CAL_DEBUG
   {
	 int i, *pint;
	 printk(KERN_INFO "[CAM]awb_calibration_front_show():\n");
	 pint = (int *)buf;
	 for (i = 0; i < 898; i++)
	   printk(KERN_INFO "%x\n", pint[i]);

   }
#endif

	return ret;
}

static ssize_t awb_calibration_sub_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char *ptr;

	ptr = get_cam_awb_cal();
	/* fixed : workaround because of defined 8 parameters now */

	ret = sizeof(struct qct_awb_lsc_struct);/* 8*4; */
	printk(KERN_INFO "[CAM]awb_calibration_sub_show(%d)\n", ret);
	memcpy(buf, ptr + 0x2000U, ret);


#ifdef CAM_AWB_CAL_DEBUG
   {
	 int i, *pint;
	 printk(KERN_INFO "[CAM]awb_calibration_sub_show():\n");
	 pint = (int *)buf;
	 for (i = 0; i < 898; i++)
	   printk(KERN_INFO "%x\n", pint[i]);

   }
#endif

	return ret;
}


static ssize_t awb_calibration_3D_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = DUAL_CAL_SIZE;
	unsigned char *ptr;

	ptr = get_cam_awb_cal();
	/* fixed : workaround because of defined 8 parameters now */

	memcpy(buf, ptr + 0x3000U, ret);


#ifdef CAM_AWB_CAL_DEBUG
   {
	 int i, *pint;
	 printk(KERN_INFO "[CAM]awb_calibration_3D_show():\n");
	 pint = (int *)buf;
	 for (i = 0; i < DUAL_CAL_SIZE; i++)
	   printk(KERN_INFO "%x\n", pint[i]);

   }
#endif

	return ret;
}

static DEVICE_ATTR(awb_cal, 0444, awb_calibration_show, NULL);
static DEVICE_ATTR(awb_cal_front, 0444, awb_calibration_front_show, NULL);
static DEVICE_ATTR(awb_cal_sub, 0444, awb_calibration_sub_show, NULL);
static DEVICE_ATTR(awb_cal_3D, 0444, awb_calibration_3D_show, NULL);


static struct kobject *cam_awb_cal;

static int cam_get_awb_cal(void)
{
	int ret ;

	/* Create /sys/android_camera_awb_cal/awb_cal */
	cam_awb_cal = kobject_create_and_add("android_camera_awb_cal", NULL);
	if (cam_awb_cal == NULL) {
		pr_info("[CAM]cam_get_awb_cal: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret ;
	}

   /* dev_attr_[register_name]<== DEVICE_ATTR(awb_cal, 0444,
   awb_calibration_show, NULL); */
	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal.attr);
	if (ret) {
		pr_info("[CAM]cam_get_awb_cal:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
		goto end;
	}


	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_front.attr);
	if (ret) {
		pr_info("[CAM]cam_get_awb_cal_front:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
		goto end;
	}

	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_sub.attr);
	if (ret) {
		pr_info("[CAM]cam_get_awb_cal_sub:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
		goto end;
	}

	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_3D.attr);
	if (ret) {
		pr_info("[CAM]cam_get_awb_cal_3D:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
		goto end;
	}

end:
	return 0 ;
}

late_initcall(cam_get_awb_cal);
