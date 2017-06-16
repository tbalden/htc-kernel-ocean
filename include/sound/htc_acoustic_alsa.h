/*
 *
 * Copyright (C) 2012 HTC Corporation.
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
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/qdsp6v2/apr.h>
#include <sound/htc_audio_ioctl.h>
#include <linux/device.h>
#include <sound/apr_audio-v2.h>

#ifndef _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_QCT_ALSA_H_
#define _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_QCT_ALSA_H_

#define pr_aud_fmt(fmt) "[AUD] " KBUILD_MODNAME ": " fmt
#define pr_aud_fmt1(fmt) "[AUD]" fmt
#define pr_aud_err(fmt, ...) \
			printk(KERN_ERR pr_aud_fmt(fmt), ##__VA_ARGS__)
#define pr_aud_err1(fmt, ...) \
			printk(KERN_ERR pr_aud_fmt1(fmt), ##__VA_ARGS__)
#define pr_aud_info(fmt, ...) \
			printk(KERN_INFO pr_aud_fmt(fmt), ##__VA_ARGS__)
#define pr_aud_info1(fmt, ...) \
			printk(KERN_INFO pr_aud_fmt1(fmt), ##__VA_ARGS__)

#if defined(CONFIG_DYNAMIC_DEBUG)
#define pr_aud_debug(fmt, ...) \
	dynamic_pr_debug(pr_aud_fmt(fmt), ##__VA_ARGS__)
#elif defined(DEBUG)
#define pr_aud_debug(fmt, ...) \
	printk(KERN_DEBUG pr_aud_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_aud_debug(fmt, ...) \
	no_printk(KERN_DEBUG pr_aud_fmt(fmt), ##__VA_ARGS__)
#endif

#undef pr_debug
#undef pr_info
#undef pr_err
#define pr_debug(fmt, ...) pr_aud_debug(fmt, ##__VA_ARGS__)
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)

#define ADM_MODULE_ONEDOTONE_AUDIO       0x10000135
#define ADM_MODULE_LIMITERCOPP           0x1000012A
#define ADM_MODULE_ID_MISC_EFFECT        0x10030001
#define ADM_COPP_ID_ONEDOTONE_AUDIO           0x10000009
#define ADM_COPP_ID_HTC_HD_AUDIO              0x10000010
#define ADM_COPP_ID_HTC_USB_AUDIO              0x10000011
#define ADM_COPP_ID_ADAPTIVE_AUDIO            0x10000004
#define ADM_PARAM_ID_ONEDOTONE_AUDIO_EN       0x10000137
#define ADM_PARAM_ID_LIMITERCOPP_AUDIO_ENABLE 0x1000012C
#define ADM_PARAM_ID_MISC_SET_STEREO_TO_MONO 0x10030081
#define ADM_PARAM_ID_MISC_SET_ACOUSTIC_SHOCK_RAMP 0x10030101
#define ADM_PARAM_ID_MISC_SET_ACOUSTIC_SHOCK_MUTE 0x10030111

#define AFE_MODULE_ID_ADAPTSOUND_EFFECT          0x10000040
#define AFE_MODULE_ID_ADAPTSOUND_LIMITER         0x1000012A

#define ADM_COPP_ID_AUDIOZOOM                    0x10000008
#define ADM_MODULE_ID_AUDIOZOOM_EFFECT           0x10000047
#define ADM_PARAM_ID_AUDIOZOOM_PARAM             0x10000056

#define AFE_PARAM_ID_ADAPTSOUND_CONF_LEFT_RE     0x10000043
#define AFE_PARAM_ID_ADAPTSOUND_CONF_LEFT_IM     0x10000044
#define AFE_PARAM_ID_ADAPTSOUND_CONF_RIGHT_RE    0x10000045
#define AFE_PARAM_ID_ADAPTSOUND_CONF_RIGHT_IM    0x10000046
#define AFE_PARAM_ID_ADAPTSOUND_CONF_LIMITER     0x1000012D
#define AFE_PARAM_ID_ADAPTSOUND_EFFECT_ENABLE    0x10000042
#define AFE_PARAM_ID_ADAPTSOUND_LIMITER_ENABLE   0x1000012C

#define AFE_COPP_ID_ADAPTIVE_AUDIO_20            0x10000007

#ifdef CONFIG_HTC_HEADSET_MGR
enum HS_NOTIFY_TYPE {
	HS_AMP_N = 0,
	HS_CODEC_N,
	HS_N_MAX,
};

struct hs_notify_t {
	int used;
	void *private_data;
	int (*callback_f)(void*,int);
};
#endif

enum HTC_FEATURE {
	HTC_Q6_EFFECT = 0,
	HTC_AUD_24BIT,
};

struct avcs_crash_params {
    struct apr_hdr  hdr;
    uint32_t crash_type;
};

struct acoustic_ops {
	void (*set_q6_effect)(int mode);
	int (*get_htc_revision)(void);
	int (*get_hw_component)(void);
	char* (*get_mid)(void);
	int (*enable_digital_mic)(void);
	int (*enable_24b_audio)(void);
	int (*get_q6_effect) (void);
	/* HTC_AUD_START - AS HS 2.0 */
	int (*msm_setparam)(htc_adsp_params_ioctl_t *ctrl);
	int (*get_headsetType)(void);
	/* HTC_AUD_END */
};

void htc_acoustic_register_ops(struct acoustic_ops *ops);

#ifdef CONFIG_HTC_HEADSET_MGR
void htc_acoustic_register_hs_notify(enum HS_NOTIFY_TYPE type, struct hs_notify_t *notify);
#endif

/* To query if feature is enable */
int htc_acoustic_query_feature(enum HTC_FEATURE feature);

#endif

enum htc_effect {
	HTC_ADM_EFFECT_ONEDOTONE = 0,
	HTC_ADM_EFFECT_ONEDOTONE_MUTE,
	HTC_ADM_EFFECT_ONEDOTONE_RAMPING,
	HTC_ADM_EFFECT_ONEDOTONE_LIMITER,
	HTC_ADM_EFFECT_HD_STEREO_MONO,
	HTC_ADM_EFFECT_HD_STEREO_MONO_441,
	HTC_ADM_EFFECT_HD_STEREO_MONO_USB,
	HTC_ADM_EFFECT_USB_STEREO_MONO_USB,
	HTC_ADM_EFFECT_AS_STEREO_MONO,
	HTC_ADM_EFFECT_AS_STEREO_MONO_441,
	HTC_ADM_EFFECT_AS_DATA1,/* as_conf_left_im */
	HTC_ADM_EFFECT_AS_DATA2,/* as_conf_left_re */
	HTC_ADM_EFFECT_AS_DATA3,/* as_conf_right_im */
	HTC_ADM_EFFECT_AS_DATA4,/* as_conf_right_re */
	HTC_ADM_EFFECT_AS_DATA5,/* as_limiter_conf */
	HTC_ADM_EFFECT_AS_DATA6,/* as_limiter_enable */
	HTC_ADM_EFFECT_AS_DATA7,/* as_enable */
	HTC_ADM_EFFECT_AS_DATA1_441,/* as_conf_left_im */
	HTC_ADM_EFFECT_AS_DATA2_441,/* as_conf_left_re */
	HTC_ADM_EFFECT_AS_DATA3_441,/* as_conf_right_im */
	HTC_ADM_EFFECT_AS_DATA4_441,/* as_conf_right_re */
	HTC_ADM_EFFECT_AS_DATA5_441,/* as_limiter_conf */
	HTC_ADM_EFFECT_AS_DATA6_441,/* as_limiter_enable */
	HTC_ADM_EFFECT_AS_DATA7_441,/* as_enable */
	HTC_ADM_EFFECT_AUDIOZOOM,
	HTC_ADM_EFFECT_MAX,
};

struct htc_effect_index_s {
	int topology_id;
	u16 port_id;
};

static const struct htc_effect_index_s htc_effect_index[HTC_ADM_EFFECT_MAX] = {
	{ADM_COPP_ID_ONEDOTONE_AUDIO, AFE_PORT_ID_QUATERNARY_MI2S_RX}, /* HTC_ADM_EFFECT_ONEDOTONE */
	{ADM_COPP_ID_ONEDOTONE_AUDIO, AFE_PORT_ID_QUATERNARY_MI2S_RX}, /* HTC_ADM_EFFECT_ONEDOTONE_MUTE */
	{ADM_COPP_ID_ONEDOTONE_AUDIO, AFE_PORT_ID_QUATERNARY_MI2S_RX}, /* HTC_ADM_EFFECT_ONEDOTONE_RAMPING */
	{ADM_COPP_ID_ONEDOTONE_AUDIO, AFE_PORT_ID_QUATERNARY_MI2S_RX}, /* HTC_ADM_EFFECT_ONEDOTONE_LIMITER */
	{ADM_COPP_ID_HTC_HD_AUDIO, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX}, /* HTC_ADM_EFFECT_HD_STEREO_MONO */
	{ADM_COPP_ID_HTC_HD_AUDIO, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX}, /* HTC_ADM_EFFECT_HD_STEREO_MONO_441 */
	{ADM_COPP_ID_HTC_HD_AUDIO, AFE_PORT_ID_USB_RX}, /* HTC_ADM_EFFECT_HD_STEREO_MONO_USB */
	{ADM_COPP_ID_HTC_USB_AUDIO, AFE_PORT_ID_USB_RX}, /* HTC_ADM_EFFECT_USB_STEREO_MONO_USB */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX}, /* HTC_ADM_EFFECT_AS_STEREO_MONO */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX}, /* HTC_ADM_EFFECT_AS_STEREO_MONO_441 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX}, /* HTC_ADM_EFFECT_AS_DATA1 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX}, /* HTC_ADM_EFFECT_AS_DATA2 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX}, /* HTC_ADM_EFFECT_AS_DATA3 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX}, /* HTC_ADM_EFFECT_AS_DATA4 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX}, /* HTC_ADM_EFFECT_AS_DATA5 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX}, /* HTC_ADM_EFFECT_AS_DATA6 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_6_RX}, /* HTC_ADM_EFFECT_AS_DATA7 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX}, /* HTC_ADM_EFFECT_AS_DATA1_441 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX}, /* HTC_ADM_EFFECT_AS_DATA2_441 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX}, /* HTC_ADM_EFFECT_AS_DATA3_441 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX}, /* HTC_ADM_EFFECT_AS_DATA4_441 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX}, /* HTC_ADM_EFFECT_AS_DATA5_441 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX}, /* HTC_ADM_EFFECT_AS_DATA6_441 */
	{AFE_COPP_ID_ADAPTIVE_AUDIO_20, AFE_PORT_ID_SLIMBUS_MULTI_CHAN_5_RX}, /* HTC_ADM_EFFECT_AS_DATA7_441 */
	{ADM_COPP_ID_AUDIOZOOM, SLIMBUS_0_TX}, /* HTC_ADM_EFFECT_AUDIO_ZOOM */
};


int htc_adm_set_payload(int effect_id, uint32_t payload_size, void *payload);
void* htc_adm_get_payload(int effect_id);
