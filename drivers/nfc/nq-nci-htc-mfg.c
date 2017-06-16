#include <linux/delay.h>
#include <linux/string.h>
#include <linux/of.h>
#include "nq-nci-htc-mfg.h"
#if NFC_GET_BOOTMODE
#include <linux/htc_flags.h>
#endif //NFC_GET_BOOTMODE
#define WATCHDOG_FTM_TIMEOUT_SEC 20

#define D(x...)	\
	if (1)	\
		printk(KERN_DEBUG "[NFC] " x)
#define I(x...) printk(KERN_INFO "[NFC] " x)
#define E(x...) printk(KERN_ERR "[NFC] [Err] " x)
#define MAX_BUFFER_SIZE	512




unsigned long watchdog_counter;
unsigned int watchdogEn = 1;
unsigned int watchdog_timeout = WATCHDOG_FTM_TIMEOUT_SEC; //set default timeout value
int mfc_nfc_cmd_result = 0;
char  NCI_TMP[MAX_BUFFER_SIZE];
#if FTM_NFC_CPLC
char  RF_PARAM_CE_eSE[MAX_BUFFER_SIZE];
char  CPLC[MAX_BUFFER_SIZE];
#endif

extern void pn544_Enable(void);
extern void pn544_Disable(void);
extern int pn544_irq_gpio(void);
extern int pn544_TxData(uint8_t *txData, int length);
extern int pn544_RxData(uint8_t *rxData, int length);


typedef struct RF_entry_def {
	unsigned int RF_ID;
	unsigned int RF_Protocol;
	unsigned int RF_Technology;
} RF_Entry;

typedef struct control_msg_pack_def {
	uint8_t cmd[MAX_CMD_LEN];
	uint8_t exp_resp_content[MAX_CMD_LEN];
	uint8_t exp_ntf[MAX_CMD_LEN];
} control_msg_pack;

struct device_info_def {
	unsigned int padding_exist;
	unsigned char padding;
	unsigned long fwVersion;
	unsigned int HW_model;
	unsigned int NCI_version;
	unsigned long NFCC_Features;
	unsigned char manufactor;

	unsigned char FW_Major;
	unsigned char FW_Minor;

	unsigned int protocol_set;
	unsigned int intf_set;
	unsigned int target_rf_id;
	unsigned int activated_INTF;
	unsigned int NTF_count;
	RF_Entry NTF_queue[15];
} gDevice_info;


control_msg_pack nfc_dpc_script[] = {
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 5, 0x2F, 0x3D, 0x02, 0x01, 0x80 },
		{ 1, 0x00 },
		{ 0 },
	},
};

control_msg_pack nfc_version_script[] = {
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
};

control_msg_pack select_rf_target[] = {
	{
		{ 6, 0x21, 0x04, 0x03, 0x01, 0x04, 0x02 },
		{ 1, 0x00 },
		{ 0 },
	},
};

control_msg_pack nfc_card_script[] ={
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	{
		{ 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x03, 0x01, 0x08 },
		{ 0 },
		{ 0 },
	},
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x00 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	{
		{ 15, 0x21, 0x01, 0x0C, 0x00, 0x02, 0x00, 0x03, 0x00, 0x05, 0x01, 0x01, 0x03, 0x00, 0x01, 0x04 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 12, 0x21, 0x03, 0x09, 0x04, 0x81, 0x01, 0x82, 0x01, 0x83, 0x01, 0x85, 0x01 },
		{ 1, 0x00 },
		{ 1, 0x01 }
	}
};

control_msg_pack nfc_card_pll_script[] ={
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	{
		{ 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x03, 0x01, 0x11 },
		{ 0 },
		{ 0 },
	},
        {
                { 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x04, 0x01, 0x01 },
                { 2, 0x00, 0x00 },
                { 0 },
        },
        {
                { 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x02, 0x01, 0x01 },
                { 2, 0x00, 0x00 },
                { 0 },
        },
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x00 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	{
		{ 15, 0x21, 0x01, 0x0C, 0x00, 0x02, 0x00, 0x03, 0x00, 0x05, 0x01, 0x01, 0x03, 0x00, 0x01, 0x04 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 12, 0x21, 0x03, 0x09, 0x04, 0x81, 0x01, 0x82, 0x01, 0x83, 0x01, 0x85, 0x01 },
		{ 1, 0x00 },
		{ 1, 0x01 }
	}
};

control_msg_pack nfc_standby_enble_script[] = {
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		/*Enable Standby mode*/
		{ 4, 0x2F, 0x00, 0x01, 0x01 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		/*Set the Internal VEN to VEN = 0 state*/
		{ 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x07, 0x01, 0x02 },
		{ 1, 0x00 },
		{ 0 },
	},
};

control_msg_pack nfc_reader_script[] ={
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	{
		{ 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x03, 0x01, 0x08 },
		{ 2, 0x00, 0x00 },
		{ 0 },
	},
	{   /*NXP_CORE_CONF_EXTN*/
		{ 0x0A,
        0x20, 0x02, 0x07, 0x01, 
        0xA0, 0x0E, 0x03, 0x56, 0x24, 0x0A
        },
		{ 0 },
		{ 0 },
	},
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x00 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	/*{
		{ 13, 0x20, 0x02, 0x0A, 0x01, 0xA0, 0x4E, 0x06, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
		{ 2, 0x00, 0x00 },
		{ 0 },
	},*/
	{
		{ 7, 0x21, 0x00, 0x04, 0x01, 0x04, 0x01, 0x02 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 8, 0x21, 0x03, 0x05, 0x02, 0x00, 0x01, 0x01, 0x01 },
		{ 1, 0x00 },
		{ 1, 0x01 }
	},
};

control_msg_pack nfc_reader_pll_script[] ={
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	{
		{ 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x03, 0x01, 0x11 },
		{ 2, 0x00, 0x00 },
		{ 0 },
	},
        {
                { 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x04, 0x01, 0x01 },
                { 2, 0x00, 0x00 },
                { 0 },
        },
        {
                { 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x02, 0x01, 0x01 },
                { 2, 0x00, 0x00 },
                { 0 },
        },
	{   /*NXP_CORE_CONF_EXTN*/
		{ 0x0A,
        0x20, 0x02, 0x07, 0x01, 
        0xA0, 0x0E, 0x03, 0x56, 0x24, 0x0A
        },
		{ 0 },
		{ 0 },
	},
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x00 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	/*{
		{ 13, 0x20, 0x02, 0x0A, 0x01, 0xA0, 0x4E, 0x06, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
		{ 2, 0x00, 0x00 },
		{ 0 },
	},*/
	{
		{ 7, 0x21, 0x00, 0x04, 0x01, 0x04, 0x01, 0x02 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 8, 0x21, 0x03, 0x05, 0x02, 0x00, 0x01, 0x01, 0x01 },
		{ 1, 0x00 },
		{ 1, 0x01 }
	},
};

#if 0
static control_msg_pack nfc_off_mode_charging_enble_script[] = {
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x00 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		/*NFCEE_MODE_SET*/
		{ 5, 0x22, 0x01, 0x02, 0x02, 0x01},
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 4, 0x2F, 0x15, 0x01, 0x01 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 0x0A, 0x21,0x03,0x07,0x03,0x80,0x01,0x81,0x01,0x82,0x01},
		{ 1, 0x00 },
		{ 0 },
	},
};
#endif

#if FTM_NFC_CPLC
control_msg_pack nfc_cplc_part1_script[] ={
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x00 }, //CORE_RESET_CMD Keep Configuration
		.exp_resp_content = { 1, 0x00 }, //4000 XX "00"XXXX Status OK
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 }, //CORE_INIT_CMD
		{ 1, 0x00 }, //4001 XX "00"XXXX Status OK
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 }, //PROPRIETARY_ACT_CMD
		{ 1, 0x00 }, //4F02 XX "00"XXXX Status OK
		{ 0 },
	},
	{
		{ 4, 0x2F, 0x00, 0x01, 0x00 }, //CORE_SET_POWER_MODE_CMD  Standby Mode disabled
		{ 1, 0x00 }, //4F00 XX "00"XXXX Status OK
		{ 0 },
	},
	{
		{ 16, 0x20, 0x02, 0x0D, 0x03, 0xA0, 0xEC, 0x01, 0x00, 0xA0, 0xED, 0x01, 0x01, 0xA0, 0x12, 0x01, 0x02},
		{ 2, 0x00, 0x00 },
		{ 0 },
	},
	{
		.cmd = { 4, 0x22, 0x00, 0x01, 0x01 },//NFCEE_DISCOVER_NTF enable discovery of NFCEE
		.exp_resp_content = { 2, 0x00, 0x01 },
		.exp_ntf = { 0 },
	},
	{
		{ 9, 0x20, 0x04, 0x06, 0x03, 0x01, 0x01, 0x02, 0x01, 0x01},//Create Dynamic Logical connection to HCI network
		{ 4, 0x00, 0xFF, 0x01, 0x03 },
		{ 0 },
	},
	{
		{ 5, 0x22, 0x01, 0x02, 0x01, 0x01 },//Enable HCI network
		{ 1, 0x00},
		{ 2, 0xC0, 0x00 },
	},
	{
		{ 5, 0x03, 0x00, 0x02, 0x81, 0x03 },//HCI Open Pipe Admin
		{ 2, 0x81, 0x80},
		{ 0 },
		/*{ 6, 0x01, 0x00, 0x03, 0xC0, 0x80, 0x04 },*/
	},
	{
		{ 6, 0x03, 0x00, 0x03, 0x81, 0x02, 0x04  },//HCI Get Parameter "HOST_LIST"
		{ 4, 0x81, 0x80, 0x00, 0xC0},
		{ 0 },
	},
	{
		{ 7, 0x03, 0x00, 0x04, 0x81, 0x01, 0x03, 0xC0   },//HCI Set Parameter "WHITE_LIST" with Host ID C0
		{ 2, 0x81, 0x80},
		{ 0 },
	},

};


control_msg_pack nfc_cplc_part2_1_script[] ={
	{
		.cmd = { 6, 0x03, 0x00, 0x03, 0x99, 0x02, 0x01  },
		.exp_resp_content = { 1, 0x99 },
		.exp_ntf = { 0 },
	},
};

control_msg_pack nfc_cplc_part2_2_script[] ={
	{
		.cmd = { 10, 0x03, 0x00, 0x07, 0x99, 0x50, 0x80, 0xCA, 0x9F, 0x7F, 0x00 },
		.exp_resp_content = { 1, 0x99 },
		.exp_ntf = { 0 },
	},
};

control_msg_pack nfc_cplc_part2_3_script[] ={
	{
		.cmd = { 6, 0x20, 0x03, 0x03, 0x01, 0xA0, 0xF0 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
};

#endif //FTM_NFC_CPLC



static control_msg_pack nfc_off_mode_charging_enble_script[] = {
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x00 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		/*NFCEE_MODE_SET*/
		{ 5, 0x22, 0x01, 0x02, 0x02, 0x01},
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 4, 0x2F, 0x15, 0x01, 0x01 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 0x0A, 0x21,0x03,0x07,0x03,0x80,0x01,0x81,0x01,0x82,0x01},
		{ 1, 0x00 },
		{ 0 },
	},
};


void pn544_hw_reset_control(int en_num)
{
	switch(en_num){
	case 0:
		pn544_Enable();
		msleep(50);
		pn544_Disable();
		msleep(50);
		pn544_Enable();
		msleep(50);
		pn544_Disable();
		msleep(50);
		break;
	case 1:
	default:
		pn544_Enable();
		msleep(50);
		pn544_Disable();
		msleep(50);
		pn544_Enable();
		msleep(50);
	}
}

void nfc_nci_dump_data(unsigned char *data, int len) {
	int i = 0, j = 0;
	memset(NCI_TMP, 0x00, MAX_BUFFER_SIZE);
	for (i = 0, j = 0; i < len; i++)
		j += snprintf(NCI_TMP + j, MAX_BUFFER_SIZE*2 , " 0x%02X", data[i]);
	I("%s\r\n", NCI_TMP);
}



//	nci_reader return conditions:
//	-255: i2c error, break
//	-1: script request condition not reached, stay at current
//	 0: script restart
//	 1: script request condition reached
//	 other n: go to scriptIndex n


int nci_Reader(control_msg_pack *script, unsigned int scriptSize) {
	static control_msg_pack *previous = 0;
	static int res_achieved = 0;
	static int ntf_achieved = 0;
	static char expect_resp_header[2] = {0};
	static char expect_ntf_header[2] = {0};
	uint8_t receiverBuffer[MAX_NFC_DATA_SIZE] ={0};
	char nci_read_header[2] = {0};
	unsigned char nci_data_len = 0;
	unsigned int GOID = 0;
	int rf_support_len = 0;


	if (previous != script) {
		I("new command, reset flags.\r\n");
		previous = script;
		res_achieved = 0;
		ntf_achieved = 0;
		if (script->exp_resp_content != 0) {
			if (0x20 == (script->cmd[1] & 0xF0))
				expect_resp_header[0] = script->cmd[1] + 0x20;
			else
				expect_resp_header[0] = script->cmd[1];
			expect_resp_header[1] = script->cmd[2];
			I(": 0x%02X, 0x%02X\r\n", expect_resp_header[0], expect_resp_header[1]);
		}

		if (*(script->exp_ntf) != 0) {
			if (0x20 == (script->cmd[1] & 0xF0))
				expect_ntf_header[0] = script->cmd[1] + 0x40;
			else if (0x00 == (script->cmd[1] & 0xF0))
				expect_ntf_header[0] = 0x60;
			I("Expected NTF Header: 0x%02X\r\n", expect_ntf_header[0]);
		}
	}


	if ( pn544_RxData(nci_read_header, 2) < 0) {
		I("I2C error while read out the NCI header.\r\n");
		return -255;
	} else {
		I("NCI header read: 0x%02X, 0x%02X\r\n", nci_read_header[0], nci_read_header[1]);
		mdelay(NFC_READ_DELAY);
		if ( pn544_RxData(&nci_data_len, 1) < 0) {
			I("I2C error while read out the NCI data length.\r\n");
			return -255;
		} else {
			I("NCI data length read: %d\r\n", (int)nci_data_len);
			mdelay(NFC_READ_DELAY);
			if ( pn544_RxData(receiverBuffer, nci_data_len) < 0) {
				I("I2C error while read out the NCI data.\r\n");
				return -255;
			} else {
				I("NCI data: ");
				nfc_nci_dump_data(receiverBuffer, (int)nci_data_len);
			}
		}
	}

	/* Bypass separate package first, not used by either source currently */
	/* process responses */
	if (0x40 == (nci_read_header[0] & 0xF0)) {
		GOID = nci_read_header[0] & 0x0F;
		GOID = (GOID << 8) | nci_read_header[1];
		I("GOID: 0x%08X\r\n", GOID);

		switch (GOID) {
		case 0x0000: /* Case CORE_RESET_RSP */
			I("Response CORE_RESET_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command. exp_len:%x\r\n", script->exp_resp_content[0]);
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			gDevice_info.NCI_version = receiverBuffer[1];
			break;
		case 0x0001: /* Case CORE_INIT_RSP */
			I("Response CORE_INIT_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			rf_support_len = receiverBuffer[5];
			gDevice_info.NFCC_Features = ((unsigned int)receiverBuffer[4]) << 24 | ((unsigned int)receiverBuffer[3]) << 16 | ((unsigned int)receiverBuffer[2]) << 8 | receiverBuffer[1];
			gDevice_info.manufactor = receiverBuffer[12 + rf_support_len];
			gDevice_info.fwVersion = ((unsigned int)receiverBuffer[15 + rf_support_len]) << 8 | ((unsigned int)receiverBuffer[16 + rf_support_len]);
			I("FW Version 0x%07lX\r\n", gDevice_info.fwVersion);
			mfc_nfc_cmd_result = (int)gDevice_info.fwVersion;
			break;
		case 0x0103: /* Case RF_DISCOVER_RSP */
			I("Response RF_DISCOVER_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			if (script->cmd[5] < 0x80)
				I("Start to detect Cards.\r\n");
			else
				I("Start to listen Reader.\r\n");
			/* Set target NTF as RF_INTF_ACTIVATED_NTF */
			expect_ntf_header[1] = 0x05;
			break;
		case 0x0200: /* Case NFCEE_DISCOVER_RSP */
			I("Response NFCEE_DISCOVER_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			/* Set target NTF as NFCEE_DISCOVER_NTF */
			expect_ntf_header[1] = 0x00;
			break;
		case 0x0201: /* Case NFCEE_MODE_SET_RSP */
			I("Response NFCEE_MODE_SET_RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			/* Set target NTF as NFCEE_DISCOVER_NTF */
			expect_ntf_header[1] = 0x00;
			break;

#if FTM_NFC_CPLC
		case 0x0006: /* Case Get_ATR_RSP */
			I("Response Get ATR RSP received.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			}
			break;
#endif //FTM_NFC_CPLC
		default:
			I("Response not defined.\r\n");
			if (*(script->exp_resp_content)) {
				if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
					I("Response type matched with command.\r\n");
					if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
						I("Response matched with expected response, res_achieved set.\r\n");
						res_achieved = 1;
					} else {
						I("Not expected response! Quit now.\r\n");
						return -255;
					}
				} else {
					I("Command-Response type not matched, ignore.\r\n");
				}
			} else
				I("No response requirement.\r\n");
		}
	}

	/* process data packets */
	if (0x00 == (nci_read_header[0] & 0xF0)) {
		I("Data Packet, Connection ID:0x%02X\r\n", (nci_read_header[0] & 0x0F));
		if (*(script->exp_resp_content)) {
			if (memcmp(nci_read_header, expect_resp_header, 2) == 0){
				I("Response type matched with command.\r\n");
				if (memcmp(&script->exp_resp_content[1], receiverBuffer, script->exp_resp_content[0]) == 0) {
					I("Response matched with expected response, res_achieved set.\r\n");
					res_achieved = 1;
				} else {
					I("Not expected response! Quit now.\r\n");
					return -255;
				}
			} else {
				I("Command-Response type not matched, ignore.\r\n");
			}
		} else
			I("No response requirement.\r\n");
		if (0x00 == (nci_read_header[0] & 0xF0))
			expect_ntf_header[1] = 0x06;
	}

	/* process notifications */
	if (0x60 == (nci_read_header[0] & 0xF0)) {
		GOID = nci_read_header[0] & 0x0F;
		GOID = (GOID << 8) | nci_read_header[1];
		I("GOID: 0x%08X\r\n", GOID);

		switch (GOID) {
		case 0x0103:
			I("Notification RF_DISCOVER_NTF received.\r\n");
			if (*(script->exp_ntf)) { /*Case when expected multiple remote SE NTF coming*/
				if (memcmp(nci_read_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\r\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\r\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\r\n");
						return -1;
					}
				} else {
					//I("Command-Notification type not matched, ignore.\r\n");
					/* Case when waiting for RF_INTF_ACTIVATED_NTF but multiple NTF detected */
					gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_ID = receiverBuffer[0];
					gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_Protocol = receiverBuffer[1];
					gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_Technology = receiverBuffer[2];
					if (gDevice_info.target_rf_id == 255 &&
					 gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_Protocol == gDevice_info.protocol_set)
						gDevice_info.target_rf_id = gDevice_info.NTF_queue[gDevice_info.NTF_count].RF_ID;

					if (receiverBuffer[nci_data_len - 1] == 0) {
						I("Last INTF_NTF reached.\r\n");
						I("Card detected!\r\n");
						select_rf_target[0].cmd[4] = gDevice_info.target_rf_id;
						select_rf_target[0].cmd[5] = gDevice_info.protocol_set;
						select_rf_target[0].cmd[6] = gDevice_info.intf_set;
//						if (script_processor(select_rf_target, sizeof(select_rf_target)) == 0)
//							return 1;
//						else
//							return -1;
					}
				}
			}
			gDevice_info.NTF_count++;
			break;
		case 0x0105: /* Case RF_INTF_ACTIVATED_NTF */
			I("Notification RF_INTF_ACTIVATED_NTF received.\r\n");
			if (*(script->exp_ntf)) {
				if (memcmp(nci_read_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\r\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\r\n");
						gDevice_info.activated_INTF = receiverBuffer[0];
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\r\n");
						return -1;
					}
				} else {
					I("Command-Notification type not matched, ignore.\r\n");
				}
			}
			if (receiverBuffer[3] < 0x80)
				I("Card detected!\r\n");
			else
				I("Reader detected!\r\n");
			break;
		case 0x0200: /* Case NFCEE_DISCOVER_NTF */
			I("Notification NFCEE_DISCOVER_NTF received.\r\n");
			if (*(script->exp_ntf)) {
				if (memcmp(nci_read_header, expect_ntf_header, 2) == 0){
					I("Notification type matched with command.\r\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\r\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\r\n");
						return -1;
					}
				} else {
					I("Command-Notification type not matched, ignore.\r\n");
				}
			}
			gDevice_info.HW_model = receiverBuffer[1];
			break;
#if FTM_NFC_CPLC
		case 0x010A:/*Case Type A/B discovery*/
			I("Notification Type A/B DISCOVERY_NTF received.\r\n");
			if (*(script->exp_ntf)) {
				ntf_achieved = 1;
			}
			break;
#endif //FTM_NFC_CPLC
		default:
			I("Notification not defined.\r\n");
			if (*(script->exp_ntf)) {
				if (memcmp(nci_read_header, expect_ntf_header, 1) == 0){
					I("Notification type matched with command.\r\n");
					if (memcmp(&script->exp_ntf[1], receiverBuffer, script->exp_ntf[0]) == 0) {
						I("Notification matched with expected Notification, ntf_achieved set.\r\n");
						ntf_achieved = 1;
					} else {
						I("Not expected Notification! Wait for another.\r\n");
						return -1;
					}
				} else {
					I("Command-Notification type not matched, ignore.\r\n");
				}
			} else
				I("No Notification requirement.\r\n");
		}
	}

	if (*(script->exp_resp_content) != 0) {
		if (res_achieved) {
			if (*(script->exp_ntf) != 0) {
				if (ntf_achieved) {
					return 1;
				} else {
					I("Notification requirement not achieve, stay at current command.\r\n");
					//if (watchdogEn == 1)
					//	watchdog_counter = 0; keep counting don't reset

					return -1;
				}
			} else {
				I("No NTF requirement, step to next command.\r\n");
				return 1;
			}
		} else {
			I("Response requirement not achieve, stay at current command.\r\n");

			//if (watchdogEn == 1)
			//	watchdog_counter = 0; keep counting don't reset

			return -1;
		}
	} else if (*(script->exp_ntf) != 0) {
		if (ntf_achieved) {
			return 1;
		} else {
			I("Notification requirement not achieve, stay at current command.\r\n");

		//	if (watchdogEn == 1)
		//		watchdog_counter = 0; keep counting don't reset

			return -1;
		}
	} else {
		I("No requirement, step to next command.\r\n");
		return 1;
	}
}

#define CHECK_READER(void) \
do { \
	if (watchdogEn == 1) {\
		if (watchdog_counter++ > ((2000 / NFC_READ_DELAY) * watchdog_timeout)) {\
			I("watchdog timeout, command fail.\r\n");\
			goto TIMEOUT_FAIL; \
		} \
	} \
	if (pn544_irq_gpio()) { \
		reader_resp = nci_Reader(&script[scriptIndex], scriptSize); \
		/*I("%d returned.\r\n", reader_resp);*/ \
		switch(reader_resp) { \
		case -255: \
			/* I2C error, break*/ \
			goto I2C_FAIL; \
			break; \
		case -1: \
			/* stay at current command.*/ \
			break; \
		case 0: \
			/* script restart */ \
			scriptIndex = 0; \
			break; \
		case 1: \
			/* step to next command. */ \
			scriptIndex++; \
			break; \
		default: \
			scriptIndex = reader_resp; \
		} \
	} \
} while(0)

int script_processor(control_msg_pack *script, unsigned int scriptSize) {
	int ret;
	int scriptIndex, reader_resp;
	int last_scriptIndex;
	scriptSize = scriptSize/sizeof(control_msg_pack);
	I("script_processor script size: %d.\r\n", scriptSize);

	scriptIndex = 0;
	last_scriptIndex = -1;
	reader_resp = 1;

	do {
		if (reader_resp == -1) {
			CHECK_READER();
			mdelay(NFC_READ_DELAY);

			if (watchdogEn == 1)
				if (watchdog_counter++ > ((2000 / NFC_READ_DELAY) * watchdog_timeout)) {
					I("watchdog timeout, command fail.\r\n");
					goto TIMEOUT_FAIL;
				}

			continue;
		}

		if ( last_scriptIndex != scriptIndex) {
			I("script_processor pn544_TxData()+\r\n");
			ret = pn544_TxData(&script[scriptIndex].cmd[1], (int)script[scriptIndex].cmd[0]);
			I("script_processor pn544_TxData()-\r\n");
			if (ret < 0) {
				E("%s, i2c Tx error!\n", __func__);
				nfc_nci_dump_data(&script[scriptIndex].cmd[1], (int)script[scriptIndex].cmd[0]);
				goto I2C_FAIL;
				break;
			}
			else {
					I("i2c wrote: ");
					nfc_nci_dump_data(&script[scriptIndex].cmd[1], (int)script[scriptIndex].cmd[0]);
					mdelay(NFC_READ_DELAY + 20);
					last_scriptIndex = scriptIndex;
					I("script_processor CHECK_IRQ value :%d\r\n", pn544_irq_gpio());
					CHECK_READER();
				}
		} else {
			CHECK_READER();

			if (watchdogEn == 1)
				if (watchdog_counter++ > ((2000 / NFC_READ_DELAY) * watchdog_timeout)) {
					I("watchdog timeout, command fail.\r\n");
					goto TIMEOUT_FAIL;
				}

		}
		mdelay(NFC_READ_DELAY);
	} while(scriptIndex < scriptSize);

	return 0;
I2C_FAIL:
	E("%s, I2C_FAIL!\n", __func__);
	mfc_nfc_cmd_result = -2;
	return 1;
TIMEOUT_FAIL:
	mfc_nfc_cmd_result = 0;
	return 1;
}



int mfg_nfc_test(int arg)
{
	bool PLL_clock = true;
	gDevice_info.NTF_count = 0;
	memset(gDevice_info.NTF_queue, 0x00, sizeof(gDevice_info.NTF_queue));
	gDevice_info.protocol_set = 4;
	gDevice_info.intf_set = 2;
	gDevice_info.target_rf_id = 255;
	mfc_nfc_cmd_result = -1;
	watchdog_counter = 0; //only rest it at the beginning
	watchdogEn = 1;
	I("%s: arg = %d\n", __func__, arg);

	switch (arg) {
	case 0:  // get nfc FW version
		I("%s: get nfcversion :\n", __func__);
		pn544_hw_reset_control(1);
		if (script_processor(nfc_version_script, sizeof(nfc_version_script)) == 0) {
			I("%s: mfc_nfc_cmd_result (nfcversion) = %d\n", __func__, mfc_nfc_cmd_result);
		}
		pn544_hw_reset_control(0);
		break;
	case 1:  // reader mode test
		I("%s: nfcreader test :\n", __func__);
		pn544_hw_reset_control(1);
		if(PLL_clock) {
			if (script_processor(nfc_reader_pll_script, sizeof(nfc_reader_pll_script)) == 0) {
				I("%s: mfc_nfc_cmd_result (nfcversion) = %d\n", __func__, mfc_nfc_cmd_result);
				mfc_nfc_cmd_result = 1; // 1 for pass
			}
		}
		else {
			if (script_processor(nfc_reader_script, sizeof(nfc_reader_script)) == 0) {
				I("%s: mfc_nfc_cmd_result (nfcversion) = %d\n", __func__, mfc_nfc_cmd_result);
				mfc_nfc_cmd_result = 1; // 1 for pass
			}
		}
		pn544_hw_reset_control(0);
		break;
	case 2:  // card mode test
		I("%s: nfccard test :\n", __func__);
		pn544_hw_reset_control(1);
		if(PLL_clock) {
			if (script_processor(nfc_card_pll_script, sizeof(nfc_card_pll_script)) == 0) {
				I("%s: mfc_nfc_cmd_result (nfcversion) = %d\n", __func__, mfc_nfc_cmd_result);
				mfc_nfc_cmd_result = 1;
			}
		}
		else {
			if (script_processor(nfc_card_script, sizeof(nfc_card_script)) == 0) {
				I("%s: mfc_nfc_cmd_result (nfcversion) = %d\n", __func__, mfc_nfc_cmd_result);
				mfc_nfc_cmd_result = 1;
			}
		}
		pn544_hw_reset_control(0);
		break;
#if FTM_NFC_CPLC
	case 3:
		I("%s: nfc cplc test :\n", __func__);
		memset(CPLC, 0, MAX_BUFFER_SIZE);
		memset(RF_PARAM_CE_eSE, 0, MAX_BUFFER_SIZE);
		memset(NCI_TMP, 0, MAX_BUFFER_SIZE);
		pn544_hw_reset_control(1);

		if (script_processor(nfc_cplc_part1_script, sizeof(nfc_cplc_part1_script)) == 0) {
			I("nci_cplc_part1 complete.\r\n");
			msleep(2000);
		} else {
			I("nci_cplc_part1 script processing error!\r\n");
			break;
		}
#if 0
//Disable Get ATR_COMMAND (No response on some PN80T chips)
		if (script_processor(nfc_cplc_part2_1_script, sizeof(nfc_cplc_part2_1_script)) == 0) {
			I("nci_cplc_part2_1 complete.\r\n");
		} else {
			I("nci_cplc_part2_1 script processing error!\r\n");
			break;
		}
#endif
		if (script_processor(nfc_cplc_part2_2_script, sizeof(nfc_cplc_part2_2_script)) == 0) {
			I("nci_cplc_part2_2 complete.\r\n");
			memcpy(CPLC,NCI_TMP,MAX_BUFFER_SIZE);
		} else {
			I("nci_cplc_part2_2 script processing error!\r\n");
			break;
		}

		if (script_processor(nfc_cplc_part2_3_script, sizeof(nfc_cplc_part2_3_script)) == 0) {
			memcpy(RF_PARAM_CE_eSE,NCI_TMP,MAX_BUFFER_SIZE);
			I("nci_cplc_part2_3 complete.\r\n");
			mfc_nfc_cmd_result = 99;
		} else {
			I("nci_cplc_part2_2 script processing error!\r\n");
			break;
		}
		break;
	case 77:
		I("%s: nfc dpc test :\n", __func__);
		memset(CPLC, 0, MAX_BUFFER_SIZE);
		memset(RF_PARAM_CE_eSE, 0, MAX_BUFFER_SIZE);
		memset(NCI_TMP, 0, MAX_BUFFER_SIZE);
		pn544_hw_reset_control(1);

		if (script_processor(nfc_dpc_script, sizeof(nfc_dpc_script)) == 0) {
			I("nci_dpc complete.\r\n");
			memcpy(CPLC,NCI_TMP,MAX_BUFFER_SIZE);
			mfc_nfc_cmd_result = 99;
		} else {
			I("nci_dpc script processing error!\r\n");
			break;
		}
		break;
#endif  //FTM_NFC_CPLC
	case 88:
#ifdef SW_ENABLE_OFFMODECHARGING
		pn544_hw_reset_control(1);
		I("%s: off_mode_charging script :\n", __func__);
		if (script_processor(nfc_off_mode_charging_enble_script, sizeof(nfc_off_mode_charging_enble_script)) == 0) {
			mfc_nfc_cmd_result = 1;
			mdelay(1);
			I("%s: off_mode_charging complete!!\n",__func__);
		} else {
			E("%s: off_mode_charging fail!!\n",__func__);
			pn544_hw_reset_control(0);
		}

#endif
		break;
	case 99:
//PN548 Standby mode workaround++
		I("%s: nfc_standby_enble_script :\n", __func__);
		pn544_hw_reset_control(1);
		if (script_processor(nfc_standby_enble_script, sizeof(nfc_standby_enble_script)) == 0) {
			I("%s: mfc_nfc_cmd_result (nfcversion) = %d\n", __func__, mfc_nfc_cmd_result);
			mfc_nfc_cmd_result = 1;
		}
//PN548 Standby mode workaround--
		pn544_hw_reset_control(0);

		break;
	default:
		E("%s: case default do nothing\n", __func__);
		break;
	}
	I("%s: END\n", __func__);
	return mfc_nfc_cmd_result;
}

#if FTM_NFC_CPLC
char* mfg_nfc_cplc_result() {
	if (mfg_nfc_test(3) == 99) {
		return CPLC;
	} else {
		return NULL;
	}
}

char* mfg_nfc_cplc_result_uid() {
	return RF_PARAM_CE_eSE;
}

#endif //FTM_NFC_CPLC

char* mfg_nfc_dpc_result() {
	if (mfg_nfc_test(77) == 99) {
		return CPLC;
	} else {
		return NULL;
	}
}

/******************************************************************************
 *
 *  Function pn553_htc_check_rfskuid:
 *  Return With(1)/Without(0) NFC chip if this SKU can get RFSKUID in kernal
 *  Return is_alive(original value) by default.
 *
 ******************************************************************************/
int pn553_htc_check_rfskuid(int in_is_alive){
#if NFC_READ_RFSKUID
	int nfc_rfbandid_size = 0;
	int i;
	unsigned int *nfc_rfbandid_info;
	struct device_node *nfc_rfbandid;
	nfc_rfbandid = of_find_node_by_path("/chosen/mfg");
	if (nfc_rfbandid){
		nfc_rfbandid_info = (unsigned int *) of_get_property(nfc_rfbandid,"skuid.rf_id",&nfc_rfbandid_size);
		if (nfc_rfbandid_info == NULL)
		{
			E("%s:Get skuid.rf_id fail keep NFC by default",__func__);
			return 1;
		}
	}else {
		E("%s:Get skuid.rf_id fail (can't find /chosen/mfg) keep NFC by default",__func__);
		return 1;
	}
	if(nfc_rfbandid_size != 32) {  //32bytes = 4 bytes(int) * 8 rfbandid_info
		E("%s:Get skuid.rf_id size error keep NFC by default",__func__);
		return 1;
	}

	for ( i = 0; i < 8; i++) {
		if (nfc_rfbandid_info[i] == HAS_NFC_CHIP) {
			I("%s: Check skuid.rf_id done device has NFC chip",__func__);
			return 1;
		}
	}
	I("%s: Check skuid.rf_id done remove NFC",__func__);
	return 0;
#else //NFC_READ_RFSKUID
	return in_is_alive;
#endif //NFC_READ_RFSKUID
}

/******************************************************************************
 *
 *  Function pn548_htc_get_bootmode:
 *  Return  NFC_BOOT_MODE_NORMAL            0
 *          NFC_BOOT_MODE_FTM               1
 *          NFC_BOOT_MODE_DOWNLOAD          2
 *          NFC_BOOT_MODE_OFF_MODE_CHARGING 5
 *  Return 	NFC_BOOT_MODE_NORMAL by default
 *          if there's no bootmode infomation available
 *
 *          Bootmode strig is defined in
 *          bootable/bootloader/lk/app/aboot/aboot.c
 *          bootable/bootloader/lk/app/aboot/htc/htc_board_info_and_setting.c
 *
 ******************************************************************************/
int pn553_htc_get_bootmode(void) {
	char sbootmode[30] = "default";
#if NFC_GET_BOOTMODE
	strlcpy(sbootmode,htc_get_bootmode(),sizeof(sbootmode));
#endif  //NFC_GET_BOOTMODE
	if (strcmp(sbootmode, "offmode_charging") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_OFF_MODE_CHARGING\n",__func__);
		return NFC_BOOT_MODE_OFF_MODE_CHARGING;
	} else if (strcmp(sbootmode, "ftm") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_FTM\n",__func__);
		return NFC_BOOT_MODE_FTM;
	} else if (strcmp(sbootmode, "download") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_DOWNLOAD\n",__func__);
		return NFC_BOOT_MODE_DOWNLOAD;
	} else {
		I("%s: Check bootmode done NFC_BOOT_MODE_NORMAL mode = %s\n",__func__,sbootmode);
		return NFC_BOOT_MODE_NORMAL;
	}
}

