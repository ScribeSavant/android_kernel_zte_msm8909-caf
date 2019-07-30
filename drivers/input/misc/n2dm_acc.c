/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
 *
 * File Name          : n2dm_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (denis.ciocca@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.12
 * Date               : 2013/Nov/12
 * Description        : N2DM accelerometer driver sensor
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 ******************************************************************************
 Revision 1.0.6 15/11/2010
  first revision
  supports sysfs;
  no more support for ioctl;
 Revision 1.0.7 26/11/2010
  checks for availability of interrupts pins
  correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7
 Revision 1.0.9: 2011/May/23
  update_odr func correction;
 Revision 1.0.10: 2011/Aug/16
  introduces default_platform_data, i2c_read and i2c_write function rewritten,
  manages smbus beside i2c
 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct; modified:-update_fs_range;-set_range
  input format; allows gpio_intX to be passed as parameter at insmod time;
 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/kernel.h>
#include	<linux/device.h>
#include	<linux/module.h>
#include	<linux/moduleparam.h>
#include	<linux/regulator/consumer.h>
#include    <linux/sensors.h>
#include 	<linux/mutex.h>
#include	<linux/input/n2dm.h>
//added by chenhui for proc info node begin
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
//added by chenhui for proc info node

/*#define	DEBUG		1 */
/*#define  DEBUG_CALI        1*/
#define	G_MAX			16000


#define SENSITIVITY_2G		1     /*    High-resolution   mg    //3096		ug/LSB	*/
#define SENSITIVITY_4G		2 	/*    High-resolution   mg	//7813		ug/LSB	*/
#define SENSITIVITY_8G		4  	/*	 High-resolution   mg     //15625		ug/LSB	*/
#define SENSITIVITY_16G		12	/*	 High-resolution   mg	//875		ug/LSB	*/




/* Accelerometer Sensor Operating Mode */
#define N2DM_ACC_ENABLE		(0x01)
#define N2DM_ACC_DISABLE	(0x00)

#define	HIGH_RESOLUTION		(0x08)

#define	AXISDATA_REG		(0x28)
#define WHOAMI_N2DM_ACC	(0x33)	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		(0x1F)	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		(0x20)	/*	control reg 1		*/
#define	CTRL_REG2		(0x21)	/*	control reg 2		*/
#define	CTRL_REG3		(0x22)	/*	control reg 3		*/
#define	CTRL_REG4		(0x23)	/*	control reg 4		*/
#define	CTRL_REG5		(0x24)	/*	control reg 5		*/
#define	CTRL_REG6		(0x25)	/*	control reg 6		*/

#define	FIFO_CTRL_REG		(0x2E)	/*	FiFo control reg	*/

#define	INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define	INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define	INT_THS1		(0x32)	/*	interrupt 1 threshold	*/
#define	INT_DUR1		(0x33)	/*	interrupt 1 duration	*/


#define	TT_CFG			(0x38)	/*	tap config		*/
#define	TT_SRC			(0x39)	/*	tap source		*/
#define	TT_THS			(0x3A)	/*	tap threshold		*/
#define	TT_LIM			(0x3B)	/*	tap time limit		*/
#define	TT_TLAT			(0x3C)	/*	tap time latency	*/
#define	TT_TW			(0x3D)	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1
#define ALL_ZEROES		(0x00)

#define N2DM_ACC_PM_OFF		(0x00)
#define N2DM_ACC_ENABLE_ALL_AXES	(0x07)


#define PMODE_MASK		(0x08)
#define ODR_MASK		(0XF0)

#define N2DM_ACC_ODR1		(0x10)  /* 1Hz output data rate */
#define N2DM_ACC_ODR10		(0x20)  /* 10Hz output data rate */
#define N2DM_ACC_ODR25		(0x30)  /* 25Hz output data rate */
#define N2DM_ACC_ODR50		(0x40)  /* 50Hz output data rate */
#define N2DM_ACC_ODR100		(0x50)  /* 100Hz output data rate */
#define N2DM_ACC_ODR200		(0x60)  /* 200Hz output data rate */
#define N2DM_ACC_ODR400		(0x70)  /* 400Hz output data rate */
#define N2DM_ACC_ODR1344	(0x90)  /* 1344Hz output data rate */



#define	IA			(0x40)
#define	ZH			(0x20)
#define	ZL			(0x10)
#define	YH			(0x08)
#define	YL			(0x04)
#define	XH			(0x02)
#define	XL			(0x01)
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	(0x40)
#define	CTRL_REG4_BDU_ENABLE	(0x80)
#define	CTRL_REG4_BDU_MASK	(0x80)
#define	CTRL_REG6_I2_TAPEN	(0x80)
#define	CTRL_REG6_HLACTIVE	(0x02)
/* */
#define NO_MASK			(0xFF)
#define INT1_DURATION_MASK	(0x7F)
#define	INT1_THRESHOLD_MASK	(0x7F)
#define TAP_CFG_MASK		(0x3F)
#define	TAP_THS_MASK		(0x7F)
#define	TAP_TLIM_MASK		(0x7F)
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			(0x20)
#define	STAP			(0x10)
#define	SIGNTAP			(0x08)
#define	ZTAP			(0x04)
#define	YTAP			(0x02)
#define	XTAZ			(0x01)


#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
#define POLL_INTERVAL_MIN_MS	10
#define POLL_INTERVAL_MAX_MS	4000
#define POLL_DEFAULT_INTERVAL_MS 200
/* end RESUME STATE INDICES */

#define CALI_POLL_INTERVAL  20
#define ACC_CALI_COUNT  10
#define CAL_NUM		99

static struct sensors_classdev sensors_cdev = {
		.name = "n2dm-accel",
		.vendor = "st digital mems sensor",
		.version = 1,
		.handle = SENSORS_ACCELERATION_HANDLE,
		.type = SENSOR_TYPE_ACCELEROMETER,
		.max_range = "156.8",	/* 16g */
		.resolution = "0.156",	/* 15.6mg */
		.sensor_power = "0.01",	/* typical value */
		.min_delay = POLL_INTERVAL_MIN_MS * 1000, /* in microseconds */
		.max_delay = POLL_INTERVAL_MAX_MS,
		.max_latency = POLL_INTERVAL_MAX_MS,
		.fifo_reserved_event_count = 0,
		.fifo_max_event_count = 0,
		.enabled = 0,
		.delay_msec = POLL_DEFAULT_INTERVAL_MS, /* in millisecond */
		.sensors_enable = NULL,
		.sensors_poll_delay = NULL,
		.sensors_self_test = NULL,
};

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} n2dm_acc_odr_table[] = {
		{    1, N2DM_ACC_ODR1344 },
		{    3, N2DM_ACC_ODR400  },
		{    5, N2DM_ACC_ODR200  },
		{   10, N2DM_ACC_ODR100  },
		{   20, N2DM_ACC_ODR50   },
		{   40, N2DM_ACC_ODR25   },
		{  100, N2DM_ACC_ODR10   },
		{ 1000, N2DM_ACC_ODR1    },
};

/*added by chenhui for proc info node begin*/
#define PROC_GSENSOR_DIR  "gsensor"
#define PROC_GSENSOR_NAME  "gsensor/chip_info"

static struct proc_dir_entry *gsensor_proc_dir_entry;
static struct i2c_client *proc_client;
//static ssize_t gsensor_config_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos);
static int gsensor_config_proc_open(struct inode *inode, struct file *file);
struct list_head nd_struct_glist_n;


static const struct file_operations info_proc_file_ops = {	
	.owner	 = THIS_MODULE,    
	.open    = gsensor_config_proc_open,    
	.read	 = seq_read,	
	.llseek	 = seq_lseek,	
	.release = seq_release,	
	//.write   = gsensor_config_proc_write,
};
/*proc info node end*/


static int int1_gpio = N2DM_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = N2DM_ACC_DEFAULT_INT2_GPIO;
module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);

struct n2dm_acc_status {
	struct i2c_client *client;
	struct sensors_classdev cdev;
	struct n2dm_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;

	u32 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	//bool default_cal;
	s16 cali_data[3];
	s16 raw_data[3];
	char calibrate_buf[CAL_NUM];
       
#ifdef DEBUG
	u8 reg_addr;
#endif
};

static struct n2dm_acc_platform_data default_n2dm_acc_pdata = {
	.fs_range = N2DM_ACC_G_2G,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 1,//1,//1,
	.negate_y = 1,//0,//0,
	.negate_z = 1,//1,//0,
	.poll_interval = 100,
	.min_interval = N2DM_ACC_MIN_POLL_PERIOD_MS,
	.gpio_int1 = N2DM_ACC_DEFAULT_INT1_GPIO,
	.gpio_int2 = N2DM_ACC_DEFAULT_INT2_GPIO,
};

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator n2dm_acc_vreg[] = {
	{NULL, "vdd", 1700000, 3600000},
	{NULL, "vio", 1700000, 3600000},
};

#ifdef N2DM_VDD_HOLD_FOR_BOARD_TEST
static int n2dm_acc_config_regulator_vdd_hold(struct n2dm_acc_status *stat, bool on)
{
	int rc = 0;
	static struct regulator *vdd_hold = NULL;//vdd hold for board test, xym20141023

	if (on) {
		vdd_hold = regulator_get(&acc->client->dev, "vddhold");
		if (IS_ERR(vdd_hold)) {
			rc = PTR_ERR(vdd_hold);
			pr_err("%s:vdd_hold regulator get failed rc=%d\n", __func__, rc);
			vdd_hold = NULL;
			return rc;
			//goto error_vdd;
		}

		if (regulator_count_voltages(vdd_hold) > 0) {
			rc = regulator_set_voltage(vdd_hold, n2dm_acc_vreg[0].min_uV, n2dm_acc_vreg[0].max_uV);
			if (rc) {
				pr_err("%s: vdd_hold set voltage failed rc=%d\n", __func__, rc);
				regulator_put(vdd_hold);
				vdd_hold = NULL;
				return rc;
				//goto error_vdd;
			}
		}

		rc = regulator_enable(vdd_hold);
		if (rc) {
			pr_err("%s: vdd_hold regulator_enable failed rc =%d\n", __func__, rc);
			if (regulator_count_voltages(vdd_hold) > 0) {
				regulator_set_voltage(	vdd_hold, 0, n2dm_acc_vreg[0].max_uV);
			}
			regulator_put(vdd_hold);
			vdd_hold = NULL;
			return rc;
			//goto error_vdd;
		}
		return rc;	
	} else {

		rc = regulator_disable(vdd_hold);
		if (rc) {
			pr_err("%s: vdd_hold regulator_disable failed rc =%d\n", __func__, rc);
			return rc;
		}
		
		if (regulator_count_voltages(vdd_hold) > 0)
			regulator_set_voltage(vdd_hold, 0, n2dm_acc_vreg[0].max_uV);
		
		regulator_put(vdd_hold);
		vdd_hold = NULL;
	}
	return rc;
}
#endif

static int n2dm_acc_config_regulator(struct n2dm_acc_status *stat, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(n2dm_acc_vreg) / sizeof(struct sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			n2dm_acc_vreg[i].vreg =
				regulator_get(&stat->client->dev,
				n2dm_acc_vreg[i].name);
			if (IS_ERR(n2dm_acc_vreg[i].vreg)) {
				rc = PTR_ERR(n2dm_acc_vreg[i].vreg);
				dev_err(&stat->client->dev,
					"Regulator(%s) get failed rc=%d\n",
					n2dm_acc_vreg[i].name, rc);
				n2dm_acc_vreg[i].vreg = NULL;
				goto deinit_vregs;
			}

			if (regulator_count_voltages(
				n2dm_acc_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					n2dm_acc_vreg[i].vreg,
					n2dm_acc_vreg[i].min_uV,
					n2dm_acc_vreg[i].max_uV);
				if (rc) {
					dev_err(&stat->client->dev,
					"Regulator(%s)Set voltage failed rc=%d\n",
					n2dm_acc_vreg[i].name, rc);
					regulator_put(n2dm_acc_vreg[i].vreg);
					n2dm_acc_vreg[i].vreg = NULL;
					goto deinit_vregs;
				}
			}
		}
		return rc;
	} else {
		i = num_reg;
		goto deinit_vregs;
	}

deinit_vregs:
	while (--i >= 0) {
		if (!IS_ERR_OR_NULL(n2dm_acc_vreg[i].vreg)) {
			regulator_put(n2dm_acc_vreg[i].vreg);
			n2dm_acc_vreg[i].vreg = NULL;
		}
	}
	return rc;
}

static int n2dm_acc_set_regulator(struct n2dm_acc_status *stat, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(n2dm_acc_vreg) / sizeof(struct sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			if (!IS_ERR_OR_NULL(n2dm_acc_vreg[i].vreg)) {
				rc = regulator_enable(n2dm_acc_vreg[i].vreg);
				if (rc) {
					dev_err(&stat->client->dev,
					"Enable regulator(%s) failed rc=%d\n",
					n2dm_acc_vreg[i].name, rc);
					goto disable_regulator;
				}
			}
		}
		return rc;
	} else {
		for (i = (num_reg - 1); i >= 0; i--) {
			if (!IS_ERR_OR_NULL(n2dm_acc_vreg[i].vreg)) {
				rc = regulator_disable(n2dm_acc_vreg[i].vreg);
				if (rc)
					dev_err(&stat->client->dev,
					"Disable regulator(%s) failed rc=%d\n",
					n2dm_acc_vreg[i].name, rc);
			}
		}
		return 0;
	}

disable_regulator:
	while (--i >= 0) {
		if (!IS_ERR_OR_NULL(n2dm_acc_vreg[i].vreg))
			regulator_disable(n2dm_acc_vreg[i].vreg);
	}
	return rc;
}

static int n2dm_acc_i2c_read(struct n2dm_acc_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;

	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_i2c_block_data: ret:%d len:%d, "
				"command=0x%02x, ",
				ret, len, cmd);
			unsigned int ii;
			for (ii = 0; ii < len; ii++)
				printk(KERN_DEBUG "buf[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&stat->client->dev,
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			return 0; /* failure */
		}
		return len; /* success */
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int n2dm_acc_i2c_write(struct n2dm_acc_status *stat, u8 *buf,
									int len)
{
	int ret;
	u8 reg, value;

	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_i2c_block_data: ret=%d, "
				"len:%d, command=0x%02x, ",
				ret, len, reg);
			unsigned int ii;
			for (ii = 0; ii < (len + 1); ii++)
				printk(KERN_DEBUG "value[%d]=0x%02x,",
								ii, buf[ii]);

			printk("\n");
#endif
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}

static int n2dm_acc_hw_init(struct n2dm_acc_status *stat)
{
	int err = -1;
	u8 buf[7];

	pr_info("%s: hw init start\n", N2DM_ACC_DEV_NAME);
    printk("cgh please notify %d\n", stat->client->addr);
	buf[0] = WHO_AM_I;
	err = n2dm_acc_i2c_read(stat, buf, 1);
	if (err < 0) {
		stat->client->addr = 0x9;
		printk("cgh1 please notify %d\n", stat->client->addr);
		err = n2dm_acc_i2c_read(stat, buf, 1);
		if (err < 0){
		dev_warn(&stat->client->dev, "Error reading WHO_AM_I:"
				" is device available/working?\n");
		goto err_firstread;
		}
	} else
		stat->hw_working = 1;

	if (buf[0] != WHOAMI_N2DM_ACC) {
		dev_err(&stat->client->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n",
			WHOAMI_N2DM_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = CTRL_REG1;
	buf[1] = stat->resume_state[RES_CTRL_REG1];
	err = n2dm_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TEMP_CFG_REG;
	buf[1] = stat->resume_state[RES_TEMP_CFG_REG];
	err = n2dm_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = n2dm_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TT_THS;
	buf[1] = stat->resume_state[RES_TT_THS];
	buf[2] = stat->resume_state[RES_TT_LIM];
	buf[3] = stat->resume_state[RES_TT_TLAT];
	buf[4] = stat->resume_state[RES_TT_TW];
	err = n2dm_acc_i2c_write(stat, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = TT_CFG;
	buf[1] = stat->resume_state[RES_TT_CFG];
	err = n2dm_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = INT_THS1;
	buf[1] = stat->resume_state[RES_INT_THS1];
	buf[2] = stat->resume_state[RES_INT_DUR1];
	err = n2dm_acc_i2c_write(stat, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = stat->resume_state[RES_INT_CFG1];
	err = n2dm_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;


	buf[0] = CTRL_REG2;
	buf[1] = stat->resume_state[RES_CTRL_REG2];
	buf[2] = stat->resume_state[RES_CTRL_REG3];
	buf[3] = stat->resume_state[RES_CTRL_REG4];
	buf[4] = stat->resume_state[RES_CTRL_REG5];
	buf[5] = stat->resume_state[RES_CTRL_REG6];
	err = n2dm_acc_i2c_write(stat, buf, 5);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
	pr_info("%s: hw init done\n", N2DM_ACC_DEV_NAME);
	return 0;

err_firstread:
	stat->hw_working = 0;
err_unknown_device:
err_resume_state:
	stat->hw_initialized = 0;
	dev_err(&stat->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void n2dm_acc_device_power_off(struct n2dm_acc_status *stat)
{
	int err;
	u8 buf[2] = { CTRL_REG1, N2DM_ACC_PM_OFF };

	err = n2dm_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed: %d\n", err);
		
	n2dm_acc_set_regulator(stat, false);	

	if (stat->pdata->power_off) {
		if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			disable_irq_nosync(stat->irq2);
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}
	if (stat->hw_initialized) {
		if (stat->pdata->gpio_int1 >= 0)
			disable_irq_nosync(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			disable_irq_nosync(stat->irq2);
		stat->hw_initialized = 0;
	}

}

static int n2dm_acc_device_power_on(struct n2dm_acc_status *stat)
{
	int err = -1;
	
	err = n2dm_acc_set_regulator(stat, true);
	if (err < 0) {
		dev_err(&stat->client->dev,
				"power_on failed: %d\n", err);
		return err;
	}

	msleep(20);	

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0) {
			dev_err(&stat->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if (stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}

	if (!stat->hw_initialized) {
		err = n2dm_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			//n2dm_acc_device_power_off(stat);
			n2dm_acc_set_regulator(stat, false);
			return err;
		}
	}

	if (stat->hw_initialized) {
		if (stat->pdata->gpio_int1 >= 0)
			enable_irq(stat->irq1);
		if (stat->pdata->gpio_int2 >= 0)
			enable_irq(stat->irq2);
	}
	return 0;
}

static irqreturn_t n2dm_acc_isr1(int irq, void *dev)
{
	struct n2dm_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq1_work_queue, &stat->irq1_work);
	pr_debug("%s: isr1 queued\n", N2DM_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t n2dm_acc_isr2(int irq, void *dev)
{
	struct n2dm_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq2_work_queue, &stat->irq2_work);
	pr_debug("%s: isr2 queued\n", N2DM_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void n2dm_acc_irq1_work_func(struct work_struct *work)
{

	struct n2dm_acc_status *stat =
	container_of(work, struct n2dm_acc_status, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:n2dm_acc_get_int1_source(stat); */

	/* ; */
	pr_debug("%s: IRQ1 triggered\n", N2DM_ACC_DEV_NAME);
/* exit: */
	enable_irq(stat->irq1);
}

static void n2dm_acc_irq2_work_func(struct work_struct *work)
{

	struct n2dm_acc_status *stat =
	container_of(work, struct n2dm_acc_status, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:n2dm_acc_get_tap_source(stat); */

	/* ; */

	pr_debug("%s: IRQ2 triggered\n", N2DM_ACC_DEV_NAME);
/* exit: */
	enable_irq(stat->irq2);
}

static int n2dm_acc_update_fs_range(struct n2dm_acc_status *stat,
							u8 new_fs_range)
{
	int err = -1;

	u32 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = N2DM_ACC_FS_MASK | HIGH_RESOLUTION;
    pr_info("n2dm_acc_update_fs_range: new_fs_range = %d\n", new_fs_range);
	switch (new_fs_range) {
	case N2DM_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case N2DM_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case N2DM_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	case N2DM_ACC_G_16G:

		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid fs range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}
    pr_info("n2dm_acc_update_fs_range: sensitivity = %d\n", sensitivity); 

	/* Updates configuration register 4,
	* which contains fs range setting */
	buf[0] = CTRL_REG4;
	err = n2dm_acc_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;
	init_val = buf[0];
	stat->resume_state[RES_CTRL_REG4] = init_val;
	new_val = new_fs_range | HIGH_RESOLUTION;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	buf[1] = updated_val;
	buf[0] = CTRL_REG4;
	err = n2dm_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;
	stat->resume_state[RES_CTRL_REG4] = updated_val;
	stat->sensitivity = sensitivity;

	return err;
error:
	dev_err(&stat->client->dev,
			"update fs range failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int n2dm_acc_update_odr(struct n2dm_acc_status *stat,
							int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(n2dm_acc_odr_table) - 1; i >= 0; i--) {
		if ((n2dm_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
								|| (i == 0))
			break;
	}
	config[1] = n2dm_acc_odr_table[i].mask;

	config[1] |= N2DM_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL_REG1;
		err = n2dm_acc_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
		stat->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;

error:
	dev_err(&stat->client->dev, "update odr failed 0x%02x,0x%02x: %d\n",
			config[0], config[1], err);

	return err;
}



static int n2dm_acc_register_write(struct n2dm_acc_status *stat,
					u8 *buf, u8 reg_address, u8 new_value)
{
	int err = -1;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = n2dm_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;
	return err;
}

/*
static int n2dm_acc_register_read(struct n2dm_acc_status *stat,
							u8 *buf, u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = n2dm_acc_i2c_read(stat, buf, 1);
	return err;
}
*/

/*
static int n2dm_acc_register_update(struct n2dm_acc_status *stat,
		u8 *buf, u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = n2dm_acc_register_read(stat, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = n2dm_acc_register_write(stat, buf, reg_address,
				updated_val);
	}
	return err;
}
*/

static int n2dm_acc_get_acceleration_data(
				struct n2dm_acc_status *stat, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	acc_data[0] = (AXISDATA_REG);
	err = n2dm_acc_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (s32)(((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (s32)(((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (s32)(((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	//pr_info("%s read x1=%d, y1=%d, z1=%d\n",
	//		N2DM_ACC_DEV_NAME, hw_d[0], hw_d[1], hw_d[2]);  
			
	hw_d[0] = hw_d[0] * stat->sensitivity;
	hw_d[1] = hw_d[1] * stat->sensitivity;
	hw_d[2] = hw_d[2] * stat->sensitivity;
	hw_d[0] = (hw_d[0] * 1024)/(1000);
	hw_d[1] = (hw_d[1] * 1024)/(1000);
	hw_d[2] = (hw_d[2] * 1024)/(1000);

	//pr_info("%s read x2=%d, y2=%d, z2=%d\n",
	//		N2DM_ACC_DEV_NAME, hw_d[0], hw_d[1], hw_d[2]); 
			
	xyz[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	xyz[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	xyz[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));   

	//pr_info("%s read x=%d, y=%d, z=%d\n",
	//		N2DM_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);   
#ifdef DEBUG

	pr_debug("%s read x=%d, y=%d, z=%d\n",
			N2DM_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

#endif
	return err;
}

static void n2dm_acc_report_values(struct n2dm_acc_status *stat,
					int *xyz)
{
	ktime_t ts;
	ts = ktime_get_boottime();	
	input_report_abs(stat->input_dev, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev, ABS_Z, xyz[2]);
	input_event(stat->input_dev, EV_SYN, SYN_TIME_SEC,
			ktime_to_timespec(ts).tv_sec);
	input_event(stat->input_dev, EV_SYN, SYN_TIME_NSEC,
			ktime_to_timespec(ts).tv_nsec);	
	input_sync(stat->input_dev);
}

static int n2dm_acc_enable(struct n2dm_acc_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = n2dm_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		schedule_delayed_work(&stat->input_work,
			msecs_to_jiffies(stat->pdata->poll_interval));
	}

	return 0;
}

static int n2dm_acc_disable(struct n2dm_acc_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		cancel_delayed_work_sync(&stat->input_work);
		n2dm_acc_device_power_off(stat);
	}

	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = n2dm_acc_i2c_read(stat, &data, 1);
	if (err < 0)
		return err;
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err = -1;
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = n2dm_acc_register_write(stat, x, reg, new_val);
	if (err < 0)
		return err;
	stat->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max((unsigned int)interval_ms, stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	n2dm_acc_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	char range = 2;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range ;
	switch (val) {
	case N2DM_ACC_G_2G:
		range = 2;
		break;
	case N2DM_ACC_G_4G:
		range = 4;
		break;
	case N2DM_ACC_G_8G:
		range = 8;
		break;
	case N2DM_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = N2DM_ACC_G_2G;
		break;
	case 4:
		range = N2DM_ACC_G_4G;
		break;
	case 8:
		range = N2DM_ACC_G_8G;
		break;
	case 16:
		range = N2DM_ACC_G_16G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid range request: %lu,"
				" discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = n2dm_acc_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(&stat->client->dev, "range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		n2dm_acc_enable(stat);
	else
		n2dm_acc_disable(stat);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1, INT1_THRESHOLD_MASK, RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
		struct device_attribute *attr,	char *buf)
{

	return read_single_reg(dev, buf, TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}

static int n2dm_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
				unsigned int delay_ms)
{
	struct n2dm_acc_status *data = container_of(sensors_cdev,
					struct n2dm_acc_status, cdev);		
					
	if (delay_ms < POLL_INTERVAL_MIN_MS)
		delay_ms = POLL_INTERVAL_MIN_MS;
	if (delay_ms > POLL_INTERVAL_MAX_MS)
		delay_ms = POLL_INTERVAL_MAX_MS;
	data->pdata->poll_interval= delay_ms;

	n2dm_acc_update_odr(data, delay_ms);
      
	return 0;
}
static int n2dm_cdev_enable(struct sensors_classdev *sensors_cdev,
				unsigned int enable)
{
	struct n2dm_acc_status *stat = container_of(sensors_cdev,
					struct n2dm_acc_status, cdev);
	unsigned long val;
	val = enable;

	if (val)
		n2dm_acc_enable(stat);
	else
		n2dm_acc_disable(stat);

	return 0;
}

/*add Acc sensor calibration start by jiangchong,2015\05\06*/
static int n2dm_acc_calibrate(struct sensors_classdev *sensors_cdev,
		int axis, int apply_now){
    //struct i2c_client *client = to_i2c_client(dev);
	//struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	
       
	struct n2dm_acc_status *stat = container_of(sensors_cdev,
				struct n2dm_acc_status, cdev);
	
	s16 arry[3];
	int i;

	int val = atomic_read(&stat->enabled);
	int raw_data_mg[3] = {0};
	printk("n2dm cal(): $$$$$begin!!!!!\n");
	memset(stat->cali_data, 0 , sizeof(stat->cali_data));

	if(!val)
		{
               dev_err(&stat->client->dev, " acc has not been open before calibrate phase,erro\n");
              // add acc open here??
	}

	
	//printk("stat cal(): $$$$$begin!!!!!\n");
	arry[0] = 0;
	arry[1] = 0;
	arry[2] = 0;

	if(stat->pdata->poll_interval ==0)
	{
        n2dm_acc_update_odr(stat, CALI_POLL_INTERVAL);
	 stat->pdata->poll_interval = CALI_POLL_INTERVAL;
		}
	for(i=0; i<ACC_CALI_COUNT; i++){
	msleep(stat->pdata->poll_interval);
	n2dm_acc_get_acceleration_data(stat,raw_data_mg);
	arry[0] = arry[0] + raw_data_mg[0];
	arry[1] = arry[1] + raw_data_mg[1];
	arry[2] = arry[2] + raw_data_mg[2];
	//printk("!!!!!!acc rawdata result is %d,%d,%d\n",tj9->raw_data[0],tj9->raw_data[1],tj9->raw_data[2]);
	//printk("^^^^^^acc arry result is %d,%d,%d\n",arry[0],arry[1],arry[2]);
	}
	stat->cali_data[0] = arry[0] / ACC_CALI_COUNT;
	stat->cali_data[1] = arry[1] / ACC_CALI_COUNT;
	stat->cali_data[2] = arry[2] / ACC_CALI_COUNT - 1024;

	#ifdef DEBUG_CALI
	printk("######acc cali result is %d,%d,%d\n",stat->cali_data[0],stat->cali_data[1],stat->cali_data[2]);
	#endif
	snprintf(stat->calibrate_buf, sizeof(stat->calibrate_buf),"%d,%d,%d", stat->cali_data[0],stat->cali_data[1],stat->cali_data[2]);
	sensors_cdev->params = stat->calibrate_buf;

	return 0;
}


static int n2dm_acc_write_calibrate(struct sensors_classdev * sensors_cdev, struct cal_result_t * cal_result)
{
       struct n2dm_acc_status *stat = container_of(sensors_cdev,
					struct n2dm_acc_status, cdev);
	//printk("kxtj9 write(): write start!!!!!\n");				
	stat->cali_data[0] = cal_result->offset_x;
	stat->cali_data[1] = cal_result->offset_y;
	stat->cali_data[2] = cal_result->offset_z;
      #ifdef DEBUG_CALI
	printk("*****acc write result is %d,%d,%d\n",stat->cali_data[0],stat->cali_data[1],stat->cali_data[2]);
      #endif
      return 0;
}
/*add Acc sensor calibration end by jiangchong,2015\05\06*/
#ifdef DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = n2dm_acc_i2c_write(stat, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = n2dm_acc_i2c_read(stat, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct n2dm_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&stat->lock);
	stat->reg_addr = val;
	mutex_unlock(&stat->lock);
	return size;
}
#endif

static struct device_attribute attributes[] = {

	__ATTR(delay, 0664, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable, 0664, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(click_config, 0664, attr_get_click_cfg, attr_set_click_cfg),
	__ATTR(click_source, 0444, attr_get_click_source, NULL),
	__ATTR(click_threshold, 0664, attr_get_click_ths, attr_set_click_ths),
	__ATTR(click_timelimit, 0664, attr_get_click_tlim, attr_set_click_tlim),
	__ATTR(click_timelatency, 0664, attr_get_click_tlat,
							attr_set_click_tlat),
	__ATTR(click_timewindow, 0664, attr_get_click_tw, attr_set_click_tw),

#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void n2dm_acc_input_work_func(struct work_struct *work)
{
	struct n2dm_acc_status *stat;

	int xyz[3] = { 0 };
	int err;

	stat = container_of((struct delayed_work *)work,
			struct n2dm_acc_status, input_work);

	mutex_lock(&stat->lock);
	err = n2dm_acc_get_acceleration_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get_acceleration_data failed\n");
	else{
		xyz[0] = xyz[0] - stat->cali_data[0];
		xyz[1] = xyz[1] - stat->cali_data[1];
		xyz[2] = xyz[2] - stat->cali_data[2];
		#ifdef DEBUG_CALI
             dev_err(&stat->client->dev, "cali_data %d,%d,%d\n",stat->cali_data[0],stat->cali_data[1],stat->cali_data[2]);
		#endif
		n2dm_acc_report_values(stat, xyz);
		}
	schedule_delayed_work(&stat->input_work, msecs_to_jiffies(
			stat->pdata->poll_interval));
	mutex_unlock(&stat->lock);
}

int n2dm_acc_input_open(struct input_dev *input)
{
	struct n2dm_acc_status *stat = input_get_drvdata(input);

	return n2dm_acc_enable(stat);
}

void n2dm_acc_input_close(struct input_dev *dev)
{
	struct n2dm_acc_status *stat = input_get_drvdata(dev);

	n2dm_acc_disable(stat);
}

static int n2dm_acc_validate_pdata(struct n2dm_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)N2DM_ACC_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
		stat->pdata->axis_map_y > 2 ||
		 stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", stat->pdata->axis_map_x,
					stat->pdata->axis_map_y,
						stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1
			|| stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", stat->pdata->negate_x,
				stat->pdata->negate_y, stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int n2dm_acc_input_init(struct n2dm_acc_status *stat)
{
	int err;

	INIT_DELAYED_WORK(&stat->input_work, n2dm_acc_input_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->open = n2dm_acc_input_open;
	stat->input_dev->close = n2dm_acc_input_close;
	stat->input_dev->name = N2DM_ACC_DEV_NAME;
	/* stat->input_dev->name = "accelerometer"; */
	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, stat->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, stat->input_dev->absbit);

	input_set_abs_params(stat->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_WHEEL, INT_MIN,
								INT_MAX, 0, 0);


	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
				"unable to register input device %s\n",
				stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void n2dm_acc_input_cleanup(struct n2dm_acc_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

/*added by chenhui for chip info proc node begin*/

static void *gsensor_config_proc_start(struct seq_file *m, loff_t *pos)
{	
	return seq_list_start_head(&nd_struct_glist_n, *pos);
}

static void *gsensor_config_proc_next(struct seq_file *p, void *v, loff_t *pos)
{	
	return seq_list_next(v, &nd_struct_glist_n, pos);
}

static void gsensor_config_proc_stop(struct seq_file *m, void *v)
{
}

static int gsensor_config_proc_show(struct seq_file *m, void *v)
{
	int retval;
	
	seq_puts(m, "gsensor module\n");
	
	seq_printf(m, "chip name : %s\n", N2DM_ACC_DEV_NAME);	
	
	seq_printf(m, "i2c address  : %d-00%x\n",proc_client->adapter->nr, proc_client->addr);
	
	retval = i2c_smbus_read_byte_data(proc_client, WHO_AM_I);

	if (retval < 0) {
		dev_err(&proc_client->dev, "verify  err!\n");
		return 0;
	}
       seq_printf(m, "chip id : 0x%x\n", retval);
	   
	return 0;
}


static const struct seq_operations proc_config_ops = {	
	.start = gsensor_config_proc_start,	
	.next  = gsensor_config_proc_next,	
	.stop  = gsensor_config_proc_stop,	
	.show  = gsensor_config_proc_show,
};

static int gsensor_config_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &proc_config_ops);
}
/*
static ssize_t gsensor_config_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{

}
*/
static void remove_procfs_interfaces(void)
{
	remove_proc_entry(PROC_GSENSOR_NAME, gsensor_proc_dir_entry);
	remove_proc_entry(PROC_GSENSOR_DIR, NULL);
}
/*added by chenhui for chip info proc node end*/

static int n2dm_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct n2dm_acc_status *stat;

	u32 smbus_func = (I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK);

	int err = -1;

	dev_info(&client->dev, "probe start.\n");

	stat = kzalloc(sizeof(struct n2dm_acc_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

	/* Support for both I2C and SMBUS adapter interfaces. */
	stat->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto exit_check_functionality_failed;
		}
	}


	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->client = client;
	i2c_set_clientdata(client, stat);

	stat->pdata = kmalloc(sizeof(*stat->pdata), GFP_KERNEL);
	if (stat->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto err_mutexunlock;
	}

	if (client->dev.platform_data == NULL) {
		default_n2dm_acc_pdata.gpio_int1 = int1_gpio;
		default_n2dm_acc_pdata.gpio_int2 = int2_gpio;
		memcpy(stat->pdata, &default_n2dm_acc_pdata,
							sizeof(*stat->pdata));
		dev_info(&client->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, client->dev.platform_data,
							sizeof(*stat->pdata));
	}

	err = n2dm_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}
 

	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if (stat->pdata->gpio_int1 >= 0) {
		stat->irq1 = gpio_to_irq(stat->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d, "
							"mapped on gpio:%d\n",
			N2DM_ACC_DEV_NAME, __func__, stat->irq1,
							stat->pdata->gpio_int1);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		stat->irq2 = gpio_to_irq(stat->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d, "
							"mapped on gpio:%d\n",
			N2DM_ACC_DEV_NAME, __func__, stat->irq2,
							stat->pdata->gpio_int2);
	}

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL_REG1] = (ALL_ZEROES |
					N2DM_ACC_ENABLE_ALL_AXES);
	stat->resume_state[RES_CTRL_REG4] = (ALL_ZEROES | CTRL_REG4_BDU_ENABLE);
	
#ifdef N2DM_VDD_HOLD_FOR_BOARD_TEST
	err = n2dm_acc_config_regulator_vdd_hold(stat, true);
	if (err < 0) {
		dev_err(&client->dev, "vdd_hold power on failed: %d\n", err);
		goto err_pdata_init;
	}
#endif

	err = n2dm_acc_config_regulator(stat, true);
	if (err < 0) {
		dev_err(&client->dev, "Configure power failed: %d\n", err);
#ifdef N2DM_VDD_HOLD_FOR_BOARD_TEST
		goto err_vdd_hold_init;
#else
		goto err_pdata_init;
#endif		
	}	

	err = n2dm_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_regulator_init;
	}

	atomic_set(&stat->enabled, 1);

	err = n2dm_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = n2dm_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = n2dm_acc_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}


	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device N2DM_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}
	stat->cdev = sensors_cdev;
	stat->cdev.min_delay = POLL_INTERVAL_MIN_MS * 1000;
	stat->cdev.delay_msec = stat->pdata->poll_interval;
	stat->cdev.sensors_enable = n2dm_cdev_enable;
	stat->cdev.sensors_poll_delay = n2dm_cdev_poll_delay;
	//stat->cdev.resolution = "0.00957031";

            stat->cdev.sensors_calibrate = n2dm_acc_calibrate;
		    printk("ACC cal(): start!!!!!!!\n"); 
            stat->cdev.sensors_write_cal_params =
                        n2dm_acc_write_calibrate;

    memset(&stat->cdev.cal_result, 0 , sizeof(stat->cdev.cal_result));
       /*add ACC sensor calibration end by st jonny zhou,2015\07\06*/
	err = sensors_classdev_register(&client->dev, &stat->cdev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device N2DM_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}	

	n2dm_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	if (stat->pdata->gpio_int1 >= 0) {
		INIT_WORK(&stat->irq1_work, n2dm_acc_irq1_work_func);
		stat->irq1_work_queue =
			create_singlethread_workqueue("n2dm_acc_wq1");
		if (!stat->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(stat->irq1, n2dm_acc_isr1,
			IRQF_TRIGGER_RISING, "n2dm_acc_irq1", stat);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(stat->irq1);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		INIT_WORK(&stat->irq2_work, n2dm_acc_irq2_work_func);
		stat->irq2_work_queue =
			create_singlethread_workqueue("n2dm_acc_wq2");
		if (!stat->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(stat->irq2, n2dm_acc_isr2,
			IRQF_TRIGGER_RISING, "n2dm_acc_irq2", stat);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(stat->irq2);
	}



	mutex_unlock(&stat->lock);

/*added by chenhui for proc info node begin*/
	gsensor_proc_dir_entry = proc_mkdir(PROC_GSENSOR_DIR, NULL);	

	if (!gsensor_proc_dir_entry) {           
		dev_err(&client->dev, "gsensor_proc_dir_entry is null\n");
	}
	proc_create(PROC_GSENSOR_NAME, 0644, NULL, &info_proc_file_ops);               

	proc_client = stat->client;
/*added by chenhui for proc info node end*/

	dev_info(&client->dev, "%s: probed\n", N2DM_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue2:
	if (stat->pdata->gpio_int2 >= 0)
		destroy_workqueue(stat->irq2_work_queue);
err_free_irq1:
	free_irq(stat->irq1, stat);
err_destoyworkqueue1:
	if (stat->pdata->gpio_int1 >= 0)
		destroy_workqueue(stat->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	n2dm_acc_input_cleanup(stat);
err_power_off:
	n2dm_acc_device_power_off(stat);
err_regulator_init:
	n2dm_acc_config_regulator(stat, false);
#ifdef N2DM_VDD_HOLD_FOR_BOARD_TEST
err_vdd_hold_init:
	n2dm_acc_config_regulator_vdd_hold(stat, false);
#endif
err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);
/* err_freedata: */
	kfree(stat);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", N2DM_ACC_DEV_NAME);
	return err;
}

static int n2dm_acc_remove(struct i2c_client *client)
{

	struct n2dm_acc_status *stat = i2c_get_clientdata(client);

	if (stat->pdata->gpio_int1 >= 0) {
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->gpio_int1);
		destroy_workqueue(stat->irq1_work_queue);
	}

	if (stat->pdata->gpio_int2 >= 0) {
		free_irq(stat->irq2, stat);
		gpio_free(stat->pdata->gpio_int2);
		destroy_workqueue(stat->irq2_work_queue);
	}

	remove_procfs_interfaces();	//added by chenhui for proc node	
	n2dm_acc_input_cleanup(stat);
	n2dm_acc_device_power_off(stat);
	sensors_classdev_unregister(&stat->cdev);
	n2dm_acc_config_regulator(stat, false);
	remove_sysfs_interfaces(&client->dev);

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int n2dm_acc_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct n2dm_acc_status *stat = i2c_get_clientdata(client);
	//pr_err("%s: n2dm_acc_resume\n", N2DM_ACC_DEV_NAME);
	if (stat->on_before_suspend)
		return n2dm_acc_enable(stat);
	return 0;
}

static int n2dm_acc_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct n2dm_acc_status *stat = i2c_get_clientdata(client);
	//pr_err("%s: n2dm_acc_suspend\n", N2DM_ACC_DEV_NAME);
	stat->on_before_suspend = atomic_read(&stat->enabled);
	return n2dm_acc_disable(stat);
}
#else
#define n2dm_acc_suspend	NULL
#define n2dm_acc_resume	NULL
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(n2dm_pm_ops, n2dm_acc_suspend, n2dm_acc_resume);

static const struct i2c_device_id n2dm_acc_id[]
		= { { N2DM_ACC_DEV_NAME, 0 }, { }, };
		
static struct of_device_id n2dm_match_table[] = {
	{ .compatible = "st,n2dm", },
	{ },
};		

MODULE_DEVICE_TABLE(i2c, n2dm_acc_id);


static struct i2c_driver n2dm_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = N2DM_ACC_DEV_NAME,
			.of_match_table = n2dm_match_table,
			.pm	= &n2dm_pm_ops,			
		  },
	.probe = n2dm_acc_probe,
	.remove = n2dm_acc_remove,
	//.suspend = n2dm_acc_suspend,
	//.resume = n2dm_acc_resume,
	.id_table = n2dm_acc_id,
};

static int __init n2dm_acc_init(void)
{
	//pr_info("%s accelerometer driver: init\n",
	//					N2DM_ACC_DEV_NAME);
	return i2c_add_driver(&n2dm_acc_driver);
}

static void __exit n2dm_acc_exit(void)
{

	//pr_info("%s accelerometer driver exit\n",
	//					N2DM_ACC_DEV_NAME);

	i2c_del_driver(&n2dm_acc_driver);
	return;
}

module_init(n2dm_acc_init);
module_exit(n2dm_acc_exit);

MODULE_DESCRIPTION("n2dm accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

