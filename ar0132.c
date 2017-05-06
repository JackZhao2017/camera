/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"


#define		HOSTCMD_CONFIGPOWERMGT 			0X8102
#define 	HOSTCMD_SYETEM_SET_STATE 		0x8100 
#define 	HOSTCMD_SYETEM_GET_STATE		0x8101
#define 	HOSTCMD_REGISTER 				0x0040
#define 	AR0132_CMD_PARAM_0              0xFC00

#define  	SYSCTL_RESETREG					0x001A
#define 	SYSCTL_RESET					0X0E05
#define  	SYSCTL_MCUBOOT_OPTION			0X0020

#define     SYS_STATE_ENTER_SOFT_STANDBY    0x5000	
#define     SYS_STATE_LEAVE_SOFT_STANDBY	0x3500
#define     SYS_STATE_ENTER_STREAMING       0x3400
#define     SYS_STATE_ENTER_SUSPEND			0x4000

#define 	AR0132_CHANGE_CONFIG			0x2800
#define 	AR0132_CHIP_ID_ADDRESS			0x0000
#define 	CHIP_ID							0x0062
#define 	AR0132_RESET_REG 				0x001A
#define 	AR0132_RESET					0x0E05
#define 	AR0132_NORMAl					0x0E04


#define AR0132_VOLTAGE_ANALOG               2800000
#define AR0132_VOLTAGE_DIGITAL_CORE         1500000
#define AR0132_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define AR0132_XCLK_MIN 6000000
#define AR0132_XCLK_MAX 24000000

#define AR0132_CHIP_ID_HIGH_BYTE	0x0000
#define AR0132_CHIP_ID_LOW_BYTE		0x0001

enum ar0132_mode {
	ar0132_mode_MIN = 0,
	ar0132_mode_MAX = 1
};

enum ar0132_frame_rate {
	ar0132_30_fps
};

static int ar0132_framerates[] = {
	[ar0132_30_fps] = 30,
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ar0132_mode_info {
	enum ar0132_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};
typedef struct ar0132reg_val{
	u32  len;
	u32 u32val;
} ar0132reg_val;
typedef struct ar0132reg_info{
	char name[20];
	u16	 regaddr;
	ar0132reg_val regval;
}ar0132reg_info;
typedef struct {
	u32 y_start;
	u32	x_start;
	u32 y_end;
	u32 x_end;
	u32	pixclk;
	u32	framelen;
	u32 linelen;
}ar0132out_fmt;
#define infosize 41
ar0132reg_info reg_addr_info[infosize]={
	{"y_start   ",0xc804,{2,0}},{"x_start   ",0xc806,{2,0}},{"y_end     ",0xc808,{2,0}},{"x_end     ",0xc80a,{2,0}},
	{"pixclk    ",0xc80c,{4,0}},{"framelen  ",0xc814,{2,0}},{"linelen   ",0xc816,{2,0}},{"integtimin",0xc810,{2,0}},
	{"integtimax",0xc812,{2,0}},{"mode      ",0xc88c,{1,0}},{"modetext  ",0xc88f,{1,0}},{"outputwith",0xca90,{2,0}},
	{"outputheig",0xca92,{2,0}},{"yuvfmt    ",0xca94,{2,0}},{"framescan ",0xc8a4,{2,0}},{"outfmt    ",0xca96,{1,0}},
	{"state     ",0xcc00,{2,0}},{"mgr_mode  ",0xcc02,{2,0}},{"sysmgr    ",0xDC07,{1,0}},{"bayerpath ",0xca97,{1,0}},
	{"bayerwidth",0xca98,{1,0}},
	{"y_offset  ",0xca99,{1,0}},{"parallectl",0xca9c,{2,0}},{"sysmgrstat",0xDC00,{2,0}},
	{"cfgmod    ",0xdc07,{1,0}},{"cfgstatus ",0xdc09,{1,0}},{"altmmode  ",0xc988,{2,0}},{"altmlgamma",0xc990,{2,0}},
	{"altmhgamma",0xc992,{2,0}},{"altmk1slop",0xc994,{2,0}},{"altmk1min ",0xc996,{2,0}},{"altmk1max ",0xc998,{2,0}},
	{"altmdarkbm",0xc99a,{2,0}},{"altmbright",0xc99c,{2,0}},{"altmk1damp",0xc99e,{2,0}},{"sharpdark ",0xc9a0,{2,0}},
	{"sharpbrigt",0xc9a2,{2,0}},{"strength_d",0xc9a4,{2,0}},{"strength_b",0xc9a6,{2,0}},{"fcorrect	",0xc818,{2,0}},
	{"cfgyuning ",0xc830,{4,0}},{"baseaddr0 ",0xc834,{1,0}},{"baseaddr1 ",0xc835,{1,0}},{"calib_xoff",0xc8a8,{1,0}},
	{"calib_yoff",0xc8a9,{1,0}},
};
/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data ar0132_data;
static int pwn_gpio;

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;

static int ar0132_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ar0132_remove(struct i2c_client *client);

static s32 ar0132_read_reg(u16 reg, u8 *val);
static s32 ar0132_read_reg16(u16 reg, u16 *val);
static s32 ar0132_read_regn(ar0132reg_info *reginfo);

static s32 ar0132_write_reg(u16 reg, u8 val);
static s32 ar0132_write_reg16(u16 reg,u16 val);
static s32 ar0132_write_regn(ar0132reg_info reginfo);

static int ar0132_config_cmd(u16 val);
static int ar0132_get_reg(int inforang);
static int ar0132_write_HCI(u16 state);
static int ar0132_retset(void);
static int ar0132_isdoorbellClear(void);

static const struct i2c_device_id ar0132_id[] = {
	{"ov564x", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ar0132_id);

static struct i2c_driver ar0132_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov564x",
		  },
	.probe  = ar0132_probe,
	.remove = ar0132_remove,
	.id_table = ar0132_id,
};

static void ar0132_standby(s32 enable)
{
	if (enable)
		gpio_set_value(pwn_gpio, 1);
	else
		gpio_set_value(pwn_gpio, 0);

	msleep(2);
}

static void ar0132_reset(void)
{
	/* camera power down */
	gpio_set_value(pwn_gpio, 1);
	msleep(5);

	gpio_set_value(pwn_gpio, 0);
	msleep(5);

	gpio_set_value(pwn_gpio, 1);
}
static s32 ar0132_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(ar0132_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}
	return 0;
}
static s32 ar0132_write_reg16(u16 reg,u16 val)
{
	u8 au8Buf[4] = {0};
	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val >>8;
	au8Buf[3] = val&0xff;
	if (i2c_master_send(ar0132_data.i2c_client, au8Buf, 4) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}
	return 0;
}
static s32 ar0132_write_regn(ar0132reg_info reginfo)
{
	u8 au8Buf[6] = {0},i=0;
	ar0132reg_val regval=reginfo.regval;
	au8Buf[0] = reginfo.regaddr>>8;
	au8Buf[1] = reginfo.regaddr& 0xff;	
	for(i=2;i<2+regval.len;i++){
		au8Buf[i]=regval.u32val>>((regval.len+1-i)*8);
	}
	if (i2c_master_send(ar0132_data.i2c_client, au8Buf, regval.len+2) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reginfo.regaddr, regval.u32val);
		return -1;
	}
	return 0;
}
static s32 ar0132_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ar0132_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ar0132_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}
static s32 ar0132_read_reg16(u16 reg, u16 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal[2] = {0};
	u16 u16RdVal=0;
	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;
	
	if (2 != i2c_master_send(ar0132_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (2 != i2c_master_recv(ar0132_data.i2c_client, &u8RdVal, 2)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u16RdVal);
		return -1;
	}
	u16RdVal=u8RdVal[0];
	u16RdVal=(u16RdVal<<8);
	u16RdVal+=u8RdVal[1];
	*val = u16RdVal;

	return u16RdVal;
}
static s32 ar0132_read_regn(ar0132reg_info *reginfo)
{
	u8 au8RegBuf[2] = {0},i=0;
	u8 u8RdVal[4] = {0};
	u32 u32RdVal=0;
	struct ar0132reg_val regval=reginfo->regval;
	au8RegBuf[0] = reginfo->regaddr>> 8;
	au8RegBuf[1] = reginfo->regaddr & 0xff;
	if (2 != i2c_master_send(ar0132_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reginfo->regaddr);
		return -1;
	}

	if (regval.len!= i2c_master_recv(ar0132_data.i2c_client, &u8RdVal, regval.len)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__,reginfo->regaddr, u32RdVal);
		return -1;
	}
	u32RdVal=0;
	for(i=0;i<regval.len;i++)
	{
		u32RdVal=u32RdVal<<8;
		u32RdVal+=u8RdVal[i];
	}
	reginfo->regval.u32val=u32RdVal;

	return u32RdVal;
}

static 	int ar0132_getoutfmt(ar0132out_fmt *fmt)
{
	s32 i=0;
	ar0132reg_info reginfo;
	u32 *fmtval=NULL;
	fmtval=(u32*)fmt;
	int ret;
	for(i=0;i<7;i++){
		reginfo=reg_addr_info[i];
		ret=ar0132_read_regn(&reginfo);
		if(ret<0){
			pr_err("%s:read_regn %d\n",__func__,ret);
			goto err;
		}
		fmtval[i]=reginfo.regval.u32val;
	}
err:
	return 0;
}

static int ar0132_get_reg(int inforang)
{
	s32 i = 0;
	ar0132reg_info reginfo;
	int start=0,stop=inforang;
	int retval = 0;	
	pr_err("-------------%d -- %d reginfo ---------------------------- \n",start,stop);
	for(i=start;i<stop;++i){
		reginfo=reg_addr_info[i];
		retval=ar0132_read_regn(&reginfo);
		if(retval<0){
			pr_err("%d\n",retval);
			goto err;
		}
		pr_err("%s:   reg=0x%x  val=0x%-8x    dec=%d\n",reginfo.name,reginfo.regaddr,reginfo.regval.u32val,reginfo.regval.u32val);
	}
	pr_err("------------------------------------------------------\n");
err:
	return retval;
}
static int ar0132_isdoorbellClear()
{
	int ret=0,count=0;
	u16 retval;
	ret=ar0132_read_reg16(HOSTCMD_REGISTER,&retval);
	pr_err("<%s>:doorbell is  %4x \n",__func__,retval);
	while(retval&0x8000||retval>0)
	{
		ar0132_read_reg16(HOSTCMD_REGISTER,&retval);
		pr_err("<%s>:doorbell is  %4x \n",__func__,retval);
		if(count++>5){
			pr_err("<%s>:doorbell is not clear %x \n",__func__,retval);
			return 1;
		}
		msleep(20);
	}
	return 0;
}
static int ar0132_write_HCI(u16 state)
{
	u16 data;
	int count,ret;
	printk("<%s>:state = % x\n",__func__,state);
	if(ar0132_isdoorbellClear()){
		printk("<%s>:doorbell is not clear \n",__func__);
		return 1;
	}

	ar0132_write_reg16(HOSTCMD_REGISTER,state);
	ar0132_read_reg16(HOSTCMD_REGISTER,&data);
	count=0;
	while(data){
		ar0132_read_reg16(HOSTCMD_REGISTER,&data);
		if(count++>10){
	        ar0132_write_reg16(HOSTCMD_REGISTER,state);
			ar0132_read_reg16(HOSTCMD_REGISTER,&data);
			printk(KERN_INFO "<%s>:Failed to 0x%x STATE: ERROR = 0x%x\n",__func__,state,data);
			break;
		}
		msleep(20);
	}
	if(state !=HOSTCMD_SYETEM_GET_STATE)
		goto out;
	ar0132_read_reg16(AR0132_CMD_PARAM_0,&data);
	data=data>>8;
	switch(data){
	case 0x20:
		printk(KERN_INFO"Current state of AP0100 = idle\n");
		break;
	case 0x31:
		printk(KERN_INFO"<%s>:Current state of AP0100 = streaming\n",__func__);
		break;
	case 0x41:
		printk(KERN_INFO"Current state of AP0100 = suspended\n");
		break;
	case 0x51:
		printk(KERN_INFO"Enter state of AP0100   =soft standby");
		break;
	case 0x53:
		printk(KERN_INFO"Current state of AP0100 = soft standby\n");
		break;
	case 0x55:
		printk(KERN_INFO"Leave state of AP0100   =soft standby");
		break;		
	case 0x5b:
		printk(KERN_INFO"Current state of AP0100 = hard standby\n");
		break;
	default:
		printk(KERN_INFO"<%s>:Current state of AP0100 = unknown 0x%x\n",__func__, data);
		break;
	}
out:
	return 0;
}

static int ar0132_config_cmd(u16 val)
{
	int ret=0,count=0;
	u16 data;
	u16 retval;

	if(ar0132_isdoorbellClear()){
		printk(KERN_INFO"<%s>:doorbell is not clear \n",__func__);
		return -1;
	}

	ret=ar0132_write_reg16(AR0132_CMD_PARAM_0,val);
	if(ret<0){
		pr_err("<%s>:write reg16 %d\n",__func__,ret);
		return ret;
	}
	ret=ar0132_write_reg16(HOSTCMD_REGISTER,HOSTCMD_SYETEM_SET_STATE);
	if(ret<0){
		pr_err("write reg16HOSTCMD_SYETEM_SET_STATE %d\n",ret);
		return ret;
	}
	ret=ar0132_read_reg16(HOSTCMD_REGISTER,&retval);	
	pr_err("<%s>:host register return is  0x%x\n",__func__,retval);
	if(ret<0){
		pr_err("host register return is  0x 0x%x\n",retval);
	}

	count=0;
	while(retval){
		ret=ar0132_read_reg16(HOSTCMD_REGISTER,&retval);
		if(ret<0){
			pr_err("read reg16HOSTCMD_SYETEM_SET_STATE 0x%x\n",retval);
		}
		if(count++>10){
			//ar0132_write_reg16(HOSTCMD_REGISTER,HOSTCMD_SYETEM_SET_STATE);
			ar0132_read_reg16(HOSTCMD_REGISTER,&retval);
			pr_err("Failed to set change config state: ERROR = 0x%x\n",retval);
			break;
		}
		msleep(1000);
	}
	return ret;
}




static int ar0132_retset()
{
	int ret=0;
	ret=ar0132_write_reg16(AR0132_RESET_REG,AR0132_RESET);
	if(ret<0){
		pr_err("reset faild\n");
		return ret;
	}
	msleep(20);
	ret=ar0132_write_reg16(AR0132_RESET_REG,AR0132_NORMAl);
	return ret;
}

static int ar0132_set_rot_mode(struct reg_value *rot_mode)
{

	return 0;
}
static int ar0132_init_mode(enum ar0132_frame_rate frame_rate,
		enum ar0132_mode mode);
static int ar0132_write_snapshot_para(enum ar0132_frame_rate frame_rate,
		enum ar0132_mode mode);
static int ar0132_change_mode(enum ar0132_frame_rate new_frame_rate,
		enum ar0132_frame_rate old_frame_rate,
		enum ar0132_mode new_mode,
		enum ar0132_mode orig_mode)
{

	int retval = 0;
	return retval;
}
static int ar0132_init_mode(enum ar0132_frame_rate frame_rate,
			    enum ar0132_mode mode)
{
	int retval = 0;
	return retval;
}

static int ar0132_write_snapshot_para(enum ar0132_frame_rate frame_rate,
       enum ar0132_mode mode)
{
	int ret = 0;
	return ret ;
}


/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ar0132_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ar0132_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = AR0132_XCLK_MIN;
	p->u.bt656.clock_max = AR0132_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	if (on && !sensor->on) {
		if (io_regulator)
			if (regulator_enable(io_regulator) != 0)
				return -EIO;
		if (core_regulator)
			if (regulator_enable(core_regulator) != 0)
				return -EIO;
		if (gpo_regulator)
			if (regulator_enable(gpo_regulator) != 0)
				return -EIO;
		if (analog_regulator)
			if (regulator_enable(analog_regulator) != 0)
				return -EIO;
		/* Make sure power on */
		ar0132_standby(0);
	} else if (!on && sensor->on) {
		if (analog_regulator)
			regulator_disable(analog_regulator);
		if (core_regulator)
			regulator_disable(core_regulator);
		if (io_regulator)
			regulator_disable(io_regulator);
		if (gpo_regulator)
			regulator_disable(gpo_regulator);

		ar0132_standby(1);
	}

	sensor->on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps, old_fps;	/* target frames per secound */
	enum ar0132_frame_rate new_frame_rate, old_frame_rate;
	int ret = 0;

	/* Make sure power on */
	ar0132_standby(0);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (sensor->streamcap.timeperframe.numerator != 0)
			old_fps = sensor->streamcap.timeperframe.denominator /
				sensor->streamcap.timeperframe.numerator;
		else
			old_fps = 30;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ar0132_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ar0132_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ar0132_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ar0132_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ar0132_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ar0132_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ar0132_data.ae_mode;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	struct sensor_data *sensor = s->priv;
	__u32 captureMode = sensor->streamcap.capturemode;
	struct reg_value *rot_mode = NULL;

	pr_debug("In ar0132:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_MXC_ROT:
	case V4L2_CID_MXC_VF_ROT:
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > ar0132_mode_MAX)
		return -EINVAL;
	fsize->pixel_format = ar0132_data.pix.pixelformat;
	fsize->discrete.width =ar0132_data.pix.width;
	fsize->discrete.height =ar0132_data.pix.height;
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	int i, j, count;

	if (fival->index < 0 || fival->index > ar0132_mode_MAX)
		return -EINVAL;

	if (fival->pixel_format == 0 || fival->width == 0 ||
			fival->height == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;

	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ar0132_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > 0)	/* only 1 pixelformat support so far */
		return -EINVAL;
	fmt->pixelformat = ar0132_data.pix.pixelformat;
	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ar0132_frame_rate frame_rate;

	ar0132_data.on = true;

	/* mclk */
	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

err:
	return retval;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ar0132_ioctl_desc[] = {
	{ vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ vidioc_int_dev_exit_num, ioctl_dev_exit},
	{ vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power },
	{ vidioc_int_g_ifparm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ifparm },
/*	{ vidioc_int_g_needs_reset_num,
	  (v4l2_int_ioctl_func *)ioctl_g_needs_reset }, */
/*	{ vidioc_int_reset_num,
	  (v4l2_int_ioctl_func *)ioctl_reset }, */
	{ vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init },
	{ vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
/*	{ vidioc_int_try_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_try_fmt_cap }, */
	{ vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
/*	{ vidioc_int_s_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_s_fmt_cap }, */
	{ vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm },
/*	{ vidioc_int_queryctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_queryctrl }, */
	{ vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl },
	{ vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ vidioc_int_enum_frameintervals_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ vidioc_int_g_chip_ident_num,
	  (v4l2_int_ioctl_func *)ioctl_g_chip_ident },
};

static struct v4l2_int_slave ar0132_slave = {
	.ioctls = ar0132_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ar0132_ioctl_desc),
};

static struct v4l2_int_device ar0132_int_device = {
	.module = THIS_MODULE,
	.name = "ar0132",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ar0132_slave,
	},
};

/*!
 * ar0132 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ar0132_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;
	ar0132out_fmt fmt;
	u16  u16val;
	/* ar0132 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "ar0132 setup pinctrl failed!");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(pwn_gpio)) {
		dev_warn(dev, "no sensor pwdn pin available");
		return -EINVAL;
	}
	printk(KERN_INFO"f_get_named_gpio pwn_gpio %d \n",pwn_gpio);
	retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
					"ar0132_pwdn");
	if (retval < 0)
		return retval;

	/* Set initial values for the sensor struct. */
	memset(&ar0132_data, 0, sizeof(ar0132_data));
	ar0132_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ar0132_data.sensor_clk)) {
		/* assuming clock enabled by default */
		ar0132_data.sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(ar0132_data.sensor_clk);
	}
	printk(KERN_INFO"devm_clk_get ar0132_data.sensor_clk %d \n",ar0132_data.sensor_clk);
	retval = of_property_read_u32(dev->of_node, "mclk",
					(u32 *) &(ar0132_data.mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}
	printk(KERN_INFO"of_property_read_u32 ar0132_data.mclk %d \n",ar0132_data.mclk);
	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ar0132_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}
	printk(KERN_INFO"of_property_read_u32 ar0132_data.mclk_source %d \n",ar0132_data.mclk_source);
	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ar0132_data.csi));
	if (retval) {
		dev_err(dev, "csi_id missing or invalid\n");
		return retval;
	}
	printk(KERN_INFO"of_property_read_u32 ar0132_data.csi %d \n",ar0132_data.csi);
	clk_prepare_enable(ar0132_data.sensor_clk);

	ar0132_data.io_init = ar0132_reset;
	ar0132_data.i2c_client = client;
	ar0132_data.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	ar0132_data.pix.width = 640;
	ar0132_data.pix.height = 480;
	ar0132_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ar0132_data.streamcap.capturemode = 0;
	ar0132_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ar0132_data.streamcap.timeperframe.numerator = 1;

	ar0132_reset();

	ar0132_standby(0);

	retval = ar0132_read_reg(AR0132_CHIP_ID_HIGH_BYTE, &chip_id_high);
	if (retval < 0 || chip_id_high != 0x0) {
		pr_warning("camera ar0132 is not found %d\n",chip_id_high);
		clk_disable_unprepare(ar0132_data.sensor_clk);
		return -ENODEV;
	}
	retval = ar0132_read_reg(AR0132_CHIP_ID_LOW_BYTE, &chip_id_low);
	if (retval < 0 || chip_id_low != 0x62) {
		pr_warning("camera ar0132 is not found %d\n",chip_id_low);
		clk_disable_unprepare(ar0132_data.sensor_clk);
		return -ENODEV;
	}

	pr_warning("reset camera \n");
	ar0132_retset();
	ar0132_get_reg(infosize);
    ar0132_config_cmd(AR0132_CHANGE_CONFIG);
    ar0132_get_reg(infosize);
    ar0132_getoutfmt(&fmt);
    ar0132_read_reg16(0xca90,&u16val);
 	ar0132_data.pix.width  =u16val;
 	ar0132_read_reg16(0xca92,&u16val);
	ar0132_data.pix.height =u16val;   
	if((fmt.framelen>0)&&(fmt.linelen>0))
	ar0132_data.streamcap.timeperframe.denominator = fmt.pixclk/fmt.framelen/fmt.linelen;
	ar0132_data.streamcap.timeperframe.numerator = 1;

	pr_err(  "y_start:%-8d  \n"
             "x_start:%-8d  \n"
             "y_end:  %-8d  \n"
             "x_end:  %-8d  \n"
             "pixclk: %-8d  \n"
             "framelen:%-8d \n"
             "linelen: %-8d \n",fmt.y_start,fmt.x_start,fmt.y_end,fmt.x_end,fmt.pixclk,fmt.framelen,fmt.linelen);

	pr_err(  "width : %d  \n"
             "height: %d  \n"
             "fps   : %d  \n",ar0132_data.pix.width,ar0132_data.pix.height,ar0132_data.streamcap.timeperframe.denominator);

	if(ar0132_write_HCI(HOSTCMD_SYETEM_GET_STATE)){
		pr_warning("write hosecmd_system_get_state faild\n");
	}
	ar0132_standby(1);
	ar0132_int_device.priv = &ar0132_data;
	retval = v4l2_int_device_register(&ar0132_int_device);
	clk_disable_unprepare(ar0132_data.sensor_clk);
	pr_info("camera ar0132 is found\n");
	return retval;
}

/*!
 * ar0132 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ar0132_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&ar0132_int_device);

	if (gpo_regulator)
		regulator_disable(gpo_regulator);

	if (analog_regulator)
		regulator_disable(analog_regulator);

	if (core_regulator)
		regulator_disable(core_regulator);

	if (io_regulator)
		regulator_disable(io_regulator);

	return 0;
}

/*!
 * ar0132 init function
 * Called by insmod ar0132_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ar0132_init(void)
{
	u8 err;

	err = i2c_add_driver(&ar0132_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * AR0132 cleanup function
 * Called on rmmod ar0132_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ar0132_clean(void)
{
	i2c_del_driver(&ar0132_i2c_driver);
}

module_init(ar0132_init);
module_exit(ar0132_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ar0132 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
