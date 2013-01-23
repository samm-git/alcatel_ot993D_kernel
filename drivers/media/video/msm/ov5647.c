/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define DEBUG
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <mach/camera.h>
#include <media/msm_camera.h>
#include "ov5647.h"

/* 16bit address - 8 bit context register structure */
#define Q8	0x00000100
#define Q10	0x00000400
#define OV5647_AUTO_FOCUS

#define REG_OV5647_GAIN_MSB           0x350A
#define REG_OV5647_GAIN_LSB           0x350B
#define REG_OV5647_LINE_HSB           0x3500
#define REG_OV5647_LINE_MSB           0x3501
#define REG_OV5647_LINE_LSB           0x3502

/* MCLK */
#define OV5647_MASTER_CLK_RATE 24000000

/* AF Total steps parameters */
#define OV5647_TOTAL_STEPS_NEAR_TO_FAR	32

#define OV5647_REG_PREV_FRAME_LEN_1	31
#define OV5647_REG_PREV_FRAME_LEN_2	32
#define OV5647_REG_PREV_LINE_LEN_1	33
#define OV5647_REG_PREV_LINE_LEN_2	34

#define OV5647_REG_SNAP_FRAME_LEN_1	15
#define OV5647_REG_SNAP_FRAME_LEN_2	16
#define OV5647_REG_SNAP_LINE_LEN_1	17
#define OV5647_REG_SNAP_LINE_LEN_2	18
#define MSB                             1
#define LSB                             0
static int is_flash = 0;
/* Debug switch */
#ifdef CDBG
#undef CDBG
#endif
#ifdef CDBG_HIGH
#undef CDBG_HIGH
#endif

//#define OV5647_VERBOSE_DGB

#ifdef OV5647_VERBOSE_DGB
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#define CDBG_HIGH(fmt, args...) pr_debug(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#define CDBG_HIGH(fmt, args...) pr_debug(fmt, ##args)
#endif

#ifdef OV5647_AUTO_FOCUS
static uint8_t  mode_mask = 0x09;
#endif
struct ov5647_work_t {
	struct work_struct work;
};

static struct ov5647_work_t *ov5647_sensorw;
#ifdef OV5647_AUTO_FOCUS
static struct ov5647_work_t *ov5647_af_sensorw;
static struct i2c_client *ov5647_af_client;
#endif
static struct i2c_client *ov5647_client;

struct ov5647_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;

	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum ov5647_resolution_t prev_res;
	enum ov5647_resolution_t pict_res;
	enum ov5647_resolution_t curr_res;
	enum ov5647_test_mode_t  set_test;
};

static bool CSI_CONFIG;
static struct ov5647_ctrl_t *ov5647_ctrl;

static DECLARE_WAIT_QUEUE_HEAD(ov5647_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(ov5647_af_wait_queue);
DEFINE_MUTEX(ov5647_mut);

static uint16_t prev_line_length_pck;
static uint16_t prev_frame_length_lines;
static uint16_t snap_line_length_pck;
static uint16_t snap_frame_length_lines;

static int ov5647_i2c_rxdata(unsigned short saddr,
		unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(ov5647_client->adapter, msgs, 2) < 0) {
		CDBG("ov5647_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

static int32_t ov5647_i2c_txdata(unsigned short saddr,
		unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	if (i2c_transfer(ov5647_client->adapter, msg, 1) < 0) {
		CDBG("ov5647_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

int32_t ov5647_i2c_read(unsigned short raddr,
		unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[2];

	if (!rdata)
		return -EIO;
	CDBG("%s:saddr:0x%x raddr:0x%x data:0x%x",__func__, ov5647_client->addr, raddr, *rdata);
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = ov5647_i2c_rxdata(ov5647_client->addr >> 1, buf, 1);
	if (rc < 0) {
		CDBG("ov5647_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = buf[0];
	CDBG("ov5647_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);

	return rc;
}
EXPORT_SYMBOL(ov5647_i2c_read);

int32_t ov5647_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	rc = ov5647_i2c_txdata(ov5647_client->addr >> 1, buf, 3);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
				waddr, bdata);
	}
	return rc;
}
EXPORT_SYMBOL(ov5647_i2c_write_b_sensor);
static int32_t ov5647_i2c_write_b_table(struct ov5647_i2c_reg_conf const
		*reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num; i++) {
		rc = ov5647_i2c_write_b_sensor(reg_conf_tbl->waddr,
				reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}
#ifdef OV5647_AUTO_FOCUS
static int32_t ov5647_af_i2c_txdata(unsigned short saddr,
		unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};
	if (i2c_transfer(ov5647_af_client->adapter, msg, 1) < 0) {
		pr_err("ov5647_af_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t ov5647_af_i2c_write_b_sensor(uint8_t waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));
	buf[0] = waddr;
	buf[1] = bdata;
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = ov5647_af_i2c_txdata(ov5647_af_client->addr, buf, 2);
	if (rc < 0) {
		pr_err("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
				waddr, bdata);
	}
	return rc;
}
#endif
static void ov5647_start_stream(void)
{
    CDBG("CAMERA_DBG: 0x4202 0x0, stream on...\r\n");
    ov5647_i2c_write_b_sensor(0x4202, 0x00);/* streaming on */
}

static void ov5647_stop_stream(void)
{
    CDBG("CAMERA_DBG: 0x4202 0xf, stream off...\r\n");
    ov5647_i2c_write_b_sensor(0x4202, 0x0f);/* streaming off */
}

static void ov5647_group_hold_on(void)
{
	ov5647_i2c_write_b_sensor(0x3208, 0x0);
}

static void ov5647_group_hold_off(void)
{
	ov5647_i2c_write_b_sensor(0x3208, 0x10);
	ov5647_i2c_write_b_sensor(0x3208, 0xa0);
}

static void ov5647_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;
	uint32_t preview_pclk=0x37, snapshot_pclk=0x4f;

	d1 = (prev_frame_length_lines * 0x00000400) / snap_frame_length_lines;
	d2 = (prev_line_length_pck * 0x00000400) / snap_line_length_pck;
	divider = (d1 * d2*preview_pclk/snapshot_pclk) / 0x400;
	CDBG(KERN_ERR "ov5647_get_pict_fps divider = %d", divider );
	/*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
}

static uint16_t ov5647_get_prev_lines_pf(void)
{
	if (ov5647_ctrl->prev_res == QTR_SIZE)
		return prev_frame_length_lines;
	else
		return snap_frame_length_lines;
}

static uint16_t ov5647_get_prev_pixels_pl(void)
{
	if (ov5647_ctrl->prev_res == QTR_SIZE)
		return prev_line_length_pck;
	else
		return snap_line_length_pck;
}

static uint16_t ov5647_get_pict_lines_pf(void)
{
	if (ov5647_ctrl->pict_res == QTR_SIZE)
		return prev_frame_length_lines;
	else
		return snap_frame_length_lines;
}

static uint16_t ov5647_get_pict_pixels_pl(void)
{
	if (ov5647_ctrl->pict_res == QTR_SIZE)
		return prev_line_length_pck;
	else
		return snap_line_length_pck;
}

static uint32_t ov5647_get_pict_max_exp_lc(void)
{
	return snap_frame_length_lines * 24;
}

static int32_t ov5647_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;

	ov5647_ctrl->fps_divider = fps->fps_div;
	ov5647_ctrl->pict_fps_divider = fps->pict_fps_div;

	if (ov5647_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		total_lines_per_frame = (uint16_t)
		((prev_frame_length_lines * ov5647_ctrl->fps_divider) / 0x400);
	} else {
		total_lines_per_frame = (uint16_t)
		((snap_frame_length_lines * ov5647_ctrl->fps_divider) / 0x400);
	}

	ov5647_group_hold_on();
	rc = ov5647_i2c_write_b_sensor(0x380e,
			((total_lines_per_frame & 0xFF00) >> 8));
	rc = ov5647_i2c_write_b_sensor(0x380f,
			(total_lines_per_frame & 0x00FF));
	ov5647_group_hold_off();

	return rc;
}

static inline uint8_t ov5647_byte(uint16_t word, uint8_t offset)
{
	return word >> (offset * BITS_PER_BYTE);
}

static int32_t ov5647_write_exp_gain(uint16_t gain, uint32_t line)
{
	int rc = 0;
	static uint16_t max_line = 984;
	u8 intg_time_hsb,intg_time_msb, intg_time_lsb;
	uint8_t gain_lsb, gain_hsb;
	ov5647_ctrl->my_reg_gain = gain;
	ov5647_ctrl->my_reg_line_count = (uint16_t)line;

	CDBG(KERN_ERR "exposure set during preview for register is 0x%x, 0x%x, %d",\
		gain, line, line);

	gain_lsb = (uint8_t) (ov5647_ctrl->my_reg_gain);
	gain_hsb = (uint8_t)((ov5647_ctrl->my_reg_gain& 0x300)>>8);
	//adjust frame rate
	ov5647_group_hold_on();
	if (line > 980) {
		rc = ov5647_i2c_write_b_sensor( 0x380E, (uint8_t)((line+4) >> 8) ) ;
		rc = ov5647_i2c_write_b_sensor( 0x380F, (uint8_t)((line+4) & 0x00FF) ) ;
		max_line = line + 4;
	} else if (max_line > 984) {
		rc = ov5647_i2c_write_b_sensor( 0x380E, (uint8_t)(984 >> 8) ) ;
		rc = ov5647_i2c_write_b_sensor( 0x380F, (uint8_t)(984 & 0x00FF) ) ;
		max_line = 984;
	}

	ov5647_group_hold_off();
	line = line<<4;
	// ov5647 need this operation
	intg_time_hsb = (u8)(line>>16);
	intg_time_msb = (u8) ((line & 0xFF00) >> 8);
	intg_time_lsb = (u8) (line& 0x00FF);

	rc = ov5647_i2c_write_b_sensor( REG_OV5647_LINE_HSB, intg_time_hsb) ;
	udelay(500);
	rc = ov5647_i2c_write_b_sensor( REG_OV5647_LINE_MSB, intg_time_msb) ;
	udelay(500);
	rc = ov5647_i2c_write_b_sensor( REG_OV5647_LINE_LSB, intg_time_lsb) ;
	udelay(500);


	rc = ov5647_i2c_write_b_sensor( REG_OV5647_GAIN_MSB, gain_hsb) ;
	udelay(500);
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_LSB, gain_lsb) ;
	udelay(500);

	return rc;
}


static int32_t ov5647_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	static uint16_t max_line = 1964;
	int rc = 0;
	uint8_t gain_lsb, gain_hsb;
	u8 intg_time_hsb,intg_time_msb, intg_time_lsb;

	ov5647_ctrl->my_reg_gain = gain;
	ov5647_ctrl->my_reg_line_count = (uint16_t)line;

	gain_lsb = (uint8_t) (ov5647_ctrl->my_reg_gain);
	gain_hsb = (uint8_t)((ov5647_ctrl->my_reg_gain& 0x300)>>8);

	CDBG(KERN_ERR "exposure set during snapshot for register is 0x%x, 0x%x, %d",\
	gain, line, line);

	ov5647_group_hold_on();
	if (line > 1964) {
		rc = ov5647_i2c_write_b_sensor(0x380E, (uint8_t)((line+4) >> 8) ) ;
		rc = ov5647_i2c_write_b_sensor( 0x380F, (uint8_t)((line+4) & 0x00FF) ) ;
		max_line = line + 4;
	} else if (max_line > 1968) {
		rc = ov5647_i2c_write_b_sensor( 0x380E, (uint8_t)(1968 >> 8) ) ;
		rc = ov5647_i2c_write_b_sensor( 0x380F, (uint8_t)(1968 & 0x00FF) ) ;
		max_line = 1968;
	}
	ov5647_group_hold_off();
	line = line<<4;
	// ov5647 need this operation
	intg_time_hsb = (u8)(line>>16);
	intg_time_msb = (u8) ((line & 0xFF00) >> 8);
	intg_time_lsb = (u8) (line& 0x00FF);

	//FIXME for BLC trigger
	rc = ov5647_i2c_write_b_sensor( REG_OV5647_LINE_HSB, intg_time_hsb) ;
	udelay(500);
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_MSB, intg_time_msb) ;
	udelay(500);
	rc = ov5647_i2c_write_b_sensor( REG_OV5647_LINE_LSB, intg_time_lsb) ;
	udelay(500);

	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_MSB, gain_hsb) ;
	udelay(500);
	rc = ov5647_i2c_write_b_sensor( REG_OV5647_GAIN_LSB, gain_lsb^0x1) ;

	mdelay(170);

	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_HSB, intg_time_hsb) ;
	udelay(500);
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_MSB, intg_time_msb) ;
	udelay(500);
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_LINE_LSB, intg_time_lsb) ;
	udelay(500);

	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_MSB, gain_hsb) ;
	udelay(500);
	rc = ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_LSB, gain_lsb) ;


	return rc;

}
#ifdef OV5647_AUTO_FOCUS
static int32_t ov5647_move_focus(int direction, int32_t num_steps)
{
	uint8_t   code_val_msb = 0;
	uint8_t   code_val_lsb = 0;
	int16_t   step_direction,actual_step,next_position;
	int rc;

	if (num_steps == 0) {
		return 0;
	}

	if ( direction == MOVE_NEAR ) {
		step_direction = 20;
	} else if ( direction == MOVE_FAR) {
		step_direction = -20;
	} else {
		return -EINVAL;
	}

	actual_step = (int16_t)(step_direction * num_steps);
	next_position = (int16_t)ov5647_ctrl->curr_lens_pos + actual_step;
	if(next_position < 0) {
		CDBG(KERN_ERR "ov5647_move_focus:OV5647 position(=%d) out of range",next_position);
		next_position = 0;
	}
	if(next_position > 0x3FF) {
		CDBG(KERN_ERR "ov5647_move_focus:OV5647 position(=%d) out of range",next_position);
		next_position = 0x3FF;
	}
	ov5647_ctrl->curr_lens_pos = next_position;

	code_val_msb = (uint8_t)((ov5647_ctrl->curr_lens_pos & 0x03FF) >> 4);
	code_val_lsb = (uint8_t)((ov5647_ctrl->curr_lens_pos & 0x000F) << 4);
	code_val_lsb |= mode_mask;

	rc = ov5647_af_i2c_write_b_sensor(code_val_msb, code_val_lsb);
	if ( rc != 0) {// DAC setting
		//printk(KERN_ERR "ov5647_move_focus:WRITE ERROR lsb = 0x%x, msb = 0x%x",code_val_lsb, code_val_msb );
	} else {
		//printk(KERN_ERR "ov5647_move_focus:Successful lsb = 0x%x, msb = 0x%x",code_val_lsb, code_val_msb );
		mdelay(10); //delay may set based on the steps moved when I2C write successful.
	}
	return 0;
}

static int32_t ov5647_set_default_focus(uint8_t af_step)
{
	uint8_t  code_val_msb = 0;
	uint8_t  code_val_lsb = 0;
	int 	rc = 0;

	ov5647_ctrl->curr_lens_pos = 200;


	code_val_msb = (ov5647_ctrl->curr_lens_pos & 0x03FF) >> 4;
	code_val_lsb = (ov5647_ctrl->curr_lens_pos & 0x000F) << 4;
	code_val_lsb |= mode_mask;

	CDBG(KERN_ERR "ov5647_set_default_focus:lens pos = %d", ov5647_ctrl->curr_lens_pos);
	rc = ov5647_af_i2c_write_b_sensor(code_val_msb, code_val_lsb);
	if ( rc != 0) // DAC setting
		CDBG(KERN_ERR "ov5647_set_default_focus:WRITE ERROR lsb = 0x%x, msb = 0x%x", code_val_lsb, code_val_msb );
	else
		//printk(KERN_ERR "ov5647_set_default_focus:WRITE successful lsb = 0x%x, msb = 0x%x",code_val_lsb, code_val_msb );
	mdelay(10);
	return 0;
}
#endif

static int32_t ov5647_test(enum ov5647_test_mode_t mo)
{
	int32_t rc = 0;

	if (mo != TEST_OFF)
		rc = ov5647_i2c_write_b_sensor(0x0601, (uint8_t) mo);

	return rc;
}

static void ov5647_reset_sensor(void)
{
	ov5647_i2c_write_b_sensor(0x103, 0x1);
}

static int is_first_preview = 0;
static int32_t ov5647_sensor_setting(int update_type, int rt)
{

	int32_t rc = 0;
	struct msm_camera_csi_params ov5647_csi_params;
	if(update_type != REG_INIT){
		ov5647_stop_stream();
		//wait for clk/data really stop
		if ((rt == FULL_SIZE) ||
			(CSI_CONFIG == 0) )
		{
			mdelay(66);
		}
		else{
			mdelay(150);//266
		}

		CDBG("CAMERA_DBG1: 0x4800 regVal:0x25\r\n");
		ov5647_i2c_write_b_sensor(0x4800, 0x25);/* streaming off */
		mdelay(10);
	}

	if (update_type == REG_INIT) {
		ov5647_reset_sensor();
		ov5647_i2c_write_b_table(ov5647_regs.rec_settings, ov5647_regs.rec_size);
		CSI_CONFIG = 0;
		is_first_preview = 0;
	} else if (update_type == UPDATE_PERIODIC) {
		CDBG("UPDATE_PERIODIC E update_type:%d\n", update_type);
		if (rt == QTR_SIZE)
		{
			ov5647_i2c_write_b_table(ov5647_regs.reg_prev, ov5647_regs.reg_prev_size);
			ov5647_i2c_write_b_sensor(REG_OV5647_GAIN_LSB, 0x0);
			CDBG("CAMERA_DBG:preview settings...\r\n");
		} else if (rt == FULL_SIZE)
		{
			ov5647_i2c_write_b_table(ov5647_regs.reg_snap, ov5647_regs.reg_snap_size);
			CDBG("CAMERA_DBG:snapshot settings...\r\n");
		} else if (rt == HFR_60FPS) {
			rc = ov5647_i2c_write_b_table(ov5647_regs.reg_hfr_60fps, ov5647_regs.reg_hfr_60fps_size);
			CDBG("CAMERA_DBG:hfr 60fps settings...\r\n");
		} else if (rt == HFR_90FPS) {
			rc = ov5647_i2c_write_b_table(ov5647_regs.reg_hfr_90fps, ov5647_regs.reg_hfr_90fps_size);
			CDBG("CAMERA_DBG:hfr 90fps settings...\r\n");
		}
		CDBG("write reg finished\n");
		ov5647_i2c_write_b_sensor(0x0100, 0x01);/* exit standby state */
		mdelay(20);
		if (!CSI_CONFIG) {
			msm_camio_vfe_clk_rate_set(192000000);
			ov5647_csi_params.data_format = CSI_8BIT;
			ov5647_csi_params.lane_cnt = 2;
			ov5647_csi_params.lane_assign = 0xe4;
			ov5647_csi_params.dpcm_scheme = 0;
			ov5647_csi_params.settle_cnt = 10;
			rc = msm_camio_csi_config(&ov5647_csi_params);
			mdelay(20);
			CSI_CONFIG = 1;
			//ov5647_i2c_write_b_sensor(0x0100, 0x01);/* exit standby state */
		}
		CDBG("CAMERA_DBG: 0x4800 regVal:0x04\r\n");
		ov5647_i2c_write_b_sensor(0x4800, 0x04);
		if(is_first_preview)
		{
			//snapshot-preview
			mdelay(133);
		}
		else
		{
			//first run in to preview
			mdelay(10);
			is_first_preview = 1;
		}

		ov5647_start_stream();/* streaming on */
		mdelay(30);
		otp_update_wb(0x48, 0x64);

	}
	return rc;
}

static int32_t ov5647_video_config(int mode)
{
	int32_t rc = 0;
	CDBG("video config\n");
	if (ov5647_sensor_setting(UPDATE_PERIODIC, ov5647_ctrl->prev_res) < 0)
		return rc;
	if (ov5647_ctrl->set_test) {
		if (ov5647_test(ov5647_ctrl->set_test) < 0)
			return  rc;
	}

	ov5647_ctrl->curr_res = ov5647_ctrl->prev_res;
	ov5647_ctrl->sensormode = mode;
	return rc;
}

static int32_t ov5647_snapshot_config(int mode)
{
	int32_t rc = 0;
	
	if(is_flash ==1)
		shutdown_flash_led();

	/*change sensor resolution if needed */
	if (ov5647_ctrl->curr_res != ov5647_ctrl->pict_res) {
		if (ov5647_sensor_setting(UPDATE_PERIODIC, ov5647_ctrl->pict_res) < 0)
			return rc;
	}

	ov5647_ctrl->curr_res = ov5647_ctrl->pict_res;
	ov5647_ctrl->sensormode = mode;

	/*Add by wqf, for flash_led setting
	 * This interface shoud be called after delay ms for next time. 
	 */
	if(is_flash == 1)
		set_flash_led(LEVEL_162MA, 1000);
	
	return rc;
}

static int32_t ov5647_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	/* change sensor resolution if needed */
	if (ov5647_ctrl->curr_res != ov5647_ctrl->pict_res) {
		if (ov5647_sensor_setting(UPDATE_PERIODIC, ov5647_ctrl->pict_res) < 0)
			return rc;
	}

	ov5647_ctrl->curr_res = ov5647_ctrl->pict_res;
	ov5647_ctrl->sensormode = mode;
	return rc;
}

static int32_t ov5647_set_sensor_mode(int mode,
		int res)
{
	int32_t rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		ov5647_ctrl->prev_res = res;
		rc = ov5647_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		ov5647_ctrl->prev_res = res;
		rc = ov5647_snapshot_config(mode);
		break;
	case SENSOR_HFR_60FPS_MODE:
		CDBG("%s HFR 60fps", __func__);
		ov5647_ctrl->prev_res = res;
		rc = ov5647_video_config(mode);
		break;
	case SENSOR_HFR_90FPS_MODE:
		CDBG("%s HFR 90fps", __func__);
		ov5647_ctrl->prev_res = res;
		rc = ov5647_video_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = ov5647_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t ov5647_power_down(void)
{
	ov5647_stop_stream();
	return 0;
}

static int ov5647_probe_init_done(const struct msm_camera_sensor_info *data)
{
	CDBG("probe done\n");
	gpio_direction_output(data->sensor_pwd, 1);
	return 0;
}

static int ov5647_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t regaddress1 = 0x300a;
	uint16_t regaddress2 = 0x300b;
	uint16_t chipid1 = 0;
	uint16_t chipid2 = 0;

	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG(" ov5647_probe_init_sensor is called\n");

	gpio_direction_output(data->sensor_pwd, 0);
	mdelay(5);
	gpio_direction_output(data->sensor_reset, 1);
	mdelay(25);

	ov5647_i2c_read(regaddress1, &chipid1);
	if (chipid1 != 0x56) {
		rc = -ENODEV;
		CDBG("ov5647_probe_init_sensor fail chip id doesnot match\n");
		goto init_probe_fail;
	}

	ov5647_i2c_read(regaddress2, &chipid2);
	if (chipid2 != 0x47) {
		rc = -ENODEV;
		CDBG("ov5647_probe_init_sensor fail chip id doesnot match\n");
		goto init_probe_fail;
	}

	CDBG("ID1: 0x%x\n", chipid1);
	CDBG("ID2: 0x%x\n", chipid2);
	goto init_probe_done;

	init_probe_fail:
	CDBG(" ov5647_probe_init_sensor fails\n");
	ov5647_probe_init_done(data);
	return rc;
	init_probe_done:
	CDBG(" ov5647_probe_init_sensor finishes\n");
	gpio_direction_output(data->sensor_pwd, 1);
	return rc;
}

extern int lcd_camera_power_onoff(int on);

int ov5647_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("Calling ov5647_sensor_open_init\n");

	ov5647_ctrl = kzalloc(sizeof(struct ov5647_ctrl_t), GFP_KERNEL);
	if (!ov5647_ctrl) {
		CDBG("ov5647_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	ov5647_ctrl->fps_divider = 1 * 0x00000400;
	ov5647_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov5647_ctrl->set_test = TEST_OFF;
	ov5647_ctrl->prev_res = QTR_SIZE;
	ov5647_ctrl->pict_res = FULL_SIZE;

	if (data)
		ov5647_ctrl->sensordata = data;

	prev_frame_length_lines =0x3d8;

	prev_line_length_pck =0x768*2;

	snap_frame_length_lines =0x7b0;

	snap_line_length_pck =0xa8c;

	CSI_CONFIG = 0;

	/* enable mclk first */
	msm_camio_clk_rate_set(OV5647_MASTER_CLK_RATE);
//ZhiShen 2011-11-08 add for merge mojito_qrd_pre_cs
	//lcd_camera_power_onoff(1); //power on camera ldo and vreg
	
	gpio_direction_output(data->sensor_pwd, 1);
	gpio_direction_output(data->sensor_reset, 0);
	mdelay(10);
	lcd_camera_power_onoff(1); //power on camera ldo and vreg
	mdelay(10); //waiting for ldo stable
	gpio_direction_output(data->sensor_pwd, 0);
	mdelay(20);
	gpio_direction_output(data->sensor_reset, 1);
	mdelay(25);

	CDBG("init settings\n");

	rc = ov5647_sensor_setting(REG_INIT, ov5647_ctrl->prev_res);
	ov5647_ctrl->fps = 30 * Q8;

	/* enable AF actuator */
	if (ov5647_ctrl->sensordata->vcm_enable) {
		CDBG("enable AF actuator, gpio = %d\n",
			 ov5647_ctrl->sensordata->vcm_pwd);
		rc = gpio_request(ov5647_ctrl->sensordata->vcm_pwd,
						"ov5647_af");
		if (!rc)
			gpio_direction_output(
				ov5647_ctrl->sensordata->vcm_pwd,
				 1);
		else {
			pr_err("ov5647_ctrl gpio request failed!\n");
			goto init_fail;
		}
		mdelay(20);
#ifdef OV5647_AUTO_FOCUS
		rc = ov5647_set_default_focus(0);
#endif
		if (rc < 0) {
			gpio_direction_output(ov5647_ctrl->sensordata->vcm_pwd,
								0);
			gpio_free(ov5647_ctrl->sensordata->vcm_pwd);
		}
	}
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;
init_fail:
	CDBG("init_fail\n");
	ov5647_probe_init_done(data);
	//lcd_camera_power_onoff(0);
init_done:
	CDBG("init_done\n");
	return rc;
}

static int ov5647_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int ov5647_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov5647_wait_queue);
	return 0;
}
#ifdef OV5647_AUTO_FOCUS
static int ov5647_af_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov5647_af_wait_queue);
	return 0;
}
#endif
static const struct i2c_device_id ov5647_af_i2c_id[] = {
	{"ov5647_af", 0},
	{ }
};
#ifdef OV5647_AUTO_FOCUS
static int ov5647_af_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("ov5647_af_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	ov5647_af_sensorw = kzalloc(sizeof(struct ov5647_work_t), GFP_KERNEL);
	if (!ov5647_af_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov5647_af_sensorw);
	ov5647_af_init_client(client);
	ov5647_af_client = client;

	mdelay(50);

	CDBG("ov5647_af_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("ov5647_af_probe failed! rc = %d\n", rc);
	return rc;
}
#endif
static const struct i2c_device_id ov5647_i2c_id[] = {
	{"ov5647", 0},{}
};

static int ov5647_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("ov5647_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	ov5647_sensorw = kzalloc(sizeof(struct ov5647_work_t), GFP_KERNEL);
	if (!ov5647_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov5647_sensorw);
	ov5647_init_client(client);
	ov5647_client = client;

	mdelay(50);

	CDBG("ov5647_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("ov5647_probe failed! rc = %d\n", rc);
	return rc;
}

static int __devexit ov5647_remove(struct i2c_client *client)
{
	struct ov5647_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	ov5647_client = NULL;
	kfree(sensorw);
	return 0;
}
#ifdef OV5647_AUTO_FOCUS
static int __devexit ov5647_af_remove(struct i2c_client *client)
{
	struct ov5647_work_t *ov5647_af = i2c_get_clientdata(client);
	free_irq(client->irq, ov5647_af);
	ov5647_af_client = NULL;
	kfree(ov5647_af);
	return 0;
}
#endif
static struct i2c_driver ov5647_i2c_driver = {
	.id_table = ov5647_i2c_id,
	.probe  = ov5647_i2c_probe,
	.remove = ov5647_i2c_remove,
	.driver = {
		.name = "ov5647",
	},
};
#ifdef OV5647_AUTO_FOCUS
static struct i2c_driver ov5647_af_i2c_driver = {
	.id_table = ov5647_af_i2c_id,
	.probe  = ov5647_af_i2c_probe,
	.remove = __exit_p(ov5647_af_i2c_remove),
	.driver = {
		.name = "ov5647_af",
	},
};
#endif
int ov5647_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
				(void *)argp,
				sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&ov5647_mut);
	CDBG("ov5647_sensor_config: cfgtype = %d\n",
			cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		ov5647_get_pict_fps(
			cdata.cfg.gfps.prevfps,
			&(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf =
			ov5647_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl =
			ov5647_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf =
			ov5647_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl =
			ov5647_get_pict_pixels_pl();
		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc =
			ov5647_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = ov5647_set_fps(&(cdata.cfg.fps));
		break;
	case CFG_SET_EXP_GAIN:
		rc = ov5647_write_exp_gain(cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
		break;
	case CFG_SET_PICT_EXP_GAIN:
		rc = ov5647_set_pict_exp_gain(cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
		break;
	case CFG_SET_MODE:
		rc = ov5647_set_sensor_mode(cdata.mode, cdata.rs);
		break;
	case CFG_PWR_DOWN:
		rc = ov5647_power_down();
		break;
#ifdef OV5647_AUTO_FOCUS
	case CFG_MOVE_FOCUS:
		if(is_flash == 1)
			set_flash_led(LEVEL_81MA,-1);
		rc = ov5647_move_focus(cdata.cfg.focus.dir,
				cdata.cfg.focus.steps);
		break;
	case CFG_SET_DEFAULT_FOCUS:
		rc = ov5647_set_default_focus(cdata.cfg.focus.steps);
		break;

	case CFG_GET_AF_MAX_STEPS:
		cdata.max_steps = OV5647_TOTAL_STEPS_NEAR_TO_FAR;
		if (copy_to_user((void *)argp,
					&cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_EFFECT:
		rc = ov5647_set_default_focus(cdata.cfg.effect);
		break;
	case CFG_SET_AUTOFLASH:
		is_flash = cdata.cfg.is_autoflash;
		printk("--CAMERA-- is autoflash %d\r\n",is_flash);
		if(is_flash == 2)
			set_flash_led(LEVEL_81MA,-1);
		if(is_flash == 0)
			shutdown_flash_led();
		break;

#endif
	default:
		rc = -EFAULT;
		break;
	}
	mutex_unlock(&ov5647_mut);

	return rc;
}

static int ov5647_sensor_release(void)
{
	int rc = -EBADF;
	uint16_t val=0;
	mutex_lock(&ov5647_mut);
	ov5647_i2c_read(0x3018,&val);
	val |= 0x18;
	ov5647_i2c_write_b_sensor(0x3018, val);
	ov5647_power_down();
	mdelay(20);
	gpio_set_value(ov5647_ctrl->sensordata->sensor_pwd, 1);
	usleep_range(5000, 5100);
	if (ov5647_ctrl->sensordata->vcm_enable) {
		gpio_direction_output(ov5647_ctrl->sensordata->vcm_pwd, 0);
		gpio_free(ov5647_ctrl->sensordata->vcm_pwd);
	}

	//lcd_camera_power_onoff(0); //turn off ldo and vreg

	kfree(ov5647_ctrl);
	ov5647_ctrl = NULL;
	CSI_CONFIG = 0;
	CDBG("ov5647_release completed\n");
	mutex_unlock(&ov5647_mut);

	return rc;
}

static int ov5647_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	CDBG("%s E", __func__);

	gpio_direction_output(info->sensor_pwd, 1);
	gpio_direction_output(info->sensor_reset, 0);
	mdelay(1);
	//lcd_camera_power_onoff(1); //turn on ldo and vreg

	rc = i2c_add_driver(&ov5647_i2c_driver);
	if (rc < 0 || ov5647_client == NULL) {
		rc = -ENOTSUPP;
		CDBG("I2C add driver ov5647 failed");
		goto probe_fail_2;
	}
#ifdef OV5647_AUTO_FOCUS
	rc = i2c_add_driver(&ov5647_af_i2c_driver);
	if (rc < 0 || ov5647_af_client == NULL) {
		rc = -ENOTSUPP;
		CDBG("I2C add driver ov5647 af failed");
		goto probe_fail_3;
	}
#endif
	msm_camio_clk_rate_set(OV5647_MASTER_CLK_RATE);

	rc = ov5647_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail_1;

	s->s_init = ov5647_sensor_open_init;
	s->s_release = ov5647_sensor_release;
	s->s_config  = ov5647_sensor_config;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;
	gpio_set_value(info->sensor_pwd, 1);
	ov5647_probe_init_done(info);

	//lcd_camera_power_onoff(0); //turn off ldo and vreg

	CDBG("%s X", __func__);
	return rc;
#ifdef OV5647_AUTO_FOCUS
probe_fail_3:
	i2c_del_driver(&ov5647_af_i2c_driver);
#endif
probe_fail_2:
	i2c_del_driver(&ov5647_i2c_driver);
probe_fail_1:
	//lcd_camera_power_onoff(0);
	CDBG("ov5647_sensor_probe: SENSOR PROBE FAILS!\n");
	CDBG("%s X", __func__);
	return rc;
}

static int __devinit ov5647_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, ov5647_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = ov5647_probe,
	.driver = {
		.name = "msm_camera_ov5647",
		.owner = THIS_MODULE,
	},
};

static int __init ov5647_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov5647_init);
MODULE_DESCRIPTION("Samsung 5 MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
