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

#ifndef OV5647_H
#define OV5647_H
#include <linux/types.h>
#include <mach/board.h>
#include <linux/pmic8029-flash-led.h>

extern struct ov5647_reg ov5647_regs;

struct ov5647_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

enum ov5647_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum ov5647_resolution_t {
  QTR_SIZE,
  FULL_SIZE,
  HFR_60FPS,
  HFR_90FPS,
  HFR_120FPS,
  INVALID_SIZE
};

enum ov5647_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

enum ov5647_reg_pll {
	E013_VT_PIX_CLK_DIV,
	E013_VT_SYS_CLK_DIV,
	E013_PRE_PLL_CLK_DIV,
	E013_PLL_MULTIPLIER,
	E013_OP_PIX_CLK_DIV,
	E013_OP_SYS_CLK_DIV
};

enum ov5647_reg_mode {
	E013_X_ADDR_START,
	E013_X_ADDR_END,
	E013_Y_ADDR_START,
	E013_Y_ADDR_END,
	E013_X_OUTPUT_SIZE,
	E013_Y_OUTPUT_SIZE,
	E013_DATAPATH_SELECT,
	E013_READ_MODE,
	E013_ANALOG_CONTROL5,
	E013_DAC_LD_4_5,
	E013_SCALING_MODE,
	E013_SCALE_M,
	E013_LINE_LENGTH_PCK,
	E013_FRAME_LENGTH_LINES,
	E013_COARSE_INTEGRATION_TIME,
	E013_FINE_INTEGRATION_TIME,
	E013_FINE_CORRECTION
};

struct ov5647_reg {
	const struct ov5647_i2c_reg_conf *rec_settings;
	const unsigned short rec_size;
	const struct ov5647_i2c_reg_conf *reg_prev;
	const unsigned short reg_prev_size;
	const struct ov5647_i2c_reg_conf *reg_snap;
	const unsigned short reg_snap_size;
    const struct ov5647_i2c_reg_conf *reg_hfr_60fps;
	const unsigned short reg_hfr_60fps_size;
    const struct ov5647_i2c_reg_conf *reg_hfr_90fps;
	const unsigned short reg_hfr_90fps_size;
};
bool otp_update_wb(unsigned char golden_rg, unsigned char golden_bg) ;
int32_t ov5647_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata);
int32_t ov5647_i2c_read(unsigned short raddr,unsigned short *rdata);
#endif /* OV5647_H */
