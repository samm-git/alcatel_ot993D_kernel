/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include "ov7692.h"

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define Q8    0x00000100

/* Omnivision8810 product ID register address */
#define REG_OV7692_MODEL_ID_MSB                       0x0A
#define REG_OV7692_MODEL_ID_LSB                       0x0B

#define OV7692_MODEL_ID                       0x7692
/* Omnivision8810 product ID */

/* Time in milisecs for waiting for the sensor to reset */
#define OV7692_RESET_DELAY_MSECS    66
#define OV7692_DEFAULT_CLOCK_RATE   24000000
/* Registers*/

/* Color bar pattern selection */
#define OV7692_COLOR_BAR_PATTERN_SEL_REG     0x82
/* Color bar enabling control */
#define OV7692_COLOR_BAR_ENABLE_REG           0x601
/* Time in milisecs for waiting for the sensor to reset*/
#define OV7692_RESET_DELAY_MSECS    66

static int ov7692_pwdn_gpio;
static int ov7692_reset_gpio;

/*============================================================================
							DATA DECLARATIONS
============================================================================*/
static bool OV7692_CSI_CONFIG;
/* 816x612, 24MHz MCLK 96MHz PCLK */
uint32_t OV7692_FULL_SIZE_WIDTH        = 640;
uint32_t OV7692_FULL_SIZE_HEIGHT       = 480;

uint32_t OV7692_QTR_SIZE_WIDTH         = 640;
uint32_t OV7692_QTR_SIZE_HEIGHT        = 480;

uint32_t OV7692_HRZ_FULL_BLK_PIXELS    = 16;
uint32_t OV7692_VER_FULL_BLK_LINES     = 12;
uint32_t OV7692_HRZ_QTR_BLK_PIXELS     = 16;
uint32_t OV7692_VER_QTR_BLK_LINES      = 12;

struct ov7692_work_t {
	struct work_struct work;
};
static struct  ov7692_work_t *ov7692_sensorw;
static struct  i2c_client *ov7692_client;
struct ov7692_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;
	uint32_t sensormode;
	uint32_t fps_divider;		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;	/* init to 1 * 0x00000400 */
	uint32_t fps;
	int32_t  curr_lens_pos;
	uint32_t curr_step_pos;
	uint32_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint32_t total_lines_per_frame;
	enum ov7692_resolution_t prev_res;
	enum ov7692_resolution_t pict_res;
	enum ov7692_resolution_t curr_res;
	enum ov7692_test_mode_t  set_test;
	unsigned short imgaddr;
};
static struct ov7692_ctrl_t *ov7692_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(ov7692_wait_queue);
DEFINE_MUTEX(ov7692_mut);

/*=============================================================*/

static int ov7692_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 1,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(ov7692_client->adapter, msgs, 2) < 0) {
		CDBG("ov7692_i2c_rxdata failed!\n");
		return -EIO;
	}
	return 0;
}
static int32_t ov7692_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = 2,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(ov7692_client->adapter, msg, 1) < 0) {
		CDBG("ov7692_i2c_txdata faild 0x%x\n", ov7692_client->addr);
		return -EIO;
	}

	return 0;
}

static int32_t ov7692_i2c_read(uint8_t raddr,
	uint8_t *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[1];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = ov7692_i2c_rxdata(ov7692_client->addr >> 1, buf, rlen);
	if (rc < 0) {
		CDBG("ov7692_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = buf[0];
	return rc;
}
static int32_t ov7692_i2c_write_b_sensor(uint8_t waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];
	memset(buf, 0, sizeof(buf));
	buf[0] = waddr;
	buf[1] = bdata;
	rc = ov7692_i2c_txdata(ov7692_client->addr >> 1, buf, 2);
	if (rc < 0)
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	return rc;
}

static int32_t ov7692_sensor_setting(int update_type, int rt)
{
	int32_t i, array_length;
	int32_t rc = 0;
	struct msm_camera_csi_params ov7692_csi_params;
	switch (update_type) {
	case REG_INIT:
		OV7692_CSI_CONFIG = 0;
		ov7692_i2c_write_b_sensor(0x0e, 0x08);
		return rc;
		break;
	case UPDATE_PERIODIC:
		if (!OV7692_CSI_CONFIG) {
			ov7692_csi_params.lane_cnt = 1;
			ov7692_csi_params.data_format = CSI_8BIT;
			ov7692_csi_params.lane_assign = 0xe4;
			ov7692_csi_params.dpcm_scheme = 0;
			ov7692_csi_params.settle_cnt = 0x14;

			rc = msm_camio_csi_config(&ov7692_csi_params);
			msleep(10);
			array_length = sizeof(ov7692_init_settings_array) /
				sizeof(ov7692_init_settings_array[0]);
			for (i = 0; i < array_length; i++) {
				rc = ov7692_i2c_write_b_sensor(
					ov7692_init_settings_array[i].reg_addr,
					ov7692_init_settings_array[i].reg_val);
				if (rc < 0)
					return rc;
			}
			OV7692_CSI_CONFIG = 1;
			msleep(20);
			return rc;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t ov7692_video_config(int mode)
{
	int32_t rc = 0;
	int rt;
	/* change sensor resolution if needed */
	rt = RES_PREVIEW;

	if (ov7692_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	ov7692_ctrl->curr_res = ov7692_ctrl->prev_res;
	ov7692_ctrl->sensormode = mode;
	return rc;
}

static int32_t ov7692_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = ov7692_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
	case SENSOR_RAW_SNAPSHOT_MODE:
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}
static void ov7692_power_on(void)
{
   CDBG("%s\n",__func__);
   gpio_set_value(ov7692_pwdn_gpio, 0);
}

static void ov7692_power_down(void)
{
    CDBG("%s\n",__func__);
       gpio_set_value(ov7692_pwdn_gpio, 1);
}

static void ov7692_sw_reset(void)
{
    CDBG("%s\n",__func__);
    ov7692_i2c_write_b_sensor(0x12, 0x80);
}

static void ov7692_hw_reset(void)
{
    CDBG("--CAMERA-- %s ... (Start...)\n",__func__);
    gpio_set_value(ov7692_reset_gpio, 1);   //reset camera reset pin
    mdelay(5);
    gpio_set_value(ov7692_reset_gpio, 0);
    mdelay(5);
    gpio_set_value(ov7692_reset_gpio, 1);
    mdelay(1);

    CDBG("--CAMERA-- %s ... (End...)\n",__func__);
}



static int ov7692_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	uint8_t model_id_msb, model_id_lsb = 0;
    uint16_t model_id = 0;
	int32_t rc = 0;
	/*The reset pin is not physically connected to the sensor.
	The standby pin will do the reset hence there is no need
	to request the gpio reset*/

	/* Read sensor Model ID: */
	rc = ov7692_i2c_read(REG_OV7692_MODEL_ID_MSB, &model_id_msb, 1);
	if (rc < 0)
		goto init_probe_fail;
	rc = ov7692_i2c_read(REG_OV7692_MODEL_ID_LSB, &model_id_lsb, 1);
	if (rc < 0)
		goto init_probe_fail;
	model_id = (model_id_msb << 8) | ((model_id_lsb & 0x00FF)) ;
	CDBG("ov7692 model_id = 0x%x, 0x%x, 0x%x\n",
		 model_id, model_id_msb, model_id_lsb);
	/* 4. Compare sensor ID to OV7692 ID: */
	if (model_id != OV7692_MODEL_ID) {
		rc = -ENODEV;
		goto init_probe_fail;
	}
	goto init_probe_done;
init_probe_fail:
	pr_warning(" ov7692_probe_init_sensor fails\n");
init_probe_done:
	CDBG(" ov7692_probe_init_sensor finishes\n");
	return rc;
}

int ov7692_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("Calling ov7692_sensor_open_init\n");
	ov7692_ctrl = kzalloc(sizeof(struct ov7692_ctrl_t), GFP_KERNEL);
	if (!ov7692_ctrl) {
		CDBG("ov7692_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	ov7692_ctrl->fps_divider = 1 * 0x00000400;
	ov7692_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov7692_ctrl->fps = 30 * Q8;
	ov7692_ctrl->set_test = TEST_OFF;
	ov7692_ctrl->prev_res = QTR_SIZE;
	ov7692_ctrl->pict_res = FULL_SIZE;
	ov7692_ctrl->curr_res = INVALID_SIZE;

	if (data)
		ov7692_ctrl->sensordata = data;

	/* enable mclk first */

	msm_camio_clk_rate_set(24000000);
	msleep(20);
    ov7692_power_on();
    msleep(5);

	rc = ov7692_probe_init_sensor(data);
	if (rc < 0) {
		CDBG("Calling ov7692_sensor_open_init fail\n");
		goto init_fail;
	}

	rc = ov7692_sensor_setting(REG_INIT, RES_PREVIEW);
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;

init_fail:
	CDBG(" ov7692_sensor_open_init fail\n");
	kfree(ov7692_ctrl);
init_done:
	CDBG("ov7692_sensor_open_init done\n");
	return rc;
}

static int ov7692_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov7692_wait_queue);
	return 0;
}

static const struct i2c_device_id ov7692_i2c_id[] = {
	{"ov7692", 0},
	{ }
};

static int ov7692_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("ov7692_i2c_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	ov7692_sensorw = kzalloc(sizeof(struct ov7692_work_t), GFP_KERNEL);
	if (!ov7692_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov7692_sensorw);
	ov7692_init_client(client);
	ov7692_client = client;

	CDBG("ov7692_i2c_probe success! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("ov7692_i2c_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit ov7692_remove(struct i2c_client *client)
{
	struct ov7692_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	ov7692_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver ov7692_i2c_driver = {
	.id_table = ov7692_i2c_id,
	.probe  = ov7692_i2c_probe,
	.remove = __exit_p(ov7692_i2c_remove),
	.driver = {
		.name = "ov7692",
	},
};

int ov7692_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&ov7692_mut);
	switch (cdata.cfgtype) {
	case CFG_SET_MODE:
		rc = ov7692_set_sensor_mode(cdata.mode,
			cdata.rs);
		break;
	case CFG_PWR_DOWN:
		ov7692_power_down();
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&ov7692_mut);

	return rc;
}
static int ov7692_sensor_release(void)
{
	int rc = -EBADF;
    int32_t i, array_length;
    struct reg_addr_val_pair_struct ov7692_mipi_standby[] = {
		{0xff, 0x01},
		{0xb4, 0x40},
		{0xb5, 0x30},
		{0xff, 0x00},
		};
	mutex_lock(&ov7692_mut);
	array_length = sizeof(ov7692_mipi_standby) /
		sizeof(ov7692_mipi_standby[0]);
	for (i = 0; i < array_length; i++) {
		rc = ov7692_i2c_write_b_sensor(
			ov7692_mipi_standby[i].reg_addr,
			ov7692_mipi_standby[i].reg_val);
		if (rc < 0)
			return rc;
	}

    ov7692_sw_reset();
	ov7692_power_down();
	kfree(ov7692_ctrl);
	ov7692_ctrl = NULL;
	CDBG("ov7692_release completed\n");
	mutex_unlock(&ov7692_mut);

	return rc;
}
static int ov7692_probe_init_gpio(const struct msm_camera_sensor_info *data)
{
    int rc = 0;

    ov7692_pwdn_gpio = data->sensor_pwd;
    ov7692_reset_gpio = data->sensor_reset ;
    
    if(data->sensor_reset_enable)
    {
        gpio_direction_output(data->sensor_reset, 1);
    }

    gpio_direction_output(data->sensor_pwd, 1);

    return rc;

}


static int ov7692_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&ov7692_i2c_driver);
	if (rc < 0 || ov7692_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}
    rc = ov7692_probe_init_gpio(info);
    if (rc < 0) {
        CDBG("%s: gpio init failed\n", __func__);
        goto probe_fail;
    }

    ov7692_power_down();

	msm_camio_clk_rate_set(24000000);
    msleep(5);

    ov7692_power_on();
    msleep(5);

    if(info->sensor_reset_enable)
    {
        ov7692_hw_reset();
    }
    else
    {
        ov7692_sw_reset();
    }
	rc = ov7692_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	s->s_init = ov7692_sensor_open_init;
	s->s_release = ov7692_sensor_release;
	s->s_config  = ov7692_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;
    s->s_mount_angle = info->sensor_platform_info->mount_angle;
    ov7692_power_down();
	return rc;

probe_fail:
	CDBG("ov7692_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&ov7692_i2c_driver);
	return rc;
}

static int __ov7692_probe(struct platform_device *pdev)
{

	return msm_camera_drv_start(pdev, ov7692_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov7692_probe,
	.driver = {
		.name = "msm_camera_ov7692",
		.owner = THIS_MODULE,
	},
};

static int __init ov7692_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov7692_init);

MODULE_DESCRIPTION("OMNI VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");
