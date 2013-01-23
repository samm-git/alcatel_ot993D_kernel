/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  version: v1.1
 *  version: v1.2 , fix flick issue for gamma testing.
 */

//#define DEBUG

#include "mach/gpio.h"

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_hx8369b.h"

//static int prev_bl = 17;
//static int gpio_backlight_en = 0xff;

static struct msm_panel_common_pdata *mipi_hx8369b_pdata=NULL;
static struct dsi_buf hx8369b_tx_buf;
static struct dsi_buf hx8369b_rx_buf;

static char hx8369b_display_off[] = {0x28};
static char hx8369b_enter_sleep[] = {0x10};

/* same shared by hx8369 and hx8363A  */
static struct dsi_cmd_desc hx8369b_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(hx8369b_display_off), hx8369b_display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(hx8369b_enter_sleep), hx8369b_enter_sleep}
};


/* common setting */
static char SET_CMD_1[]={0x11};
static char SET_CMD_2[]={0xB9,0xFF,0x83,0x69};
static char SET_CMD_3[]={0x3A,0x77};
static char SET_CMD_4[]={0xB1,0x12,0x83,0x77,0x00,0x8F,0x0F,0x1A,0x1A,0x0C,0x0A};
static char SET_CMD_5[]={0xB2,0x00,0x10,0x02};
static char SET_CMD_6[]={0xB3,0x83,0x00,0x31,0x03};//80 60 26
static char SET_CMD_7[]={0xB4,0x80};
static char SET_CMD_9[]={0xB6,0xac,0xac};   // difference: {0xB6,0xBD,0xBD}
static char SET_CMD_10[]={0xCC,0x00};
static char SET_CMD_11[]={0xD5,
	0x00,0x00,0x12,0x00,0x00,0x00,0x00,0x20,0x00,0x00, //1~10,   different
    0x00,0x00,0x02,0x5B,0x33,0x00,0x00,0x14,0x02,0x2f, //11~20, different
    0x13,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x03, //~30
    0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x00, //~40
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //~50 
    0x00,0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //~60
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33, //~70
    0x11,0x11,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00, //~80
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, //~90, different
    0x01,0x5A}; //~92
static char SET_CMD_12[]={0xEA,0x62}; 
static char SET_CMD_13[]={0xBA,
0x11,0x00,0x16,0xC6,0x80,0x0A,0x00,0x10,0x24,0x02,0x21,0x21,0x9A,0x11,0x14}; 	
static char SET_CMD_14[]={0xE0,
0x00,0x00,0x01,0x0E,0x0E,0x3F,0x28,0x34,0x05,0x14, //P 1~10
    0x11,0x15,0x17,0x16,0x16,0x12,0x16, //P 11~17
    0x00,0x00,0x01,0x0E,0x0E,0x3F,0x28,0x34,0x05,0x14, //N 1~10
    0x11,0x15,0x17,0x16,0x16,0x12,0x16,0x01}; //N 11~17
static char SET_CMD_15[]={0x29};
static char SET_CMD_16[]={0x2C};

static struct dsi_cmd_desc hx8369b_video_display_on_cmds[] = {
	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_2), SET_CMD_2},			
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_3), SET_CMD_3},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_4), SET_CMD_4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_5), SET_CMD_5},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_6), SET_CMD_6},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_7), SET_CMD_7},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_9), SET_CMD_9},	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_10), SET_CMD_10},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_11), SET_CMD_11},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_12), SET_CMD_12},	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_13), SET_CMD_13},	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_14), SET_CMD_14},	
	{DTYPE_DCS_WRITE, 1, 0, 0, 150, sizeof(SET_CMD_1), SET_CMD_1},
	{DTYPE_DCS_WRITE, 1, 0, 0, 15, sizeof(SET_CMD_15), SET_CMD_15},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(SET_CMD_16), SET_CMD_16},
};

#define GPIO_HX8369B_LCD_RESET   129
static int mipi_hx8369b_lcd_reset(void)
{
    int rc = 0;

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_HX8369B_LCD_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		gpio_free(GPIO_HX8369B_LCD_RESET);
		return rc;
	}

    //gpio_set_value(GPIO_HX8369B_LCD_RESET, 1);
    //msleep(10);        
    gpio_set_value(GPIO_HX8369B_LCD_RESET, 0);
    msleep(10);        
    gpio_set_value(GPIO_HX8369B_LCD_RESET, 1);
    msleep(100);          

        return 0;
}

static int mipi_hx8369b_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	
	printk("======> mipi_hx8369b_lcd_on start! \n");

	mfd = platform_get_drvdata(pdev);

	if (!mfd) {
		return -ENODEV;
	}
	
	if (mfd->key != MFD_KEY) {
		return -EINVAL;
	}


	mipi  = &mfd->panel_info.mipi;

	if (mipi_hx8369b_lcd_reset() < 0) 
	{
       	printk("mipi_hx8369b_lcd_reset error\n");
       	return -EINVAL;
    }
	
	if (mipi->mode == DSI_VIDEO_MODE) 
	{		
		mipi_dsi_cmds_tx(
			mfd, 
			&hx8369b_tx_buf, 
			hx8369b_video_display_on_cmds,
			ARRAY_SIZE(hx8369b_video_display_on_cmds) 
			);
	} 
	else       /* for HX8369B driver IC, only supports MIPI video mode.  */
	{
		printk("Error: %s called with wrong mipi mode support of LCD! \n", __func__);
	}
	printk("mipi_hx8369b_lcd_on end! \n");

	return 0;
}

static int mipi_hx8369b_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	printk("mipi_hx8369b_lcd_off start! \n");

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmds_tx(mfd, &hx8369b_tx_buf, hx8369b_display_off_cmds,
			ARRAY_SIZE(hx8369b_display_off_cmds));

#if 1
    if (mipi_hx8369b_lcd_reset() < 0) {
        printk("mipi_hx8369b_lcd_reset error\n");
        return -EINVAL;
    }

    gpio_set_value(GPIO_HX8369B_LCD_RESET, 0);

#endif

	printk("mipi_hx8369b_lcd_off end! \n");

	return 0;
}

static int __devinit mipi_hx8369b_lcd_probe(struct platform_device *pdev)
{
	int rc = 0;
	printk("%s-------------pdev->id=%d-----%s\n", __func__,pdev->id,pdev->name);
	if (pdev->id == 0) {
		mipi_hx8369b_pdata = pdev->dev.platform_data;
		return rc;
	}
	
	msm_fb_add_device(pdev);
		
	return rc;
}

static struct platform_driver this_driver = {
	.probe  = mipi_hx8369b_lcd_probe,
	.driver = {
		.name   = "mipi_hx8369b",
	},
};


static void mipi_hx8369b_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level;
	bl_level = mfd->bl_level;

	
	if (mipi_hx8369b_pdata && mipi_hx8369b_pdata->pmic_backlight)
		mipi_hx8369b_pdata->pmic_backlight(bl_level);
	else
		pr_err("%s(): Backlight level set failed", __func__);
}


static struct msm_fb_panel_data hx8369b_panel_data = {
	.on	= mipi_hx8369b_lcd_on,
	.off = mipi_hx8369b_lcd_off,
	.set_backlight = mipi_hx8369b_set_backlight,
};

static int ch_used[3];

int mipi_hx8369b_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	printk("mipi_hx8369b_device_register\n");

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;
	pdev = platform_device_alloc("mipi_hx8369b", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	hx8369b_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &hx8369b_panel_data,
		sizeof(hx8369b_panel_data));
	if (ret) {
		printk("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_hx8369b_lcd_init(void)
{
	if(gpio_get_value(49)) 
		return -1;
	
	printk("%s called! \n", __func__);
	
	mipi_dsi_buf_alloc(&hx8369b_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&hx8369b_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(mipi_hx8369b_lcd_init);
