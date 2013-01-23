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
 *  version: v2.0 fixed bug 265565, but with bad gamma settings.
 *  version: v3.0 official gamma settings.
 *  version: v4.0 official gamma settings from truly, but not very good.
 */

//#define DEBUG

#include "mach/gpio.h"

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_hx8363a.h"

//static int prev_bl = 17;
//static int gpio_backlight_en = 0xff;

static struct msm_panel_common_pdata *mipi_hx8363a_pdata=NULL;
static struct dsi_buf hx8363a_tx_buf;
static struct dsi_buf hx8363a_rx_buf;



#define HX8363A_CMD_DELAY 0
#define HX8363A_PANEL_SET_DELAY 5
#define HX8363A_GAMMA_SET_DELAY 5
#define HX8363A_MIPI_SETTING_DELAY 10
#define HX8363A_SLEEP_OFF_DELAY 150
#define HX8363A_DISPLAY_ON_DELAY 150

/* common setting */
/* same shared by hx8369 and hx8363A  */
static char hx8363a_exit_sleep[] = {0x11};
static char hx8363a_display_on[] = {0x29};
static char hx8363a_display_off[] = {0x28};
static char hx8363a_enter_sleep[] = {0x10};

/* same shared by hx8369 and hx8363A  */
static struct dsi_cmd_desc hx8363a_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(hx8363a_display_off), hx8363a_display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(hx8363a_enter_sleep), hx8363a_enter_sleep}
};

static char hx8363a_video_vcom_setting[] = {0xB6, 0x24 /*0x00*/};
static char hx8363a_video_pannel_setting[] = {0xCC, 0x09 /*0x00*/ };

/* video mode setting */
static char hx8363a_video_wave_cycle_setting[] = 
{
	/* Set_CYC CPT   */
#if 1
	0xB4, 0x08, 0x12, 0x72, 
	0x12, 0x06, 0x03, 0x54, 
	0x03, 0x4e
#else
	0xB4, 0x08, 0x03, 0xe0, 
	0x30, 0x01, 0x12, 0x64, 
	0x01, 0xff, 0x00, 0x00
#endif
};


static char hx8363a_video_power_setting[] = {
#if 1
	0xB1, 0x81, 0x34, 0x08,
	0x34, 0x02, 0x13, 0x11,
	0x00, 0x32, 0x3A, 0x3F,  // 0x32, 0x39
	0x3F
#else
	0xB1, 0x81, 0x30, 0x08,
	0x34, 0x01, 0x13, 0x16,
	0x00, 0x35, 0x3E, 0x16,
	0x16,
#endif
};

static char hx8363a_video_setrgbif[]=
{
	0xB3, 0x00,    //0xB3, 0x01,
};

static char hx8363a_video_gamma_setting[] = {

#if 1
	0xE0,
	0x00,0x1e,0x63,
	0x50,0x0e,0x17,0x04,
	0x8c,0xce,0xd4,0x15,
	0xD5,0x95,0x13,0x18,

    0x00,0x1e,0x63,
	0x50,0x0e,0x17,0x04,
	0x8c,0xce,0xd4,0x15,
	0xD5,0x95,0x13,0x18,
	
#else
	0xE0,0x00,0x45,0x0A,
	0x92,0x8C,0x3F,0x07,
	0xCF,0x0F,0x14,0x17,
	0xD5,0x95,0x10,0x12,
	0x00,0x45,0x0A,0x92,
	0x8C,0x3F,0x07,0xCF, 
	0x0F,0x14,0x17,0xD5, 
	0x95,0x10,0x12,
#endif
};

// the following are newly defined! 
static char hx8363a_video_colmod[]=
{
	0x3A, 0x70    //0x3A, 0x77,
};

#if 0
/* Memory Access Control : vertical flip,  horizontal flip */
static char hx8363a_video_madctl[]=   
{
	0x36, 0x0a
};
#endif

static char hx8363a_video_ptba[]=
{
	0xBF, 0x00, 0x10
};

static char hx8363a_video_set_disp[]=
{
	0xB2, 0x33, 0x33, 0x22
};

static char hx8363a_extend_cmd_enable[] = 
{
	0xB9, 0xFF, 0x83, 0x63
};

static char hx8363a_video_mipi_setting[] = {
	0xBA, 0x80, 0x00, 0x10,
	0x08, 0x08, 0x10, 0x7C,
	0x6E, 0x6D, 0x0A, 0x01,      
	0x84, 0x43                        
};

static struct dsi_cmd_desc hx8363a_video_display_on_cmds[] = {
	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_extend_cmd_enable), hx8363a_extend_cmd_enable},			
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_power_setting), hx8363a_video_power_setting},
	{DTYPE_DCS_WRITE, 1, 0, 0, 150, sizeof(hx8363a_exit_sleep), hx8363a_exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_set_disp), hx8363a_video_set_disp},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_colmod), hx8363a_video_colmod},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_wave_cycle_setting), hx8363a_video_wave_cycle_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_ptba), hx8363a_video_ptba},	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 5, sizeof(hx8363a_video_vcom_setting), hx8363a_video_vcom_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_pannel_setting), hx8363a_video_pannel_setting},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_gamma_setting), hx8363a_video_gamma_setting},
	//{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_dgc_settings), hx8363a_video_dgc_settings},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_mipi_setting), hx8363a_video_mipi_setting},	
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hx8363a_video_setrgbif), hx8363a_video_setrgbif},	
	{DTYPE_DCS_WRITE, 1, 0, 0, 150, sizeof(hx8363a_display_on), hx8363a_display_on},
};

#define GPIO_HX8363A_LCD_RESET   129
static int mipi_hx8363a_lcd_reset(void)
{
    int rc = 0;

	rc = gpio_tlmm_config(GPIO_CFG(GPIO_HX8363A_LCD_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
		GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		gpio_free(GPIO_HX8363A_LCD_RESET);
		return rc;
	}

    //gpio_set_value(GPIO_HX8363A_LCD_RESET, 1);
    //msleep(10);        
    gpio_set_value(GPIO_HX8363A_LCD_RESET, 0);
    msleep(10);        
    gpio_set_value(GPIO_HX8363A_LCD_RESET, 1);
    msleep(100);          

        return 0;
}

static int mipi_hx8363a_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	
	pr_debug("======> mipi_hx8363a_lcd_on start! \n");

	mfd = platform_get_drvdata(pdev);

	if (!mfd) {
		return -ENODEV;
	}
	
	if (mfd->key != MFD_KEY) {
		return -EINVAL;
	}


	mipi  = &mfd->panel_info.mipi;

	if (mipi_hx8363a_lcd_reset() < 0) 
	{
       	pr_debug("mipi_hx8363a_lcd_reset error\n");
       	return -EINVAL;
    }
	
	if (mipi->mode == DSI_VIDEO_MODE) 
	{		
		mipi_dsi_cmds_tx(
			mfd, 
			&hx8363a_tx_buf, 
			hx8363a_video_display_on_cmds,
			ARRAY_SIZE(hx8363a_video_display_on_cmds) 
			);
	} 
	else       /* for HX8363A driver IC, only supports MIPI video mode.  */
	{
		pr_debug("Error: %s called with wrong mipi mode support of LCD! \n", __func__);
	}
	pr_debug("mipi_hx8363a_lcd_on end! \n");

	return 0;
}

static int mipi_hx8363a_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	pr_debug("mipi_hx8363a_lcd_off start! \n");

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi_dsi_cmds_tx(mfd, &hx8363a_tx_buf, hx8363a_display_off_cmds,
			ARRAY_SIZE(hx8363a_display_off_cmds));

#if 1
    if (mipi_hx8363a_lcd_reset() < 0) {
        pr_debug("mipi_hx8363a_lcd_reset error\n");
        return -EINVAL;
    }

    gpio_set_value(GPIO_HX8363A_LCD_RESET, 0);

#endif

	pr_debug("mipi_hx8363a_lcd_off end! \n");

	return 0;
}

static int __devinit mipi_hx8363a_lcd_probe(struct platform_device *pdev)
{
	int rc = 0;
	pr_debug("%s-------------pdev->id=%d-----%s\n", __func__,pdev->id,pdev->name);
	if (pdev->id == 0) {
		mipi_hx8363a_pdata = pdev->dev.platform_data;
		return rc;
	}
	
	msm_fb_add_device(pdev);
		
	return rc;
}

static struct platform_driver this_driver = {
	.probe  = mipi_hx8363a_lcd_probe,
	.driver = {
		.name   = "mipi_hx8363a",
	},
};


static void mipi_hx8363a_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level;
	bl_level = mfd->bl_level;

	
	if (mipi_hx8363a_pdata && mipi_hx8363a_pdata->pmic_backlight)
		mipi_hx8363a_pdata->pmic_backlight(bl_level);
	else
		pr_err("%s(): Backlight level set failed", __func__);
}


static struct msm_fb_panel_data hx8363a_panel_data = {
	.on	= mipi_hx8363a_lcd_on,
	.off = mipi_hx8363a_lcd_off,
	.set_backlight = mipi_hx8363a_set_backlight,
};

static int ch_used[3];

int mipi_hx8363a_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	pr_debug("mipi_hx8363a_device_register\n");

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;
	pdev = platform_device_alloc("mipi_hx8363a", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	hx8363a_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &hx8363a_panel_data,
		sizeof(hx8363a_panel_data));
	if (ret) {
		pr_debug("%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_debug("%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init mipi_hx8363a_lcd_init(void)
{
	if(!gpio_get_value(49)) 
		return -1;
	
	pr_debug("%s called! \n", __func__);
	
	mipi_dsi_buf_alloc(&hx8363a_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&hx8363a_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

#if 0
static char manufacture_id_1[] = { 0x0c}; /* Read RDID2 */

static struct dsi_cmd_desc hx8363a_manufacture_id_cmd_1 = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id_1), manufacture_id_1
};


static char manufacture_id_2[] = { 0x0a }; /* Read RDID2 */

static struct dsi_cmd_desc hx8363a_manufacture_id_cmd_2 = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id_2), manufacture_id_2
};

static char manufacture_id_3[] = { 0xdb}; /* Read RDID2 */

static struct dsi_cmd_desc hx8363a_manufacture_id_cmd_3 = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id_3), manufacture_id_3
};

static char manufacture_id_4[] = { 0xba }; /* Read RDID2 */

static struct dsi_cmd_desc hx8363a_manufacture_id_cmd_4 = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id_4), manufacture_id_4
};




#if 0
static struct dsi_cmd_desc hx8363a_manufacture_id_read_prepare[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, HX8363A_CMD_DELAY, sizeof(hx8363a_extend_cmd_enable), hx8363a_extend_cmd_enable},	
};
#endif

void mipi_hx8363a_manufacture_id(struct msm_fb_data_type *mfd)
{
	struct dsi_buf *rp, *tp;
	struct dsi_cmd_desc *cmd;
	int len;
	int i;

	pr_debug("%s\n", __func__);

	#if 0
	mipi_dsi_cmds_tx(
			mfd, 
			&hx8363a_tx_buf, 
			hx8363a_manufacture_id_read_prepare,
			ARRAY_SIZE(hx8363a_manufacture_id_read_prepare) 
			);
	#endif
	
	tp = &hx8363a_tx_buf;
	rp = &hx8363a_rx_buf;
	
	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);

	cmd = &hx8363a_manufacture_id_cmd_1;
	len = mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 4);

	pr_debug("==0x0c===> %s: len = %d\n", __func__, len);
	
	for (i = 0; i < len; i++)
		pr_debug("=====> [%d] : 0x%x, ", i, rp->data[i]);

	pr_debug("\n\n");

	//----
	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);

	cmd = &hx8363a_manufacture_id_cmd_2;
	len = mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 4);

	pr_debug("===0x0a==> %s: len = %d\n", __func__, len);
	
	for (i = 0; i < len; i++)
		pr_debug("=====> [%d] : 0x%x, ", i, rp->data[i]);

	pr_debug("\n\n");

	// ---
	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);

	cmd = &hx8363a_manufacture_id_cmd_3;
	len = mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 4);

	pr_debug("===0xdb==> %s: len = %d\n", __func__, len);
	
	for (i = 0; i < len; i++)
		pr_debug("=====> [%d] : 0x%x, ", i, rp->data[i]);

	pr_debug("\n\n");

	// ---
	mipi_dsi_buf_init(rp);
	mipi_dsi_buf_init(tp);

	cmd = &hx8363a_manufacture_id_cmd_4;
	len = mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 13);

	pr_debug("===0xba==> %s: len = %d\n", __func__, len);
	
	for (i = 0; i < len; i++)
		pr_debug("=====> [%d] : 0x%x, ", i, rp->data[i]);

	pr_debug("\n\n");
	
	return;
}
#endif


module_init(mipi_hx8363a_lcd_init);
