/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __ASM_ARCH_MSM_GPIO_QRD7627A_H__
#define __ASM_ARCH_MSM_GPIO_QRD7627A_H__

#include <linux/gpio.h>

#define GPIO_INVALID NR_MSM_GPIOS
enum {
	GPIO_HOST_VBUS_EN_INDEX	= 0,
	GPIO_BT_SYS_REST_EN_INDEX,
	GPIO_CAM_FRONT_PWDN_INDEX,	/* CAM_VGA */
	GPIO_CAM_BACK_SHDN_EN_INDEX,
	GPIO_CAM_FRONT_RESET_INDEX,
	GPIO_CAM_BACK_RESET_INDEX,
	GPIO_FM_RDS_INT_INDEX,
	GPIO_SDC1_HW_DET_INDEX,
	UART1DM_RX_GPIO_INDEX,
	FT5206_IRQ_GPIO_INDEX,
	FT5206_RESET_GPIO_INDEX,
	CLEARPAD3000_ATTEN_GPIO_INDEX,
	CLEARPAD3000_RESET_GPIO_INDEX,
	KYPD_DRV0_INDEX,
	KYPD_DRV1_INDEX,
	KEYSENSE_N0_INDEX,
	KEYSENSE_N1_INDEX,
	GPIO_COMPASS_DRDY_INDEX,
	GPIO_GYRO_INT_N_INDEX,
	GPIO_ACC_INT_INDEX,
	GPIO_LTR502_INT_INDEX,
	LED_BLUE_INDEX,
	LED_RED_INDEX,
	LED_GREEN_INDEX,
	LED_FLASH_INDEX,
	GPIO_VDD33_EN_INDEX,

	GPIO_TOTAL_NUM
};

enum {
	GPIO_BACKLIGHT_EN_INDEX,
	GPIO_SPI_MOSI_INDEX,
	GPIO_SPI_CLK_INDEX,
	GPIO_SPI_CS0_N_INDEX,
	GPIO_DISPLAY_RESET_INDEX,
	GPIO_DISPLAY_PWR_EN_INDEX,
	GPIO_LCDC_BRDG_PD_INDEX,
	GPIO_LCDC_BRDG_RESET_N_INDEX,
	GPIO_LCD_DSI_SEL_INDEX,
	MDP_303_VSYNC_GPIO_INDEX,
	GPIO_LCDC_PCLK_INDEX,
	LCD_CAMERA_LDO_2V8_INDEX, /*SKU1&SKU3 2.8V LDO */
	LCD_CAMERA_LDO_1V8_INDEX, /*SKU1 1.8V LDO*/

	LCD_GPIO_TOTAL_NUM
};

extern unsigned int * msm_gpio_table_qrd;
extern unsigned int msm_gpio_table_qrd_7227a_sku1[GPIO_TOTAL_NUM];
extern unsigned int msm_gpio_table_qrd_7227a_sku3[GPIO_TOTAL_NUM];

extern unsigned int * msm_lcd_gpio_table_qrd;
extern unsigned int msm_lcd_gpio_table_qrd_7227a_sku1[LCD_GPIO_TOTAL_NUM];
extern unsigned int msm_lcd_gpio_table_qrd_7227a_sku3[LCD_GPIO_TOTAL_NUM];
extern unsigned int msm_lcd_gpio_table_qrd_7227a_sku3_1[LCD_GPIO_TOTAL_NUM];

#endif
