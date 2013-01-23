/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	include/linux/pmic8029-kp-bl.h
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/

#ifndef LINUX_PM8029_FLASH_LED_MODULE_H
#define LINUX_PM8029_FLASH_LED_MODULE_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

#define FLASH_LED_IOC_MAGIC 'f'

/* IOCTL MACROS */
#define FLASH_LED_IOCTL_TURNOFF			_IO(FLASH_LED_IOC_MAGIC, 0x00)
#define FLASH_LED_IOCTL_SET_BRIGHTNESS		_IOW(FLASH_LED_IOC_MAGIC, 0x01, int)

#if 0 //For CAT3612
typedef enum {
	LEVEL_300MA = 0,
	LEVEL_290MA,
	LEVEL_280MA,
	LEVEL_271MA,
	LEVEL_262MA,
	LEVEL_252MA,
	LEVEL_242MA,
	LEVEL_232MA,
	LEVEL_222MA,
	LEVEL_213MA,
	LEVEL_203MA,
	LEVEL_194MA,
	LEVEL_184MA,
	LEVEL_174MA,
	LEVEL_164MA,
	LEVEL_155MA,
	LEVEL_145MA,
	LEVEL_136MA,
	LEVEL_126MA,
	LEVEL_116MA,
	LEVEL_106MA,
	LEVEL_97MA,
	LEVEL_87MA,
	LEVEL_78MA,
	LEVEL_68MA,
	LEVEL_58MA,
	LEVEL_48MA,
	LEVEL_38MA,
	LEVEL_29MA,
	LEVEL_20MA,	
	LEVEL_10MA,
	LEVEL_0MA,		
}flash_led_level_t;
#else //For SGM3146
typedef enum {
	LEVEL_216MA = 0,
	LEVEL_202MA,
	LEVEL_189MA,
	LEVEL_175MA,
	LEVEL_162MA,
	LEVEL_148MA,
	LEVEL_135MA,
	LEVEL_121MA,
	LEVEL_108MA,
	LEVEL_94MA,
	LEVEL_81MA,
	LEVEL_67MA,
	LEVEL_54MA,
	LEVEL_40MA,
	LEVEL_27MA,
	LEVEL_13MA,
	LEVEL_NUM,
}flash_led_level_t;
#endif

int set_flash_led(flash_led_level_t level, int delay_ms);
void shutdown_flash_led(void);

#endif /* LINUX_PM8029_FLASH_LED_MODULE_H */
