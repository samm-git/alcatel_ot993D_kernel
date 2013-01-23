/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <mach/pmic.h>

#define MAX_KEYPAD_BL_LEVEL	16
#define KP_BL_LED_GPIO  122
extern unsigned int fastboot_power_off;
extern unsigned int fastboot_bklight_off;

void msm_keypad_bl_led_set_fastboot(enum led_brightness value)
{
	int ret;

      
	if (value == LED_OFF)
	{
                ret = pmic_secure_mpp_config_i_sink(PM_MPP_8,
                                PM_MPP__I_SINK__LEVEL_5mA,
                                PM_MPP__I_SINK__SWITCH_DIS);
	}	
	else if (value == LED_FULL)
	{
                ret = pmic_secure_mpp_config_i_sink(PM_MPP_8,
                                PM_MPP__I_SINK__LEVEL_10mA,
                                PM_MPP__I_SINK__SWITCH_ENA);
	}
	else 
	{
                ret = pmic_secure_mpp_config_i_sink(PM_MPP_8,
                                PM_MPP__I_SINK__LEVEL_5mA,
                                PM_MPP__I_SINK__SWITCH_ENA);

	}
	//ret = pmic_set_led_intensity(LED_KEYPAD, value / MAX_KEYPAD_BL_LEVEL);
	if (ret)
		printk("can't set keypad backlight\n");
}

static void msm_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	int ret;

	if(fastboot_power_off == 3)
	{
	
		if(value != LED_OFF)
			return;
	}	

	if(fastboot_power_off == 2)
	{
		if(fastboot_bklight_off == 1)
		{	
		if(value != LED_OFF)
			return;
		}
	}	
        //printk("kp_bl-------------kp_bl_ioctl set %d\n", value);
	if (value == LED_OFF)
	{
                ret = pmic_secure_mpp_config_i_sink(PM_MPP_8,
                                PM_MPP__I_SINK__LEVEL_5mA,
                                PM_MPP__I_SINK__SWITCH_DIS);
               gpio_set_value(KP_BL_LED_GPIO, 0);						
	}	
	else if (value == LED_FULL)
	{
                ret = pmic_secure_mpp_config_i_sink(PM_MPP_8,
                                PM_MPP__I_SINK__LEVEL_10mA,
                                PM_MPP__I_SINK__SWITCH_ENA);
               gpio_set_value(KP_BL_LED_GPIO, 1);					
	}
	else 
	{
                ret = pmic_secure_mpp_config_i_sink(PM_MPP_8,
                                PM_MPP__I_SINK__LEVEL_5mA,
                                PM_MPP__I_SINK__SWITCH_ENA);
               gpio_set_value(KP_BL_LED_GPIO, 1);		
	}
	//ret = pmic_set_led_intensity(LED_KEYPAD, value / MAX_KEYPAD_BL_LEVEL);
	if (ret)
		dev_err(led_cdev->dev, "can't set keypad backlight\n");
}

static struct led_classdev msm_kp_bl_led = {
	.name			= "keyboard-backlight",
	.brightness_set		= msm_keypad_bl_led_set,
	.brightness		= LED_OFF,
};

static int msm_pmic_led_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &msm_kp_bl_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led class driver\n");
		return rc;
	}

       rc = gpio_request((KP_BL_LED_GPIO), "kp_bl_led");	
       if (rc){
           pr_err("KP_BL_LED_GPIO request error\n");
           return -rc;
       }
       gpio_tlmm_config(GPIO_CFG(KP_BL_LED_GPIO, 0, GPIO_CFG_OUTPUT, 
				GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
	 
	msm_keypad_bl_led_set(&msm_kp_bl_led, LED_OFF);
	return rc;
}

static int __devexit msm_pmic_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&msm_kp_bl_led);

	return 0;
}

#ifdef CONFIG_PM
static int msm_pmic_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&msm_kp_bl_led);

	return 0;
}

static int msm_pmic_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&msm_kp_bl_led);

	return 0;
}
#else
#define msm_pmic_led_suspend NULL
#define msm_pmic_led_resume NULL
#endif

static struct platform_driver msm_pmic_led_driver = {
	.probe		= msm_pmic_led_probe,
	.remove		= __devexit_p(msm_pmic_led_remove),
	.suspend	= msm_pmic_led_suspend,
	.resume		= msm_pmic_led_resume,
	.driver		= {
		.name	= "pmic-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_pmic_led_init(void)
{
	return platform_driver_register(&msm_pmic_led_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&msm_pmic_led_driver);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("MSM PMIC LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-leds");
