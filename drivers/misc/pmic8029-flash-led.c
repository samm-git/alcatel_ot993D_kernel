/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/misc/pmic8029-flash-led.c
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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <mach/pmic.h>
#include <linux/workqueue.h>
#include <linux/pmic8029-flash-led.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("flash_led");

#define GPIO_FLASHLED 96
#define GPIO_FLASHLED_2 107

static void delay_shutdown_flash_led(struct work_struct *work);
static struct workqueue_struct *flash_led_control_workqueue;
static DECLARE_DELAYED_WORK(delayed_work_flash_led_control, delay_shutdown_flash_led);
static bool flash_led_is_using = false;
static struct wake_lock flash_led_wake_lock;

static int init_flash_led(void)
{
	int ret;

	flash_led_control_workqueue = create_singlethread_workqueue("flash_led_control");
        if (!flash_led_control_workqueue) {
                return -ENOMEM;
        }

        ret = gpio_request(GPIO_FLASHLED, "flash_led");
        if (ret) {
                pr_err("gpio %d for flash_led request error,return vale is %d\n", GPIO_FLASHLED, ret);
                return -EIO;
        }
        gpio_tlmm_config(GPIO_CFG(GPIO_FLASHLED, 0, GPIO_CFG_OUTPUT,
                                GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
        gpio_set_value(GPIO_FLASHLED, 0);
	   
        ret = gpio_request(GPIO_FLASHLED_2, "flash_led");
        if (ret) {
                pr_err("gpio %d for flash_led request error,return vale is %d\n", GPIO_FLASHLED_2, ret);
                return -EIO;
        }
        gpio_tlmm_config(GPIO_CFG(GPIO_FLASHLED_2, 0, GPIO_CFG_OUTPUT,
                                GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), GPIO_CFG_ENABLE);
        gpio_set_value(GPIO_FLASHLED_2, 0);

        return 0;
}

int set_flash_led(flash_led_level_t level, int delay_ms)
{
	int ret = 0;
	int i;

	if (! flash_led_control_workqueue)
	{
		printk("wqf---flash led has not init\n");
		return -EINVAL;
	}
	if (level < 0 || level >= LEVEL_NUM)
	{
		printk("wqf---flash led: level is out of range\n");
		return -EINVAL;
	}	
	if (flash_led_is_using)
	{
		printk("wqf---flash led is in using\n");
		return -EINVAL;
	}

	flash_led_is_using = true;
	
#if 0
	pmic_secure_mpp_control_digital_output(
                                    PM_MPP_5,
                                    PM_MPP__DLOGIC__LVL_VDD + level,
                                    PM_MPP__DLOGIC_OUT__CTRL_HIGH);
#endif

        gpio_set_value(GPIO_FLASHLED, 1);
        udelay(40);
        for (i = 0; i < level; i++)
        {
           gpio_set_value(GPIO_FLASHLED, 0);
           udelay(1);
           gpio_set_value(GPIO_FLASHLED, 1);
           udelay(1);

        }

        gpio_set_value(GPIO_FLASHLED_2, 1);
        udelay(40);
        for (i = 0; i < level; i++)
        {
           gpio_set_value(GPIO_FLASHLED_2, 0);
           udelay(1);
           gpio_set_value(GPIO_FLASHLED_2, 1);
           udelay(1);

        }

	if (delay_ms >= 0)
	{       
        	queue_delayed_work(flash_led_control_workqueue,
                                    &delayed_work_flash_led_control,
                                    msecs_to_jiffies(delay_ms));	
	}

	return ret;
}

void shutdown_flash_led(void)
{
	if (flash_led_is_using == true){
#if 0
		pmic_secure_mpp_control_digital_output(
                                    PM_MPP_5,
                                    PM_MPP__DLOGIC__LVL_VDD,
                                    PM_MPP__DLOGIC_OUT__CTRL_LOW);
#endif
		gpio_set_value(GPIO_FLASHLED, 0);
		gpio_set_value(GPIO_FLASHLED_2, 0);

		msleep(2);
		flash_led_is_using = false;
	}
}

static void delay_shutdown_flash_led(struct work_struct *work)
{
#if 0
	pmic_secure_mpp_control_digital_output(
                                    PM_MPP_5,
                                    PM_MPP__DLOGIC__LVL_VDD,
                                    PM_MPP__DLOGIC_OUT__CTRL_LOW);
#endif
	gpio_set_value(GPIO_FLASHLED, 0);
	gpio_set_value(GPIO_FLASHLED_2, 0);

	msleep(2);
	flash_led_is_using = false;
}

static long flash_led_ioctl(struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int brightness;

	switch (cmd) {
	case FLASH_LED_IOCTL_SET_BRIGHTNESS:
		if (copy_from_user(&brightness, argp, sizeof(brightness)))
			return -EFAULT;
		//printk("wqf-------------flash_led_ioctl set %d\n", brightness);
		shutdown_flash_led();
		set_flash_led(brightness, -1);
                if(!wake_lock_active(&flash_led_wake_lock))
                        wake_lock(&flash_led_wake_lock);
		break;

	case FLASH_LED_IOCTL_TURNOFF:
		//printk("wqf-------------flash_led_ioctl turn off\n");
		shutdown_flash_led();
                if(wake_lock_active(&flash_led_wake_lock))
                        wake_unlock(&flash_led_wake_lock);
		break;
	}

	return 0;
}

static const struct file_operations flash_led_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = flash_led_ioctl,
};

static struct miscdevice flash_led_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "flash_led",
	.fops = &flash_led_fops,
};

static int __init flash_led_init(void)
{
        if (misc_register(&flash_led_device)) {
                printk(KERN_ERR "flash_led: failed to register device\n" );
                return -1;
        }
	if (init_flash_led()) {
                printk(KERN_ERR "flash_led: failed to init device\n" );
                return -1;
        }
        wake_lock_init(&flash_led_wake_lock, WAKE_LOCK_SUSPEND, "flash_led_wake_lock");

	return 0;
}

module_init(flash_led_init);

static void __exit flash_led_exit(void)
{
        wake_lock_destroy(&flash_led_wake_lock);

	gpio_free(GPIO_FLASHLED);
	gpio_free(GPIO_FLASHLED_2);
	return;
}

module_exit(flash_led_exit);
