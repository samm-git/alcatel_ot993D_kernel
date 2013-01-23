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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <linux/usb/android.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/mfd/marimba.h>
#include <linux/power_supply.h>
#include <linux/input/rmi_platformdata.h>
#include <linux/input/rmi_i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#ifdef CONFIG_TOUCHSCREEN_FT5206
#include <linux/input/ft5x06_ts.h>
#endif
#include <asm/mach/mmc.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/usbdiag.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_serial_hs.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include <mach/vreg.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/socinfo.h>
#include <mach/oem_rapi_client.h>
#include "board-msm7x27a-regulator.h"
#include "board-qrd7627a-gpio.h"
#include "devices.h"
#include "devices-msm7x2xa.h"
#include "pm.h"
#include "timer.h"
#include "pm-boot.h"
#include "board-msm7x27a-regulator.h"
#include "board-msm7627a.h"

#ifdef  CONFIG_INPUT_LTR502
#include <linux/input/ltr502.h>
#endif

#ifdef  CONFIG_INPUT_LIS3DH
#include <linux/input/lis3dh.h>
#endif
#ifdef  CONFIG_INPUT_L3G4200D
#include <linux/input/k3g.h>
#endif
#ifdef CONFIG_SENSORS_AK8975
#include <linux/akm8975.h>
#endif

#define PMEM_KERNEL_EBI1_SIZE	0x3A000
#define MSM_PMEM_AUDIO_SIZE	0x5B000
#define BAHAMA_SLAVE_ID_FM_REG 0x02
int lcd_camera_power_onoff(int on);

#define GPIO_INVALID NR_MSM_GPIOS

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
extern struct bt_vreg_info bt_vregs[2];
#endif
#define GPIO_VDD33_EN (machine_is_msm7627a_qrd1()?119:(GPIO_INVALID))
enum {
	GPIO_VOTE_VDD33_WLAN ,
	GPIO_VOTE_VDD33_BT,
};
#define GPIO_VOTE_VDD33_WLAN_MASK (1 << GPIO_VOTE_VDD33_WLAN)
#define GPIO_VOTE_VDD33_BT_MASK (1 << GPIO_VOTE_VDD33_BT)
#define GPIO_VOTE_VDD33_ALL_MASK (GPIO_VOTE_VDD33_WLAN_MASK | GPIO_VOTE_VDD33_BT_MASK)
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
static unsigned long gpio_voter_table[NR_MSM_GPIOS];

static void gpio_set_value_vote(unsigned int gpio,int value,int voter_id,int condition_mask)
{
	int all_vote_ok = 0;
	if(gpio >= NR_MSM_GPIOS) {
		pr_err("Invalid GPIO number %d\n", gpio);
		return;
	}
	if(value) {
		set_bit(voter_id,&gpio_voter_table[gpio]);
		all_vote_ok = ((gpio_voter_table[gpio] & condition_mask) == condition_mask);
	}else{
		clear_bit(voter_id,&gpio_voter_table[gpio]);
		all_vote_ok = !(gpio_voter_table[gpio] & condition_mask);
	}
	if(all_vote_ok){
		gpio_set_value(gpio, !!value);
	}
}
#endif
#ifdef  CONFIG_INPUT_L3G4200D

static struct k3g_platform_data k3g_pdata = {
	.axis_map_x	= 1,
	.axis_map_y	= 1,
	.axis_map_z	= 1,
	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,
};

static struct i2c_board_info gyro_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("k3g", 0x68),
		.platform_data =  &k3g_pdata,
		.irq = -1,
	},
};

//#define GYRO_USE_IRQ
#ifdef GYRO_USE_IRQ

static struct msm_gpio gyro_gpio_cfg_data[] = {
	{
		GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
		"gyro_int"
	},
};

static int gyro_gpio_setup(void) {
	int ret = 0;
	gyro_gpio_cfg_data[0].gpio_cfg =
				GPIO_CFG(msm_gpio_table_qrd[GPIO_GYRO_INT_N_INDEX], 0,
				GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA);
	ret = msm_gpios_request_enable(gyro_gpio_cfg_data,
				 sizeof(gyro_gpio_cfg_data)/sizeof(struct msm_gpio));
	if( ret<0 )
		printk(KERN_ERR "%s: Failed to obtain gyro int GPIO %d. Code: %d\n",
				__func__, msm_gpio_table_qrd[GPIO_GYRO_INT_N_INDEX], ret);
	else {
		gyro_i2c_info[0].irq = gpio_to_irq(msm_gpio_table_qrd[GPIO_GYRO_INT_N_INDEX]);
		printk("gyro_i2c_info[0].irq is %d\n",gyro_i2c_info[0].irq);
	}
	return ret;
}
#endif
#endif

#ifdef CONFIG_BOSCH_BMA250
static struct i2c_board_info bma250_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("bma250", 0x18),
	},
};
#endif

#ifdef  CONFIG_INPUT_LIS3DH

static struct lis3dh_acc_platform_data lis3dh_acc_pdata = {
	.poll_interval	= 5,
	.min_interval	= 10,

	.g_range	= LIS3DH_ACC_G_2G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,

	.gpio_int1	= -EINVAL,//GPIO_ACC_INT_INDEX, not use interrupt
	.gpio_int2	= -EINVAL,

};

static struct i2c_board_info lis3dh_acc_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("lis3dh_acc", 0x18),
		.platform_data =  &lis3dh_acc_pdata,
		.irq = -1,
	},
};

static struct msm_gpio lis3dh_acc_gpio_cfg_data[] = {
	{
		GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),
		"lis3dh_acc_int"
	},
};

static int lis3dh_acc_gpio_setup(void) {
	int ret = 0;
        lis3dh_acc_gpio_cfg_data[0].gpio_cfg =
                                GPIO_CFG(msm_gpio_table_qrd[GPIO_ACC_INT_INDEX], 0,
                                GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA);
	ret = msm_gpios_request_enable(lis3dh_acc_gpio_cfg_data,
				 sizeof(lis3dh_acc_gpio_cfg_data)/sizeof(struct msm_gpio));
	if( ret<0 )
		printk(KERN_ERR "%s: Failed to obtain acc int GPIO %d. Code: %d\n",
				__func__, msm_gpio_table_qrd[GPIO_ACC_INT_INDEX], ret);
	//lis3dh_acc_i2c_info[0].irq = gpio_to_irq(msm_gpio_table_qrd[GPIO_ACC_INT_INDEX]);
	return ret;
}
#endif

#ifdef CONFIG_SENSORS_AK8975
static struct akm8975_platform_data akm_platform_data_8975 = {
		.gpio_DRDY = GPIO_INVALID,
};

static struct i2c_board_info akm8975_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("akm8975", 0x0e),
		.platform_data =  &akm_platform_data_8975,
		.flags = I2C_CLIENT_WAKE,
		.irq = GPIO_INVALID,//MSM_GPIO_TO_INT(GPIO_COMPASS_DRDY_INDEX),
	},
};
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_INPUT_LTR502)

static struct ltr502_platform_data ltr502_pdata = {
	.int_gpio = GPIO_INVALID,
};

static struct i2c_board_info ltr502_light_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("ltr502", 0x1c),
		.platform_data =  &ltr502_pdata,
	},
};

static struct msm_gpio ltr502_light_gpio_cfg_data[] = {
	{GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "ltr502_light_int"},
};

static int ltr502_light_gpio_setup(void) {
	int ret = 0;
	ltr502_pdata.int_gpio = msm_gpio_table_qrd[GPIO_LTR502_INT_INDEX];
	ltr502_light_gpio_cfg_data[0].gpio_cfg =
                                GPIO_CFG(msm_gpio_table_qrd[GPIO_LTR502_INT_INDEX], 0,
                                GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA);
	ret = msm_gpios_request_enable(ltr502_light_gpio_cfg_data, 1);
	if(ret < 0)
		printk(KERN_ERR "%s: Failed to obtain acc int GPIO %d. Code: %d\n",
				__func__, msm_gpio_table_qrd[GPIO_LTR502_INT_INDEX], ret);

	return ret;
}
#endif

#if 0
static struct platform_device msm_wlan_ar6000_pm_device = {
	.name           = "wlan_ar6000_pm_dev",
	.id             = -1,
};
#endif

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(60, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
	{ GPIO_CFG(131, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_scl" },
	{ GPIO_CFG(132, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
		"qup_sda" },
};

static void gsbi_qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc;

	if (adap_id < 0 || adap_id > 1)
		return;

	/* Each adapter gets 2 lines from the table */
	if (config_type)
		rc = msm_gpios_request_enable(&qup_i2c_gpios_hw[adap_id*2], 2);
	else
		rc = msm_gpios_request_enable(&qup_i2c_gpios_io[adap_id*2], 2);
	if (rc < 0)
		pr_err("QUP GPIO request/enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_gsbi0_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

static struct msm_i2c_platform_data msm_gsbi1_qup_i2c_pdata = {
	.clk_freq		= 100000,
	.msm_i2c_config_gpio	= gsbi_qup_i2c_gpio_config,
};

#ifdef CONFIG_ARCH_MSM7X27A
#define MSM_PMEM_MDP_SIZE       0x1DD1000
#define MSM_PMEM_ADSP_SIZE      0x1100000

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_SIZE		0x260000
#else
#define MSM_FB_SIZE		0x195000
#endif

#endif

#ifdef CONFIG_LEDS_TRICOLOR_FLAHSLIGHT
static struct msm_gpio tricolor_leds_gpio_cfg_data[] = {
	{
		GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"blue led"
	},{
		GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"red led"
	},{
		GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"green led"
	},{
		GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"flashlight"
	},
};

static int tricolor_leds_gpio_setup(void) {
	int ret = 0;
	ret = msm_gpios_request_enable(tricolor_leds_gpio_cfg_data,
			sizeof(tricolor_leds_gpio_cfg_data)/sizeof(struct msm_gpio));
	if( ret<0 )
		printk(KERN_ERR "%s: Failed to obtain tricolor_leds GPIO . Code: %d\n",
				__func__, ret);
	return ret;
}

static struct platform_device msm_device_tricolor_leds = {
	.name   = "tricolor leds and flashlight",
	.id = -1,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5206

static struct msm_gpio ft5206_cfg_data[] = {
{GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),"ft5206_irq"},
{GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA),"ft5206_reset"},
};

static ssize_t
ft5206_virtual_keys_register(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf,
                 __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":40:510:80:60"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":120:510:80:60"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":200:510:80:60"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":280:510:80:60"
            "\n");
}

static struct kobj_attribute ft5206_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ft5x0x_ts",
		.mode = S_IRUGO,
	},
	.show = &ft5206_virtual_keys_register,
};

static struct attribute *ft5206_virtual_key_properties_attrs[] = {
        &ft5206_virtual_keys_attr.attr,
        NULL
};

static struct attribute_group ft5206_virtual_key_properties_attr_group = {
        .attrs = ft5206_virtual_key_properties_attrs,
};

struct kobject *ft5206_virtual_key_properties_kobj;

static struct ft5x0x_ts_platform_data ft5206_platformdata;

static struct i2c_board_info i2c_info_ft5206 = {
	I2C_BOARD_INFO("ft5x0x_ts", 0x38),
	.platform_data = &ft5206_platformdata,
};

static int ft5206_touchpad_setup(void)
{
	int retval = 0;

	pr_info("the ft5206_touchpad_setup is setup\n");
    ft5206_virtual_key_properties_kobj = kobject_create_and_add( \
    "board_properties", NULL);
    if (ft5206_virtual_key_properties_kobj)
            retval = sysfs_create_group(ft5206_virtual_key_properties_kobj, \
            &ft5206_virtual_key_properties_attr_group);
    if (!ft5206_virtual_key_properties_kobj || retval)
            pr_err("failed to create ft5206 board_properties\n");

	retval = msm_gpios_request_enable(ft5206_cfg_data, sizeof(ft5206_cfg_data)/sizeof(struct msm_gpio));
	if(retval) {
		pr_err("%s: Failed to obtain touchpad GPIO. Code: %d.", __func__, retval);
	}

	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
			&i2c_info_ft5206, 1);

	return retval;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C) || \
defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C_MODULE)

static int synaptics_touchpad_setup(void);

static struct msm_gpio clearpad3000_cfg_data[] = {
	{GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_6MA), "rmi4_attn"},
	{GPIO_CFG(GPIO_INVALID, 0, GPIO_CFG_OUTPUT,
			GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), "rmi4_reset"},
};

static struct rmi_XY_pair rmi_offset = {.x = 0, .y = 0};
static struct rmi_range rmi_clipx = {.min = 48, .max = 980};
static struct rmi_range rmi_clipy = {.min = 7, .max = 1647};
static struct rmi_f11_functiondata synaptics_f11_data = {
	.swap_axes = false,
	.flipX = false,
	.flipY = false,
	.offset = &rmi_offset,
	.button_height = 113,
	.clipX = &rmi_clipx,
	.clipY = &rmi_clipy,
};

#define MAX_LEN		100

static ssize_t clearpad3000_virtual_keys_register(struct kobject *kobj,
		     struct kobj_attribute *attr, char *buf)
{
	char *virtual_keys = __stringify(EV_KEY) ":" __stringify(KEY_MENU) \
			     ":60:830:120:60" ":" __stringify(EV_KEY) \
			     ":" __stringify(KEY_HOMEPAGE)   ":180:830:120:60" \
				":" __stringify(EV_KEY) ":" \
				__stringify(KEY_SEARCH) ":300:830:120:60" \
				":" __stringify(EV_KEY) ":" \
			__stringify(KEY_BACK)   ":420:830:120:60" "\n";

	return snprintf(buf, strnlen(virtual_keys, MAX_LEN) + 1 , "%s",
			virtual_keys);
}

static struct kobj_attribute clearpad3000_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.sensor00fn11",
		.mode = S_IRUGO,
	},
	.show = &clearpad3000_virtual_keys_register,
};

static struct attribute *virtual_key_properties_attrs[] = {
	&clearpad3000_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group virtual_key_properties_attr_group = {
	.attrs = virtual_key_properties_attrs,
};

struct kobject *virtual_key_properties_kobj;

static struct rmi_functiondata synaptics_functiondata[] = {
	{
		.function_index = RMI_F11_INDEX,
		.data = &synaptics_f11_data,
	},
};

static struct rmi_functiondata_list synaptics_perfunctiondata = {
	.count = ARRAY_SIZE(synaptics_functiondata),
	.functiondata = synaptics_functiondata,
};

static struct rmi_sensordata synaptics_sensordata = {
	.perfunctiondata = &synaptics_perfunctiondata,
	.rmi_sensor_setup	= synaptics_touchpad_setup,
};

static struct rmi_i2c_platformdata synaptics_platformdata = {
	.i2c_address = 0x2c,
	.irq_type = IORESOURCE_IRQ_LOWLEVEL,
	.sensordata = &synaptics_sensordata,
};

static struct i2c_board_info synaptic_i2c_clearpad3k[] = {
	{
	I2C_BOARD_INFO("rmi4_ts", 0x2c),
	.platform_data = &synaptics_platformdata,
	},
};

static int synaptics_touchpad_setup(void)
{
	int retval = 0;

	virtual_key_properties_kobj =
		kobject_create_and_add("board_properties", NULL);
	if (virtual_key_properties_kobj)
		retval = sysfs_create_group(virtual_key_properties_kobj,
				&virtual_key_properties_attr_group);
	if (!virtual_key_properties_kobj || retval)
		pr_err("failed to create ft5202 board_properties\n");

	retval = msm_gpios_request_enable(clearpad3000_cfg_data,
		    sizeof(clearpad3000_cfg_data)/sizeof(struct msm_gpio));
	if (retval) {
		pr_err("%s:Failed to obtain touchpad GPIO %d. Code: %d.",
				__func__, msm_gpio_table_qrd[CLEARPAD3000_ATTEN_GPIO_INDEX], retval);
		retval = 0; /* ignore the err */
	}
	synaptics_platformdata.irq = gpio_to_irq(msm_gpio_table_qrd[CLEARPAD3000_ATTEN_GPIO_INDEX]);

	gpio_set_value(msm_gpio_table_qrd[CLEARPAD3000_RESET_GPIO_INDEX], 0);
	usleep(10000);
	gpio_set_value(msm_gpio_table_qrd[CLEARPAD3000_RESET_GPIO_INDEX], 1);
	usleep(50000);

	return retval;
}
#endif


static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc = 0;
	unsigned gpio;

	gpio = msm_gpio_table_qrd[GPIO_HOST_VBUS_EN_INDEX];

	rc = gpio_request(gpio,	"i2c_host_vbus_en");
	if (rc < 0) {
		pr_err("failed to request %d GPIO\n", gpio);
		return;
	}
	gpio_direction_output(gpio, !!on);
	gpio_set_value_cansleep(gpio, !!on);
	gpio_free(gpio);
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
};

static void __init msm7627a_init_host(void)
{
	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct regulator *reg_hsusb;
static int msm_hsusb_ldo_init(int init)
{
	int rc = 0;

	if (init) {
		reg_hsusb = regulator_get(NULL, "usb");
		if (IS_ERR(reg_hsusb)) {
			rc = PTR_ERR(reg_hsusb);
			pr_err("%s: could not get regulator: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_set_voltage(reg_hsusb, 3300000, 3300000);
		if (rc) {
			pr_err("%s: could not set voltage: %d\n",
					__func__, rc);
			goto reg_free;
		}

		return 0;
	}
	/* else fall through */
reg_free:
	regulator_put(reg_hsusb);
out:
	reg_hsusb = NULL;
	return rc;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (IS_ERR_OR_NULL(reg_hsusb))
		return reg_hsusb ? PTR_ERR(reg_hsusb) : -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	return enable ?
		regulator_enable(reg_hsusb) :
		regulator_disable(reg_hsusb);
}

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret = 0;

	if (init)
		ret = msm_pm_app_rpc_init(callback);
	else
		msm_pm_app_rpc_deinit(callback);

	return ret;
}
#endif

static struct msm_otg_platform_data msm_otg_pdata = {
#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init	 = msm_hsusb_pmic_notif_init,
#else
	.vbus_power		 = msm_hsusb_vbus_power,
#endif
	.rpc_connect		 = hsusb_rpc_connect,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.chg_init		 = hsusb_chg_init,
	.chg_connected		 = hsusb_chg_connected,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
};
#endif

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup	= 1,
	.rx_to_inject		= 0xFD,
};
#endif
static struct msm_pm_platform_data msm7627a_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 16000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 12000,
					.residency = 20000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 0,
					.suspend_enabled = 1,
					.latency = 2000,
					.residency = 0,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
					.idle_supported = 1,
					.suspend_supported = 1,
					.idle_enabled = 1,
					.suspend_enabled = 1,
					.latency = 2,
					.residency = 0,
	},
};

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_PHYS,
	.p_addr = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}

early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}

early_param("fb_size", fb_size_setup);

static struct resource msm_fb_resources[] = {
	{
		.flags	= IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -ENODEV;

	if (machine_is_msm7627a_qrd1()) {
		if (!strncmp(name, "mipi_video_truly_wvga", 21))
			ret = 0;
	}else if (machine_is_msm7627a_qrd3()) {
		if (!strncmp(name, "lcdc_truly_hvga_ips3p2335_pt", 28))
			ret = 0;
	}else {
		ret = -ENODEV;
	}

	return ret;
}

static int mipi_truly_set_bl(int on)
{
	gpio_set_value(msm_lcd_gpio_table_qrd[GPIO_BACKLIGHT_EN_INDEX], !!on);

	return 1;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct msm_panel_common_pdata mipi_truly_pdata = {
	.pmic_backlight = mipi_truly_set_bl,
};

static struct platform_device mipi_dsi_truly_panel_device = {
	.name	= "mipi_truly",
	.id	= 0,
	.dev	= {
		.platform_data = &mipi_truly_pdata,
	}
};

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(FM_DIGITAL_STEREO_HEADSET, 26),
	SND(FM_DIGITAL_SPEAKER_PHONE, 27),
	SND(FM_DIGITAL_BT_A2DP_HEADSET, 28),
	SND(STEREO_HEADSET_AND_SPEAKER, 31),
	SND(CURRENT, 0x7FFFFFFE),
	SND(FM_ANALOG_STEREO_HEADSET, 35),
	SND(FM_ANALOG_STEREO_HEADSET_CODEC, 36),
    SND(NO_MIC_HEADSET, 37),
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design     = 3500,
	.voltage_max_design     = 4200,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage	 = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};
static struct platform_device msm_wlan_ar6000_pm_device = {
	.name	=	"wlan_ar6000_pm_dev",
	.id	=	1,
	.num_resources	=	0,
	.resource	=	NULL,
};
#define GPIO_WLAN_CHIP_PWD_L_old 124
#define GPIO_WLAN_CHIP_PWD_L 40
static struct vreg *vreg_wlan;
static void wlan_power_on(void)
{
	int status;
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	const char *btid = "BTPW";
#endif
	/* Voting for 1.8V Regulator */
        vreg_wlan = vreg_get(NULL , "wlan4");
        if (IS_ERR(vreg_wlan)) {
                pr_err("%s: vreg get failed with : (%ld)\n",
                        __func__, PTR_ERR(vreg_wlan));
                return ;
        }

        /* Set the voltage level to 1.8V */
        status = vreg_set_level(vreg_wlan, 1800);
        if (status < 0) {
                pr_err("%s: set regulator level failed with :(%d)\n",
                        __func__, status);
                goto wlan_vreg_fail;
        }

        /* Enabling the 1.8V regulator */
        status = vreg_enable(vreg_wlan);
        if (status) {
                pr_err("%s: enable regulator failed with :(%d)\n",
                        __func__, status);
                goto wlan_vreg_fail;
        }

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	if(machine_is_msm7627a_qrd1()){
		status = gpio_tlmm_config(GPIO_CFG(GPIO_VDD33_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
		if(status){
			pr_err("%s gpio_tlmm_config %d failed,error = %d\n", __func__,GPIO_VDD33_EN,status);
		}
		gpio_set_value_vote(GPIO_VDD33_EN, 1, GPIO_VOTE_VDD33_WLAN,GPIO_VOTE_VDD33_WLAN_MASK);
	}else {
		status = regulator_set_voltage(bt_vregs[1].reg,bt_vregs[1].min_level,bt_vregs[1].max_level);
		if (status < 0) {
			pr_err("%s: vreg set level failed (%d)\n",__func__, status);
		}
		status = regulator_enable(bt_vregs[1].reg);

		if (status) {
			pr_err("%s: vreg %s failed(%d)\n",__func__, bt_vregs[1].name,status);
		}
		if (bt_vregs[1].is_pin_controlled == 1) {
			status = pmapp_vreg_lpm_pincntrl_vote(btid,
							bt_vregs[1].pmapp_id,
							PMAPP_CLOCK_ID_D1,
							PMAPP_CLOCK_VOTE_ON);
			if (status < 0) {
				pr_err("%s: vreg %s pin ctrl failed(%d)\n",
					__func__, bt_vregs[1].name,
					status);
			}
		}

	}
	return ;
#endif
wlan_vreg_fail:
	vreg_put(vreg_wlan);
}

static void wlan_power_off(void)
{
        int status;

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	const char *btid = "BTPW";
	if(machine_is_msm7627a_qrd1()){
		status = gpio_tlmm_config(GPIO_CFG(GPIO_VDD33_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
		if(status){
			pr_err("%s gpio_tlmm_config 119 failed,error = %d\n", __func__,status);
		}
		gpio_set_value_vote(GPIO_VDD33_EN, 0, GPIO_VOTE_VDD33_WLAN,GPIO_VOTE_VDD33_ALL_MASK);
	}else {
		if (bt_vregs[1].is_pin_controlled == 1) {
			status = pmapp_vreg_lpm_pincntrl_vote(btid,
								bt_vregs[1].pmapp_id,
								PMAPP_CLOCK_ID_D1,
								PMAPP_CLOCK_VOTE_OFF);
			if (status < 0) {
				pr_err("%s: vreg %s pin ctrl failed(%d)\n",
					__func__, bt_vregs[1].name,status);
			}
		}
		status = regulator_disable(bt_vregs[1].reg);
		if (status < 0) {
			pr_err("%s: vreg %s disable failed(%d)\n",__func__, bt_vregs[1].name,status);
		}
	}
#endif
	if (vreg_wlan != NULL) {
		status = vreg_disable(vreg_wlan);
		if (status)
			pr_err("%s: disable regulator failed:(%d)\n",__func__, status);
		vreg_wlan = NULL;
	}

}

int wlan_power_ctrl(unsigned char on)
{
	int rc = 0;
	const char *id = "WLPW";
	if(on){
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0,
		PMAPP_CLOCK_VOTE_ON);
		if (rc < 0) {
			pr_err("Failed to vote for TCXO_A0 ON(WLAN)\n");
			return rc;
		}

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0,
		PMAPP_CLOCK_VOTE_PIN_CTRL);
		if (rc < 0)
			pr_err("%s:Pin Control Failed, rc = %d", __func__, rc);

		wlan_power_on();
#if 1
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_CHIP_PWD_L_old, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
		if(rc){
			pr_err("%s gpio_tlmm_config 40 failed,error = %d\n", __func__,rc);
		}
		gpio_set_value(GPIO_WLAN_CHIP_PWD_L_old, 1);
#endif		
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_CHIP_PWD_L, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
		if(rc){
			pr_err("%s gpio_tlmm_config 40 failed,error = %d\n", __func__,rc);
		}
		gpio_set_value(GPIO_WLAN_CHIP_PWD_L, 1);
	}else{
#if 1
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_CHIP_PWD_L_old, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
		if(rc){
			pr_err("%s gpio_tlmm_config 40 failed,error = %d\n", __func__,rc);
		}
		gpio_set_value(GPIO_WLAN_CHIP_PWD_L_old, 0);
#endif	
		rc = gpio_tlmm_config(GPIO_CFG(GPIO_WLAN_CHIP_PWD_L, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
		if(rc){
			pr_err("%s gpio_tlmm_config 40 failed,error = %d\n", __func__,rc);
		}
		gpio_set_value(GPIO_WLAN_CHIP_PWD_L, 0);
		wlan_power_off();
		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A0,
		PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0) {
			pr_err("Failed to vote for TCXO_A0 OFF(WLAN)\n");
		}
	}
	return rc;
}
EXPORT_SYMBOL(wlan_power_ctrl);

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static void qrd1_camera_gpio_cfg(void)
{

	int rc = 0;

	rc = gpio_request(msm_gpio_table_qrd[GPIO_CAM_BACK_SHDN_EN_INDEX], "cam_back_pwdn");
	if (rc < 0)
		pr_err("%s: gpio_request---GPIO_CAM_BACK_SHDN_EN failed!", __func__);

	rc = gpio_tlmm_config(GPIO_CFG(msm_gpio_table_qrd[GPIO_CAM_BACK_SHDN_EN_INDEX], 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to enable Power Down gpio for main camera!\n", __func__);
		gpio_free(msm_gpio_table_qrd[GPIO_CAM_BACK_SHDN_EN_INDEX]);
	}

	rc = gpio_request(msm_gpio_table_qrd[GPIO_CAM_BACK_RESET_INDEX], "cam_back_rst");
	if (rc < 0) {
		pr_err("%s: gpio_request---GPIO_CAM_BACK_RESET failed!",
				__func__);
		gpio_free(msm_gpio_table_qrd[GPIO_CAM_BACK_SHDN_EN_INDEX]);
	}

	rc = gpio_tlmm_config(GPIO_CFG(msm_gpio_table_qrd[GPIO_CAM_BACK_RESET_INDEX], 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to enable reset gpio for main camera!\n", __func__);
		gpio_free(msm_gpio_table_qrd[GPIO_CAM_BACK_RESET_INDEX]);
	}

	rc = gpio_request(msm_gpio_table_qrd[GPIO_CAM_FRONT_PWDN_INDEX], "cam_front_pwdn");
	if (rc < 0)
		pr_err("%s: gpio_request---GPIO_CAM_FRONT_PWDN failed!", __func__);

	rc = gpio_tlmm_config(GPIO_CFG(msm_gpio_table_qrd[GPIO_CAM_FRONT_PWDN_INDEX], 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to enable Power Down gpio for front camera!\n", __func__);
		gpio_free(msm_gpio_table_qrd[GPIO_CAM_FRONT_PWDN_INDEX]);
	}

	rc = gpio_request(msm_gpio_table_qrd[GPIO_CAM_FRONT_RESET_INDEX], "cam_front_rst");
	if (rc < 0)
		pr_err("%s: gpio_request---GPIO_CAM_FRONT_RESET failed!", __func__);

	rc = gpio_tlmm_config(GPIO_CFG(msm_gpio_table_qrd[GPIO_CAM_FRONT_RESET_INDEX], 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("%s: unable to enable reset gpio for front camera!\n", __func__);
		gpio_free(msm_gpio_table_qrd[GPIO_CAM_FRONT_RESET_INDEX]);
	}

	gpio_direction_output(msm_gpio_table_qrd[GPIO_CAM_BACK_SHDN_EN_INDEX], 1);
	gpio_direction_output(msm_gpio_table_qrd[GPIO_CAM_BACK_RESET_INDEX], 0);
	gpio_direction_output(msm_gpio_table_qrd[GPIO_CAM_FRONT_PWDN_INDEX], 1);
	gpio_direction_output(msm_gpio_table_qrd[GPIO_CAM_FRONT_RESET_INDEX], 0);
}

#endif
static struct regulator_bulk_data regs_camera[] = {
	{ .supply = "msme1", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "gp2",   .min_uV = 2850000, .max_uV = 2850000 },
	{ .supply = "usb2",  .min_uV = 1800000, .max_uV = 1800000 },
};

static void __init msm_camera_vreg_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_camera), regs_camera);
	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		return;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_camera), regs_camera);
	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		return;
	}
}

static void msm_camera_vreg_config(int vreg_en)
{
	int rc = vreg_en ?
		regulator_bulk_enable(ARRAY_SIZE(regs_camera), regs_camera) :
		regulator_bulk_disable(ARRAY_SIZE(regs_camera), regs_camera);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, vreg_en ? "en" : "dis", rc);
}
static int config_gpio_table(uint32_t *table, int len)
{
	int rc = 0, i = 0;

	for (i = 0; i < len; i++) {
		rc = gpio_tlmm_config(table[i], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s not able to get gpio\n", __func__);
			for (i--; i >= 0; i--)
				gpio_tlmm_config(camera_off_gpio_table[i],
							GPIO_CFG_ENABLE);
			break;
		}
	}
	return rc;
}

static int config_camera_on_gpios_rear(void)
{
	int rc = 0;

	msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_rear(void)
{
	msm_camera_vreg_config(0);
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

static int config_camera_on_gpios_front(void)
{
	int rc = 0;

	msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_front(void)
{
	msm_camera_vreg_config(0);

	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

struct msm_camera_device_platform_data msm_camera_data_rear = {
	.camera_gpio_on		= config_camera_on_gpios_rear,
	.camera_gpio_off	= config_camera_off_gpios_rear,
	.ioext.csiphy		= 0xA1000000,
	.ioext.csisz		= 0x00100000,
	.ioext.csiirq		= INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate	= 24000000,
	.ioclk.vfe_clk_rate	= 192000000,
	.ioext.appphy		= MSM_CLK_CTL_PHYS,
	.ioext.appsz		= MSM_CLK_CTL_SIZE,
};

struct msm_camera_device_platform_data msm_camera_data_front = {
	.camera_gpio_on		= config_camera_on_gpios_front,
	.camera_gpio_off	= config_camera_off_gpios_front,
	.ioext.csiphy		= 0xA0F00000,
	.ioext.csisz		= 0x00100000,
	.ioext.csiirq		= INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate	= 24000000,
	.ioclk.vfe_clk_rate	= 192000000,
	.ioext.appphy		= MSM_CLK_CTL_PHYS,
	.ioext.appsz		= MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_OV5647
static struct msm_camera_sensor_platform_info ov5647_sensor_7627a_info = {
        .mount_angle = 90
};

static struct msm_camera_sensor_flash_src msm_flash_src_ov5647 = {
        .flash_sr_type = MSM_CAMERA_FLASH_SRC_LED,
        ._fsrc.led_src.led_name = "flashlight",
        ._fsrc.led_src.led_name_len = 10,
};

static struct msm_camera_sensor_flash_data flash_ov5647 = {
        .flash_type             = MSM_CAMERA_FLASH_LED,
        .flash_src              = &msm_flash_src_ov5647,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov5647_data = {
        .sensor_name    = "ov5647",
        .sensor_reset_enable = 1,
        .sensor_reset   = 23,
        .sensor_pwd     = 93,
        .vcm_pwd        = 30,
        .vcm_enable     = 0,
        .pdata          = &msm_camera_data_rear,
        .flash_data     = &flash_ov5647,
        .sensor_platform_info   = &ov5647_sensor_7627a_info,
        .csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov5647 = {
        .name      = "msm_camera_ov5647",
        .dev       = {
                .platform_data = &msm_camera_sensor_ov5647_data,
        },
};
#endif

#ifdef CONFIG_OV5640
static struct msm_camera_sensor_platform_info ov5640_sensor_info = {
	.mount_angle	= 90
};

static struct msm_camera_sensor_flash_src msm_flash_src_ov5640 = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_LED,
	._fsrc.led_src.led_name = "flashlight",
	._fsrc.led_src.led_name_len = 10,
};

static struct msm_camera_sensor_flash_data flash_ov5640 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
	.flash_src	= &msm_flash_src_ov5640,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov5640_data = {
	.sensor_name		= "ov5640",
	.sensor_reset_enable	= 1,
	.vcm_pwd		= 0,
	.vcm_enable		= 0,
	.pdata			= &msm_camera_data_rear,
	.flash_data		= &flash_ov5640,
	.sensor_platform_info	= &ov5640_sensor_info,
	.csi_if			= 1,
};

static struct platform_device msm_camera_sensor_ov5640 = {
	.name	= "msm_camera_ov5640",
	.dev	= {
		.platform_data = &msm_camera_sensor_ov5640_data,
	},
};
#endif

#ifdef CONFIG_WEBCAM_OV7692_QRD
static struct msm_camera_sensor_platform_info ov7692_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_ov7692 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_info msm_camera_sensor_ov7692_data = {
	.sensor_name		= "ov7692",
	.sensor_reset_enable	= 0,
	.sensor_reset		= 0,
	.vcm_pwd		= 0,
	.vcm_enable		= 0,
	.pdata			= &msm_camera_data_front,
	.flash_data		= &flash_ov7692,
	.sensor_platform_info   = &ov7692_sensor_7627a_info,
	.csi_if			= 1,
};

static struct platform_device msm_camera_sensor_ov7692 = {
	.name	= "msm_camera_ov7692",
	.dev	= {
		.platform_data = &msm_camera_sensor_ov7692_data,
	},
};
#endif

static struct i2c_board_info i2c_camera_devices[] = {
	#ifdef CONFIG_OV5647
	{
		I2C_BOARD_INFO("ov5647", 0x36 << 1),
	},
	{
		I2C_BOARD_INFO("ov5647_af", 0x18 >> 1),
	},
	#endif
	#ifdef CONFIG_OV5640
	{
		I2C_BOARD_INFO("ov5640", 0x78 >> 1),
	},
	#endif
	#ifdef CONFIG_WEBCAM_OV7692_QRD
	{
		I2C_BOARD_INFO("ov7692", 0x78),
	},
	#endif
};

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static void __init msm_msm7627a_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n", size,
						addr, __pa(addr));
}

static struct memtype_reserve msm7627a_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
	.mdp_rev = MDP_REV_303,
};

#ifdef CONFIG_FB_MSM_MDP303
enum {
	DSI_SINGLE_LANE = 1,
	DSI_TWO_LANES,
};

static int msm_fb_get_lane_config(void)
{
	int rc = DSI_TWO_LANES;
	if (cpu_is_msm7x25a() || cpu_is_msm7x25aa()) {
		rc = DSI_SINGLE_LANE;
		pr_info("DSI Single Lane\n");
	} else {
		pr_info("DSI Two Lanes\n");
	}

	return DSI_TWO_LANES;
}

static int mipi_truly_sel_mode(int video_mode)
{
	int rc = 0;

	rc = gpio_request(msm_lcd_gpio_table_qrd[GPIO_LCD_DSI_SEL_INDEX], "lcd_dsi_sel");
	if (rc < 0)
		goto gpio_error;

	rc = gpio_tlmm_config( GPIO_CFG(msm_lcd_gpio_table_qrd[GPIO_LCD_DSI_SEL_INDEX], 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,
				            GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc)
		goto gpio_error;

	rc = gpio_direction_output(msm_lcd_gpio_table_qrd[GPIO_LCD_DSI_SEL_INDEX], 1);
	if (!rc) {
		gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[GPIO_LCD_DSI_SEL_INDEX], video_mode);
		return rc;
	} else {
		goto gpio_error;
	}

gpio_error:
	pr_err("mipi_truly_sel_mode failed\n");
	gpio_free(msm_lcd_gpio_table_qrd[GPIO_LCD_DSI_SEL_INDEX]);
	return rc;
}

static int msm_fb_dsi_client_qrd1_reset(void)
{
	int rc = 0;

	rc = gpio_request(msm_lcd_gpio_table_qrd[GPIO_LCDC_BRDG_RESET_N_INDEX], "lcdc_brdg_reset_n");
	if (rc < 0) {
		pr_err("failed to request lcd brdg reset_n\n");
		return rc;
	}

	rc = gpio_tlmm_config(GPIO_CFG(msm_lcd_gpio_table_qrd[GPIO_LCDC_BRDG_RESET_N_INDEX], 0, GPIO_CFG_OUTPUT,
				        GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (rc < 0) {
		pr_err("Failed to enable LCDC Bridge reset enable\n");
		return rc;
	}

	rc = gpio_direction_output(msm_lcd_gpio_table_qrd[GPIO_LCDC_BRDG_RESET_N_INDEX], 1);
	if (rc < 0) {
		pr_err("Failed GPIO bridge pd\n");
		gpio_free(msm_lcd_gpio_table_qrd[GPIO_LCDC_BRDG_RESET_N_INDEX]);
		return rc;
	}

	mipi_truly_sel_mode(1);

	return rc;
}

static int msm_fb_dsi_client_reset(void)
{
	int rc = 0;

	rc = msm_fb_dsi_client_qrd1_reset();
	return rc;
}

static int dsi_gpio_initialized;

static int mipi_dsi_panel_qrd1_power(int on)
{
	int rc = 0;

	if (!dsi_gpio_initialized) {
		rc = gpio_request(msm_lcd_gpio_table_qrd[GPIO_BACKLIGHT_EN_INDEX], "gpio_bkl_en");
		if (rc < 0)
			return rc;

		rc = gpio_tlmm_config(GPIO_CFG(msm_lcd_gpio_table_qrd[GPIO_BACKLIGHT_EN_INDEX], 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
		if (rc < 0) {
			pr_err("failed GPIO_BACKLIGHT_EN tlmm config\n");
			return rc;
		}

		rc = gpio_direction_output(msm_lcd_gpio_table_qrd[GPIO_BACKLIGHT_EN_INDEX], 0);
		if (rc < 0) {
			pr_err("failed to enable backlight\n");
			gpio_free(msm_lcd_gpio_table_qrd[GPIO_BACKLIGHT_EN_INDEX]);
			return rc;
		}
		dsi_gpio_initialized = 1;
	}
	gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[GPIO_LCD_DSI_SEL_INDEX], 0);
  	gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[GPIO_LCDC_BRDG_RESET_N_INDEX], 0);

	lcd_camera_power_onoff(on);

	return rc;
}

static int mipi_dsi_panel_power(int on)
{
	int rc = 0;

	rc = mipi_dsi_panel_qrd1_power(on);
	return rc;
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.dsi_power_save		= mipi_dsi_panel_power,
	.dsi_client_reset	= msm_fb_dsi_client_reset,
	.get_lane_config	= msm_fb_get_lane_config,
};
#endif

static struct vreg *lcd_camera_vreg = NULL; //SKU3 VREG 1.8V
static atomic_t lcd_camera_power_cnt = ATOMIC_INIT(-1);

static int lcd_camera_control_is_needed(void)
{
  int need = 0;

  if (machine_is_msm7627a_qrd3()) {
    if (!hw_version_ge(2,0)) {
      pr_info("need not in old board\n");
      need = 0;
    } else {
      need = 1;
    }
  } else if (machine_is_msm7627a_qrd1()) {
    /* TODO: check for SKU1 */
    need = 1;
  } else {
    pr_info("need not in surf & ffa board\n");
    need = 0;
  }

  pr_info("%s: need = %d\n", __func__, need);

  return need;
}

static void lcd_camera_power_init(void)
{
  int rc = 0;

  pr_info("lcd_camera_power_init\n");

  if (lcd_camera_control_is_needed() == 0)
    return;

  //LDO_EXT2V8
  if (gpio_request(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_2V8_INDEX], "lcd_camera_ldo_2v8")) {
    pr_err("failed to request gpio lcd_camera_ldo_2v8\n");
    return;
  }

  rc = gpio_tlmm_config(GPIO_CFG(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_2V8_INDEX], 0,
        GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
        GPIO_CFG_2MA), GPIO_CFG_ENABLE);
  if (rc < 0) {
    pr_err("%s: unable to enable lcd_camera_ldo_2v8!\n", __func__);
    goto fail_gpio2;
  }

  if (machine_is_msm7627a_qrd1() || (machine_is_msm7627a_qrd3() && hw_version_ge(3,0))) {
    //LDO_EVT1V8
    if (gpio_request(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_1V8_INDEX], "lcd_camera_ldo_1v8")) {
      pr_err("failed to request gpio lcd_camera_ldo_1v8\n");
      goto fail_gpio2;
    }

    rc = gpio_tlmm_config(GPIO_CFG(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_1V8_INDEX], 0,
            GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
            GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if (rc < 0) {
      pr_err("%s: unable to enable lcdc_camera_ldo_1v8!\n", __func__);
      goto fail_gpio1;
    }
  }

  if (machine_is_msm7627a_qrd3()) {
    //VREG_L1, SKU3 PVT uses it for CTP 1.8V, DVT uses it for LCD/Camera 1.8V
    lcd_camera_vreg = vreg_get(NULL, "rfrx1");
    rc = vreg_set_level(lcd_camera_vreg, 1800);
    if (rc < 0) {
      pr_err("%s: set regulator level failed "
        "with :(%d)\n", __func__, rc);
	  /* Don't put it since it will be used by power_onff */
     //vreg_put(lcd_camera_vreg);
      goto fail_gpio2;
    }
  }

  return;

fail_gpio1:
  gpio_free(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_1V8_INDEX]);
fail_gpio2:
  gpio_free(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_2V8_INDEX]);
  return;
}

static int lcd_camera_power_on_qrd1(void)
{
  pr_info("turn on qrd1 lcd_camera_ldo\n");
  gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_1V8_INDEX], 1);
  gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_2V8_INDEX], 1);

  return 0;
}

static int lcd_camera_power_off_qrd1(void)
{
  pr_info("turn off qrd1 lcd_camera_ldo\n");

  gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_1V8_INDEX], 0);
  gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_2V8_INDEX], 0);

  return 0;
}

static int lcd_camera_power_on_qrd3(void)
{
  int rc = 0;

  pr_info("turn on qrd3 lcd_camera_vreg\n");
  if (hw_version_ge(3,0)) {
	  gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_1V8_INDEX], 1);
  }

  rc = vreg_enable(lcd_camera_vreg);
  if (rc) {
	  pr_err("vreg_enable: lcd_camera_vreg vreg failed\n");
  }

  pr_info("turn on qrd3 lcd_camera_ldo\n");
  gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_2V8_INDEX], 1);

  return rc;
}

static int lcd_camera_power_off_qrd3(void)
{
  int rc = 0;

  pr_info("turn off qrd3 lcd_camera_vreg\n");
  if (hw_version_ge(3,0)) {
	  gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_1V8_INDEX], 0);
  }

  rc = vreg_disable(lcd_camera_vreg);
  if (rc) {
	  pr_err("vreg_disable: lcd_camera_vreg vreg failed\n");
  }

  pr_info("turn off qrd3 lcd_camera_ldo\n");
  gpio_set_value_cansleep(msm_lcd_gpio_table_qrd[LCD_CAMERA_LDO_2V8_INDEX], 0);

  return rc;
}

int lcd_camera_power_onoff(int on)
{
  int rc = 0;

  pr_info("lcd_camera_power_onoff on = %d\n", on);

  if (lcd_camera_control_is_needed() == 0)
    return rc;

  if (on) {
    if (atomic_inc_and_test(&lcd_camera_power_cnt)) {
      if (machine_is_msm7627a_qrd1()) {
        rc = lcd_camera_power_on_qrd1();
      } else {
        rc = lcd_camera_power_on_qrd3();
      }
    }
  } else {
    if (atomic_sub_return(1, &lcd_camera_power_cnt) == -1) {
      if (machine_is_msm7627a_qrd1()) {
        rc = lcd_camera_power_off_qrd1();
      } else {
        rc = lcd_camera_power_off_qrd3();
      }
    }
  }

  return rc;
}
EXPORT_SYMBOL(lcd_camera_power_onoff);

#ifdef CONFIG_FB_MSM_LCDC
static uint32_t lcdc_gpio_initialized = 0;

static uint32_t lcdc_gpio_table[] = {
	19,
	20,
	21,
	89,
	90,
};

static char *lcdc_gpio_name_table[6] = {
	"spi_mosi",
	"spi_clk",
	"spi_cs",
	"gpio_bkl_en",
	"gpio_disp_reset",
};

static void lcdc_truly_gpio_init(void)
{
	int i;
	int rc = 0;

	pr_info("%s: lcdc_gpio_initialized = %d\n", __func__, lcdc_gpio_initialized);

	if (!lcdc_gpio_initialized) {
		for (i = 0; i < ARRAY_SIZE(lcdc_gpio_table); i++) {
			printk("%s: config %d\n", __func__, lcdc_gpio_table[i]);
			rc = gpio_request(lcdc_gpio_table[i], lcdc_gpio_name_table[i]);
			if (rc < 0) {
				pr_err("Error request gpio %s\n", lcdc_gpio_name_table[i]);
				break;
			}
			rc = gpio_tlmm_config(GPIO_CFG(lcdc_gpio_table[i], 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			if (rc < 0) {
				pr_err("Error config lcdc gpio:%d\n", lcdc_gpio_table[i]);
				break;
			}
			rc = gpio_direction_output(lcdc_gpio_table[i], 0);
			if (rc < 0) {
				pr_err("Error direct lcdc gpio:%d\n", lcdc_gpio_table[i]);
				break;
			}
		}

		if (rc < 0) {
			for (; i >= 0; i--) {
				gpio_free(lcdc_gpio_table[i]);
			}
			lcdc_gpio_initialized = 0;
		} else {
			lcdc_gpio_initialized = 1;
		}
	}

	return;
}

static struct msm_panel_common_pdata lcdc_truly_panel_data = {
	.panel_config_gpio = NULL,
	.gpio_num	  = lcdc_gpio_table,
};

static struct platform_device lcdc_truly_panel_device = {
	.name   = "lcdc_truly_hvga_ips3p2335_pt",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_truly_panel_data,
	}
};

static int msm_fb_lcdc_power_save(int on)
{
	int rc = 0;

  pr_info("%s\n", __func__);

  if (on) {
    lcd_camera_power_onoff(1);

    lcdc_truly_gpio_init();

    if (lcdc_gpio_initialized) {
      /*LCD reset*/
      gpio_set_value(msm_lcd_gpio_table_qrd[GPIO_DISPLAY_RESET_INDEX], 1);
      msleep(10);
      gpio_set_value(msm_lcd_gpio_table_qrd[GPIO_DISPLAY_RESET_INDEX], 0);
      msleep(10);
      gpio_set_value(msm_lcd_gpio_table_qrd[GPIO_DISPLAY_RESET_INDEX], 1);
      msleep(10);
    }
  } else {
     /* pull down LCD IO to avoid current leakage */
     gpio_set_value(msm_lcd_gpio_table_qrd[GPIO_SPI_MOSI_INDEX], 0);
     gpio_set_value(msm_lcd_gpio_table_qrd[GPIO_SPI_CLK_INDEX], 0);
     gpio_set_value(msm_lcd_gpio_table_qrd[GPIO_SPI_CS0_N_INDEX], 0);
     gpio_set_value(msm_lcd_gpio_table_qrd[GPIO_DISPLAY_RESET_INDEX], 0);

     lcd_camera_power_onoff(0);
  }

	return rc;
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = NULL,
	.lcdc_power_save   = msm_fb_lcdc_power_save,
};
#endif

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
#ifdef CONFIG_FB_MSM_MDP303
	if(machine_is_msm7627a_qrd1())
		msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
#endif
#ifdef CONFIG_FB_MSM_LCDC
	if(machine_is_msm7627a_qrd3())
		msm_fb_register_device("lcdc", &lcdc_pdata);
#endif
}

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	android_pmem_adsp_pdata.size = pmem_adsp_size;
	android_pmem_pdata.size = pmem_mdp_size;
	android_pmem_audio_pdata.size = pmem_audio_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7627a_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	msm7627a_reserve_table[MEMTYPE_EBI1].size += pmem_kernel_ebi1_size;
#endif
}

static void __init msm7627a_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7627a_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info msm7627a_reserve_info __initdata = {
	.memtype_reserve_table = msm7627a_reserve_table,
	.calculate_reserve_sizes = msm7627a_calculate_reserve_sizes,
	.paddr_to_memtype = msm7627a_paddr_to_memtype,
};

static void __init msm7627a_reserve(void)
{
	reserve_info = &msm7627a_reserve_info;
	msm_reserve();
}

static void __init msm_device_i2c_init(void)
{
	msm_gsbi0_qup_i2c_device.dev.platform_data = &msm_gsbi0_qup_i2c_pdata;
	msm_gsbi1_qup_i2c_device.dev.platform_data = &msm_gsbi1_qup_i2c_pdata;
}

#define KEYPAD_ROW 31, 32
#define KEYPAD_COL 36, 37

static unsigned int kp_row_gpios[] = {
	KEYPAD_ROW
};

static unsigned int kp_col_gpios[] = {
	KEYPAD_COL
};

#define KP_INDEX(row, col) ((row)*ARRAY_SIZE(kp_col_gpios) + (col))
#define KEYPAD_LAYOUT				\
        [KP_INDEX(0, 0)] = KEY_VOLUMEUP,	\
        [KP_INDEX(0, 1)] = KEY_VOLUMEDOWN,	\
        [KP_INDEX(1, 0)] = KEY_CAMERA_FOCUS,	\
        [KP_INDEX(1, 1)] = KEY_CAMERA,

static const unsigned short keymap[ARRAY_SIZE(kp_col_gpios) *
					  ARRAY_SIZE(kp_row_gpios)] = {
	KEYPAD_LAYOUT
};

/* SURF keypad platform device information */
static struct gpio_event_matrix_info kp_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keymap,
	.output_gpios	= kp_row_gpios,
	.input_gpios	= kp_col_gpios,
	.noutputs	= ARRAY_SIZE(kp_row_gpios),
	.ninputs	= ARRAY_SIZE(kp_col_gpios),
	.settle_time.tv_nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv_nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS,
};

static struct gpio_event_info *kp_info[] = {
	&kp_matrix_info.info
};

static struct gpio_event_platform_data kp_pdata = {
	.name		= "7x27a_kp",
	.info		= kp_info,
	.info_count	= ARRAY_SIZE(kp_info)
};

static struct platform_device kp_pdev = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &kp_pdata,
	},
};

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 0, /* 0 will disable end key */
};

static struct platform_device hs_pdev = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct platform_device *qrd1_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart1,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&msm_fb_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_batt_device,
#ifdef CONFIG_OV5640
	&msm_camera_sensor_ov5640,
#endif
#ifdef CONFIG_WEBCAM_OV7692_QRD
	&msm_camera_sensor_ov7692,
#endif
	&msm_kgsl_3d0,
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	&msm_bt_power_device,
#endif
	&mipi_dsi_truly_panel_device,
//	&msm_wlan_ar6000_pm_device,
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#ifdef CONFIG_LEDS_TRICOLOR_FLAHSLIGHT
	&msm_device_tricolor_leds,
#endif
};

static struct platform_device *qrd3_devices[] __initdata = {
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_uart1,
	&msm_device_uart_dm1,
	&msm_gsbi0_qup_i2c_device,
	&msm_gsbi1_qup_i2c_device,
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&msm_fb_device,
	&android_pmem_audio_device,
	&msm_device_snd,
	&msm_device_adspdec,
	&msm_batt_device,
#ifdef CONFIG_OV5647
	&msm_camera_sensor_ov5647,
#endif
#ifdef CONFIG_OV5640
	&msm_camera_sensor_ov5640,
#endif
#ifdef CONFIG_WEBCAM_OV7692_QRD
	&msm_camera_sensor_ov7692,
#endif
	&msm_kgsl_3d0,
#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	&msm_bt_power_device,
#endif
	&mipi_dsi_truly_panel_device,
//	&msm_wlan_ar6000_pm_device,
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#ifdef CONFIG_LEDS_TRICOLOR_FLAHSLIGHT
	&msm_device_tricolor_leds,
#endif
#ifdef CONFIG_FB_MSM_LCDC
	&lcdc_truly_panel_device,
#endif
	&msm_device_nand,
};

static struct platform_device msm_proccomm_regulator_dev = {
	.name   = PROCCOMM_REGULATOR_DEV_NAME,
	.id     = -1,
	.dev    = {
		.platform_data = &msm7x27a_proccomm_regulator_data
	}
};

static void __init msm7627a_init_regulators(void)
{
	int rc = platform_device_register(&msm_proccomm_regulator_dev);
	if (rc)
		pr_err("%s: could not register regulator device: %d\n",
				__func__, rc);
}

static int msm7627a_init_gpio(void)
{
	int ret = -ENODEV;

	if (machine_is_msm7627a_qrd1()) {
		pr_info("%s is machine_is_msm7627a_qrd1\n", __func__);
		msm_gpio_table_qrd = msm_gpio_table_qrd_7227a_sku1;
		msm_lcd_gpio_table_qrd = msm_lcd_gpio_table_qrd_7227a_sku1;
		ret = 0;
	} else if(machine_is_msm7627a_qrd3()) {
		pr_info("%s is machine_is_msm7627a_qrd3\n", __func__);
		msm_gpio_table_qrd = msm_gpio_table_qrd_7227a_sku3;
		if (hw_version_is(10, 0))
			msm_lcd_gpio_table_qrd = msm_lcd_gpio_table_qrd_7227a_sku3_1;
		else
			msm_lcd_gpio_table_qrd = msm_lcd_gpio_table_qrd_7227a_sku3;
		ret = 0;
	}
	else {
		return ret;
	}

	/*need modify some static gpio value for some global data*/
#ifdef CONFIG_MSM_CAMERA
#ifdef CONFIG_WEBCAM_OV7692_QRD
	msm_camera_sensor_ov7692_data.sensor_reset = msm_gpio_table_qrd[GPIO_CAM_FRONT_RESET_INDEX];
	msm_camera_sensor_ov7692_data.sensor_pwd = msm_gpio_table_qrd[GPIO_CAM_FRONT_PWDN_INDEX];
#endif
#ifdef CONFIG_OV5640
	msm_camera_sensor_ov5640_data.sensor_reset	= msm_gpio_table_qrd[GPIO_CAM_BACK_RESET_INDEX];
	msm_camera_sensor_ov5640_data.sensor_pwd	= msm_gpio_table_qrd[GPIO_CAM_BACK_SHDN_EN_INDEX];
#endif
#ifdef CONFIG_OV5647
	msm_camera_sensor_ov5647_data.sensor_reset	= msm_gpio_table_qrd[GPIO_CAM_BACK_RESET_INDEX];
	msm_camera_sensor_ov5647_data.sensor_pwd	= msm_gpio_table_qrd[GPIO_CAM_BACK_SHDN_EN_INDEX];
#endif
#endif

#ifdef CONFIG_FB_MSM_MDP303
	mipi_dsi_pdata.vsync_gpio	= msm_lcd_gpio_table_qrd[MDP_303_VSYNC_GPIO_INDEX];
#endif


#ifdef CONFIG_FB_MSM_LCDC
	lcdc_gpio_table[0] = msm_lcd_gpio_table_qrd[GPIO_SPI_MOSI_INDEX];  /* spi_sdi */
	lcdc_gpio_table[1] = msm_lcd_gpio_table_qrd[GPIO_SPI_CLK_INDEX];   /* spi_clk */
	lcdc_gpio_table[2] = msm_lcd_gpio_table_qrd[GPIO_SPI_CS0_N_INDEX]; /* spi_cs  */
	lcdc_gpio_table[3] = msm_lcd_gpio_table_qrd[GPIO_BACKLIGHT_EN_INDEX]; /* backlight  */
	lcdc_gpio_table[4] = msm_lcd_gpio_table_qrd[GPIO_DISPLAY_RESET_INDEX]; /* reset */
#endif

#ifdef CONFIG_TOUCHSCREEN_FT5206
	ft5206_platformdata.irq = MSM_GPIO_TO_INT(msm_gpio_table_qrd[FT5206_IRQ_GPIO_INDEX]);
	ft5206_platformdata.reset = msm_gpio_table_qrd[FT5206_RESET_GPIO_INDEX];

	ft5206_cfg_data[0].gpio_cfg = GPIO_CFG(msm_gpio_table_qrd[FT5206_IRQ_GPIO_INDEX], 0, GPIO_CFG_INPUT,
									GPIO_CFG_NO_PULL, GPIO_CFG_6MA);
	ft5206_cfg_data[1].gpio_cfg = GPIO_CFG(msm_gpio_table_qrd[FT5206_RESET_GPIO_INDEX], 0, GPIO_CFG_INPUT,
									GPIO_CFG_NO_PULL, GPIO_CFG_6MA);
#endif

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C) || \
	defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C_MODULE)
	clearpad3000_cfg_data[0].gpio_cfg = GPIO_CFG(msm_gpio_table_qrd[CLEARPAD3000_ATTEN_GPIO_INDEX], 0, GPIO_CFG_INPUT,
											GPIO_CFG_NO_PULL, GPIO_CFG_6MA);
	clearpad3000_cfg_data[1].gpio_cfg = GPIO_CFG(msm_gpio_table_qrd[CLEARPAD3000_RESET_GPIO_INDEX], 0, GPIO_CFG_OUTPUT,
											GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA);
#endif
	kp_row_gpios[0] = msm_gpio_table_qrd[KYPD_DRV0_INDEX];
	kp_row_gpios[1] = msm_gpio_table_qrd[KYPD_DRV1_INDEX];
	kp_col_gpios[0] = msm_gpio_table_qrd[KEYSENSE_N0_INDEX];
	kp_col_gpios[1] = msm_gpio_table_qrd[KEYSENSE_N1_INDEX];

#ifdef CONFIG_LEDS_TRICOLOR_FLAHSLIGHT
	tricolor_leds_gpio_cfg_data[0].gpio_cfg = GPIO_CFG(msm_gpio_table_qrd[LED_BLUE_INDEX], 0, GPIO_CFG_OUTPUT,
											GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
	tricolor_leds_gpio_cfg_data[1].gpio_cfg = GPIO_CFG(msm_gpio_table_qrd[LED_RED_INDEX], 0, GPIO_CFG_OUTPUT,
											GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
	tricolor_leds_gpio_cfg_data[2].gpio_cfg = GPIO_CFG(msm_gpio_table_qrd[LED_GREEN_INDEX], 0, GPIO_CFG_OUTPUT,
											GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
	tricolor_leds_gpio_cfg_data[3].gpio_cfg = GPIO_CFG(msm_gpio_table_qrd[LED_FLASH_INDEX], 0, GPIO_CFG_OUTPUT,
											GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
#endif
	return ret;
}

#define NV_ITEM_WLAN_MAC_ADDR   4678
extern int msm_read_nv(unsigned int nv_item, void *buf);
unsigned char wlan_mac_addr[6];
static int __init msm7x27a_init_ar6000pm(void)
 {
        msm_read_nv(NV_ITEM_WLAN_MAC_ADDR,wlan_mac_addr);
	msm_wlan_ar6000_pm_device.dev.platform_data = &wlan_power_ctrl;
        return platform_device_register(&msm_wlan_ar6000_pm_device);
 }



static void __init msm_qrd1_init(void)
{

	if(msm7627a_init_gpio())
		pr_err("no proper GPIO for this platform!\n");

	msm7x2x_misc_init();
	msm7627a_init_regulators();
	msm_device_i2c_init();
	lcd_camera_power_init();
	qrd1_camera_gpio_cfg();
#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(msm_gpio_table_qrd[UART1DM_RX_GPIO_INDEX]);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_otg_pdata.swfi_latency = msm7627a_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#endif
	msm_device_gadget_peripheral.dev.platform_data =
		&msm_gadget_pdata;
	if(machine_is_msm7627a_qrd3())
		platform_add_devices(qrd3_devices,
				ARRAY_SIZE(qrd3_devices));

	if(machine_is_msm7627a_qrd1())
		platform_add_devices(qrd1_devices,
				ARRAY_SIZE(qrd1_devices));

/* Ensure ar6000pm device is registered before MMC/SDC */
	msm7x27a_init_ar6000pm();

	msm7627a_init_mmc();

#ifdef CONFIG_USB_EHCI_MSM_72K
	msm7627a_init_host();
#endif
	msm_pm_set_platform_data(msm7627a_pm_data,
				ARRAY_SIZE(msm7627a_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));

	msm_fb_add_devices();

#if defined(CONFIG_BT) && defined(CONFIG_MARIMBA_CORE)
	msm7627a_bt_power_init();
#endif

	msm_camera_vreg_init();
	i2c_register_board_info(MSM_GSBI0_QUP_I2C_BUS_ID, i2c_camera_devices,
			ARRAY_SIZE(i2c_camera_devices));

#ifdef CONFIG_TOUCHSCREEN_FT5206
	if(machine_is_msm7627a_qrd3())
	{
		pr_info("the platform is qrd3\n");
		ft5206_touchpad_setup();
	}
#endif
#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C) || \
	defined(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C_MODULE)
	if(machine_is_msm7627a_qrd1())
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				synaptic_i2c_clearpad3k,
				ARRAY_SIZE(synaptic_i2c_clearpad3k));
#endif

#ifdef CONFIG_INPUT_L3G4200D

#ifdef GYRO_USE_IRQ
	gyro_gpio_setup();
#endif

	pr_info("i2c_register_board_info GYRO\n");
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				gyro_i2c_info,
				ARRAY_SIZE(gyro_i2c_info));
#endif

#ifdef CONFIG_BOSCH_BMA250
	if (machine_is_msm7627a_qrd3()) {
		pr_info("i2c_register_board_info BMA250 ACC\n");
		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
					bma250_i2c_info,
					ARRAY_SIZE(bma250_i2c_info));
	}
#endif

#ifdef CONFIG_INPUT_LIS3DH
	if (machine_is_msm7627a_qrd1()) {
		lis3dh_acc_gpio_setup();
		pr_info("i2c_register_board_info LIS3DH ACC\n");

		i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
					lis3dh_acc_i2c_info,
					ARRAY_SIZE(lis3dh_acc_i2c_info));
	}
#endif


#ifdef CONFIG_SENSORS_AK8975
	pr_info("i2c_register_board_info AKM8975\n");
	akm_platform_data_8975.gpio_DRDY = msm_gpio_table_qrd[GPIO_COMPASS_DRDY_INDEX];
	akm8975_i2c_info[0].irq = gpio_to_irq(msm_gpio_table_qrd[GPIO_COMPASS_DRDY_INDEX]);
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				akm8975_i2c_info,
				ARRAY_SIZE(akm8975_i2c_info));
#endif

#ifdef CONFIG_INPUT_LTR502
	pr_info("i2c_register_board_info LTR502\n");
	ltr502_light_gpio_setup();
	i2c_register_board_info(MSM_GSBI1_QUP_I2C_BUS_ID,
				ltr502_light_i2c_info,
				ARRAY_SIZE(ltr502_light_i2c_info));
#endif
	platform_device_register(&hs_pdev);
	platform_device_register(&kp_pdev);

#ifdef CONFIG_MSM_RPC_VIBRATOR
	msm_init_pmic_vibrator();
#endif
    /*7x25a kgsl initializations*/
	msm7x25a_kgsl_3d0_init();
#ifdef CONFIG_LEDS_TRICOLOR_FLAHSLIGHT
	    /*tricolor leds init*/
	tricolor_leds_gpio_setup();
#endif
}

static void __init qrd7627a_init_early(void)
{
	msm_msm7627a_allocate_memory_regions();
}

#ifdef CONFIG_MSM_AMSS_ENHANCE_DEBUG
static int __init qrd7627a_logbuf_init(void)
{
	nzi_buf_item_type input;
	extern char __log_buf[];

	input.extension.len = 0;
	input.address = (uint32_t)__virt_to_phys(__log_buf);
	input.size = (1 << CONFIG_LOG_BUF_SHIFT);
	strncpy(input.file_name, "dmesg", NZI_ITEM_FILE_NAME_LENGTH);
	input.file_name[NZI_ITEM_FILE_NAME_LENGTH - 1] = 0;
	return send_modem_logaddr(&input);
}
late_initcall(qrd7627a_logbuf_init);
#endif

MACHINE_START(MSM7627A_QRD1, "QRD MSM7627a QRD1")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7627a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm_qrd1_init,
	.timer		= &msm_timer,
	.init_early	= qrd7627a_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
MACHINE_START(MSM7627A_QRD3, "QRD MSM7627a QRD3")
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm_common_io_init,
	.reserve	= msm7627a_reserve,
	.init_irq	= msm_init_irq,
	.init_machine	= msm_qrd1_init,
	.timer		= &msm_timer,
	.init_early	= qrd7627a_init_early,
	.handle_irq	= vic_handle_irq,
MACHINE_END
