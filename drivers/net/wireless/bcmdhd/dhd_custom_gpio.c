/*
* Customer code to add GPIO control during WLAN start/stop
* Copyright (C) 1999-2011, Broadcom Corporation
* 
*         Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2 (the "GPL"),
* available at http://www.broadcom.com/licenses/GPLv2.php, with the
* following added to such license:
* 
*      As a special exception, the copyright holders of this software give you
* permission to link this software with independent modules, and to copy and
* distribute the resulting executable under terms of your choice, provided that
* you also meet, for each linked independent module, the terms and conditions of
* the license of that module.  An independent module is a module which is not
* derived from this software.  The special exception does not apply to any
* modifications of the software.
* 
*      Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*
* $Id: dhd_custom_gpio.c 280266 2011-08-28 04:18:20Z $
*/

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>

#include <dngl_stats.h>
#include <dhd.h>

#include <wlioctl.h>
#include <wl_iw.h>

#define WL_ERROR(x) printf x
#define WL_TRACE(x)

#ifdef CUSTOMER_HW
extern  void bcm_wlan_power_off(int);
extern  void bcm_wlan_power_on(int);
#endif /* CUSTOMER_HW */
#if defined(CUSTOMER_HW2)
#ifdef CONFIG_WIFI_CONTROL_FUNC
int wifi_set_power(int on, unsigned long msec);
int wifi_get_irq_number(unsigned long *irq_flags_ptr);
int wifi_get_mac_addr(unsigned char *buf);
void *wifi_get_country_code(char *ccode);
#else
int wifi_set_power(int on, unsigned long msec) { return -1; }
int wifi_get_irq_number(unsigned long *irq_flags_ptr) { return -1; }
int wifi_get_mac_addr(unsigned char *buf) { return -1; }
void *wifi_get_country_code(char *ccode) { return NULL; }
#endif /* CONFIG_WIFI_CONTROL_FUNC */
#endif /* CUSTOMER_HW2 */

#if defined(OOB_INTR_ONLY)

#if defined(BCMLXSDMMC)
extern int sdioh_mmc_irq(int irq);
#endif /* (BCMLXSDMMC)  */

#ifdef CUSTOMER_HW3
#include <mach/gpio.h>
#endif

/* Customer specific Host GPIO defintion  */
static int dhd_oob_gpio_num = -1;

module_param(dhd_oob_gpio_num, int, 0644);
MODULE_PARM_DESC(dhd_oob_gpio_num, "DHD oob gpio number");

/* This function will return:
 *  1) return :  Host gpio interrupt number per customer platform
 *  2) irq_flags_ptr : Type of Host interrupt as Level or Edge
 *
 *  NOTE :
 *  Customer should check his platform definitions
 *  and his Host Interrupt spec
 *  to figure out the proper setting for his platform.
 *  Broadcom provides just reference settings as example.
 *
 */
int dhd_customer_oob_irq_map(unsigned long *irq_flags_ptr)
{
	int  host_oob_irq = 0;

#ifdef CUSTOMER_HW2
	host_oob_irq = wifi_get_irq_number(irq_flags_ptr);

#else
#if defined(CUSTOM_OOB_GPIO_NUM)
	if (dhd_oob_gpio_num < 0) {
		dhd_oob_gpio_num = CUSTOM_OOB_GPIO_NUM;
	}
#endif /* CUSTOMER_HW2 */

	if (dhd_oob_gpio_num < 0) {
		WL_ERROR(("%s: ERROR customer specific Host GPIO is NOT defined \n",
			__FUNCTION__));
		return (dhd_oob_gpio_num);
	}

	WL_ERROR(("%s: customer specific Host GPIO number is (%d)\n",
	         __FUNCTION__, dhd_oob_gpio_num));

#if defined CUSTOMER_HW
	host_oob_irq = MSM_GPIO_TO_INT(dhd_oob_gpio_num);
#elif defined CUSTOMER_HW3
	gpio_request(dhd_oob_gpio_num, "oob irq");
	host_oob_irq = gpio_to_irq(dhd_oob_gpio_num);
	gpio_direction_input(dhd_oob_gpio_num);
#endif /* CUSTOMER_HW */
#endif /* CUSTOMER_HW2 */

	return (host_oob_irq);
}
#endif /* defined(OOB_INTR_ONLY) */

/* Customer function to control hw specific wlan gpios */
void
dhd_customer_gpio_wlan_ctrl(int onoff)
{
	switch (onoff) {
		case WLAN_RESET_OFF:
			WL_TRACE(("%s: call customer specific GPIO to insert WLAN RESET\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_off(2);
#endif /* CUSTOMER_HW */
#ifdef CUSTOMER_HW2
			wifi_set_power(0, 0);
#endif
			WL_ERROR(("=========== WLAN placed in RESET ========\n"));
		break;

		case WLAN_RESET_ON:
			WL_TRACE(("%s: callc customer specific GPIO to remove WLAN RESET\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_on(2);
#endif /* CUSTOMER_HW */
#ifdef CUSTOMER_HW2
			wifi_set_power(1, 0);
#endif
			WL_ERROR(("=========== WLAN going back to live  ========\n"));
		break;

		case WLAN_POWER_OFF:
			WL_TRACE(("%s: call customer specific GPIO to turn off WL_REG_ON\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_off(1);
#endif /* CUSTOMER_HW */
		break;

		case WLAN_POWER_ON:
			WL_TRACE(("%s: call customer specific GPIO to turn on WL_REG_ON\n",
				__FUNCTION__));
#ifdef CUSTOMER_HW
			bcm_wlan_power_on(1);
			/* Lets customer power to get stable */
			OSL_DELAY(200);
#endif /* CUSTOMER_HW */
		break;
	}
}

#ifdef GET_CUSTOM_MAC_ENABLE
//Added by zhangjie for bug 249006
//JRD function.
/* FIXME:hard code.
 * filename: /system/etc/wifi/macaddr
 *     this file will be updated by tool [wifi_addr] when boot-up. 
 *     refer to script [init.rc].
 * string format: [00:12:34:AB:cd:ef]
 *     with hex value.
 * */
#define MAC_LEN                6
#define MAC_STR_LEN    17
#define MAC_CHAR_NUM   2

static inline int is_in_range(unsigned char c, 
               unsigned char low, unsigned char high)
{
       if((c > high) || (c < low)){
               return 0;
       }else{
               return 1;
       }
}

//convert a hex string to long integer.
static int hex_strtol(const char *p)
{
       int i;
       int acc = 0;
       unsigned char c;
       
       for(i=0; i<MAC_CHAR_NUM; i++){
               c = (unsigned char)(p[i]);
               if (is_in_range(c, '0', '9'))
                       c -= '0';
               else if (is_in_range(c, 'a', 'f'))
                       c -= ('a' - 10);
               else if (is_in_range(c, 'A', 'F'))
                       c -= ('A' - 10);
               else
                       break;

               acc = (acc << 4) + c;
       }

       return acc;
}

static int str2wa(const char *str, struct ether_addr *wa)
{
       struct ether_addr tmp;
       int l;
       const char *ptr = str;
       int i;
       int result = 0;
       
       for (i = 0; i < MAC_LEN; i++) {
               l = hex_strtol(ptr);
               if((l > 255) || (l < 0)){
                       result = -1;
                       break;  
               }

               tmp.octet[i] = (uint8_t)l;

               if(i == MAC_LEN - 1){
                       break; //done
               }
               
               ptr = strchr(ptr, ':');
               if(ptr == NULL){
                       result = -1;
                       break;
               }
               ptr++;
       }

       if(result == 0){
               memcpy((char *)wa, (char*)(&tmp), MAC_LEN);
       }

       return result;
}


static int jrd_get_mac_addr(struct ether_addr *eaddr)
{
       struct file *fp = NULL;
       char fn[100] = {0};
       char dp[MAC_STR_LEN + 1] = {0};
       int l,len;
       loff_t pos;
       mm_segment_t old_fs;
       //FIXME:hard code.
       strcpy(fn, "/system/etc/wifi/macaddr");
       fp = filp_open(fn, O_RDONLY, 0);
       if(IS_ERR(fp)){
               printk(KERN_INFO "Unable to open '%s'.\n", fn);
               goto err_open;
       }

       l = fp->f_path.dentry->d_inode->i_size;
       if(l != MAC_STR_LEN){
               printk(KERN_INFO "Invalid macaddr file '%s' %d\n", fn,l);
               goto err_format;
       }

       pos = 0;
       old_fs=get_fs();
       set_fs(KERNEL_DS);
       len=vfs_read(fp, dp, l, &pos);
       set_fs(old_fs); 
       if(len!= l)
       {
               printk(KERN_INFO "Failed to read '%s'  %d  %s.\n", fn,len,dp );
               goto err_format;
       }

       dp[MAC_STR_LEN] = '\0';
       str2wa(dp, eaddr);
       
       filp_close(fp, NULL);
       return 0;

err_format:
       filp_close(fp, NULL);
err_open:
       return -1;
}
//Added end

/* Function to get custom MAC address */
int
dhd_custom_get_mac_address(unsigned char *buf)
{
	int ret = 0;

	WL_TRACE(("%s Enter\n", __FUNCTION__));
	if (!buf)
		return -EINVAL;

	/* Customer access to MAC address stored outside of DHD driver */
	

//Added by zhangjie for bug 249006
       //JRD mac
       {
               //Friday Apr 8th, 2012.
               struct ether_addr ea_jrd = {{0x00, 0x20, 0x10, 0xDE, 0xC1, 0x0F}};
               struct ether_addr ea_example = {{0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}};
               jrd_get_mac_addr(&ea_jrd);
                if((ea_jrd.octet[0]==0x00 && ea_jrd.octet[1]==0x00 && ea_jrd.octet[2]==0x00 && ea_jrd.octet[3]==0x00 && ea_jrd.octet[4]==0x00 && ea_jrd.octet[5]==0x00) || (ea_jrd.octet[0]==0xFF && ea_jrd.octet[1]==0xFF && ea_jrd.octet[2]==0xFF && ea_jrd.octet[3]==0xFF && ea_jrd.octet[4]==0xFF && ea_jrd.octet[5]==0xFF) ){
               bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
               }else{
               bcopy((char *)&ea_jrd, buf, sizeof(struct ether_addr));
               }       
       }
//Added end

       /* Customer access to MAC address stored outside of DHD driver */
//Commented by zhangjie for bug 249006
//#if defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
//     ret = wifi_get_mac_addr(buf);
//#endif
//Commented end

#ifdef EXAMPLE_GET_MAC
	/* EXAMPLE code */
	{
		struct ether_addr ea_example = {{0x00, 0x11, 0x22, 0x33, 0x44, 0xFF}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
	}
#endif /* EXAMPLE_GET_MAC */

	return ret;
}
#endif /* GET_CUSTOM_MAC_ENABLE */

/* Customized Locale table : OPTIONAL feature */
const struct cntry_locales_custom translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
#ifdef EXAMPLE_TABLE
	{"",   "XY", 4},  /* Universal if Country code is unknown or empty */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries to : EU regrev 05 */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3}, /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3},
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
#endif /* EXMAPLE_TABLE */
};


/* Customized Locale convertor
*  input : ISO 3166-1 country abbreviation
*  output: customized cspec
*/
void get_customized_country_code(char *country_iso_code, wl_country_t *cspec)
{
#if defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))

	struct cntry_locales_custom *cloc_ptr;

	if (!cspec)
		return;

	cloc_ptr = wifi_get_country_code(country_iso_code);
	if (cloc_ptr) {
		strlcpy(cspec->ccode, cloc_ptr->custom_locale, WLC_CNTRY_BUF_SZ);
		cspec->rev = cloc_ptr->custom_locale_rev;
	}
	return;
#else
	int size, i;

	size = ARRAYSIZE(translate_custom_table);

	if (cspec == 0)
		 return;

	if (size == 0)
		 return;

	for (i = 0; i < size; i++) {
		if (strcmp(country_iso_code, translate_custom_table[i].iso_abbrev) == 0) {
			memcpy(cspec->ccode,
				translate_custom_table[i].custom_locale, WLC_CNTRY_BUF_SZ);
			cspec->rev = translate_custom_table[i].custom_locale_rev;
			return;
		}
	}
#ifdef EXAMPLE_TABLE
	/* if no country code matched return first universal code from translate_custom_table */
	memcpy(cspec->ccode, translate_custom_table[0].custom_locale, WLC_CNTRY_BUF_SZ);
	cspec->rev = translate_custom_table[0].custom_locale_rev;
#endif /* EXMAPLE_TABLE */
	return;
#endif /* defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)) */
}
