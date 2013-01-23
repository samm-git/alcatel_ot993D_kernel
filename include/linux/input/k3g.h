/*
 *  Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *  Copyright (C) 2010, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */
 /*
 * STMicroelectronics K3G gyro sensor header file
 */

#ifndef __K3G_H__
#define __K3G_H__

#ifdef __KERNEL__
struct k3g_platform_data {
	u8 fs_range;
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;
	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};

#endif /* __KERNEL__ */

#endif  /* __K3G_H__ */
