/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-24     DELL       the first version
 */
#ifndef BSP_BENG_H_
#define BSP_BENG_H_

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"
#define LED0_PIN    GET_PIN(C,7)
extern void control_led(int command);
#endif /* BSP_BENG_H_ */
