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

#define PWM_DEV_NAME "pwm4"//PD14
#define PWM_DEV_CHANNEL 3

#define SERVO_MIN_PULSE_WIDTH 500000  // 0.5ms, 逆时针最大位置 (0度)
#define SERVO_MAX_PULSE_WIDTH 2500000  // 2.5ms, 顺时针最大位置 (180度)
#define SERVO_MIDDLE_PULSE_WIDTH 1500000  // 1.5ms, 中间位置 (90度)

extern struct rt_device_pwm *pwm_dev;
extern rt_uint32_t period;
extern rt_uint32_t pulse;
extern rt_err_t ret;

extern void pwm_init(void);
extern void servo_rotate_to(struct rt_device_pwm *pwm_dev, rt_uint32_t channel, rt_uint32_t pulse_width);
#endif /* BSP_BENG_H_ */
