/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-24     DELL       the first version
 */
#include "beng.h"
struct rt_device_pwm *pwm_dev;  // pwm设备句柄
rt_uint32_t period = 20000000;  // 单位us，20ms
//rt_uint32_t pulse = 1000000;
rt_err_t ret = RT_EOK; // 返回值校验

void pwm_init(void)
{
    /* 查找pwm设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
        return;
    }
    /* 设置PWM周期 */
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, SERVO_MIDDLE_PULSE_WIDTH);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);
}

void servo_rotate_to(struct rt_device_pwm *pwm_dev, rt_uint32_t channel, rt_uint32_t pulse_width)
{
    /* 设置PWM脉冲宽度 */
    rt_pwm_set(pwm_dev, channel, period, pulse_width);
}
