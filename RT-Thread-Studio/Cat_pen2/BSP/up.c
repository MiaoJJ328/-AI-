/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-04     DELL       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
int up_over=0;
int down_over=0;

struct rt_device_pwm * dev_pwm = RT_NULL;   //定义设备

int servo_init(void)
{
    dev_pwm = ( struct rt_device_pwm *)rt_device_find("pwm4");  //查找设备
    if(dev_pwm == RT_NULL)
    {
        LOG_E("find device ERROR...\r\n");
        return 0;
    }
    //初始状态为静止
    rt_pwm_set(dev_pwm, 2, 20000000, 0);
    rt_pwm_set(dev_pwm, 3, 20000000, 0);
    if(rt_pwm_enable(dev_pwm, 2) == RT_EOK && rt_pwm_enable(dev_pwm, 3) == RT_EOK)
    {
        LOG_D("pwm Init is ok\r\n");
        return 1;
    }
    return 0;

}

//舵机上升
void up()
{
    rt_pwm_set(dev_pwm, 3, 40001000, 2000000);
    rt_pwm_set(dev_pwm, 2, 40000000, 1000000);
    rt_thread_delay(RT_TICK_PER_SECOND * 10);
    rt_pwm_set(dev_pwm, 3, 40000000, 0);
    rt_pwm_set(dev_pwm, 2, 40000000, 0);
    up_over=1;
}

//舵机下降
void down()
{
    rt_pwm_set(dev_pwm, 3, 40000000, 1000000);
    rt_pwm_set(dev_pwm, 2, 40000000, 2000000);
    rt_thread_delay(RT_TICK_PER_SECOND * 10);
    rt_pwm_set(dev_pwm, 3, 40000000, 0);
    rt_pwm_set(dev_pwm, 2, 40000000, 0);
    down_over=1;
}
