/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-04-19     wangl        the first version
 * 2025-04-20     AI Assistant add thread encapsulation
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include <fan.h>
#include <board.h>

#define FAN_PIN_A GET_PIN(C, 2)
#define FAN_PIN_B GET_PIN(D, 12)
#define FAN_THREAD_STACK_SIZE 512
#define FAN_THREAD_PRIORITY   25
#define FAN_THREAD_TIMESLICE  5

/* 定义线程控制块 */
static struct rt_thread fan_thread;
static rt_uint8_t fan_thread_stack[FAN_THREAD_STACK_SIZE];

/* 风扇状态标志 */
rt_bool_t fan_status = RT_FALSE;

/* 线程入口函数 */
static void fan_thread_entry(void *parameter)
{
    while (1)
    {
        if (fan_status == RT_TRUE)
        {
            fan_ON();
        }
        else
        {
            fan_OFF();
        }

        /* 挂起线程500ms */
        rt_thread_mdelay(10);
    }
}

void fan_init(void)
{
    rt_pin_mode(FAN_PIN_A, PIN_MODE_OUTPUT);
    rt_pin_mode(FAN_PIN_B, PIN_MODE_OUTPUT);
    fan_OFF();

    /* 初始化并启动风扇控制线程 */
    rt_thread_init(&fan_thread,
                  "fan_ctrl",
                  fan_thread_entry,
                  RT_NULL,
                  &fan_thread_stack[0],
                  FAN_THREAD_STACK_SIZE,
                  FAN_THREAD_PRIORITY,
                  FAN_THREAD_TIMESLICE);

    rt_thread_startup(&fan_thread);
}

void fan_ON(void)
{
    rt_pin_write(FAN_PIN_A, PIN_HIGH);
    rt_pin_write(FAN_PIN_B, PIN_LOW);
    LOG_D("Fan turned ON");
}

void fan_OFF(void)
{
    rt_pin_write(FAN_PIN_A, PIN_LOW);
    rt_pin_write(FAN_PIN_B, PIN_LOW);
    LOG_D("Fan turned OFF");
}

/* 查询风扇状态接口 */
rt_bool_t get_fan_status(void)
{
    return fan_status;
}

/* 导出到MSH命令（可选） */
static void fan_ctrl(int argc, char **argv)
{
    if (argc > 1)
    {
        if (!rt_strcmp(argv[1], "on"))
        {
            fan_ON();
        }
        else if (!rt_strcmp(argv[1], "off"))
        {
            fan_OFF();
        }
        else
        {
            rt_kprintf("Usage: fan_ctrl [on|off]\n");
        }
    }
    else
    {
        rt_kprintf("Fan status: %s\n", fan_status ? "ON" : "OFF");
    }
}
INIT_APP_EXPORT(fan_init);
MSH_CMD_EXPORT(fan_ctrl, fan control command);
