/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-24     DELL       the first version
 */
#include <rtthread.h>
#include "serial1.h"
#include <rtdbg.h>
#include "mqtt.h"
#define THREAD_STACK_SIZE 2048+512  // 定义线程栈大小
static struct rt_thread publish_thread;  // 定义线程控制块
static rt_uint8_t publish_stack[THREAD_STACK_SIZE]; // 定义线程栈空间
void serial_init(void)
{
    int ret;
    ret=rt_thread_init(&publish_thread,
            "publish",  // 线程名称
            publish_entry, // 线程入口函数
            RT_NULL, // 线程入口函数参数
            &publish_stack[0], // 线程栈起始地址
            sizeof(publish_stack), // 线程栈大小
            10, // 线程优先级
            20); // 线程时间片
    if(ret==RT_EOK)
    {
        rt_thread_startup(&publish_thread);
    }
    else{
        LOG_D("can't init publish thread!");
        return -1;
    }
    return RT_EOK;
}

