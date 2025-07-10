/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-04-17     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "BSP/mqtt.h"
extern void *ppclient;
extern char subscribe_topic[];
int main(void)
{
    LOG_D("Hello Cat_Box!");

    //初始化mqtt协议
    mqtt_init(subscribe_topic);

    while(1){
        IOT_MQTT_Yield(ppclient, 200);
        rt_thread_mdelay(100);
    }
    return RT_EOK;
}
