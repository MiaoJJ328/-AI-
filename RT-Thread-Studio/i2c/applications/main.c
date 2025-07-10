/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-22     RT-Thread    first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "mx30102.h"
#include "blood.h"
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "mqtt.h"
#include "serial.h"

extern  struct rt_i2c_bus_device *i2c_bus;
extern void *ppclient;
extern char subscribe_topic[];
rt_thread_t a_thread = RT_NULL;
void a_entry(void* parameter)
{
    while(1)
    {
        rt_thread_mdelay(200);
    }
}

int main(void)
{
        mqtt_init(subscribe_topic);
        serial_init();
        health_thread_start();
        publish(ppclient,"/k1r45F1HfgR/heart_health/user/heart_healthtx","{\"params\":\"hello world!\"}");
        while(1){
            IOT_MQTT_Yield(ppclient, 200);
            rt_thread_mdelay(100);
        }

    return 0;
}

