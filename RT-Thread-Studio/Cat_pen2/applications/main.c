/*
 * Copyright (c) 2006-2025, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-24     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <BSP/step_dianji_1.h>
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "mqtt.h"
#include "serial1.h"
#include "up.h"
extern void *ppclient;
extern char subscribe_topic[];


int main(void)
{
    //初始化舵机、步进电机
    if(servo_init() != 1)
    {
        LOG_E("pwm int failed");
        return 0;
    }

    stepper_init1();

    //初始化mqtt协议
    mqtt_init(subscribe_topic);
    //serial_init();
    publish(ppclient,"/k1r45F1HfgR/Cat_app/user/apptx","{\"params\":\"hello world!\"}");
    while(1){
        IOT_MQTT_Yield(ppclient, 200);
        rt_thread_mdelay(100);
    }
    return RT_EOK;
}
