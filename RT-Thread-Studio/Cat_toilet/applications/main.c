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
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <ap3216c.h>
#include "mqtt.h"
#include "beng.h"
#include "led.h"
#include "serial1.h"
extern rt_uint8_t light;
extern void *ppclient;
extern char subscribe_topic[];
extern uint8_t lamp_flag;
extern cleanning;
int main(void)
{
    ap3216c_device_t devc = ap3216c_init("i2c3");
    //char subscribe_topic[] = "/k1r4aWQApD8/stm32/user/stm32rx";
    pwm_init();
    mqtt_init(subscribe_topic);
    //serial_init();
    publish(ppclient,"/k1r45F1HfgR/Cat_toilet/user/toilettx","{\"params\":\"hello world!\"}");
    while(1){
        IOT_MQTT_Yield(ppclient, 200);
        rt_thread_mdelay(100);

       light = (int) ap3216c_read_ambient_light(devc);
       if(cleanning)
       {
           if(light<5)
          {
              control_led(1);
          }
          else
          {
              control_led(0);
          }
       }
    }

    return RT_EOK;
}
