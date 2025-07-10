/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-11-21     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <mqtt.h>
#include <mqtt_api.h>
#include "Rtc.h"
#include "hx711.h"
#include "play.h"
#include "mlx90614.h"
#include "AT24C128.h"

#define feedtime_Address 0x01
#define max_weight 0x02
#define max_water_weight 0x03
extern char subscribe_topic[];
extern rt_uint32_t feedtime;

rt_thread_t aht10_thread = RT_NULL; //温度传感器线程
rt_thread_t publish_thread = RT_NULL;   //mqtt线程
rt_thread_t get_weight_thread = RT_NULL;
void thread_create(void)
{
    get_weight_thread=rt_thread_create("getWeight", get_weight_entry, RT_NULL,1024, 20, 20);//获取重量线程
    if(get_weight_thread==RT_NULL)
    {
        rt_kprintf("getWeight failed...\r\n");
    }
    else{
        rt_kprintf("getWeight succeed\r\n");
        rt_thread_startup(get_weight_thread);
    }
    aht10_thread = rt_thread_create("aht10", myaht10_entry, RT_NULL, 1024, 20, 20); //温湿度传感器
    if(aht10_thread == RT_NULL)
    {
        rt_kprintf("aht10 failed...\r\n");
    }
    else {
    rt_kprintf("aht10 succeed\r\n");
        rt_thread_startup(aht10_thread);
    }

    publish_thread = rt_thread_create("pub", publish_entry, RT_NULL, 2048, 19, 100);    //发布线程
    if(publish_thread == RT_NULL)
    {
        rt_kprintf("pub failed...\r\n");
    }
    else
    {
        rt_kprintf("pub succeed\r\n");
        rt_thread_startup(publish_thread);
    }


}
extern void *ppclient;

int main(void)
{

    I2C_Init();
    Bump_Init();
    servo_init();
    mqtt_init(subscribe_topic);
    thread_create();
    rtc_sample_init();
    servo_timer_init();
    Servo3_thread_Start();
    mlx_thread_init();
    handle_water_start();
    handle_food_start();
    periodic_task_start();

//    AT24CXX_WriteByte(feedtime_Address, 0x05);
//
//    feedtime = AT24CXX_ReadByte(feedtime_Address);
//    AT24CXX_ReadByte(max_weight);
//   AT24CXX_ReadByte(max_water_weight);

    publish(ppclient, "/%s/%s/user/nesttx", "{\"params\":\"hello world!\"}");
      while(1)
      {
          IOT_MQTT_Yield(ppclient, 200);
          //rt_kprintf("feedtime:%d\n", feedtime);
          rt_thread_mdelay(100);

      }

    return RT_EOK;
}
