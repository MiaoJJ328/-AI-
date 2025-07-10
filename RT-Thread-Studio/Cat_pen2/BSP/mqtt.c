/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-24     DELL       the first version
 */
#include "rtthread.h"
#include "dev_sign_api.h"
#include "mqtt_api.h"
#include "CJSON.h"
#include <string.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include "mqtt.h"
void *ppclient = RT_NULL;
char subscribe_topic[] = "/k1r45F1HfgR/cat_basin/user/basinrx";
#include "up.h"
#include "step_dianji_1.h"
//#include "step_dianji_2.h"
extern paw_clean;
extern up_over;
extern down_over;
char str[128] = {0};
uint8_t clean_state = 0;    //清理状态机,0为清理完成，1为正在清理，2为外界异常干扰
void message_analysis(char *str)
{
    cJSON * root = RT_NULL;
    cJSON * name = RT_NULL;
    if(strstr(str, "cat_is_here") != RT_NULL)
    {

        root = cJSON_Parse(str);    //将字符串转化为json格式的对象
        name = cJSON_GetObjectItemCaseSensitive(root, "cat_is_here");

       if(name -> valueint == 1)
       {
           rt_kprintf("111111111111");
           //识别到猫，步进电机停止清理动作
           clean_state = 2; //清理状态机置2
       }

       //外界异常排除，继续清理操作
       else if(name -> valueint == 0)
       {
           //猫离开，步进电机继续清理动作
            clean_state = 1; //清理状态机置1
       }

    }

    //一体化闭环清理
    if(strstr(str, "basin_clean") != RT_NULL)
    {
        root = cJSON_Parse(str);    //将字符串转化为json格式的对象
        name = cJSON_GetObjectItemCaseSensitive(root, "basin_clean");

        if(name -> valueint == 1)
       {
            //执行步进电机清理动作
            clean_state = 1;
            //给车发送动作完成
            HAL_Snprintf(str, 128, "{\"mg996_state\":%d}", 1);
            publish(ppclient, "/k1r45F1HfgR/cat_basin/user/basintx", str);
       }

    }

    //当有清洁不干净的时来进行手动清洁，只让步进电机运动
    if(strstr(str, "clean_feces") != RT_NULL)
    {
        root = cJSON_Parse(str);    //将字符串转化为json格式的对象
           name = cJSON_GetObjectItemCaseSensitive(root, "clean_feces");

           if(name -> valueint == 1)
          {
               //执行步进电机清理动作
               clean_state = 1;
          }
    }

    cJSON_Delete(root); //清理解析的JSON对象

}
//消息到达函数
void message_arrive(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    iotx_mqtt_topic_info_t     *topic_info = (iotx_mqtt_topic_info_pt) msg->msg;

    switch (msg->event_type) {
        case IOTX_MQTT_EVENT_PUBLISH_RECEIVED:
            /* 打印话题名称和话题消息内容*/
            EXAMPLE_TRACE("Message Arrived:");
            EXAMPLE_TRACE("Topic  : %.*s", topic_info->topic_len, topic_info->ptopic);
            EXAMPLE_TRACE("Payload: %.*s", topic_info->payload_len, topic_info->payload);
            EXAMPLE_TRACE("\n");
            message_analysis(topic_info->payload);
            break;
        default:
            break;
    }
}

//消息订阅函数
int subscribe(void *handle, char *Topic)
{
    int res = 0;
    const char *fmt = Topic;
    char *topic = NULL;
    int topic_len = 0;

    topic_len = strlen(fmt) + strlen(DEMO_PRODUCT_KEY) + strlen(DEMO_DEVICE_NAME) + 1;
    topic = HAL_Malloc(topic_len);
    if (topic == NULL) {
        EXAMPLE_TRACE("memory not enough");
        return -1;
    }

    memset(topic, 0, topic_len);

    HAL_Snprintf(topic, topic_len, fmt, DEMO_PRODUCT_KEY, DEMO_DEVICE_NAME);    //填充topic

    res = IOT_MQTT_Subscribe(handle, topic, IOTX_MQTT_QOS0, message_arrive, NULL);
    if (res < 0) {
        EXAMPLE_TRACE("subscribe failed");
        HAL_Free(topic);
        return -1;
    }

    HAL_Free(topic);
    return 0;
}

//消息发布函数
int publish(void *handle, char *Topic, char* message)
{
   int             res = 0;
   const char     *fmt = Topic;
   char           *topic = NULL;
   int             topic_len = 0;
   char           *payload = message;

   topic_len = strlen(fmt) + strlen(DEMO_PRODUCT_KEY) + strlen(DEMO_DEVICE_NAME) + 1;
   topic = HAL_Malloc(topic_len);
   if (topic == NULL) {
       EXAMPLE_TRACE("memory not enough");
       return -1;
   }
   memset(topic, 0, topic_len);
   HAL_Snprintf(topic, topic_len, fmt, DEMO_PRODUCT_KEY, DEMO_DEVICE_NAME); //填充topic

   res = IOT_MQTT_Publish_Simple(0, topic, IOTX_MQTT_QOS0, payload, strlen(payload));   //发布消息
   if (res < 0) {
       EXAMPLE_TRACE("publish failed, res = %d", res);
       HAL_Free(topic);
       return -1;
   }

   HAL_Free(topic);
       return 0;
}


void event_handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    EXAMPLE_TRACE("msg->event_type : %d", msg->event_type);
}


int mqtt_init(char *subscribe_Topic)
{
    void                   *pclient = NULL;
    int                     res = 0;
    int                     loop_cnt = 0;
    iotx_mqtt_param_t       mqtt_params;

    HAL_GetProductKey(DEMO_PRODUCT_KEY);
    HAL_GetDeviceName(DEMO_DEVICE_NAME);
    HAL_GetDeviceSecret(DEMO_DEVICE_SECRET);

    EXAMPLE_TRACE("mqtt example");

    memset(&mqtt_params, 0x0, sizeof(mqtt_params)); //初始化mqtt连接参数为0

    mqtt_params.handle_event.h_fp = event_handle;   //设置参数回调函数

    pclient = IOT_MQTT_Construct(&mqtt_params);     //与阿里云建立连接
    if (NULL == pclient) {
        EXAMPLE_TRACE("MQTT construct failed");
        return -1;
    }

    res = subscribe(pclient, subscribe_Topic);   //订阅话题
    if (res < 0) {
        IOT_MQTT_Destroy(&pclient);
        return -1;
    }

    ppclient = pclient;
}

//发布数据线程
void publish_entry(void* parameter)
{
    char str[128] = {0};
    while(1)
    {
//        HAL_Snprintf(str, 128, "{\"paw_clean\":%d}",paw_clean);
//        HAL_Snprintf(str, 128, "{\"up_over\":%d}",up_over);
//        HAL_Snprintf(str, 128, "{\"down_over\":%d}",down_over);
        HAL_Snprintf(str, 128, "{\"down_over\":%d,\"up_over\":%d,\"paw_clean\":%d}", down_over, up_over, paw_clean);
        publish(ppclient, "/k1r45F1HfgR/cat_basin/user/basintx", str);
        rt_thread_mdelay(2000);   //每2000ms发布一次数据
    }
}

