/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-04-17     wangl       the first version
 */

#include <rtthread.h>
#include <board.h>
#define DBG_TAG "MG996"

#define PWM_DEV_NAME_BEFORE_AND_AFTER        "pwm4" /* 前后运动舵机对应的PWM设备名称 */
#define PWM_DEV_BEFORE_AND_AFTER_CHANNEL_3       3   /* PWM通 道 PB8 小盒子*/
#define PWM_DEV_BEFORE_AND_AFTER_CHANNEL_4       4   /* PWM通 道 PB9*/

#define PWM_DEV_NAME_UP_AND_DOWM              "pwm2" /* 上下运动舵机对应的PWM设备名称 */
#define PWM_DEV_UP_AND_DOWM_CHANNEL_3            3   /* PWM通 道 PB10 RIGHT*/
#define PWM_DEV_UP_AND_DOWM_CHANNEL_4            4   /* PWM通 道 PB11*/

struct rt_device_pwm *pwm_dev_before_and_after; /* PWM设 备 句 柄 */
struct rt_device_pwm *pwm_dev_up_and_down; /* PWM设 备 句 柄 */
rt_uint32_t positive_pulse = 1400000;
rt_uint32_t opposite_pulse = 1580000;
rt_uint32_t stop_pulse = 1500000;
rt_uint32_t period = 20000000; /* 周 期 为20ms， 单 位 为 纳 秒ns */
extern uint8_t mg996_state;
extern void *ppclient;
char str[128];
/*
 *@brief 舵机控制线程入口函数
 *@param 无
 *@retval 无
 */
static void mg996_thread_entry(void *parameter)
{

    /*
     * PWM脉 冲 宽 度 值， 单 位 为 纳秒ns
     * 1.1ms、1.2ms、1.3ms、1.4ms对应正转，速度依次递减
     * 1.5ms停止
     * 1.6ms、1.7ms、1.8ms、1.9ms对应反转，速度依次递增
     */

    /* 查 找 设 备 */
    pwm_dev_before_and_after = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_BEFORE_AND_AFTER);
    pwm_dev_up_and_down = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME_UP_AND_DOWM);

    if (pwm_dev_before_and_after == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME_BEFORE_AND_AFTER);
        return RT_ERROR;
    }

    if (pwm_dev_up_and_down == RT_NULL)
    {
        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME_UP_AND_DOWM);
        return RT_ERROR;
    }

    rt_kprintf("pwm sample run !");

     /* 使 能 设 备 */
     rt_pwm_enable(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_3);
     rt_pwm_enable(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_4);

     rt_pwm_enable(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_3);
     rt_pwm_enable(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_4);

    /* 设 置PWM周 期 和 脉 冲 宽 度 默 认 值 */
    rt_pwm_set(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_3, period, 500000);
    rt_pwm_set(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_4, period, stop_pulse);

    rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_3, period, stop_pulse);
    rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_4, period, stop_pulse);

    while (1)
    {
        if(mg996_state == 1)
        {
            // 上升
            rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_3, period, positive_pulse);
            rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_4, period, opposite_pulse);
            rt_thread_mdelay(32000);
            rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_3, period, stop_pulse);
            rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_4, period, stop_pulse);

            //前进
            rt_pwm_set(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_4, period, positive_pulse);
            rt_thread_mdelay(8000);
            rt_pwm_set(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_4, period, stop_pulse);
            mg996_state = 0;
            HAL_Snprintf(str, 128, "{\"paw_clean\":%d}", 1);
            publish(ppclient, "/k1r45F1HfgR/cat_basin/user/basintx", str);
        }
        else if(mg996_state == 2)   //车停在马桶正前方
        {
            //打开盒子 + 关闭盒子
              rt_pwm_set(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_3, period, 1000000);
              rt_thread_mdelay(3000);
              rt_pwm_set(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_3, period, 500000);
              mg996_state = 0;
              HAL_Snprintf(str, 128, "{\"box_close\":%d}", 1); //告诉车回到原位置
              publish(ppclient, "/k1r45F1HfgR/cat_basin/user/basintx", "{\"box_close\":1}");
        }
        else if(mg996_state == 3)   //车导航回原位置
        {
            servo_return_init();
            mg996_state = 0;
        }
        rt_thread_mdelay(10);
    }
}

/*
 * 开盒子、回退、下降
 */
void servo_return_init()
{
    //后退
    rt_pwm_set(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_4, period, opposite_pulse);
    rt_thread_mdelay(5500);
    rt_pwm_set(pwm_dev_before_and_after, PWM_DEV_BEFORE_AND_AFTER_CHANNEL_4, period, stop_pulse);

    //下降
    rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_4, period, 1300000);
    rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_3, period, 1600000);
    rt_thread_mdelay(10000);
    rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_4, period, stop_pulse);
    rt_pwm_set(pwm_dev_up_and_down, PWM_DEV_UP_AND_DOWM_CHANNEL_3, period, stop_pulse);
}


/*
 *@brief 舵机控制线程
 *@param 无
 *@retval 线程申请是否成功
 */
static int mg996_init(void)
{
    static rt_thread_t mg996_thread = RT_NULL;

    rt_err_t ret = RT_EOK;

   /* 创建 serial 线程 */
    mg996_thread = rt_thread_create("t_pwmled",mg996_thread_entry, RT_NULL, 2048, 24, 10);
   /* 创建成功则启动线程 */
   if (mg996_thread != RT_NULL)
   {
       rt_thread_startup(mg996_thread);
   }
   else
   {
       ret = RT_ERROR;
   }

   return ret;

}

INIT_APP_EXPORT(mg996_init);
