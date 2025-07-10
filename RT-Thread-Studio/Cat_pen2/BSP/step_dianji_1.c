/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-24     DELL       the first version
 */
#include <BSP/step_dianji_1.h>
#include <BSP/mqtt.h>
#include <BSP/fan.h>
//#include "up_down.h"
#include <stdlib.h> // 包含 abs 函数的头文件
#include <rtdbg.h>
int paw_clean=0;
int clean_step = 30000;
#define THREAD_PRIORITY   10
#define THREAD_STACK_SIZE 2048
#define THREAD_TIMESLICE  5
extern int clean_state;
static rt_thread_t stepper_thread = RT_NULL;  // 线程句柄
extern rt_bool_t fan_status;
extern char str[128];
extern void *ppclient;
// 初始化步进电机控制引脚
void stepper_init1(void)
{
    rt_pin_mode(MOTORXDIRPIN1, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTORXSTEPPIN1, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTORXEN1, PIN_MODE_OUTPUT);
    rt_pin_write(MOTORXEN1, PIN_LOW);
    rt_pin_mode(MOTORXDIRPIN2, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTORXSTEPPIN2, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTORXEN2, PIN_MODE_OUTPUT);
    rt_pin_write(MOTORXEN2, PIN_LOW);
    LOG_D("stepper_init ok\r\n");
}

// 设置步进电机方向
void stepper_set_direction1(int direction)
{
    if (direction > 0)
    {
        rt_pin_write(MOTORXDIRPIN1, PIN_HIGH); // 正方向
        rt_pin_write(MOTORXDIRPIN2, PIN_HIGH);
    }
    else
    {
        rt_pin_write(MOTORXDIRPIN1, PIN_LOW); // 反方向
        rt_pin_write(MOTORXDIRPIN2, PIN_LOW);
    }
}

// 发送一个步进脉冲
void stepper_step1(void)
{
    rt_pin_write(MOTORXSTEPPIN1, PIN_HIGH);
    rt_pin_write(MOTORXSTEPPIN2, PIN_HIGH);
    rt_hw_us_delay(1);
    rt_pin_write(MOTORXSTEPPIN1, PIN_LOW);
    rt_pin_write(MOTORXSTEPPIN2, PIN_LOW);
    rt_hw_us_delay(1);
}

// 控制步进电机转动指定的步数
void stepper_rotate1(int steps,int speed)
{
    int i;
    int delay = 1000000 / speed; // 计算每步的延迟时间，单位为微秒
    for (i = 0; i < abs(steps); i++)
    {
        while(clean_state == 2);
        stepper_step1();
        rt_thread_mdelay(delay / 1000); // 转换为毫秒并延迟
        clean_step --;
    }


}

void stepper_GO1(void)
{
    // 控制步进电机转动600步，转速为500步/秒（反方向）
    stepper_set_direction1(-1); // 设置为反方向
    stepper_rotate1(clean_step, 5000); // 转动600步，每秒500步
    rt_thread_mdelay(2000); // 等待2秒

    clean_step = 80000;
    stepper_set_direction1(1); // 设置为正方向
    stepper_rotate1(clean_step, 5000);
    paw_clean=1;
}

static void stepper_thread_entry(void *parameter)
{
    stepper_init1();  // 初始化电机

    while (1)
    {
        if(clean_state == 1)
        {
            clean_step = 80000;
            fan_status = RT_TRUE;
            rt_pin_write(MOTORXEN1, PIN_LOW);
            rt_pin_write(MOTORXEN2, PIN_LOW);
            stepper_GO1();
            clean_state = 0;
            fan_status = RT_FALSE;
            fan_OFF();
            HAL_Snprintf(str, 128, "{\"mg996_state\":%d}", 1);
            publish(ppclient, "/k1r45F1HfgR/cat_basin/user/basintx", str);
        }
        else if(clean_state == 2)
        {
            rt_pin_write(MOTORXEN1, PIN_HIGH);
            rt_pin_write(MOTORXEN2, PIN_HIGH);
        }

    }
    rt_thread_mdelay(10);
}

int stepper_thread_init(void)
{
    // 创建线程
    stepper_thread = rt_thread_create("stepper",
                      stepper_thread_entry,
                      RT_NULL,
                      THREAD_STACK_SIZE,
                      THREAD_PRIORITY,
                      THREAD_TIMESLICE);

    // 启动线程
    if (stepper_thread != RT_NULL) {
        rt_thread_startup(stepper_thread);
        LOG_D("Stepper thread created successfully\n");
    } else {
        LOG_E("Failed to create stepper thread\n");
        return -1;
    }

    return 0;
}

INIT_APP_EXPORT(stepper_thread_init);  // 添加自动初始化
