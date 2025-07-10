/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-24     DELL       the first version
 */
#ifndef BSP_STEP_DIANJI_1_H_
#define BSP_STEP_DIANJI_1_H_

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"
// 定义步进电机控制引脚
#define MOTORXDIRPIN1 GET_PIN(A, 4) // X轴方向控制引脚
#define MOTORXSTEPPIN1 GET_PIN(A, 8) // X轴步进控制引脚
#define MOTORXEN1 GET_PIN(B, 8)
// 定义步进电机控制引脚
#define MOTORXDIRPIN2 GET_PIN(B, 10)
#define MOTORXSTEPPIN2 GET_PIN(B, 11)
#define MOTORXEN2 GET_PIN(B, 12)
// 初始化步进电机控制引脚
void stepper_init1(void);

// 设置步进电机方向
void stepper_set_direction1(int direction);

// 发送一个步进脉冲
void stepper_step1(void);
void stepper_STOP1(void);
// 控制步进电机转动指定的步数
void stepper_rotate1(int steps,int speed);
void stepper_GO1(void);
int stepper_thread_init(void);
#endif /* BSP_STEP_DIANJI_1_H_ */
