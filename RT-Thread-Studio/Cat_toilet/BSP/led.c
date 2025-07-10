/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-24     DELL       the first version
 */
#include "led.h"

void control_led(int command)
{
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    if(command==1)
    {

        rt_pin_write(LED0_PIN, PIN_HIGH);
    }
    else if(command==0)
    {

            rt_pin_write(LED0_PIN, PIN_LOW);


        //rt_pin_write(LED0_PIN, PIN_LOW);
    }
}
