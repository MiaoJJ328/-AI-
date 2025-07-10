/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-24     DELL       the first version
 */
#ifndef BSP_BLOOD_H_
#define BSP_BLOOD_H_

#include "mx30102.h"
#include "algorithm.h"
#include "math.h"
void blood_data_translate(void);
void blood_data_update(void);
void blood_Loop(void);
void health_entry(void);
void health_thread_start(void);
#endif /* BSP_BLOOD_H_ */
