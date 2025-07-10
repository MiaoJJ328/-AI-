/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-11-21     RealThread   first version
 */

#include <rtthread.h>
#include <board.h>
#include <drv_common.h>

RT_WEAK void rt_hw_board_init()
{
    extern void hw_board_init(char *clock_src, int32_t clock_src_freq, int32_t clock_target_freq);

    /* Heap initialization */
#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *) HEAP_BEGIN, (void *) HEAP_END);
#endif

    hw_board_init(BSP_CLOCK_SOURCE, BSP_CLOCK_SOURCE_FREQ_MHZ, BSP_CLOCK_SYSTEM_FREQ_MHZ);

    /* Set the shell console output device */
#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

}

/**
* @brief TIM_PWM MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_pwm: TIM_PWM handle pointer
* @retval None
*/
//void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
//{
//  if(htim_pwm->Instance==TIM4)
//  {
//  /* USER CODE BEGIN TIM4_MspInit 0 */
//
//  /* USER CODE END TIM4_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_TIM4_CLK_ENABLE();
//  /* USER CODE BEGIN TIM4_MspInit 1 */
//
//  /* USER CODE END TIM4_MspInit 1 */
//  }
//
//}
//
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(htim->Instance==TIM4)
//  {
//  /* USER CODE BEGIN TIM4_MspPostInit 0 */
//
//  /* USER CODE END TIM4_MspPostInit 0 */
//
//    __HAL_RCC_GPIOD_CLK_ENABLE();
//    /**TIM4 GPIO Configuration
//    PD12     ------> TIM4_CH1
//    PD13     ------> TIM4_CH2
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN TIM4_MspPostInit 1 */
//
//  /* USER CODE END TIM4_MspPostInit 1 */
//  }
//
//}
///**
//* @brief TIM_PWM MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param htim_pwm: TIM_PWM handle pointer
//* @retval None
//*/
//void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
//{
//  if(htim_pwm->Instance==TIM4)
//  {
//  /* USER CODE BEGIN TIM4_MspDeInit 0 */
//
//  /* USER CODE END TIM4_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM4_CLK_DISABLE();
//  /* USER CODE BEGIN TIM4_MspDeInit 1 */
//
//  /* USER CODE END TIM4_MspDeInit 1 */
//  }
//
//}
//void HAL_QSPI_MspInit(QSPI_HandleTypeDef* hqspi)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(hqspi->Instance==QUADSPI)
//  {
//  /* USER CODE BEGIN QUADSPI_MspInit 0 */
//
//  /* USER CODE END QUADSPI_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_QSPI_CLK_ENABLE();
//
//    __HAL_RCC_GPIOE_CLK_ENABLE();
//    /**QUADSPI GPIO Configuration
//    PE10     ------> QUADSPI_CLK
//    PE11     ------> QUADSPI_NCS
//    PE12     ------> QUADSPI_BK1_IO0
//    PE13     ------> QUADSPI_BK1_IO1
//    PE14     ------> QUADSPI_BK1_IO2
//    PE15     ------> QUADSPI_BK1_IO3
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
//                          |GPIO_PIN_14|GPIO_PIN_15;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
//    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN QUADSPI_MspInit 1 */
//
//  /* USER CODE END QUADSPI_MspInit 1 */
//  }
//
//}
//
///**
//* @brief QSPI MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param hqspi: QSPI handle pointer
//* @retval None
//*/
//void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* hqspi)
//{
//  if(hqspi->Instance==QUADSPI)
//  {
//  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */
//
//  /* USER CODE END QUADSPI_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_QSPI_CLK_DISABLE();
//
//    /**QUADSPI GPIO Configuration
//    PE10     ------> QUADSPI_CLK
//    PE11     ------> QUADSPI_NCS
//    PE12     ------> QUADSPI_BK1_IO0
//    PE13     ------> QUADSPI_BK1_IO1
//    PE14     ------> QUADSPI_BK1_IO2
//    PE15     ------> QUADSPI_BK1_IO3
//    */
//    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
//                          |GPIO_PIN_14|GPIO_PIN_15);
//
//  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */
//
//  /* USER CODE END QUADSPI_MspDeInit 1 */
//  }
//
//}
