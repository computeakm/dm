/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch3;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch3;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 540-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}
/* TIM12 init function */
void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 540-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 250-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}
/* TIM13 init function */
void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}
/* TIM14 init function */
void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}
/* TIM15 init function */
void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 270-1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1000-1;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}
/* TIM16 init function */
void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 2250-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}
/* TIM17 init function */
void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 180-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 DMA Init */
    /* TIM1_CH1 Init */
    hdma_tim1_ch1.Instance = DMA1_Stream0;
    hdma_tim1_ch1.Init.Request = DMA_REQUEST_TIM1_CH1;
    hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tim1_ch1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim1_ch1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim1_ch1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim1_ch1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_tim1_ch1);

    /* TIM1_CH3 Init */
    hdma_tim1_ch3.Instance = DMA1_Stream1;
    hdma_tim1_ch3.Init.Request = DMA_REQUEST_TIM1_CH3;
    hdma_tim1_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim1_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim1_ch3.Init.Mode = DMA_NORMAL;
    hdma_tim1_ch3.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tim1_ch3.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim1_ch3.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim1_ch3.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim1_ch3.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim1_ch3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC3],hdma_tim1_ch3);

  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 DMA Init */
    /* TIM2_CH1 Init */
    hdma_tim2_ch1.Instance = DMA1_Stream2;
    hdma_tim2_ch1.Init.Request = DMA_REQUEST_TIM2_CH1;
    hdma_tim2_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch1.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tim2_ch1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim2_ch1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim2_ch1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim2_ch1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim2_ch1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],hdma_tim2_ch1);

    /* TIM2_CH3 Init */
    hdma_tim2_ch3.Instance = DMA1_Stream3;
    hdma_tim2_ch3.Init.Request = DMA_REQUEST_TIM2_CH3;
    hdma_tim2_ch3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim2_ch3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch3.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch3.Init.Mode = DMA_NORMAL;
    hdma_tim2_ch3.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tim2_ch3.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_tim2_ch3.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim2_ch3.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim2_ch3.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_tim2_ch3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC3],hdma_tim2_ch3);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM12)
  {
  /* USER CODE BEGIN TIM12_MspInit 0 */

  /* USER CODE END TIM12_MspInit 0 */
    /* TIM12 clock enable */
    __HAL_RCC_TIM12_CLK_ENABLE();
  /* USER CODE BEGIN TIM12_MspInit 1 */

  /* USER CODE END TIM12_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM13)
  {
  /* USER CODE BEGIN TIM13_MspInit 0 */

  /* USER CODE END TIM13_MspInit 0 */
    /* TIM13 clock enable */
    __HAL_RCC_TIM13_CLK_ENABLE();
  /* USER CODE BEGIN TIM13_MspInit 1 */

  /* USER CODE END TIM13_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM14)
  {
  /* USER CODE BEGIN TIM14_MspInit 0 */

  /* USER CODE END TIM14_MspInit 0 */
    /* TIM14 clock enable */
    __HAL_RCC_TIM14_CLK_ENABLE();
  /* USER CODE BEGIN TIM14_MspInit 1 */

  /* USER CODE END TIM14_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspInit 0 */

  /* USER CODE END TIM15_MspInit 0 */
    /* TIM15 clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();

    /* TIM15 interrupt Init */
    HAL_NVIC_SetPriority(TIM15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM15_IRQn);
  /* USER CODE BEGIN TIM15_MspInit 1 */

  /* USER CODE END TIM15_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspInit 0 */

  /* USER CODE END TIM16_MspInit 0 */
    /* TIM16 clock enable */
    __HAL_RCC_TIM16_CLK_ENABLE();

    /* TIM16 interrupt Init */
    HAL_NVIC_SetPriority(TIM16_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE BEGIN TIM16_MspInit 1 */

  /* USER CODE END TIM16_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM17)
  {
  /* USER CODE BEGIN TIM17_MspInit 0 */

  /* USER CODE END TIM17_MspInit 0 */
    /* TIM17 clock enable */
    __HAL_RCC_TIM17_CLK_ENABLE();

    /* TIM17 interrupt Init */
    HAL_NVIC_SetPriority(TIM17_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM17_IRQn);
  /* USER CODE BEGIN TIM17_MspInit 1 */

  /* USER CODE END TIM17_MspInit 1 */
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE9     ------> TIM1_CH1
    PE13     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspPostInit 0 */

  /* USER CODE END TIM2_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA0     ------> TIM2_CH1
    PA2     ------> TIM2_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM2_MspPostInit 1 */

  /* USER CODE END TIM2_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB1     ------> TIM3_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM12)
  {
  /* USER CODE BEGIN TIM12_MspPostInit 0 */

  /* USER CODE END TIM12_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM12 GPIO Configuration
    PB15     ------> TIM12_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM12;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM12_MspPostInit 1 */

  /* USER CODE END TIM12_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 DMA DeInit */
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC3]);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 DMA DeInit */
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC3]);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM12)
  {
  /* USER CODE BEGIN TIM12_MspDeInit 0 */

  /* USER CODE END TIM12_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM12_CLK_DISABLE();
  /* USER CODE BEGIN TIM12_MspDeInit 1 */

  /* USER CODE END TIM12_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM13)
  {
  /* USER CODE BEGIN TIM13_MspDeInit 0 */

  /* USER CODE END TIM13_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM13_CLK_DISABLE();
  /* USER CODE BEGIN TIM13_MspDeInit 1 */

  /* USER CODE END TIM13_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM14)
  {
  /* USER CODE BEGIN TIM14_MspDeInit 0 */

  /* USER CODE END TIM14_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM14_CLK_DISABLE();
  /* USER CODE BEGIN TIM14_MspDeInit 1 */

  /* USER CODE END TIM14_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspDeInit 0 */

  /* USER CODE END TIM15_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM15_CLK_DISABLE();

    /* TIM15 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM15_IRQn);
  /* USER CODE BEGIN TIM15_MspDeInit 1 */

  /* USER CODE END TIM15_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspDeInit 0 */

  /* USER CODE END TIM16_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM16_CLK_DISABLE();

    /* TIM16 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM16_IRQn);
  /* USER CODE BEGIN TIM16_MspDeInit 1 */

  /* USER CODE END TIM16_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM17)
  {
  /* USER CODE BEGIN TIM17_MspDeInit 0 */

  /* USER CODE END TIM17_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM17_CLK_DISABLE();

    /* TIM17 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM17_IRQn);
  /* USER CODE BEGIN TIM17_MspDeInit 1 */

  /* USER CODE END TIM17_MspDeInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include "pid.h"
#include "BMI088Middleware.h"
#include "BMI088driver.h"
#include "MahonyAHRS.h"
#include "CRSF.h"
#include "motor.h"
#include "main.h"
#include "dshot.h"
#include "filter_rc_bandpass.h"
#include "mtf01.h"

extern int cotrol_mode;
float angle_roll_error, angle_pitch_error, angle_yaw_error;
float gyro[3], accel[3],temperate;
float a[3];
extern AccelCalibParam accelCalibParam;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

extern PIDController x_speed_pid,x_Angle,y_speed_pid,y_Angle,z_speed_pid,z_Angle,hight_speed_pid,hight_position_pid,x_position_pid,y_position_pid;
extern PIDController_Angle pid_angle_speed;
extern PIDController_Angle pid_angle_position;
extern PIDController pid_temp;
extern MahonyAHRS mahonyAHRS;
extern crsf_channels_t rcData;
float SYStime;
float time1, time2,dt;
uint16_t speed[4];
uint16_t pwm[4];
uint8_t tx_sreed[12]; 
float uav_x,uav_y,uav_z,uav_h;
float v_body_lpf[3] = {0.0f, 0.0f, 0.0f}; // 涓婁竴娆℃护娉?锟???
float alpha = 0.09f; 
float alpha_g = 0.3f; 
float alpha_a = 0.2f; 
int losstime=10;
extern float ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7;
float x_older,y_older,z_older;
float x_lpf,y_lpf,z_lpf,gyro_x_lpf,gyro_y_lpf,gyro_z_lpf,accel_x_lpf,accel_y_lpf,accel_z_lpf;

int tim1_data_num=0;
int tim2_data_num=0;
extern uint16_t test2_speed[4];
extern uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
extern uint32_t motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
extern uint32_t motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];  
extern uint32_t motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
bool pwm1_updata,pwm2_updata;
int  slewLimiter=10;
extern bool dshot;
int upspeed_time;
extern int max,min;
int scope;
float motor1_speed,motor2_speed,motor3_speed,motor4_speed;
int old_motor1_speed,old_motor2_speed,old_motor3_speed,old_motor4_speed;
int old_x_speed,old_y_speed;
float angle_alpha=0.7f;
extern FilterRCBandstop bandstop_filter_gx;
extern FilterRCBandstop bandstop_filter_gy;
extern FilterRCBandstop bandstop_filter_gz;
extern FilterRCBandstop bandstop_filter_ax;
extern FilterRCBandstop bandstop_filter_ay;
extern FilterRCBandstop bandstop_filter_az;
extern  FilterRCLowpass lowpass_filter_accel_x;
extern  FilterRCLowpass lowpass_filter_accel_y;
extern  FilterRCLowpass lowpass_filter_accel_z;
extern  FilterRCLowpass lowpass_filter_gyro_x;
extern  FilterRCLowpass lowpass_filter_gyro_y;
extern  FilterRCLowpass lowpass_filter_gyro_z;;
extern FilterRCLowpass lowpass_filter_speed_x;
extern FilterRCLowpass lowpass_filter_speed_y;
extern FilterRCLowpass lowpass_filter_speed_z;
float word_speed[3];
float body_speed[3];
UAV_xyz word_accle;
UAV_xyz body_accle;
UAV_xyz word_speed2;
UAV_xyz word_speed3;
UAV_xyz body_speed2;
UAV_xyz mtf_01_speed;
UAV_xyz word_hight;
UAV_xyz body_hight;
UAV_xyz UAV_position;
extern MovAvgState mov_avg_speed_x;
extern MovAvgState mov_avg_speed_y;
extern MovAvgState mov_avg_speed_z;
extern MovAvgState msg_speed_x_filter;
extern MovAvgState msg_speed_y_filter;
extern MovAvgState msg_speed_z_filter;


uint32_t old_time;
uint32_t old_distance;
float time_s;
float angle[3];
extern MICOLINK_MSG_t msg;
extern  MICOLINK_PAYLOAD_RANGE_SENSOR_t mtf_01;
float lpf(float last, float input, float alpha)
{
    return last * (1.0f - alpha) + input * alpha;
}
float constrain_float(float input, float min, float max)
{
    if(input < min) return min;
    if(input > max) return max;
    return input;
}

int compute_speed(float input,int scope)
{
    int speed;
    speed = (int)(constrain_float(input, 0.0f, 100.0f) * scope / 100.0f) + min;
    return speed;
}


int constrain_int(int input, int min, int max)
{
    if(input < min) return min;
    if(input > max) return max;
    return input;
}

int slewFilter(int input, int old, int slewLimiter)
{
    int diff = input - old;
    if(diff > slewLimiter) return input;
    if(diff < -slewLimiter)  return input;
    return old;
}
void  datainline(float new_data,float *data)
{
  data[2]=data[1];
  data[1]=data[0];
  data[0]=new_data;
}

void linearSmooth3 ( float *in,float *out )
{
    int i;
        out[0] = ( 5.0 * in[0] + 2.0 * in[1] - in[2] ) / 6.0;
        out[1] = ( in[0] + in[1] + in[2] ) / 3.0;
        out[2] = ( 5.0 * in[2] + 2.0 * in[1] - in[0] ) / 6.0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM15)//1000hz 电机pid更新环
 {
    
				scope = max - min;
    slewLimiter= (int)(scope*0.05);
	  switch (cotrol_mode)
        {
        case 0:
          x_older=(ch1-50.0f)/10.0f;
				  y_older=(ch0-50.0f)/10.0f;
				  z_older=(ch3-50.0f)/10.0f;
          PIDController_Update(&x_speed_pid, x_older, gyro_x);
          PIDController_Update(&y_speed_pid, y_older, -gyro_y);
          PIDController_Update(&z_speed_pid, z_older, -gyro_z);
        uav_h=ch2;
        uav_x=x_speed_pid.out;
        uav_y=y_speed_pid.out;
        uav_z=z_speed_pid.out;
      
        motor1_speed=uav_h - uav_x + uav_y + uav_z; //FL
        motor2_speed=uav_h + uav_x + uav_y - uav_z; //FR
        motor3_speed=uav_h + uav_x - uav_y + uav_z; //BR
        motor4_speed=uav_h - uav_x -  uav_y - uav_z; //BL
          break;
        case 1:
					 x_older=(ch1-50.0f)/600.0f*3.14f;
				  y_older=(ch0-50.0f)/600.0f*3.14f;
				  z_older=(ch3-50.0f)/600.0f*3.14f;
           //angle loo

       Quaternion_compute_to_errorangle(mahonyAHRS.yaw,y_older,x_older,&mahonyAHRS, &angle[0]);
        //Quaternion_compute_to_errorangle(0,(ch0-50.0f)/100.0f*3.14f, (ch1-50.0f)/100.0f*3.14f,&mahonyAHRS, &angle[0]);
//			
//        Quaternion_PIDController(&x_Angle,angle[0]);
//        Quaternion_PIDController(&y_Angle,-angle[1]);
        
        PIDController_Update(&y_Angle,y_older, -mahonyAHRS.pitch);
        PIDController_Update(&x_Angle,x_older,   mahonyAHRS.roll);
			
        //PIDController_Update(&z_Angle, ch3-50.0f, mahonyAHRS.yaw);
        PIDController_Update(&x_speed_pid, x_Angle.out, gyro_x);
        PIDController_Update(&y_speed_pid, y_Angle.out, -gyro_y);
        PIDController_Update(&z_speed_pid, (ch3-50.0f)/10.0f, -gyro_z);
      
        uav_h=ch2;
        uav_x=x_speed_pid.out;
        uav_y=y_speed_pid.out;
        uav_z=z_speed_pid.out;
        motor1_speed=uav_h - uav_x + uav_y + uav_z; //FL
        motor2_speed=uav_h + uav_x + uav_y - uav_z; //FR
        motor3_speed=uav_h + uav_x - uav_y + uav_z; //BR
        motor4_speed=uav_h - uav_x -  uav_y - uav_z; //BL
          break;
        case 2:
//        Quaternion_compute_to_errorangle((ch3-50.0f)/100.0f*3.14f,(ch0-50.0f)/100.0f*3.14f, (ch1-50.0f)/100.0f*3.14f,&mahonyAHRS, &angle[0]);
//        Quaternion_PIDController(&x_Angle,angle[0]);
//        Quaternion_PIDController(&y_Angle,angle[1]);
//				Quaternion_PIDController(&z_Angle,angle[2]);
//        body_hight.z=mtf_01.distance/1000.0f;
//        turn_body_to_world(mahonyAHRS,&word_hight,&body_hight);
//        PIDController_Update(&y_Angle, (ch0-50.0f)/2.0f, -mahonyAHRS.pitch);
//        PIDController_Update(&x_Angle, (ch1-50.0f)/2.0f, mahonyAHRS.roll);
        //PIDController_Update(&z_Angle, ch3-50.0f, mahonyAHRS.yaw);
//        PIDController_Update(&x_speed_pid, x_Angle.out, gyro_x);
//        PIDController_Update(&y_speed_pid, y_Angle.out, -gyro_y);
//				PIDController_Update(&z_speed_pid,z_Angle.out, -gyro_z);
        //PIDController_Update(&z_speed_pid, (ch3-50.0f)/10.0f, -gyro_z);
//        PIDIncremental_Update(&hight_position_pid, ch2/20.0f, word_hight.z);
//        word_speed[2]=hight_position_pid.out;
//        WorldToBodyFromMahony(&mahonyAHRS,&word_speed[0],&body_speed[0]);
//        uav_h=body_speed[2];
//        uav_x=x_speed_pid.out;
//        uav_y=y_speed_pid.out;
//        uav_z=z_speed_pid.out;
//       motor1_speed=uav_h - uav_x + uav_y + uav_z; //FL
//        motor2_speed=uav_h + uav_x + uav_y - uav_z; //FR
//        motor3_speed=uav_h + uav_x - uav_y + uav_z; //BR
//        motor4_speed=uav_h - uav_x -  uav_y - uav_z; //BL



          break;
        default:
          break;
        }
	      speed[0]=compute_speed(motor1_speed,scope);
        speed[1]=compute_speed(motor2_speed,scope);
        speed[2]=compute_speed(motor3_speed,scope);
        speed[3]=compute_speed(motor4_speed,scope);
        //speed slew rate limiter
        speed[0]=slewFilter(speed[0],old_motor1_speed,slewLimiter);
        speed[1]=slewFilter(speed[1],old_motor2_speed,slewLimiter);
        speed[2]=slewFilter(speed[2],old_motor3_speed,slewLimiter);
        speed[3]=slewFilter(speed[3],old_motor4_speed,slewLimiter);
        old_motor1_speed=speed[0];
        old_motor2_speed=speed[1];
        old_motor3_speed=speed[2];
        old_motor4_speed=speed[3];
	 
 }   



	if(htim->Instance == TIM16) // 1200hz
    {
      
        //temperature control
				PIDTemp_Update(&pid_temp, 40.0f, temperate);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (int)(pid_temp.out));
        //attitude loop
        MahonyAHRSupdateIMU(&mahonyAHRS, gyro_x, gyro_y , gyro_z,accel_x, accel_y, accel_z);
        Mahony_computeAngles(&mahonyAHRS);
			  body_accle.x=accel_x;
				 body_accle.y=accel_y;
			 body_accle.z=accel_z;
				
        //speed loop
        /*
        //angle position loop
         PIDControllerAngle_Update(&pid_angle_position, 0.0f, mahonyAHRS.roll, 0.0f, mahonyAHRS.pitch, 0.0f, mahonyAHRS.yaw);
        PIDControllerAngle_Update(&pid_angle_speed,pid_angle_position.x_out, gyro_x, pid_angle_position.y_out, gyro_y, pid_angle_position.z_out, gyro_z);
        */
        //uav_x=pid_angle_speed.x_out;
        //uav_y=pid_angle_speed.y_out;
        //uav_z=pid_angle_speed.z_out;
        
        
         turn_body_to_world(mahonyAHRS,&word_accle,&body_accle);
				 word_accle.z=word_accle.z-9.8f;
				 word_speed3.x+=word_accle.x*0.00083333f;
				 word_speed3.y+=word_accle.y*0.00083333f;
				 word_speed3.z+=word_accle.z*0.00083333f;
        filter_rc_lowpass_process(&lowpass_filter_speed_x,word_speed3.x);
        filter_rc_lowpass_process(&lowpass_filter_speed_y,word_speed3.y);
        filter_rc_lowpass_process(&lowpass_filter_speed_z,word_speed3.z);
				 word_speed2.x=word_speed3.x;
				 word_speed2.y=word_speed3.y;
				 word_speed2.z=word_speed3.z;
				 if(mtf_01.ready==1)
				 {
					
				 time_s=(mtf_01.time_ms-old_time);
         if(mtf_01.flow_quality>30&&gyro_x<0.5f&&gyro_x>-0.5f&&gyro_y<0.5f&&gyro_y>-0.5f)
         {
				 mtf_01_speed.x=(float)mtf_01.flow_vel_x*mtf_01.distance/100000.0f;
				 mtf_01_speed.y=(float)mtf_01.flow_vel_y*mtf_01.distance/100000.0f;
         if(mtf_01.strength>30)
         {
           mtf_01_speed.z= ((float)mtf_01.distance-(float)old_distance)/time_s/1000.0f;
         }
				
				 turn_body_to_world(mahonyAHRS,&mtf_01_speed,&mtf_01_speed);
					 
				 word_speed2.x=0.9f*mtf_01_speed.x+0.1f*word_speed3.x;
         word_speed2.y=0.9f*mtf_01_speed.y+0.1f*word_speed3.y;
         word_speed2.z=0.9f*mtf_01_speed.z+0.1f*word_speed3.z;
				//  word_speed2.y=mtf_01_speed.y;
				// 	word_speed2.z=mtf_01_speed.z;
       
					 word_speed3.x=word_speed2.x;
					 word_speed3.y=word_speed2.y;
					 word_speed3.z=word_speed2.z;
				 	word_speed3.x=movavg_process(&msg_speed_x_filter, word_speed3.x);
          word_speed3.y=movavg_process(&msg_speed_y_filter, word_speed3.y);
          word_speed3.z=movavg_process(&msg_speed_z_filter, word_speed3.z); 
					 
         }else if(mtf_01.flow_quality>30)
         {
         mtf_01_speed.x=(float)mtf_01.flow_vel_x*mtf_01.distance/100000.0f;
				 mtf_01_speed.y=(float)mtf_01.flow_vel_y*mtf_01.distance/100000.0f;
				 mtf_01_speed.z= ((float)mtf_01.distance-(float)old_distance)/time_s/1000.0f;
				 turn_body_to_world(mahonyAHRS,&mtf_01_speed,&mtf_01_speed);
					 
				 word_speed2.x=0.2f*mtf_01_speed.x+0.8f*word_speed3.x;
         word_speed2.y=0.2f*mtf_01_speed.y+0.8f*word_speed3.y;
         word_speed2.z=0.2f*mtf_01_speed.z+0.8f*word_speed3.z;
				//  word_speed2.y=mtf_01_speed.y;
				// 	word_speed2.z=mtf_01_speed.z;
       
					 word_speed3.x=word_speed2.x;
					 word_speed3.y=word_speed2.y;
					 word_speed3.z=word_speed2.z;
				 	word_speed3.x=movavg_process(&msg_speed_x_filter, word_speed3.x);
          word_speed3.y=movavg_process(&msg_speed_y_filter, word_speed3.y);
          word_speed3.z=movavg_process(&msg_speed_z_filter, word_speed3.z); 
         }
				 old_time=mtf_01.time_ms;
					old_distance=mtf_01.distance;
					 mtf_01.ready=0;
					 
				 }

        word_speed2.x=filter_rc_lowpass_process(&lowpass_filter_speed_x,word_speed2.x);
        word_speed2.y=filter_rc_lowpass_process(&lowpass_filter_speed_y,word_speed2.y);
        word_speed2.z=filter_rc_lowpass_process(&lowpass_filter_speed_z,word_speed2.z);
        word_speed2.x=movavg_process(&mov_avg_speed_x, word_speed2.x);
        word_speed2.y=movavg_process(&mov_avg_speed_y, word_speed2.y);
        word_speed2.z=movavg_process(&mov_avg_speed_z, word_speed2.z);
        WorldToBodyFromMahony_xyz(&mahonyAHRS,&word_speed2,&body_speed2);
        UAV_position.x=UAV_position.x+word_speed2.x*0.00083333f;
        UAV_position.y=UAV_position.y+word_speed2.y*0.00083333f;
        if(mahonyAHRS.pitch<0.1f&&mahonyAHRS.pitch>-0.1f&&mahonyAHRS.roll<0.1f&&mahonyAHRS.roll>-0.1f)
        {
          UAV_position.z=(float)mtf_01.distance/1000.0f;
        }else
        {
          UAV_position.z=UAV_position.z+word_speed2.z*0.00083333f;
        }
        if(UAV_position.z<0.0f)
        {
          UAV_position.z=0.0f;
        }
      

				 
				
				 
}


        
       

				
          
         
  
				//
        
				
        

     
     

		


       
    
    if(htim->Instance == TIM17) // 1500hz 
    {
			
			  BMI088_read(&gyro[0], &accel[0], &temperate);
        
        gyro[0]=(int)(gyro[0]*25000)/25000.0f+0.002f;
        gyro[1]=(int)(gyro[1]*25000)/25000.0f+0.002f;
        gyro[2]=(int)(gyro[2]*25000)/25000.0f+0.002f;
        gyro_x=filter_rc_bandstop_process(&bandstop_filter_gx, gyro[0]);
        gyro_y=filter_rc_bandstop_process(&bandstop_filter_gy, gyro[1]);
        gyro_z=filter_rc_bandstop_process(&bandstop_filter_gz, gyro[2]);
//        gyro_x=filter_rc_lowpass_process(&lowpass_filter_gyro_x, gyro[0]);
//        gyro_y=filter_rc_lowpass_process(&lowpass_filter_gyro_y, gyro[1]);
//        gyro_z=filter_rc_lowpass_process(&lowpass_filter_gyro_z, gyro[2]);
				gyro_x=filter_rc_lowpass_process(&lowpass_filter_gyro_x, gyro_x);
        gyro_y=filter_rc_lowpass_process(&lowpass_filter_gyro_y, gyro_y);
        gyro_z=filter_rc_lowpass_process(&lowpass_filter_gyro_z, gyro_z);

        accel_x=filter_rc_bandstop_process(&bandstop_filter_ax,accel[0]);
        accel_y=filter_rc_bandstop_process(&bandstop_filter_ay,accel[1]);
        accel_z=filter_rc_bandstop_process(&bandstop_filter_az,accel[2]);



			
      losstime++;
//			upspeed_time++;
//			if(upspeed_time>5)
//			{
//		
//			upspeed_time=0;
//			}
//					pwm1_updata=true;
//			pwm2_updata=true;
			
//		dshot_write(test2_speed);
      if(losstime>500)
      {
        losstime=500;
      }
      SYStime += 0.0001f;
			
      if(SYStime >= 1.0f)
      {
        SYStime = 0.0f;
      } 
      // Your code here
    }
		

//		if(htim->Instance == htim1.Instance) // check if the interrupt comes from TIM1
//    {
//        // Your code here
//       
//       		
//			if(dshot==true)
//			{	
//				if(tim1_data_num>DSHOT_DMA_BUFFER_SIZE-1)
//        {
//          tim1_data_num=0;
//					pwm1_updata=false;
////					pwm1_updata=true;
//					
//        }else
//				{
//					if(pwm1_updata==true)
//					{
//						__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, motor1_dmabuffer[tim1_data_num]);
//        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, motor2_dmabuffer[tim1_data_num]);
//					 tim1_data_num++;
//					}
//					 
//				}
//				
//				
//			}
//        
//					
//       
//       
//    }
//		
//		

//	if(htim->Instance == htim2.Instance) // check if the interrupt comes from TIM2
//  {
//        // Your code here
//        
//    if(dshot==true)
//	  {  
//		 
//		  if(tim2_data_num>DSHOT_DMA_BUFFER_SIZE-1)
//      {
//        tim2_data_num=0;
//				pwm2_updata=false;
////					pwm2_updata=true;
//      }else
//			{
//				if(pwm2_updata==true)
//				{
//						
//				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, motor3_dmabuffer[tim2_data_num]);
//        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, motor4_dmabuffer[tim2_data_num]);
//				tim2_data_num++;
//						
//				}
//					
//			}
//				
//		 
//		 
//		 
//		}
//        
//        
//				
//        
//  }
//		
//		 
//  
//		
//		
		
    

}



/* USER CODE END 1 */
