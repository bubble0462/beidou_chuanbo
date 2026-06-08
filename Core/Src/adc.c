/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance                   = ADC1;
  /* Async clock mode requires ADCSEL in RCC_CCIPR (never configured on this
   * board).  Use synchronous clock from APB bus instead — always available. */
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel      = ADC_CHANNEL_5;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC1_Init 2 */
  (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (adcHandle->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0   ------> ADC1_IN5
    */
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle)
{

  if (adcHandle->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0   ------> ADC1_IN5
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* VREFINT factory calibration (acquired at 30°C, VDDA = 3000 mV) */
#include "stm32l4xx_ll_adc.h"

/**
  * @brief  Read actual VDDA using internal voltage reference
  * @note   Switches ADC to VREFINT channel, reads it, then restores PA0 channel
  * @retval VDDA in millivolts, 3300 on failure (fallback)
  */
static uint32_t App_ReadVDDA_mV(void)
{
  ADC_ChannelConfTypeDef chVref = {0};
  ADC_ChannelConfTypeDef chOrig = {0};
  uint32_t vdda_mV = 3300U;

  /* Prepare VREFINT channel config */
  chVref.Channel      = ADC_CHANNEL_VREFINT;
  chVref.Rank         = ADC_REGULAR_RANK_1;
  chVref.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  chVref.SingleDiff   = ADC_SINGLE_ENDED;
  chVref.OffsetNumber = ADC_OFFSET_NONE;
  chVref.Offset       = 0;

  /* Prepare restore config (PA0 / Channel 5) */
  chOrig = chVref;
  chOrig.Channel = ADC_CHANNEL_5;

  /* Enable internal voltage reference path */
  LL_ADC_SetCommonPathInternalCh(
    __LL_ADC_COMMON_INSTANCE(hadc1.Instance),
    LL_ADC_PATH_INTERNAL_VREFINT
  );

  /* Switch to VREFINT channel */
  (void)HAL_ADC_ConfigChannel(&hadc1, &chVref);

  /* Dummy conversion after channel switch */
  (void)HAL_ADC_Start(&hadc1);
  (void)HAL_ADC_PollForConversion(&hadc1, 10U);
  (void)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  /* Real VREFINT read */
  (void)HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
  {
    uint32_t raw_vref = HAL_ADC_GetValue(&hadc1);
    if (raw_vref > 0U)
    {
      /* VDDA = 3000mV * VREFINT_CAL / raw_vref */
      vdda_mV = (3000U * (uint32_t)(*VREFINT_CAL_ADDR)) / raw_vref;
    }
  }
  (void)HAL_ADC_Stop(&hadc1);

  /* Disable internal reference, restore original channel */
  LL_ADC_SetCommonPathInternalCh(
    __LL_ADC_COMMON_INSTANCE(hadc1.Instance),
    LL_ADC_PATH_INTERNAL_NONE
  );
  (void)HAL_ADC_ConfigChannel(&hadc1, &chOrig);

  return vdda_mV;
}

/**
  * @brief  Read battery voltage in millivolts
  * @note   Uses VREFINT to compensate for VDDA variation.
  *         Discards first sample after potential channel switch.
  *         Averages 4 samples for noise reduction.
  * @retval Battery voltage in mV
  */
uint32_t App_ReadVbat_mV(void)
{
  uint32_t sum = 0U;

  /* Measure actual VDDA via internal reference (compensates for LDO tolerance) */
  uint32_t vdda_mV = App_ReadVDDA_mV();

  /* Dummy conversion after channel restore */
  (void)HAL_ADC_Start(&hadc1);
  (void)HAL_ADC_PollForConversion(&hadc1, 10U);
  (void)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);

  /* 4 real samples for averaging */
  for (uint8_t i = 0U; i < 4U; i++)
  {
    (void)HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK)
    {
      sum += HAL_ADC_GetValue(&hadc1);
    }
    (void)HAL_ADC_Stop(&hadc1);
  }

  /* VBAT = raw_avg * VDDA * 2 / 4095
   * R32/R33 = 100K/100K voltage divider: ADC = VBAT / 2
   * 12-bit ADC: raw / 4095 = ADC_IN / VDDA
   * => VBAT = raw * VDDA * 2 / 4095
   */
  return (sum / 4U * vdda_mV * 2U) / 4095U;
}

/* USER CODE END 1 */
