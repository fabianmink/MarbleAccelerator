/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
//#include <string.h>
#include "libcontrol/control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


//Power stage XXX
static calib_data_t calib_data = {
		.ia = {.offs = 0, .gain = -1900},
		.ib = {.offs = 0, .gain = -1900},
		.ic = {.offs = 0, .gain = -1900},
		.i_autoOffs = true,
		.vbus = {.offs = 47, .gain = 551},
		.poti = {.offs = 0,  .gain = 256}
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CORDIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  LL_PWR_DisableUCPDDeadBattery();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_OPAMP3_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */

  //Enable Opamps
  LL_OPAMP_Enable(OPAMP1);
  LL_OPAMP_Enable(OPAMP2);
  LL_OPAMP_Enable(OPAMP3);


  //Enable ADCs
  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);

  //LL_ADC_EnableIT_EOCS(ADC1);
  //LL_ADC_REG_StartConversionSWStart(ADC1);
  LL_ADC_INJ_StartConversion(ADC1);
  //LL_ADC_EnableIT_EOCS(ADC2);
  //LL_ADC_REG_StartConversionSWStart(ADC2);
  LL_ADC_INJ_StartConversion(ADC2);


  //TIM 4 is for Encoder
  LL_TIM_EnableCounter(TIM4);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3); //Zero marker position


  //TIM1 is for PWM
  LL_TIM_EnableCounter(TIM1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
  //LL_TIM_EnableAllOutputs(TIM1); //PWM on

  LL_TIM_EnableIT_UPDATE(TIM1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 80, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1�s transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < 100);
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(160000000);

  LL_SetSystemCoreClock(160000000);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration
  PA0   ------> ADC1_IN1
  PB12   ------> ADC1_IN11
  PB14   ------> ADC1_IN5
  */
  GPIO_InitStruct.Pin = VBUS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(VBUS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = POTI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(POTI_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TEMP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(TEMP_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC1, 0);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
  ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS;
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);
  LL_ADC_INJ_SetQueueMode(ADC1, LL_ADC_INJ_QUEUE_DISABLE);
  LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_11);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);
  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VOPAMP1, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_VOPAMP1, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetOffset(ADC1, LL_ADC_OFFSET_1, LL_ADC_CHANNEL_VOPAMP1, 2490);
  LL_ADC_SetOffsetSign(ADC1, LL_ADC_OFFSET_1, LL_ADC_OFFSET_SIGN_NEGATIVE);
  LL_ADC_SetOffsetSaturation(ADC1, LL_ADC_OFFSET_1, LL_ADC_OFFSET_SATURATION_DISABLE);
  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_3, LL_ADC_CHANNEL_11);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */


  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC2, 0);
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
  ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);
  LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);
  LL_ADC_INJ_SetTriggerEdge(ADC2, LL_ADC_INJ_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC2);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC2);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_VOPAMP2);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_VOPAMP2, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_VOPAMP2, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetOffset(ADC2, LL_ADC_OFFSET_1, LL_ADC_CHANNEL_VOPAMP2, 2490);
  LL_ADC_SetOffsetSign(ADC2, LL_ADC_OFFSET_1, LL_ADC_OFFSET_SIGN_NEGATIVE);
  LL_ADC_SetOffsetSaturation(ADC2, LL_ADC_OFFSET_1, LL_ADC_OFFSET_SATURATION_DISABLE);
  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_VOPAMP3_ADC2);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_VOPAMP3_ADC2, LL_ADC_SAMPLINGTIME_2CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_VOPAMP3_ADC2, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetOffset(ADC2, LL_ADC_OFFSET_2, LL_ADC_CHANNEL_VOPAMP3_ADC2, 2490);
  LL_ADC_SetOffsetSign(ADC2, LL_ADC_OFFSET_2, LL_ADC_OFFSET_SIGN_NEGATIVE);
  LL_ADC_SetOffsetSaturation(ADC2, LL_ADC_OFFSET_2, LL_ADC_OFFSET_SATURATION_DISABLE);
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */

  /* nothing else to be configured */

  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**OPAMP1 GPIO Configuration
  PA1   ------> OPAMP1_VINP
  PA3   ------> OPAMP1_VINM0
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_NORMALSPEED;
  OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_PGA_IO0_BIAS;
  OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO0;
  LL_OPAMP_Init(OPAMP1, &OPAMP_InitStruct);
  LL_OPAMP_SetInputsMuxMode(OPAMP1, LL_OPAMP_INPUT_MUX_DISABLE);
  LL_OPAMP_SetInternalOutput(OPAMP1, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
  LL_OPAMP_SetPGAGain(OPAMP1, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
  LL_OPAMP_SetTrimmingMode(OPAMP1, LL_OPAMP_TRIMMING_FACTORY);
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**OPAMP2 GPIO Configuration
  PA5   ------> OPAMP2_VINM0
  PA7   ------> OPAMP2_VINP
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_NORMALSPEED;
  OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_PGA_IO0_BIAS;
  OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO0;
  LL_OPAMP_Init(OPAMP2, &OPAMP_InitStruct);
  LL_OPAMP_SetInputsMuxMode(OPAMP2, LL_OPAMP_INPUT_MUX_DISABLE);
  LL_OPAMP_SetInternalOutput(OPAMP2, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
  LL_OPAMP_SetPGAGain(OPAMP2, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
  LL_OPAMP_SetTrimmingMode(OPAMP2, LL_OPAMP_TRIMMING_FACTORY);
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  LL_OPAMP_InitTypeDef OPAMP_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**OPAMP3 GPIO Configuration
  PB0   ------> OPAMP3_VINP
  PB2   ------> OPAMP3_VINM0
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_NORMALSPEED;
  OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_PGA_IO0_BIAS;
  OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO0;
  LL_OPAMP_Init(OPAMP3, &OPAMP_InitStruct);
  LL_OPAMP_SetInputsMuxMode(OPAMP3, LL_OPAMP_INPUT_MUX_DISABLE);
  LL_OPAMP_SetInternalOutput(OPAMP3, LL_OPAMP_INTERNAL_OUPUT_ENABLED);
  LL_OPAMP_SetPGAGain(OPAMP3, LL_OPAMP_PGA_GAIN_16_OR_MINUS_15);
  LL_OPAMP_SetTrimmingMode(OPAMP3, LL_OPAMP_TRIMMING_FACTORY);
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_UP_TIM16_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP;
  TIM_InitStruct.Autoreload = 2499;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 1;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_1;
  TIM_BDTRInitStruct.DeadTime = 32;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
  TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
  TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
  TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
  TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PC13   ------> TIM1_CH1N
  PB15   ------> TIM1_CH3N
  PA8   ------> TIM1_CH1
  PA9   ------> TIM1_CH2
  PA10   ------> TIM1_CH3
  PA12   ------> TIM1_CH2N
  */
  GPIO_InitStruct.Pin = PWM_1N_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(PWM_1N_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_3N_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(PWM_3N_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(PWM_1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(PWM_2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(PWM_3_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PWM_2N_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(PWM_2N_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = J3_PWM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(J3_PWM_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ENC_A_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ENC_A_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ENC_B_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ENC_B_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ENC_Z_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(ENC_Z_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



#define PWM_MAX 2500

static void pwm_setrefint_3ph_tim1(int16_t ref[3]){
	int i;
	int32_t refi[3];
	uint32_t refu[3];

	for(i=0;i<3;i++){
		refi[i] = ( (int32_t)ref[i]);
		refi[i] += 32768;
		refu[i] = refi[i];
		refu[i] = refu[i] * PWM_MAX;
		refu[i] = refu[i] >> 16;
	}

	TIM1->CCR1 = refu[0];
	TIM1->CCR2 = refu[1];
	TIM1->CCR3 = refu[2];
}


typedef struct
{
	int16_t kp;       //  P-Gain Q13.2
	int16_t ki;       //  I-Gain Q7.8
	int16_t min;      //  lower output limit
	int16_t max;      //  upper output limit
	int32_t i_val;    //  current integral value
} control_pictrl_i16_t;


int16_t control_pictrl_i16(control_pictrl_i16_t* controller, int16_t ref, int16_t act){

	int32_t ctrldiff;
	int32_t p_val, i_val;
	int32_t tmp_out;
	//int16_t out;

	ctrldiff = ((int32_t)ref - (int32_t)act);
	p_val = (((int32_t)controller->kp) * ctrldiff) >> 2;
	if(controller->ki){
		i_val = ((((int32_t)controller->ki) * ctrldiff)>>8) + controller->i_val;
	}
	else {
		i_val = 0.0f;
	}

	tmp_out = (p_val + i_val);

	if(tmp_out > controller->max){
		tmp_out = controller->max;
		if(i_val > controller->i_val) i_val = controller->i_val;
		//i_val = controller->max - p_val;
	}
	if(tmp_out < controller->min){
		tmp_out = controller->min;
		if(i_val < controller->i_val) i_val = controller->i_val;
		//i_val = controller->min - p_val;
	}

	controller->i_val = i_val;
	return((int16_t)tmp_out);
}


//Linear scale / offset:  out = gain*(in + offs)
int16_t interp_linearTrsfm_i16(interp_lin_i16_t* coeffs, int16_t in){
	int32_t accu;

	accu = in + coeffs->offs;
	accu *= coeffs->gain;

	return( (int16_t) (accu>>8));
}

//Control data struct
typedef struct{
	int16_t ia;
	int16_t ib;
	int16_t ic;
	int16_t ua;
	int16_t ub;
	int16_t uc;
	int16_t vbus;
	int16_t iaref;
	int16_t iaval;
	int16_t ibref;
	int16_t ibval;
	uint32_t cnt_ccon;
	uint32_t cnt_period;
	uint32_t cnt_reTrig;
	uint16_t cnt_starta;
	uint16_t cnt_stopa;
	uint16_t cnt_startb;
	uint16_t cnt_stopb;
	int16_t poti;
} ctrl_data_t;
static ctrl_data_t myctrl;

static uint8_t imax = 20;
static uint8_t umax = 25;
static uint8_t umin = 8;
static int32_t ia_offs_sum;
static int32_t ib_offs_sum;
static int32_t ic_offs_sum;


typedef enum {
	sens_state_wft = 0,
	sens_state_run = 1,
	sens_state_rdy = 2
} sens_states_t;

typedef struct{
	uint16_t cnt;
	uint16_t spd;
	uint32_t pos;
	uint16_t level;
	uint16_t hyst;
	sens_states_t state;
} sens_data_t;
static sens_data_t mysens = {
		.cnt = 0,
		.spd = 0,
		.pos = 0,
		.level = 3500,
		.hyst = 100,
		.state = sens_state_wft
};


static bool button_pressed = 0;  //indicate a button press
static bool trigger_in = 0;  //indicate trigger event

//Data recorder (debugging)
typedef enum {
	datarec_state_startup = 0,
	datarec_state_wft = 1,
	datarec_state_rec = 2,
	datarec_state_rdy = 3
} datarec_states_t;


#define BUFLEN 400 //Buffer length
#define CYCLEDIV 4 //cycle div

static struct{
	int16_t ia_buf[BUFLEN];
	int16_t ib_buf[BUFLEN];
	int16_t ic_buf[BUFLEN];
	int16_t ua_buf[BUFLEN];
	int16_t vbus_buf[BUFLEN];
} myDataRecData;

static int i_elecnt = 0;
static int dataRecTrig = 0;

static datarec_states_t myDataRecState = datarec_state_startup;



typedef enum {
	ctrl_sm_state_startup = 0,
	ctrl_sm_state_curroffs = 1,
	ctrl_sm_state_ready = 3,
	ctrl_sm_state_ccon = 4,
	ctrl_sm_state_error = 8
} ctrl_sm_states_t;



//Current controllers
static control_pictrl_i16_t pi_a = {
		.i_val = 0,
		.ki = 10,
		.kp = 10,
		.max = 0,  // will be adapted "online" to vbus!!
		.min = 0
};

static control_pictrl_i16_t pi_b = {
		.i_val = 0,
		.ki = 10,
		.kp = 10,
		.max = 0,
		.min = 0
};



static ctrl_sm_states_t ctrl_sm_state = ctrl_sm_state_startup;
static int ctrl_sm_cnt = 0;
static int ctrl_sm_edge_detect = 0;



void control_sm_to_ready_mode(void){
	//stop PWM
	LL_TIM_DisableAllOutputs(TIM1); //PWM off
	ctrl_sm_state = ctrl_sm_state_ready;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	ctrl_sm_edge_detect = 0;
}

void control_sm_to_error(void){
	//stop PWM
	LL_TIM_DisableAllOutputs(TIM1); //PWM off
	ctrl_sm_state = ctrl_sm_state_error;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	ctrl_sm_edge_detect = 0;
}

void control_sm_to_ccon_mode(void){
	//Transfer to pos mode, do initialization
	pi_a.i_val = 0;
	pi_b.i_val = 0;
	//pi_q.i_val = 0;
	myctrl.cnt_ccon = 0;
	ctrl_sm_state = ctrl_sm_state_ccon;
	int16_t refs[3] = {0,0,0};
	pwm_setrefint_3ph_tim1(refs);
	LL_TIM_EnableAllOutputs(TIM1); //PWM on
	ctrl_sm_edge_detect = 0;
}

void datarec_sm(void){
	if(myDataRecState == datarec_state_startup){
		myDataRecState = datarec_state_wft;
	}
	else if(myDataRecState == datarec_state_wft){
		if(dataRecTrig){
			i_elecnt = 0;
			myDataRecState = datarec_state_rec;
		}
	}
	else if(myDataRecState == datarec_state_rec){
		//store debug data to buffers
		int i_elecnt_dn = i_elecnt / CYCLEDIV;
		myDataRecData.ia_buf[i_elecnt_dn] = myctrl.ia;
		myDataRecData.ib_buf[i_elecnt_dn] = myctrl.ib;
		myDataRecData.ic_buf[i_elecnt_dn] = myctrl.ic;
		myDataRecData.ua_buf[i_elecnt_dn] = myctrl.ua;
		myDataRecData.vbus_buf[i_elecnt_dn] = myctrl.vbus;

		i_elecnt++;
		if(i_elecnt/CYCLEDIV >= BUFLEN) {
			myDataRecState = datarec_state_rdy;
		}
	}
	else if(myDataRecState == datarec_state_rdy){
		if(dataRecTrig == 0){ //wait for manual restart (use debugger!!)
			myDataRecState = datarec_state_wft;
		}
	}






}


void control_sm(void){

	//master_data[master_data_pread] is always consistent, because WFB ISR can not interrupt control ISR

	if(ctrl_sm_state == ctrl_sm_state_startup){
		ctrl_sm_cnt++;
		//delay 0.5s
		if(ctrl_sm_cnt >= 8000){
			ctrl_sm_cnt = 0;
			ia_offs_sum = ib_offs_sum = ic_offs_sum = 0;
			ctrl_sm_state = ctrl_sm_state_curroffs;

			myctrl.cnt_period = 160000; //10s
			myctrl.cnt_reTrig = 1600;
			myctrl.cnt_starta = 100;
			myctrl.cnt_stopa = 550;
			myctrl.iaval = 3000; //ca. 10A

			myctrl.cnt_startb = 550;
			myctrl.cnt_stopb = 690;
			myctrl.ibval = 3500;
		}
	}
	else if(ctrl_sm_state == ctrl_sm_state_ready){
		ctrl_sm_cnt++;
		//if(button_pressed){
		if(button_pressed || ctrl_sm_cnt >= 8000){  //with autostart after 0.5s
			if(ctrl_sm_edge_detect){
				control_sm_to_ccon_mode();
			}
		}
		else{
			ctrl_sm_edge_detect = 1;
		}
	}
	else if(ctrl_sm_state == ctrl_sm_state_error){
		ctrl_sm_cnt = 0;
		if(button_pressed){
			if(ctrl_sm_edge_detect){
				control_sm_to_ready_mode();
			}
		}
		else{
			ctrl_sm_edge_detect = 1;
		}
	}
	else if(ctrl_sm_state == ctrl_sm_state_curroffs){
		ctrl_sm_cnt++;
		//wait 0.5s for offset determination
		if((!calib_data.i_autoOffs) || (ctrl_sm_cnt >= 8000)){
			control_sm_to_ready_mode();
			if (calib_data.i_autoOffs){
				calib_data.ia.offs = ia_offs_sum / 8000;
				calib_data.ib.offs = ib_offs_sum / 8000;
				calib_data.ic.offs = ic_offs_sum / 8000;
			}
			//Optionally, do not do this automatically, but only after command from master
			ctrl_sm_cnt = 0;
			control_sm_to_ready_mode();
		}

	}
	else if(ctrl_sm_state == ctrl_sm_state_ccon){
		ctrl_sm_cnt = 0;
		if(button_pressed){
			if(ctrl_sm_edge_detect){
				control_sm_to_ready_mode();
			}
		}
		else{
			ctrl_sm_edge_detect = 1;
		}
	}

}

void sens_eval(void){
	if( mysens.state == sens_state_wft){
		mysens.cnt = 0;
		if(myctrl.poti < mysens.level-mysens.hyst){
			mysens.state = sens_state_run;
		}
	}
	else if( mysens.state == sens_state_run){
		mysens.cnt++;
		if(myctrl.poti > mysens.level+mysens.hyst){
			mysens.spd = 1600000/mysens.cnt;
			mysens.pos = 0;
			mysens.cnt = 0;
			mysens.state = sens_state_rdy;
		}
	}
	else if( mysens.state == sens_state_rdy){
		mysens.cnt++;
		mysens.pos += mysens.spd;
		if(mysens.cnt > 32000){
			mysens.state = sens_state_wft;
		}
	}

}

//Vermutl. 16kHz
void main_pwm_ctrl(void){
	if(LL_GPIO_IsInputPinSet(BUTTON_GPIO_Port, BUTTON_Pin)){
		button_pressed = 0;
	} else{
		button_pressed = 1;
	}
	LL_GPIO_SetPinPull(BUTTON_GPIO_Port, BUTTON_Pin, LL_GPIO_PULL_DOWN);

	if(LL_GPIO_IsInputPinSet(ENC_A_GPIO_Port, ENC_A_Pin)){
		trigger_in = 0;
	} else{
		trigger_in = 1;
	}


	control_sm();

	// *** CURRENT MEASUREMENT and transformation ***
	while(!LL_ADC_IsActiveFlag_JEOS(ADC1)){
		//Error after certain time, if adc will not get ready -> signal error!
		//should not happen, because due to encoder read and conversion, a lot of time has passed
	}

	//Currents are in int16, Q7.8
	int16_t ia_raw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
	myctrl.ia = interp_linearTrsfm_i16(&calib_data.ia, ia_raw);

	//1024 = 9.0V,  1143 = 10.0V, 1262 = 11.0V
	int16_t vbusraw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
	myctrl.vbus = interp_linearTrsfm_i16(&calib_data.vbus, vbusraw);
	if( myctrl.vbus < (5<<8) ) {
		myctrl.vbus = (5<<8);
	}
	if( myctrl.vbus > (40<<8) ){
		myctrl.vbus = (40<<8);
	}
	int16_t divvbus = 32767/myctrl.vbus;

	int16_t potiraw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);
	myctrl.poti = interp_linearTrsfm_i16(&calib_data.poti, potiraw);

	int16_t ib_raw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
	myctrl.ib = interp_linearTrsfm_i16(&calib_data.ib, ib_raw);

	int16_t ic_raw = (int16_t) LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);
	myctrl.ic = interp_linearTrsfm_i16(&calib_data.ic, ic_raw);

	if(ctrl_sm_state >= ctrl_sm_state_ready){
		//signal over- / undervoltage and overcurrent
		if(     (myctrl.ia > ((int16_t)imax)*256)  ||    (myctrl.ia < ((int16_t)imax)*-256)       ){
			control_sm_to_error();
		}
		if(     (myctrl.ib > ((int16_t)imax)*256)  ||    (myctrl.ib < ((int16_t)imax)*-256)       ){
			control_sm_to_error();
		}
		if(     (myctrl.ib > ((int16_t)imax)*256)  ||    (myctrl.ib < ((int16_t)imax)*-256)       ){
			control_sm_to_error();
		}
		if(     (myctrl.vbus > ((int16_t)umax)*256)  ||    (myctrl.vbus < ((int16_t)umin)*256)       ){
			control_sm_to_error();
		}
	}

	sens_eval();

	if (ctrl_sm_state == ctrl_sm_state_curroffs) {
		ia_offs_sum -= ia_raw;
		ib_offs_sum -= ib_raw;
		ic_offs_sum -= ic_raw;
	}


	// *** CONTROLLERS: Position / Current ***
	if(ctrl_sm_state == ctrl_sm_state_ccon){
		myctrl.cnt_ccon++;
		if( myctrl.cnt_ccon >= myctrl.cnt_period ){
			myctrl.cnt_ccon=0;
			dataRecTrig = 1;
		}
		else if( (myctrl.cnt_ccon >= myctrl.cnt_reTrig) &&  trigger_in){
			myctrl.cnt_ccon=0;
			dataRecTrig = 1;
		}


		myctrl.iaref = 256*0; //0A
		myctrl.ibref = 256*0; //0A
		//ibref = 0;

		if(   (myctrl.cnt_ccon > myctrl.cnt_starta) && (myctrl.cnt_ccon < myctrl.cnt_stopa)){
			myctrl.iaref = myctrl.iaval; //for testing
		}
		if(   (myctrl.cnt_ccon > myctrl.cnt_startb) && (myctrl.cnt_ccon < myctrl.cnt_stopb)){
			myctrl.ibref = myctrl.ibval; //for testing
		}


	}

	if( ctrl_sm_state == ctrl_sm_state_ccon ){
		//Current Controllers
		pi_a.max = myctrl.vbus/2;
		pi_a.min = -myctrl.vbus/2;
		pi_b.max = myctrl.vbus/2;
		pi_b.min = -myctrl.vbus/2;

		myctrl.ua = control_pictrl_i16(&pi_a,myctrl.iaref,myctrl.ia);
		myctrl.ub = control_pictrl_i16(&pi_b,myctrl.ibref,myctrl.ib);
		myctrl.uc = -(myctrl.ua/2 + myctrl.ub/2);
	}


	int16_t refs[3];
	int32_t tmpref = myctrl.ua*divvbus;
	if(tmpref > 32767) tmpref = 32767;
	if(tmpref < -32768) tmpref = -32768;
	refs[0] = tmpref;

	tmpref = myctrl.ub*divvbus;
	if(tmpref > 32767) tmpref = 32767;
	if(tmpref < -32768) tmpref = -32768;
	refs[1] = tmpref;

	tmpref = myctrl.uc*divvbus;
	if(tmpref > 32767) tmpref = 32767;
	if(tmpref < -32768) tmpref = -32768;
	refs[2] = tmpref;

	if(ctrl_sm_state == ctrl_sm_state_ready){
		refs[0] = 0;
		refs[1] = 0;
		refs[2] = 0;
	}

	//PWM output
	pwm_setrefint_3ph_tim1(refs);

	datarec_sm();

	LL_GPIO_SetPinPull(BUTTON_GPIO_Port, BUTTON_Pin, LL_GPIO_PULL_NO);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

