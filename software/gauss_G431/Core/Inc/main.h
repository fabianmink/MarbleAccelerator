/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	uint8_t slaveNum; //I am slave no...
	uint8_t slaveCnt; //Total no of slaves at bus
	uint16_t slaveAdr;
	uint16_t masterAdr;
	uint8_t havePA : 1;  //Module with PA/LNA (MRF24J40ME) mounted
} wfb_slave_settings_t;


typedef struct
{
	int16_t gain;        //  gain, Q7.8
	int16_t offs;        //  offset
} interp_lin_i16_t;


typedef struct
{
	interp_lin_i16_t ia;
	interp_lin_i16_t ib;
	interp_lin_i16_t ic;
	bool i_autoOffs;
	interp_lin_i16_t vbus;
	interp_lin_i16_t temp;
	interp_lin_i16_t poti;
} calib_data_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define WFB_SYNC_TIMER_PERIOD    50000   //10ms
#define WFB_SYNC_REF             8600    //@ 1.72ms = 50000 * 0.172

#define WFB_SLAVE_TX_POS_OFFS    10000   //@ 2.0ms = 50000 * 0.20, Offset for slave 0
#define WFB_SLAVE_TX_POS_SHIFT   10000   //@ 2.0ms, every slave is shifted by this value

#define WFB_MASTER_DATA_BYTES_BASIC       2  // 1 general command + 1 service
#define WFB_MASTER_DATA_BYTES_PER_SLAVE   5  // 1 individual command + 4 position
#define WFB_MASTER_DATA_BYTES_HMAC        4

#define INSYNC_CNT_DIFF         250     //50us = 50000 * 0.005
#define INSYNC_CNT_LEN          20
#define INSYNC_MAX_LOST_FRAMES  3


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void MRF24J40_updhandler(void);
extern void MRF24J40_txhandler(void);
extern void MRF24J40_rxhandler(uint32_t syncact, uint32_t syncmax, int32_t* syncval_int, uint32_t* syncval_fract);
extern void delay_ms(uint32_t ms);
extern uint8_t MRF24J40_shortAddressRead(uint8_t address);
extern void MRF24J40_shortAddressWrite(uint8_t address, uint8_t value);
extern uint8_t MRF24J40_longAddressRead(uint16_t address);
extern void MRF24J40_longAddressWrite(uint16_t address, uint8_t value);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_1N_Pin LL_GPIO_PIN_13
#define PWM_1N_GPIO_Port GPIOC
#define VBUS_Pin LL_GPIO_PIN_0
#define VBUS_GPIO_Port GPIOA
#define POTI_Pin LL_GPIO_PIN_12
#define POTI_GPIO_Port GPIOB
#define TEMP_Pin LL_GPIO_PIN_14
#define TEMP_GPIO_Port GPIOB
#define PWM_3N_Pin LL_GPIO_PIN_15
#define PWM_3N_GPIO_Port GPIOB
#define LED_Pin LL_GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#define PWM_1_Pin LL_GPIO_PIN_8
#define PWM_1_GPIO_Port GPIOA
#define PWM_2_Pin LL_GPIO_PIN_9
#define PWM_2_GPIO_Port GPIOA
#define PWM_3_Pin LL_GPIO_PIN_10
#define PWM_3_GPIO_Port GPIOA
#define PWM_2N_Pin LL_GPIO_PIN_12
#define PWM_2N_GPIO_Port GPIOA
#define J3_PWM_Pin LL_GPIO_PIN_15
#define J3_PWM_GPIO_Port GPIOA
#define BUTTON_Pin LL_GPIO_PIN_10
#define BUTTON_GPIO_Port GPIOC
#define ENC_A_Pin LL_GPIO_PIN_6
#define ENC_A_GPIO_Port GPIOB
#define ENC_B_Pin LL_GPIO_PIN_7
#define ENC_B_GPIO_Port GPIOB
#define ENC_Z_Pin LL_GPIO_PIN_8
#define ENC_Z_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
