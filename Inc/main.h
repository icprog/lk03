/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TDC_Power_Ctl_Pin GPIO_PIN_13
#define TDC_Power_Ctl_GPIO_Port GPIOC
#define TDC_Sighal_AngleMax_Pin GPIO_PIN_0
#define TDC_Sighal_AngleMax_GPIO_Port GPIOC
#define Rx_Power_Ctl_Pin GPIO_PIN_0
#define Rx_Power_Ctl_GPIO_Port GPIOA
#define TDC_rx_pwmHv_Pin GPIO_PIN_1
#define TDC_rx_pwmHv_GPIO_Port GPIOA
#define boot_vol_ctl_Pin GPIO_PIN_2
#define boot_vol_ctl_GPIO_Port GPIOA
#define ADJ_COMP_Pin GPIO_PIN_4
#define ADJ_COMP_GPIO_Port GPIOA
#define tlc5618_clk_Pin GPIO_PIN_5
#define tlc5618_clk_GPIO_Port GPIOA
#define tlc5618_dat_Pin GPIO_PIN_7
#define tlc5618_dat_GPIO_Port GPIOA
#define GP21_INTN_Pin GPIO_PIN_4
#define GP21_INTN_GPIO_Port GPIOC
#define GP21_INTN_EXTI_IRQn EXTI4_IRQn
#define GP21_EN_Stop1_Pin GPIO_PIN_5
#define GP21_EN_Stop1_GPIO_Port GPIOC
#define GP21_EN_Start_Pin GPIO_PIN_0
#define GP21_EN_Start_GPIO_Port GPIOB
#define GP21_RSTN_Pin GPIO_PIN_1
#define GP21_RSTN_GPIO_Port GPIOB
#define GP21_EN_Stop2_Pin GPIO_PIN_10
#define GP21_EN_Stop2_GPIO_Port GPIOB
#define GP21_NSS_Pin GPIO_PIN_12
#define GP21_NSS_GPIO_Port GPIOB
#define TX_Vol_Ctrl3_Pin GPIO_PIN_6
#define TX_Vol_Ctrl3_GPIO_Port GPIOC
#define TX_Vol_Ctrl2_Pin GPIO_PIN_7
#define TX_Vol_Ctrl2_GPIO_Port GPIOC
#define TX_Vol_Ctrl1_Pin GPIO_PIN_8
#define TX_Vol_Ctrl1_GPIO_Port GPIOC
#define TDC_Signal_Pin GPIO_PIN_9
#define TDC_Signal_GPIO_Port GPIOC
#define TDC_TX_Pin GPIO_PIN_9
#define TDC_TX_GPIO_Port GPIOA
#define TDC_RX_Pin GPIO_PIN_10
#define TDC_RX_GPIO_Port GPIOA
#define tlc5618_cs_Pin GPIO_PIN_11
#define tlc5618_cs_GPIO_Port GPIOA
#define High_Vol_Ctl_Pin GPIO_PIN_12
#define High_Vol_Ctl_GPIO_Port GPIOA
#define senor_switch_Pin GPIO_PIN_10
#define senor_switch_GPIO_Port GPIOC
#define TDC_Sighal_AngleReles_Pin GPIO_PIN_12
#define TDC_Sighal_AngleReles_GPIO_Port GPIOC
#define Laser_Light_Pin GPIO_PIN_2
#define Laser_Light_GPIO_Port GPIOD
#define sensor_rs485_dir_Pin GPIO_PIN_8
#define sensor_rs485_dir_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
