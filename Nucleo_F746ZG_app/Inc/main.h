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
#include "stm32f7xx_hal.h"

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
#define IMU_INT_Pin GPIO_PIN_2
#define IMU_INT_GPIO_Port GPIOE
#define IMU_RESET_Pin GPIO_PIN_3
#define IMU_RESET_GPIO_Port GPIOE
#define HALL2_TIM9_CH1_Pin GPIO_PIN_5
#define HALL2_TIM9_CH1_GPIO_Port GPIOE
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define DRV_INA_Pin GPIO_PIN_1
#define DRV_INA_GPIO_Port GPIOF
#define US3_ECHO_TIM10_CH1_Pin GPIO_PIN_6
#define US3_ECHO_TIM10_CH1_GPIO_Port GPIOF
#define US2_ECHO_TIM11_CH1_Pin GPIO_PIN_7
#define US2_ECHO_TIM11_CH1_GPIO_Port GPIOF
#define US1_ECHO_TIM13_CH1_Pin GPIO_PIN_8
#define US1_ECHO_TIM13_CH1_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define DRV_PWM_TIM2_CH1_Pin GPIO_PIN_0
#define DRV_PWM_TIM2_CH1_GPIO_Port GPIOA
#define SUP_ADC3_IN3_Pin GPIO_PIN_3
#define SUP_ADC3_IN3_GPIO_Port GPIOA
#define DRV_I1_ADC1_IN4_Pin GPIO_PIN_4
#define DRV_I1_ADC1_IN4_GPIO_Port GPIOA
#define DRV_I2_ADC1_IN5_Pin GPIO_PIN_5
#define DRV_I2_ADC1_IN5_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define DRV_INB_Pin GPIO_PIN_0
#define DRV_INB_GPIO_Port GPIOG
#define SERVO_PWM_TIM1_CH1_Pin GPIO_PIN_9
#define SERVO_PWM_TIM1_CH1_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define HALL1_TIM12_CH1_Pin GPIO_PIN_15
#define HALL1_TIM12_CH1_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define US1_TRIG_TIM4_CH1_Pin GPIO_PIN_12
#define US1_TRIG_TIM4_CH1_GPIO_Port GPIOD
#define US2_TRIG_TIM4_CH2_Pin GPIO_PIN_13
#define US2_TRIG_TIM4_CH2_GPIO_Port GPIOD
#define US3_TRIG_TIM4_CH3_Pin GPIO_PIN_14
#define US3_TRIG_TIM4_CH3_GPIO_Port GPIOD
#define BT_FACTORY_DEFAULTS_Pin GPIO_PIN_15
#define BT_FACTORY_DEFAULTS_GPIO_Port GPIOD
#define BT_AUTO_CONNECT_Pin GPIO_PIN_2
#define BT_AUTO_CONNECT_GPIO_Port GPIOG
#define BT_nRESET_Pin GPIO_PIN_3
#define BT_nRESET_GPIO_Port GPIOG
#define BT_UART_BAUD_FORCE_Pin GPIO_PIN_4
#define BT_UART_BAUD_FORCE_GPIO_Port GPIOG
#define BT_AUTO_MASTER_MODE_Pin GPIO_PIN_5
#define BT_AUTO_MASTER_MODE_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define BT_USART6_RTS_Pin GPIO_PIN_8
#define BT_USART6_RTS_GPIO_Port GPIOG
#define BT_USART6_TX_Pin GPIO_PIN_6
#define BT_USART6_TX_GPIO_Port GPIOC
#define BT_USART6_RX_Pin GPIO_PIN_7
#define BT_USART6_RX_GPIO_Port GPIOC
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LIDAR_USART2_TX_Pin GPIO_PIN_5
#define LIDAR_USART2_TX_GPIO_Port GPIOD
#define LIDAR_USART2_RX_Pin GPIO_PIN_6
#define LIDAR_USART2_RX_GPIO_Port GPIOD
#define SB_nSENSE_INT_Pin GPIO_PIN_10
#define SB_nSENSE_INT_GPIO_Port GPIOG
#define SB_nEXP_RESET_Pin GPIO_PIN_12
#define SB_nEXP_RESET_GPIO_Port GPIOG
#define BT_USART6_CTS_Pin GPIO_PIN_15
#define BT_USART6_CTS_GPIO_Port GPIOG
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define LIDAR_MOTCTRL_Pin GPIO_PIN_4
#define LIDAR_MOTCTRL_GPIO_Port GPIOB
#define SB_nLED_EN_Pin GPIO_PIN_6
#define SB_nLED_EN_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define SB_I2C1_SCL_Pin GPIO_PIN_8
#define SB_I2C1_SCL_GPIO_Port GPIOB
#define SB_I2C1_SDA_Pin GPIO_PIN_9
#define SB_I2C1_SDA_GPIO_Port GPIOB
#define IMU_UART8_RX_Pin GPIO_PIN_0
#define IMU_UART8_RX_GPIO_Port GPIOE
#define IMU_UART8_TX_Pin GPIO_PIN_1
#define IMU_UART8_TX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
