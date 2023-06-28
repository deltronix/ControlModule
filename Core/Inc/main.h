/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_IN_Pin GPIO_PIN_13
#define START_IN_GPIO_Port GPIOC
#define RESET_IN_Pin GPIO_PIN_14
#define RESET_IN_GPIO_Port GPIOC
#define CLK_OUT_Pin GPIO_PIN_15
#define CLK_OUT_GPIO_Port GPIOC
#define USB_OTG_OC_PU_Pin GPIO_PIN_0
#define USB_OTG_OC_PU_GPIO_Port GPIOC
#define ENCB_B_Pin GPIO_PIN_1
#define ENCB_B_GPIO_Port GPIOC
#define ENCB_A_Pin GPIO_PIN_2
#define ENCB_A_GPIO_Port GPIOC
#define ENCB_SW_Pin GPIO_PIN_3
#define ENCB_SW_GPIO_Port GPIOC
#define SPI2_DISP_RST_Pin GPIO_PIN_10
#define SPI2_DISP_RST_GPIO_Port GPIOB
#define SPI2_DISP_A0_Pin GPIO_PIN_11
#define SPI2_DISP_A0_GPIO_Port GPIOB
#define SPI2_DISP_CS_Pin GPIO_PIN_12
#define SPI2_DISP_CS_GPIO_Port GPIOB
#define SPI2_FLASH_CS_Pin GPIO_PIN_6
#define SPI2_FLASH_CS_GPIO_Port GPIOC
#define ENCA_SW_Pin GPIO_PIN_7
#define ENCA_SW_GPIO_Port GPIOC
#define ENCA_B_Pin GPIO_PIN_8
#define ENCA_B_GPIO_Port GPIOC
#define ENCA_A_Pin GPIO_PIN_9
#define ENCA_A_GPIO_Port GPIOC
#define SPI3_SYNC_Pin GPIO_PIN_15
#define SPI3_SYNC_GPIO_Port GPIOA
#define SPI3_RCLK_Pin GPIO_PIN_2
#define SPI3_RCLK_GPIO_Port GPIOD
#define SPI1_RCLK_Pin GPIO_PIN_6
#define SPI1_RCLK_GPIO_Port GPIOB
#define USB_OTG_VBUS_ENABLE_Pin GPIO_PIN_9
#define USB_OTG_VBUS_ENABLE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
