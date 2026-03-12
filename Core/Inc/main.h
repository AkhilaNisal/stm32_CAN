/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32f1xx_hal.h"

/* Private includes */
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types */
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants */
/* USER CODE BEGIN EC */

/* AI Control Configuration */
#define AI_CONTROL_FREQ_HZ          100         /* 100 Hz control loop */
#define AI_CONTROL_PERIOD_MS        10          /* 10 ms per cycle */

/* Motor/Drivetrain Configuration */
#define MOTOR_MAX_RPM               4000.0f
#define MOTOR_MAX_TORQUE_NM         195.0f
#define MOTOR_MIN_TORQUE_NM         0.0f

/* Steering Configuration */
#define STEER_MAX_ANGLE_DEG         21.0f
#define STEER_MIN_ANGLE_DEG         -21.0f
#define STEER_ANGLE_RESOLUTION_DEG 0.1f

/* Brake Configuration */
#define BRAKE_MAX_PRESSURE_PCT      100.0f
#define BRAKE_MIN_PRESSURE_PCT      0.0f
#define BRAKE_PRESSURE_RESOLUTION_PCT 0.5f

/* CAN Communication Timeouts */
#define CAN_RX_TIMEOUT_MS           200         /* Message timeout */
#define CAN_WATCHDOG_INTERVAL_MS    50          /* Watchdog check interval */
#define CAN_HANDSHAKE_TIMEOUT_MS    100         /* Initial handshake timeout */

/* LED Status Indicator */
#define LED_HEALTHY_PERIOD_MS       500         /* Blink period when healthy */
#define LED_FAULT_PERIOD_MS         200         /* Faster blink when faulted */

/* USER CODE END EC */

/* Exported macro */
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes */
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines */
#define CAN_CS_Pin                  GPIO_PIN_4
#define CAN_CS_GPIO_Port            GPIOA
#define LED_Pin                     GPIO_PIN_13
#define LED_Port                    GPIOC

/* USER CODE BEGIN Private defines */

/* LED Control Macros */
#define LED_ON()                    HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF()                   HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_SET)
#define LED_TOGGLE()                HAL_GPIO_TogglePin(LED_Port, LED_Pin)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
