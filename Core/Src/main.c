/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

#include "CANSPI.h"
#include "fs-ai_api_protocol.h"
//#include "fs-ai_api_protocol.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);


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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


//  uCAN_MSG rxMessage;

  // this is for as_ai_protocol
//  VCU_Status_t my_vcu_status = {0};
//  uCAN_MSG rxMessage; // This is the variable the compiler said was unused



 // this is for fs-ai_api_protocol
  uCAN_MSG rxMsg;                              /* CAN message buffer */
  fs_ai_api_vcu2ai vcu_data;                   /* VCU to AI data */
  fs_ai_api_ai2vcu ai_data;                    /* AI to VCU data */
  fs_ai_api_can_stats_t can_stats;             /* CAN statistics */

  uint32_t last_send_time_ms = 0;
  uint32_t current_time_ms = 0;
  const uint32_t SEND_INTERVAL_MS = 10;        /* Send messages every 10ms */

  //  CANSPI_Initialize();
  HAL_Delay(1000);
  /* USER CODE END 2 */
//  if(CANSPI_Initialize())
//  {
//      // Initialization successful → turn LED ON
//      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // For Blue Pill: LOW = LED ON
//  }
//  else
//  {
//      // Initialization failed → blink LED to indicate error
//      while(1)
//      {
//          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//          HAL_Delay(500);
//      }
//  }


  /* Initialize FS-AI API */
     if (!fs_ai_api_init()) {
         /* Initialization failed - handle error */
    	 while(1)
    	      {
    	          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    	          HAL_Delay(500);
    	      }
     }

     /* Initialize timing */
     last_send_time_ms = HAL_GetTick();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	  HAL_Delay(1000);

//	  	  txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
//	      txMessage.frame.id = 0x127; // ID can be between Hex1 and Hex7FF (1-2047 decimal)
//	      txMessage.frame.dlc = 8;
//	      txMessage.frame.data0 = 'S';
//	      txMessage.frame.data1 = 'T';
//	      txMessage.frame.data2 = 'M';
//	      txMessage.frame.data3 = '3';
//	      txMessage.frame.data4 = '2';
//	      txMessage.frame.data5 = '-';
//	      txMessage.frame.data6 = '-';
//	      txMessage.frame.data7 = '.';
//	      CANSPI_Transmit(&txMessage);
//          HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//
//	      HAL_Delay(1000);

	  /* 1. Receive Part */
	  // this is for fs_ai_protocol
//	          if (CANSPI_Receive(&rxMessage)) {
//	              FS_AI_ProcessIncoming(&rxMessage);
//	          }
//
//	          /* 2. Update Heartbeat / Handshake */
//	          // Toggle handshake every loop or based on a timer
//	          static uint32_t last_tick = 0;
//	          if (HAL_GetTick() - last_tick >= 10) { // 10ms Transmission Rate
//	              last_tick = HAL_GetTick();
//
//	              my_vcu_status.handshake ^= 1;
//	              my_vcu_status.as_state = AS_READY; // Example state
//	              my_vcu_status.ami_state = 1; // Example state
//
//	              FS_AI_TransmitStatus(&my_vcu_status);
//	          }



	  // this is for fs-ai_api_protocol
	  /* Get current time */
	          current_time_ms = HAL_GetTick();

	          /* ===== CHECK FOR RECEIVED CAN MESSAGES ===== */
	          if (CANSPI_messagesInBuffer() > 0) {
	              if (CANSPI_Receive(&rxMsg) > 0) {
	                  /* Process the received CAN message */
	                  fs_ai_api_process_incoming(&rxMsg);
	              }
	          }

	          /* ===== RETRIEVE VCU DATA ===== */
	          fs_ai_api_vcu2ai_get_data(&vcu_data);

	          /* Example: Check vehicle state */
	          if (vcu_data.VCU2AI_AS_STATE == AS_DRIVING) {
	              /* Vehicle is in autonomous driving mode */

	              /* Access received data from VCU */
	              float wheel_speed_fl = vcu_data.VCU2AI_FL_WHEEL_SPEED_rpm;
	              float wheel_speed_fr = vcu_data.VCU2AI_FR_WHEEL_SPEED_rpm;
	              float steer_angle = vcu_data.VCU2AI_STEER_ANGLE_deg;
	              float brake_pressure_f = vcu_data.VCU2AI_BRAKE_PRESS_F_pct;

	              /* Your AI control logic here */
	              /* Calculate desired values based on sensor inputs */

	              /* Prepare data to send to VCU */
	              ai_data.AI2VCU_MISSION_STATUS = MISSION_RUNNING;
	              ai_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_FORWARD;
	              ai_data.AI2VCU_HANDSHAKE_SEND_BIT = vcu_data.VCU2AI_HANDSHAKE_RECEIVE_BIT;

	              /* Set control requests */
	              ai_data.AI2VCU_STEER_ANGLE_REQUEST_deg = 5.5f;         /* 5.5° */
	              ai_data.AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm = 1500.0f; /* 1500 RPM */
	              ai_data.AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm = 1500.0f;
	              ai_data.AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm = 50.0f;    /* 50 Nm */
	              ai_data.AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm = 50.0f;
	              ai_data.AI2VCU_BRAKE_PRESS_F_REQUEST_pct = 0.0f;        /* No braking */
	              ai_data.AI2VCU_BRAKE_PRESS_R_REQUEST_pct = 0.0f;

	              /* Logging data (for data recording) */
	              ai_data.AI2LOG_ACCEL_LONGITUDINAL_ms2 = 2;
	              ai_data.AI2LOG_ACCEL_LATERAL_ms2 = 1;
	              ai_data.AI2LOG_YAW_RATE_degps = 5;

	              /* Update AI data */
	              fs_ai_api_ai2vcu_set_data(&ai_data);
	          }
	          else if (vcu_data.VCU2AI_AS_STATE == AS_READY) {
	              /* Vehicle is ready but not driving yet */

	              /* Mirror the handshake bit */
	              ai_data.AI2VCU_HANDSHAKE_SEND_BIT = vcu_data.VCU2AI_HANDSHAKE_RECEIVE_BIT;
	              ai_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
	              ai_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL;

	              /* Zero all control requests */
	              ai_data.AI2VCU_STEER_ANGLE_REQUEST_deg = 0.0f;
	              ai_data.AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm = 0.0f;
	              ai_data.AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm = 0.0f;
	              ai_data.AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm = 0.0f;
	              ai_data.AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm = 0.0f;
	              ai_data.AI2VCU_BRAKE_PRESS_F_REQUEST_pct = 0.0f;
	              ai_data.AI2VCU_BRAKE_PRESS_R_REQUEST_pct = 0.0f;

	              fs_ai_api_ai2vcu_set_data(&ai_data);
	          }
	          else if (vcu_data.VCU2AI_AS_STATE == AS_EMERGENCY_BRAKE) {
	              /* Emergency braking activated */

	              /* Zero torque, apply maximum braking */
	              ai_data.AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm = 0.0f;
	              ai_data.AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm = 0.0f;
	              ai_data.AI2VCU_BRAKE_PRESS_F_REQUEST_pct = 100.0f;  /* Maximum braking */
	              ai_data.AI2VCU_BRAKE_PRESS_R_REQUEST_pct = 100.0f;

	              fs_ai_api_ai2vcu_set_data(&ai_data);
	          }
	          else {
	              /* AS_OFF or AS_FINISHED - zero all commands */
	              ai_data.AI2VCU_STEER_ANGLE_REQUEST_deg = 0.0f;
	              ai_data.AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm = 0.0f;
	              ai_data.AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm = 0.0f;
	              ai_data.AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm = 0.0f;
	              ai_data.AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm = 0.0f;
	              ai_data.AI2VCU_BRAKE_PRESS_F_REQUEST_pct = 0.0f;
	              ai_data.AI2VCU_BRAKE_PRESS_R_REQUEST_pct = 0.0f;
	              ai_data.AI2VCU_MISSION_STATUS = MISSION_NOT_SELECTED;

	              fs_ai_api_ai2vcu_set_data(&ai_data);
	          }

	          /* ===== SEND MESSAGES AT 10ms INTERVAL ===== */
	          if ((current_time_ms - last_send_time_ms) >= SEND_INTERVAL_MS) {
	              last_send_time_ms = current_time_ms;

	              /* Send all AI to VCU messages */
	              fs_ai_api_send_all_messages();
	          }

	          /* ===== OPTIONAL: CHECK COMMUNICATIONS HEALTH ===== */
//	          if (current_time_ms % 1000 == 0) {  /* Every 1 second */
//	              fs_ai_api_get_can_stats(&can_stats);
//
//	              /* Check if handshake is complete */
//	              if (fs_ai_api_is_handshake_complete()) {
//	                  /* Handshake successful */
//	              }
//
//	              /* Check if communications are healthy */
//	              if (fs_ai_api_is_comms_healthy()) {
//	                  /* All messages received successfully */
//	              }
//	              else {
//	                  /* Communication issue detected */
//	                  Error_Handler();
//	              }
//	          }
	          /* ===== OPTIONAL: CHECK COMMUNICATIONS HEALTH (NON-FATAL) ===== */
	          static uint32_t last_health_check_ms = 0;
	          static uint8_t comms_fault = 0;

	          if ((current_time_ms - last_health_check_ms) >= 1000) {   // every 1 second
	              last_health_check_ms = current_time_ms;

	              fs_ai_api_get_can_stats(&can_stats);

	              /* IMPORTANT:
	                 During your Arduino-only debug, you are NOT receiving VCU2AI frames,
	                 so fs_ai_api_is_comms_healthy() will be false. Do NOT stop the MCU.
	              */
	              if (!fs_ai_api_is_comms_healthy()) {
	                  comms_fault = 1;

	                  // Example debug indicator: blink LED quickly instead of freezing
	                  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	              } else {
	                  comms_fault = 0;
	              }

	              // If you want: reset stats every second (optional)
	              // fs_ai_api_clear_can_stats();
	          }


	          static uint32_t last_recover_ms = 0;
	          if ((current_time_ms - last_recover_ms) >= 100) {
	              last_recover_ms = current_time_ms;
	              CANSPI_RecoverIfNeeded();
	          }





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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_CS_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
