/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FS-AI CAN Communication System
  *                   STM32F1 with MCP2515 SPI-to-CAN module
  *
  * This is the main application entry point for AI algorithm integration.
  * The AI algorithm uses simple function calls to:
  *   - Set control values (throttle, steering, brakes)
  *   - Get vehicle feedback (wheel speeds, steering, etc.)
  *   - Check communication health
  *
  * All CAN communication is abstracted away by ai_controller.h
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "ai_controller.h"

/* ============================================================================
   PRIVATE DATA
   ============================================================================ */

/* HAL Handles */
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

/* Timing */
static uint32_t last_led_update_ms = 0;
static uint32_t last_recovery_check_ms = 0;

/* Status flags */
static uint8_t ai_is_operational = 0;

/* ============================================================================
   FORWARD DECLARATIONS
   ============================================================================ */

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

/* ============================================================================
   PRIVATE FUNCTIONS
   ============================================================================ */

/**
 * @brief Update LED indicator based on system health
 *
 * LED behavior:
 *   - SOLID ON: System healthy and operational
 *   - SLOW BLINK (500ms): Waiting for handshake or recovering
 *   - FAST BLINK (200ms): Communication error
 */
static void led_indicator_update(uint32_t current_time_ms)
{
    static uint32_t last_toggle_time = 0;
    uint32_t blink_period = 500;  /* Default: slow blink */

    if ((current_time_ms - last_led_update_ms) >= 100) {
        last_led_update_ms = current_time_ms;

        /* Determine LED pattern based on system state */
        ai_control_state_t state = ai_controller_get_state();

        switch (state) {
            case AI_CONTROL_STATE_READY:
                /* Solid on - system ready */
                LED_ON();
                break;

            case AI_CONTROL_STATE_RUNNING:
                /* Solid on - system running */
                LED_ON();
                break;

            case AI_CONTROL_STATE_IDLE:
                /* Slow blink - waiting for handshake */
                blink_period = 500;
                if ((current_time_ms - last_toggle_time) >= blink_period) {
                    LED_TOGGLE();
                    last_toggle_time = current_time_ms;
                }
                break;

            case AI_CONTROL_STATE_FAULT:
                /* Fast blink - communication error */
                blink_period = 200;
                if ((current_time_ms - last_toggle_time) >= blink_period) {
                    LED_TOGGLE();
                    last_toggle_time = current_time_ms;
                }
                break;

            default:
                LED_OFF();
                break;
        }
    }
}

/**
 * @brief Periodic recovery check
 *
 * Checks for and recovers from CAN bus errors every 100ms.
 */
static void recovery_check(uint32_t current_time_ms)
{
    if ((current_time_ms - last_recovery_check_ms) >= 100) {
        last_recovery_check_ms = current_time_ms;

        /* MCP2515 recovery is called internally by transport layer */
        /* This is just for diagnostics if needed */
    }
}

/**
 * @brief Example AI algorithm
 *
 * Replace this with your actual AI/control algorithm.
 * This simple example demonstrates the API.
 */
static void ai_algorithm(uint32_t current_time_ms)
{
    fs_ai_vcu_data_t vcu_data;

    /* Get latest vehicle state from VCU */
    if (!ai_get_vehicle_state(&vcu_data)) {
        /* Vehicle state not available yet */
        return;
    }

    /* Example: Simple logic to control vehicle */
    float wheel_speed_avg = ai_get_average_wheel_speed();
    float steering_feedback = ai_get_steering_feedback();

    /* Your AI algorithm goes here */
    /* For now: simple demonstration */

    /* Adaptive throttle based on wheel speed */
    float throttle_nm = 50.0f;      /* Default: 50 Nm torque */
    if (wheel_speed_avg > 3000.0f) {
        throttle_nm = 30.0f;  /* Reduce throttle at high speed */
    }

    /* Dampen steering oscillation */
    float steering_deg = steering_feedback * 0.1f+1.0;

    float brake_percent = 5.0f;     /* No braking */
    float motor_speed_rpm = 2000.0f;  /* 2000 RPM */

    /* Set control values */
    ai_set_control(throttle_nm, steering_deg, brake_percent, motor_speed_rpm);

    /* Set mission status */
    ai_set_mission_status(
        MISSION_RUNNING,
        DIRECTION_FORWARD,
        ESTOP_NO
    );

    /* Set lap counter (example: lap 1, no cones hit) */
    ai_set_mission_progress(1, 0);

    /* Optional: Send dynamics data for logging */
    ai_set_dynamics(0.5f, 0.1f, 5.0f);  /* Small accelerations */
}

/* ============================================================================
   SYSTEM INITIALIZATION
   ============================================================================ */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();

    /* Initialize AI controller and CAN communication */
    if (!ai_controller_init()) {
        Error_Handler();
    }

    /* Initialize timing variables */
    last_led_update_ms = HAL_GetTick();
    last_recovery_check_ms = HAL_GetTick();

    /* ========================================================================
       MAIN CONTROL LOOP
       ======================================================================== */
    while (1) {
        uint32_t current_time_ms = HAL_GetTick();

        /* ===== UPDATE CAN TRANSPORT (handles all timing) ===== */
        ai_controller_update(current_time_ms);

        /* ===== RUN AI ALGORITHM ===== */
        /* Your AI algorithm runs here every iteration.
           The transport layer internally manages:
           - 10ms transmit cycle
           - Message reception and decoding
           - Handshake and error handling
        */
        ai_algorithm(current_time_ms);

        /* ===== SYSTEM HEALTH MONITORING ===== */
        if (ai_is_healthy()) {
            ai_is_operational = 1;
        } else {
            ai_is_operational = 0;
        }

        /* ===== PERIODIC TASKS ===== */
        led_indicator_update(current_time_ms);
        recovery_check(current_time_ms);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
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

    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure LED pin (PC13) */
    HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_Port, &GPIO_InitStruct);

    /* Configure CAN CS pin (PA4) */
    HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = CAN_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAN_CS_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Blink LED rapidly to indicate error */
        LED_TOGGLE();
        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add their own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
