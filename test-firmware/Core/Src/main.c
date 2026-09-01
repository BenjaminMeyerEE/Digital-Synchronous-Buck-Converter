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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* TIM1 runs from the 170 MHz APB2 timer clock with PSC = 0, so the switching
   frequency is 170e6 / (ARR + 1). The .ioc still says 1699/850 (100 kHz, 50 %)
   and is left that way on purpose -- these values are applied on top of the
   generated init in USER CODE BEGIN TIM1_Init 2, so regenerating from CubeMX
   never silently changes the switching frequency or the start-up duty. */
#define PWM_ARR_TICKS         424U  /* 170 MHz / (424 + 1) = 400 kHz */
#define PWM_TARGET_PULSE      212U  /* ~50 % duty at ARR 424 */

/* Nothing switches for this long after reset. Gives the rails time to settle
   and the debugger time to take hold before the half-bridge is energized. */
#define STARTUP_HOLD_MS       500U

/* Soft start: walk CCR up in small steps instead of stepping straight to 50 %.
   4 counts every 2 ms -> ~106 ms from 0 % to target. */
#define SOFTSTART_STEP        4U
#define SOFTSTART_STEP_MS     2U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* Written by BSP_PB_Callback() from EXTI15_10_IRQHandler, read by the main
   loop -- both must be volatile. */
static volatile uint8_t  power_stage_enabled = 0U;
static volatile uint32_t pwm_pulse           = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

static void PowerStage_Enable(void);
static void PowerStage_Disable(void);

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* Deliberately NOT starting the outputs here. MOE stays clear, so with OSSI
     enabled both TIM1_CH1 (PA8) and TIM1_CH1N (PA7) sit in their idle state
     (low, both FETs off). The half-bridge is energized from the main loop by
     PowerStage_Enable(), after STARTUP_HOLD_MS and the BSP init below. */

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Quiet hold before anything switches. */
  HAL_Delay(STARTUP_HOLD_MS);

  PowerStage_Enable();

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (power_stage_enabled != 0U)
    {
      if (pwm_pulse < PWM_TARGET_PULSE)
      {
        /* Soft start: ramp the duty so the input rail sees a gradual load
           instead of a 50 % step. LD2 stays solid while ramping. */
        pwm_pulse += SOFTSTART_STEP;
        if (pwm_pulse > PWM_TARGET_PULSE)
        {
          pwm_pulse = PWM_TARGET_PULSE;
        }
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_pulse);
        HAL_Delay(SOFTSTART_STEP_MS);
      }
      else
      {
        /* At target duty: slow blink. */
        BSP_LED_Toggle(LED_GREEN);
        HAL_Delay(500U);
      }
    }
    else
    {
      /* Outputs off: fast blink. */
      BSP_LED_Toggle(LED_GREEN);
      HAL_Delay(100U);
    }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1699;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 850;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 17;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* Apply the values this project actually runs, on top of whatever the .ioc
     generated above. Doing it here rather than editing the generated code (or
     the .ioc) keeps CubeMX regeneration a no-op.
       ARR  424 -> 400 kHz switching
       CCR1 0   -> high side stays off until the soft-start ramp runs */
  __HAL_TIM_SET_AUTORELOAD(&htim1, PWM_ARR_TICKS);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);

  /* ARPE and OC1PE are both enabled, so the writes above only land in the
     preload registers. HAL_TIM_Base_Init() already issued a UG that latched
     the generated 1699/850 into the shadows, so force another one to latch the
     new values now. Without this the first PWM period would run with
     CCR1 = 850 against ARR = 424 -- i.e. 100 % duty, high side full on. */
  htim1.Instance->EGR = TIM_EGR_UG;

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Energize the half-bridge at 0 % duty.
  * @note   CCR is forced to 0 before MOE is set, so the high side never turns
  *         on at start-up. The main loop ramps the duty from there.
  * @retval None
  */
static void PowerStage_Enable(void)
{
  pwm_pulse = 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  BSP_LED_On(LED_GREEN);
  power_stage_enabled = 1U;
}

/**
  * @brief  Stop switching and return both outputs to their idle state.
  * @note   Stopping both channels clears MOE, so with OSSI enabled PA7/PA8 are
  *         driven low and both FETs turn off. This is a bring-up convenience,
  *         not a hardware safety interlock -- a real cutoff belongs on TIM1
  *         BRK, which is currently disabled.
  * @retval None
  */
static void PowerStage_Disable(void)
{
  power_stage_enabled = 0U;

  (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

  pwm_pulse = 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
}

/**
  * @brief  USER button handler -- overrides the __weak BSP implementation.
  *         Acts as a stop button for the power stage.
  * @param  Button the button that fired
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    PowerStage_Disable();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Kill the outputs first -- a fault must never leave the half-bridge
     switching. htim1.Instance is NULL if we faulted before MX_TIM1_Init(). */
  if (htim1.Instance != NULL)
  {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  }

  __disable_irq();

  /* Blink LD2 (PA5) so a fault is visible without a debugger attached. Driven
     straight through the registers: BSP_LED_Init() may not have run yet, and
     HAL_Delay() is dead with interrupts off. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE5_Msk)
                 | (0x1UL << GPIO_MODER_MODE5_Pos);

  while (1)
  {
    GPIOA->ODR ^= GPIO_ODR_OD5;
    for (volatile uint32_t i = 0U; i < 2000000U; i++)
    {
    }
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
