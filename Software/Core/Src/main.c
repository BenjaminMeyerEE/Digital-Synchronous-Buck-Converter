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
/* sys_state_t moved to main.h (USER CODE ET): it is now shared with the lean
 * DMA error path in stm32g4xx_it.c, which latches SYS_STATE_FAULT directly. */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- Power stage / timing --------------------------------------------------
 * TIM1: 170 MHz, PSC = 0, ARR = 424 -> Fsw = 400 kHz, period = 2.5 us.
 * DTG = 17 -> hardware dead-time = 17 / 170 MHz = 100 ns per switching edge. */
#define PWM_PERIOD_TICKS        425.0f      /* ARR + 1                                    */
#define CTRL_TS_SEC             2.5e-6f     /* control ISR period (TIM1 update = 400 kHz) */
#define PWM_DEADTIME_NS         100.0f
#define PWM_PERIOD_NS           2500.0f
/* LS on-time loses one dead-time at each of its two transitions -> 8 % of period */
#define PWM_DEADTIME_PCT        (2.0f * PWM_DEADTIME_NS / PWM_PERIOD_NS * 100.0f)
#define CCR_TO_PCT              (100.0f / PWM_PERIOD_TICKS)

/* --- Analog scaling (precomputed reciprocals: no division in the ISR) ------ */
#define ADC_VREF_V              3.3f
#define ADC_FULL_SCALE          4095.0f
#define VFB_DIVIDER_GAIN        0.4047619f  /* V_FB divider: ADC sees Vout * this         */
#define CURR_SENSE_V_PER_A      0.25f       /* INA241A3 gain 50 x 5 mOhm shunt            */
#define ADC_RAW_TO_VOUT_V       ((ADC_VREF_V / ADC_FULL_SCALE) / VFB_DIVIDER_GAIN)   /* ~1.9910e-3 V/LSB */
#define ADC_RAW_TO_AMPS         ((ADC_VREF_V / ADC_FULL_SCALE) / CURR_SENSE_V_PER_A) /* ~3.2234e-3 A/LSB */
#define AMPS_TO_DAC_COUNTS      (CURR_SENSE_V_PER_A * ADC_FULL_SCALE / ADC_VREF_V)   /* ~310.23 LSB/A    */
#define DAC_FULL_SCALE_A        13.2f       /* 3.3 V / 0.25 V/A: max representable trip   */

/* --- Limits / interlocks ---------------------------------------------------- */
#define OC_TRIP_MIN_A           0.5f        /* start refused while set_oc_trip_A < this   */
#define DUTY_MAX_ABS            0.98f       /* absolute ceiling for set_duty_max          */
#define VOUT_SET_MAX_V          8.0f        /* V_FB divider saturates the ADC at ~8.15 V  */
#define SOFTSTART_MS_MIN        1.0f        /* also guards the ramp-increment division    */
#define SOFTSTART_MS_MAX        10000.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* ===================== CubeMonitor: monitoring (firmware -> user) ========= */
volatile float    mon_vout_V             = 0.0f;           /* measured output voltage [V]            */
volatile float    mon_current_A          = 0.0f;           /* measured output current [A]            */
volatile float    mon_pwm_hs_duty_pct    = 0.0f;           /* high-side effective duty [%]           */
volatile float    mon_pwm_ls_duty_pct    = 0.0f;           /* low-side effective duty [%]            */
volatile uint8_t  mon_pwm_hs_active      = 0u;             /* 1 = MOE on and HS switching            */
volatile uint8_t  mon_pwm_ls_active      = 0u;             /* 1 = MOE on (LS driven)                 */
volatile uint8_t  mon_prot_out_state     = 0u;             /* live PROT_OUT pin level (PA11)         */
volatile uint8_t  mon_dt_out_state       = 0u;             /* live DT_OUT comparator level (PB4)     */
volatile uint32_t mon_dt_edge_count      = 0u;             /* total DT_OUT edges captured by TIM3    */
volatile uint32_t mon_dt_pulse_width_ns  = 0u;             /* last DT_OUT high-pulse width [ns]      */
volatile uint8_t  mon_sys_state          = SYS_STATE_INIT; /* sys_state_t enum                       */
volatile uint8_t  mon_fault_latched      = 0u;             /* 1 = BRK2 fault latched                 */
volatile uint8_t  mon_interlock_oc_not_armed = 0u;         /* 1 = set_oc_trip_A < 0.5 A, start blocked */
volatile uint32_t mon_ctrl_loop_count    = 0u;             /* 400 kHz control ISR heartbeat          */

/* ===================== CubeMonitor: tunables (user -> firmware) =========== */
volatile float set_vout_V       = 6.0f;   /* output voltage setpoint [V], clamp [0, 8.0]             */
volatile float set_kp           = 0.02f;  /* PI proportional gain [duty/V], clamp >= 0               */
volatile float set_ki           = 50.0f;  /* PI integral gain [duty/(V*s)], clamp >= 0               */
volatile float set_oc_trip_A    = 0.0f;   /* OC comparator trip [A]; must be >= 0.5 A to allow start */
volatile float set_dt_ref_A     = 0.1f;   /* switching-activity comparator threshold [A]             */
volatile float set_duty_max     = 0.95f;  /* HS duty ceiling [0..1], clamp [0, 0.98]                 */
volatile float set_softstart_ms = 50.0f;  /* soft-start ramp duration [ms], clamp [1, 10000]         */

/* ===================== CubeMonitor: commands (write 1; consumed by main loop) */
volatile uint8_t cmd_start       = 0u;    /* request IDLE -> SOFT_START                              */
volatile uint8_t cmd_stop        = 0u;    /* request SOFT_START/RUN -> IDLE                          */
volatile uint8_t cmd_clear_fault = 0u;    /* request FAULT -> IDLE (only if PROT_OUT low)            */

/* ===================== internal control state ============================= */
static volatile uint32_t adc_dual_raw = 0u;       /* DMA dest: ADC1 (V_FB) low half, ADC2 (SENSE) high half */
static volatile float ctrl_integrator = 0.0f;     /* PI integrator [duty]                                   */
static volatile float ctrl_duty_limit = 0.0f;     /* soft-start ramped duty ceiling [duty]                  */
static volatile float ctrl_ss_increment = 0.0f;   /* duty-limit increment per control tick                  */
static volatile float ctrl_duty_max_cached = 0.0f;/* clamped copy of set_duty_max (main-loop owned)         */
static volatile uint8_t ctrl_ramp_done = 0u;      /* control ISR -> main loop: soft-start ramp finished     */
static float dac_oc_applied_A = -1.0f;            /* last value written to PROT_REF (-1 forces first write) */
static float dac_dt_applied_A = -1.0f;            /* last value written to DT_REF                           */
static uint32_t iwdg_last_ctrl_count = 0u;        /* main-loop copy of the control heartbeat                */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC1_Init(void);
static void MX_DAC2_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void app_clamp_tunables(void);
static void app_apply_dac_refs(void);
static void app_enter_idle(void);
static void app_arm_soft_start(void);
static void app_clear_fault(void);
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
  /* Freeze TIM1 and IWDG while the core is halted by the debugger: breakpoints
     must not leave the power stage mid-PWM-cycle or trip the watchdog.        */
  __HAL_DBGMCU_FREEZE_TIM1();
  __HAL_DBGMCU_FREEZE_IWDG();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_IWDG_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* --- ADC calibration: both units, single-ended, before any conversion --- */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  /* --- DAC comparator references: start and apply the power-on defaults.
         PROT_REF (DAC1_OUT1/PA4) from set_oc_trip_A (default 0 A = disarmed),
         DT_REF   (DAC2_OUT1/PA6) from set_dt_ref_A  (default 0.1 A).        */
  if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_Start(&hdac2, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  app_clamp_tunables();
  app_apply_dac_refs();

  /* --- Dual regular-simultaneous ADC via DMA: one circular 32-bit transfer
         from the common data register (ADC1 low half, ADC2 high half).      */
  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)&adc_dual_raw, 1u) != HAL_OK)
  {
    Error_Handler();
  }
  /* With NDTR = 1 the half-transfer event fires together with transfer-complete,
     and HAL_DMA_IRQHandler services HT/TC in separate IRQ entries — that would
     double the 400 kHz interrupt load for nothing. Only TC drives the loop.  */
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);

  /* --- TIM3 input captures for the switching-activity comparator (DT_OUT/PB4).
         Edges are serviced by a lean handler in stm32g4xx_it.c.              */
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)  /* rising edge  */
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK)  /* falling edge */
  {
    Error_Handler();
  }

  /* --- TIM1: run the counter so TRGO keeps triggering the ADCs from boot, with
         both gate outputs held inactive. HAL_TIM_PWM_Start/PWMN_Start are NOT
         used here: both set MOE internally, which would drive the LS gate for
         several microseconds at boot — dangerous after a warm reset with a
         charged output. Direct register writes enable the channels and the
         counter without ever touching MOE (CCR1 is already 0, MOE stays 0 from
         reset); OSSR = Enable means both outputs are actively driven to their
         inactive level, not floated. app_arm_soft_start() is the ONLY place in
         the firmware where MOE is ever set.                                   */
  htim1.Instance->CCR1 = 0u;
  TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);  /* HS: PA8, LS: PA7        */
  TIM1->CR1  |= TIM_CR1_CEN;                       /* TRGO = Update -> ADCs   */
  /* The generated init does not enable the break interrupt source; the NVIC
     vector alone is not enough for HAL_TIMEx_Break2Callback to fire.         */
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);

  /* INIT -> IDLE: outputs off, converter waits for cmd_start. A BRK2 event can
     already fire here (PROT_OUT stuck high at boot) — never overwrite a
     freshly latched FAULT with IDLE.                                          */
  __disable_irq();
  if (mon_sys_state != SYS_STATE_FAULT)
  {
    mon_sys_state = SYS_STATE_IDLE;
  }
  __enable_irq();
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

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
  while (1)
  {
    /* --- live pin monitors (GPIO input path is valid even in AF mode) ------ */
    mon_prot_out_state = (uint8_t)((GPIOA->IDR >> 11) & 1u); /* PA11 = PROT_OUT / TIM1_BKIN2 */
    mon_dt_out_state   = (uint8_t)((GPIOB->IDR >> 4) & 1u);  /* PB4  = DT_OUT (this poll is the single owner) */

    /* --- tunable sanitation + DAC reference updates ------------------------ */
    app_clamp_tunables();
    app_apply_dac_refs();

    /* live interlock indicator: start refused while the OC trip is below minimum.
       Single volatile read, NaN-rejecting comparison: a NaN written between the
       clamp above and this line must read as "not armed".                      */
    {
      float oc_trip = set_oc_trip_A;
      mon_interlock_oc_not_armed = (!(oc_trip >= OC_TRIP_MIN_A)) ? 1u : 0u;
    }

    /* --- consume commands / state machine ---------------------------------- */
    if (cmd_stop != 0u)
    {
      cmd_stop = 0u;
      if ((mon_sys_state == SYS_STATE_SOFT_START) || (mon_sys_state == SYS_STATE_RUN))
      {
        app_enter_idle();
      }
    }

    if (cmd_start != 0u)
    {
      cmd_start = 0u;
      if ((mon_sys_state == SYS_STATE_IDLE) && (mon_interlock_oc_not_armed == 0u))
      {
        app_arm_soft_start();
      }
    }

    if (cmd_clear_fault != 0u)
    {
      cmd_clear_fault = 0u;
      /* explicit FAULT -> IDLE transition; refuses (atomically, on the live
         PA11 level) while the OC comparator output is still tripped (high)   */
      app_clear_fault();
    }

    /* --- SOFT_START -> RUN: performed here, not in the control ISR, so the
           state machine has a single ISR writer (the break path) and the
           transition can never depend on interrupt priorities.               */
    if (ctrl_ramp_done != 0u)
    {
      __disable_irq();
      if (mon_sys_state == SYS_STATE_SOFT_START)
      {
        mon_sys_state = SYS_STATE_RUN;
      }
      ctrl_ramp_done = 0u;
      __enable_irq();
    }

    /* --- fault indicator: LD2 on while latched in FAULT --------------------- */
    if (mon_sys_state == SYS_STATE_FAULT)
    {
      BSP_LED_On(LED_GREEN);
    }
    else
    {
      BSP_LED_Off(LED_GREEN);
    }

    /* --- IWDG: refresh only while the 400 kHz control ISR is alive.
           If the ADC/DMA/trigger chain dies, the heartbeat stops advancing and
           the watchdog resets the MCU (2 s timeout).                          */
    {
      uint32_t count_now = mon_ctrl_loop_count;
      if (count_now != iwdg_last_ctrl_count)
      {
        iwdg_last_ctrl_count = count_now;
        (void)HAL_IWDG_Refresh(&hiwdg);
      }
    }
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC2_Init(void)
{

  /* USER CODE BEGIN DAC2_Init 0 */

  /* USER CODE END DAC2_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC2_Init 1 */

  /* USER CODE END DAC2_Init 1 */

  /** DAC Initialization
  */
  hdac2.Instance = DAC2;
  if (HAL_DAC_Init(&hdac2) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC2_Init 2 */

  /* USER CODE END DAC2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 1999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 424;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
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
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 2;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMAMUX_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX_OVR_IRQn);

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
  * @brief  Clamp all CubeMonitor tunables to safe ranges (main-loop context).
  *         Also publishes the clamped duty ceiling for the control ISR so the
  *         ISR never reads an unclamped value.
  */
static void app_clamp_tunables(void)
{
  /* Pattern (TOCTOU-safe): read each volatile ONCE into a local, clamp the
     local with a NaN-rejecting comparison (!(v >= min) catches NaN as well as
     underrange), write the local back, and derive any cached value from the
     LOCAL — never from a re-read of the volatile.                            */
  float v;

  v = set_vout_V;
  if (!(v >= 0.0f))             { v = 0.0f; }
  else if (v > VOUT_SET_MAX_V)  { v = VOUT_SET_MAX_V; }
  set_vout_V = v;

  v = set_kp;
  if (!(v >= 0.0f))             { v = 0.0f; }
  set_kp = v;

  v = set_ki;
  if (!(v >= 0.0f))             { v = 0.0f; }
  set_ki = v;

  v = set_duty_max;
  if (!(v >= 0.0f))             { v = 0.0f; }
  else if (v > DUTY_MAX_ABS)    { v = DUTY_MAX_ABS; }
  set_duty_max = v;
  /* duty ceiling consumed by the control ISR: derived from the clamped local */
  ctrl_duty_max_cached = v;

  v = set_softstart_ms;
  if (!(v >= SOFTSTART_MS_MIN))  { v = SOFTSTART_MS_MIN; }
  else if (v > SOFTSTART_MS_MAX) { v = SOFTSTART_MS_MAX; }
  set_softstart_ms = v;

  v = set_oc_trip_A;
  if (!(v >= 0.0f))             { v = 0.0f; }
  else if (v > DAC_FULL_SCALE_A){ v = DAC_FULL_SCALE_A; }
  set_oc_trip_A = v;

  v = set_dt_ref_A;
  if (!(v >= 0.0f))             { v = 0.0f; }
  else if (v > DAC_FULL_SCALE_A){ v = DAC_FULL_SCALE_A; }
  set_dt_ref_A = v;
}

/**
  * @brief  Write the DAC comparator references when the requested trip levels
  *         change (main-loop context, never from an ISR).
  *         V_ref = I_trip * 0.25 V/A; DAC full scale = 3.3 V -> 13.2 A.
  */
static void app_apply_dac_refs(void)
{
  /* Each trip level is re-read from the volatile here, so it gets its own
     NaN-rejecting clamp: a NaN written between app_clamp_tunables() and this
     call must not reach the float->uint32 conversion (undefined for NaN).    */
  float trip;
  uint32_t counts;

  trip = set_oc_trip_A;
  if (!(trip >= 0.0f))              { trip = 0.0f; }
  else if (trip > DAC_FULL_SCALE_A) { trip = DAC_FULL_SCALE_A; }
  if (trip != dac_oc_applied_A)
  {
    counts = (uint32_t)(trip * AMPS_TO_DAC_COUNTS + 0.5f);
    if (counts > 4095u) { counts = 4095u; }
    (void)HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, counts); /* PROT_REF, PA4 */
    dac_oc_applied_A = trip;
  }

  trip = set_dt_ref_A;
  if (!(trip >= 0.0f))              { trip = 0.0f; }
  else if (trip > DAC_FULL_SCALE_A) { trip = DAC_FULL_SCALE_A; }
  if (trip != dac_dt_applied_A)
  {
    counts = (uint32_t)(trip * AMPS_TO_DAC_COUNTS + 0.5f);
    if (counts > 4095u) { counts = 4095u; }
    (void)HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, counts); /* DT_REF, PA6 */
    dac_dt_applied_A = trip;
  }
}

/**
  * @brief  Disable the power stage and return to IDLE (stop path).
  *         NOTE: __HAL_TIM_MOE_DISABLE is a silent NO-OP while any CCx/CCxN
  *         channel is enabled (CC1E/CC1NE always are here) — the
  *         _UNCONDITIONALLY variant is the only one that actually clears MOE.
  *         A freshly latched FAULT is never overwritten with IDLE; the
  *         output-off/reset actions are safe in every state.
  */
static void app_enter_idle(void)
{
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
  htim1.Instance->CCR1 = 0u;
  __disable_irq();
  ctrl_integrator = 0.0f;
  ctrl_duty_limit = 0.0f;
  ctrl_ramp_done  = 0u;
  if (mon_sys_state != SYS_STATE_FAULT)
  {
    mon_sys_state = SYS_STATE_IDLE;
  }
  __enable_irq();
}

/**
  * @brief  Explicit FAULT -> IDLE transition (fault-clear path only).
  *         The gate condition reads the live PROT_OUT level (PA11) directly
  *         from GPIOA->IDR at the decision point, inside the critical section:
  *         clearing is refused while the OC comparator output is still high.
  *         Pending break flags are cleared and the break interrupt source is
  *         re-armed (it is masked in the break callbacks to stop an IRQ storm
  *         while BKIN2 is held asserted).
  */
static void app_clear_fault(void)
{
  __disable_irq();
  if ((mon_sys_state == SYS_STATE_FAULT) && ((GPIOA->IDR & (1u << 11)) == 0u))
  {
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);  /* already off — belt and braces */
    htim1.Instance->CCR1 = 0u;
    ctrl_integrator = 0.0f;
    ctrl_duty_limit = 0.0f;
    ctrl_ramp_done  = 0u;
    /* SR bits are rc_w0: writing 0 clears, writing 1 leaves untouched */
    TIM1->SR = ~(TIM_SR_BIF | TIM_SR_B2IF | TIM_SR_SBIF);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    mon_fault_latched = 0u;
    mon_sys_state = SYS_STATE_IDLE;
  }
  __enable_irq();
}

/**
  * @brief  IDLE -> SOFT_START: reset the controller, precompute the soft-start
  *         ramp increment (division allowed here — never in the ISR) and enable
  *         MOE. This is the ONLY place in the firmware where MOE is EVER set.
  *         Inside the critical section the arm is aborted (state stays IDLE,
  *         MOE stays off) unless ALL of the following hold:
  *           - state is still IDLE (a BRK2 event racing this call wins),
  *           - no break flag is pending in TIM1->SR (a BRK2 pulse latched as a
  *             pended ISR must run its callback, not be overtaken by MOE on),
  *           - the live PROT_OUT level (PA11) is low right now.
  */
static void app_arm_soft_start(void)
{
  /* local copy of set_softstart_ms, NaN-rejecting clamp BEFORE the division:
     a raw re-read of the volatile here could see a value written after the
     main-loop clamp (TOCTOU) or a NaN.                                       */
  float ss_ms = set_softstart_ms;
  if (!(ss_ms >= SOFTSTART_MS_MIN))  { ss_ms = SOFTSTART_MS_MIN; }
  else if (ss_ms > SOFTSTART_MS_MAX) { ss_ms = SOFTSTART_MS_MAX; }

  /* full ramp 0 -> duty_max spread over ss_ms of 400 kHz ticks */
  float ramp_ticks = ss_ms * (1.0e-3f / CTRL_TS_SEC);
  float increment  = ctrl_duty_max_cached / ramp_ticks;  /* ss_ms clamped >= 1 ms */

  __disable_irq();
  if ((mon_sys_state == SYS_STATE_IDLE) &&
      ((TIM1->SR & (TIM_SR_B2IF | TIM_SR_BIF | TIM_SR_SBIF)) == 0u) &&
      ((GPIOA->IDR & (1u << 11)) == 0u))
  {
    htim1.Instance->CCR1 = 0u;
    ctrl_integrator   = 0.0f;
    ctrl_duty_limit   = 0.0f;
    ctrl_ss_increment = increment;
    ctrl_ramp_done    = 0u;
    /* re-arm the break interrupt source (masked in the break callbacks while
       BKIN2 was held asserted); pending flags verified clear just above     */
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    mon_sys_state     = SYS_STATE_SOFT_START;
    __HAL_TIM_MOE_ENABLE(&htim1);
  }
  __enable_irq();
}

/**
  * @brief  400 kHz control ISR. Fires from the DMA1_Channel1 transfer-complete
  *         of the dual regular-simultaneous conversion (TIM1 TRGO triggered).
  *         Budget < 1 us: direct register access only, no divisions, no HAL calls.
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    uint32_t raw   = adc_dual_raw;  /* ADC1 (V_FB) low half, ADC2 (SENSE_OUT) high half */
    float    vout  = (float)(raw & 0x0FFFu) * ADC_RAW_TO_VOUT_V;
    float    iout  = (float)((raw >> 16) & 0x0FFFu) * ADC_RAW_TO_AMPS;
    uint8_t  state = mon_sys_state;

    mon_vout_V    = vout;
    mon_current_A = iout;

    if ((state == SYS_STATE_SOFT_START) || (state == SYS_STATE_RUN))
    {
      float duty_limit;
      float error;
      float integ;
      float duty;

      if (state == SYS_STATE_SOFT_START)
      {
        /* linear duty-limit ramp 0 -> duty_max over set_softstart_ms */
        duty_limit = ctrl_duty_limit + ctrl_ss_increment;
        if (duty_limit >= ctrl_duty_max_cached)
        {
          duty_limit = ctrl_duty_max_cached;
          mon_sys_state = SYS_STATE_RUN;      /* ramp complete: SOFT_START -> RUN */
        }
        ctrl_duty_limit = duty_limit;
      }
      else
      {
        duty_limit = ctrl_duty_max_cached;    /* clamped by the main loop */
        ctrl_duty_limit = duty_limit;
      }

      /* PI with anti-windup: integrator and total output clamped to [0, duty_limit] */
      error = set_vout_V - vout;
      integ = ctrl_integrator + (error * set_ki * CTRL_TS_SEC);
      if (integ > duty_limit)  { integ = duty_limit; }
      else if (integ < 0.0f)   { integ = 0.0f; }
      ctrl_integrator = integ;

      duty = (set_kp * error) + integ;
      if (duty > duty_limit)   { duty = duty_limit; }
      else if (duty < 0.0f)    { duty = 0.0f; }

      TIM1->CCR1 = (uint32_t)(duty * PWM_PERIOD_TICKS);
    }

    /* PWM monitor variables (valid in every state) */
    {
      uint32_t ccr = TIM1->CCR1;
      uint8_t  moe = ((TIM1->BDTR & TIM_BDTR_MOE) != 0u) ? 1u : 0u;

      if (moe != 0u)
      {
        float hs_pct = (float)ccr * CCR_TO_PCT;
        float ls_pct = 100.0f - hs_pct - PWM_DEADTIME_PCT;
        if (ls_pct < 0.0f) { ls_pct = 0.0f; }
        mon_pwm_hs_duty_pct = hs_pct;
        mon_pwm_ls_duty_pct = ls_pct;
      }
      else
      {
        mon_pwm_hs_duty_pct = 0.0f;
        mon_pwm_ls_duty_pct = 0.0f;
      }
      mon_pwm_hs_active = (uint8_t)((moe != 0u) && (ccr > 0u));
      mon_pwm_ls_active = moe;
    }

    mon_ctrl_loop_count++;   /* heartbeat consumed by the main-loop IWDG gate */
  }
}

/**
  * @brief  TIM1 BRK2 event (BKIN2 = PA11 = PROT_OUT, overcurrent comparator,
  *         active high). Hardware has already cleared MOE and, because
  *         AutomaticOutput is disabled, keeps it latched off. Software only
  *         latches the FAULT state — MOE is never re-enabled here.
  */
void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    TIM1->CCR1 = 0u;
    mon_fault_latched = 1u;
    mon_sys_state = SYS_STATE_FAULT;
  }
}

/**
  * @brief  TIM1 BRK1 event. BRK1 is disabled in the generated configuration, but
  *         a system break (e.g. CSS) lands on the same flag path — handle it the
  *         same way as BRK2 for defence in depth.
  */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    TIM1->CCR1 = 0u;
    mon_fault_latched = 1u;
    mon_sys_state = SYS_STATE_FAULT;
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
