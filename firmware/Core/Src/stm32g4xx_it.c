/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TIM3 runs at 170 MHz -> 5.882 ns per tick. Integer-only conversion:
 * t_ns = ticks * 100 / 17 (exact for the 170 MHz clock, no float in the ISR). */
#define DT_TICKS_TO_NS_NUM      100u
#define DT_TICKS_TO_NS_DEN      17u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Timestamp of the last DT_OUT rising edge (TIM3 CCR1 ticks). uint16_t so that
 * the falling-edge subtraction is wrap-aware across the 65535 counter rollover. */
static volatile uint16_t dt_rise_ticks = 0u;
static volatile uint8_t  dt_rise_seen  = 0u;   /* no width until first rising edge */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
/* CubeMonitor variables owned by main.c, updated by the TIM3 capture handler
   (mon_dt_out_state is owned/polled by the main loop only — not written here) */
extern volatile uint32_t mon_dt_edge_count;     /* total DT_OUT edges captured by TIM3 */
extern volatile uint32_t mon_dt_pulse_width_ns; /* last DT_OUT high-pulse width [ns]   */
/* system state owned by main.c, latched to FAULT by the lean DMA error path  */
extern volatile uint8_t  mon_sys_state;         /* sys_state_t                         */
extern volatile uint8_t  mon_fault_latched;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  /* Lean dispatch of the 400 kHz control interrupt: the HAL_DMA_IRQHandler
     call below is bypassed (unconditional return at the end of this block) —
     its flag walk plus the DMA->ADC->ConvCplt callback indirection is pure
     overhead inside a < 425-cycle budget. NDTR = 1 in circular mode sets HT
     together with TC, so the stale HT flag is cleared alongside TC (the HT
     interrupt enable itself is cleared once at start-up in main()).          */
  uint32_t isr = DMA1->ISR;

  if ((isr & DMA_ISR_TEIF1) != 0u)
  {
    /* DMA transfer error: hardware has already cleared the channel enable, so
       the ADC sample stream is dead and the control ISR will never run again.
       Kill the power stage and latch FAULT directly (lean DMA error path).   */
    DMA1->IFCR = DMA_IFCR_CTEIF1;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = 0u;
    mon_fault_latched = 1u;
    mon_sys_state = SYS_STATE_FAULT;
  }

  if ((isr & DMA_ISR_TCIF1) != 0u)
  {
    DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1;
    Buck_CtrlLoopISR();
  }
  return;
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  /* Lean DT_OUT edge-capture handler: the HAL_TIM_IRQHandler call below is
     bypassed (unconditional return at the end of this block). At up to
     2 x 400 kHz edges the HAL flag-walk + callback indirection (~130-160
     cycles/edge) would starve the CPU. Direct register access, integer math
     only, no function calls.
     CH1 = direct TI1, rising edge; CH2 = indirect TI1, falling edge.
     Reading CCRx clears the corresponding CCxIF flag.                        */
  uint32_t sr  = TIM3->SR;
  uint32_t ovc = sr & (TIM_SR_CC1OF | TIM_SR_CC2OF);   /* overcapture flags */

  if ((sr & TIM_SR_CC1IF) != 0u)                       /* rising edge  */
  {
    dt_rise_ticks = (uint16_t)TIM3->CCR1;              /* read clears CC1IF */
    dt_rise_seen  = 1u;
    mon_dt_edge_count++;
  }

  if ((sr & TIM_SR_CC2IF) != 0u)                       /* falling edge */
  {
    uint16_t fall_ticks = (uint16_t)TIM3->CCR2;        /* read clears CC2IF */

    mon_dt_edge_count++;

    if ((dt_rise_seen != 0u) && (ovc == 0u))
    {
      /* wrap-aware uint16 delta, then ticks -> ns without floats.
         Max value: 65535 * 100 / 17 = 385500 ns, well within uint32. */
      uint16_t width_ticks = (uint16_t)(fall_ticks - dt_rise_ticks);
      mon_dt_pulse_width_ns = ((uint32_t)width_ticks * DT_TICKS_TO_NS_NUM)
                              / DT_TICKS_TO_NS_DEN;
    }
  }

  if (ovc != 0u)
  {
    /* overcapture: edges were lost, so rise/fall pairing is unreliable.
       Width computation was skipped above; drop the pending rising edge so
       no width is produced until the next clean rising edge. SR bits are
       rc_w0: writing 0 clears, writing 1 leaves untouched — this write
       clears only the overcapture flags.                                    */
    TIM3->SR = ~(TIM_SR_CC1OF | TIM_SR_CC2OF);
    dt_rise_seen = 0u;
  }
  return;
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles DMAMUX overrun interrupt.
  */
void DMAMUX_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMAMUX_OVR_IRQn 0 */

  /* USER CODE END DMAMUX_OVR_IRQn 0 */
  /* USER CODE BEGIN DMAMUX_OVR_IRQn 1 */

  /* USER CODE END DMAMUX_OVR_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
