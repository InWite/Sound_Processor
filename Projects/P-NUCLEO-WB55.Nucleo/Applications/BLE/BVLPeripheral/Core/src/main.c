/**
 ******************************************************************************
 * @file    main.c
 * @author  SRA-A&SP
 * @version V 1.1.0
 * @date    19-Dic-2019
 * @brief   This software provides an application running on STM32WB to
 *          demonstrate voice streaming over Bluetooth Low Energy.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/**
 * @mainpage Documentation for FP-AUD-BVLINKWB1 Software for STM32WB and digital 
 *           MEMS microphone
 *
 * @image html st_logo.png
 *
 * <b>Introduction</b>
 *
 * This software is based on the STM32CubeHAL, the hardware abstraction layer 
 * for STM32 microcontroller. The package extends STM32Cube by providing a
 * Board Support Package (BSP) for P-NUCLEO-WB55 and MEMS Microphone 
 * expansion boards. Middleware components for 
 * communication with other Bluetooth LE devices and a dedicated profile 
 * for voice communication
 *
 * <b>Example Application</b>
 *
 * FP-AUD-BVLINKWB1 is an STM32Cube function pack that performs full-duplex
 * voice streaming or stereo music streaming over BLE using the advanced Opus
 * compression algorithm. The application runs on the STM32WB55 Nucleo and 
 * includes drivers and middleware for BLE and digital MEMS microphones.
 */

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "app_entry.h"
#include "stm32_lpm.h"
#include "stm32_seq.h"
#include "core_main.h" 
#include "otp.h"

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

/* Private function prototypes -----------------------------------------------*/
static void Reset_BackupDomain( void );
static void Init_RTC( void );
static void SystemClock_Config( void );
static void Reset_Device( void );
static void Reset_IPCC( void );
static void Config_HSE(void);

/* Functions Definition ------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main( void )
{
  HAL_Init();

  Reset_Device();

  Config_HSE();
  
  /**< Configure the system clock */
  SystemClock_Config(); 
  
  /* Configure the RTC */
  Init_RTC();

  /**
  * This prevents the CPU2 to disable the HSI48 oscillator when
  * it does not use anymore the RNG IP
  */
  LL_HSEM_1StepLock( HSEM, 5 );
  
  APPE_Init();
 
  while(1)
  {
    UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );
  }
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

static void Config_HSE(void)
{
    OTP_ID0_t * p_otp;

  /**
   * Read HSE_Tuning from OTP
   */
  p_otp = (OTP_ID0_t *) OTP_Read(0);
  if (p_otp)
  {
    LL_RCC_HSE_SetCapacitorTuning(p_otp->hse_tuning);
  }

  return;
} 

static void Reset_Device( void )
{
#if ( CFG_HW_RESET_BY_FW == 1 )
  Reset_BackupDomain();

  Reset_IPCC();
#endif

  return;
}

static void Reset_IPCC( void )
{
  LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_IPCC);

  LL_C1_IPCC_ClearFlag_CHx(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_ClearFlag_CHx(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C1_IPCC_DisableTransmitChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_DisableTransmitChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C1_IPCC_DisableReceiveChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_DisableReceiveChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  return;
}

static void Reset_BackupDomain( void )
{
  if ((LL_RCC_IsActiveFlag_PINRST() != FALSE) && (LL_RCC_IsActiveFlag_SFTRST() == FALSE))
  {
    /**< Enable access to the RTC registers */
    HAL_PWR_EnableBkUpAccess(); 

    /**
     *  Write twice the value to flush the APB-AHB bridge
     *  This bit shall be written in the register before writing the next one
     */
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
  }

  return;
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void Init_RTC( void )
{
  /**< Enable access to the RTC registers */
  HAL_PWR_EnableBkUpAccess(); 

  /**
   *  Write twice the value to flush the APB-AHB bridge
   *  This bit shall be written in the register before writing the next one
   */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE); /**< Select LSI as RTC Input */

  __HAL_RCC_RTC_ENABLE(); /**< Enable RTC */

  hrtc.Instance = RTC; /**< Define instance */

  /**
   * Bypass the shadow register
   */
  HAL_RTCEx_EnableBypassShadow(&hrtc);

  /**
   * Set the Asynchronous prescaler
   */
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  HAL_RTC_Init(&hrtc);

  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc); /**< Disable Write Protection */

  MODIFY_REG(RTC->CR, RTC_CR_WUCKSEL, CFG_RTC_WUCKSEL_DIVIDER);

  return;
}

/**
 * @brief  Configure the system clock
 *
 * @note   This API configures
 *         - The system clock source
 *           - The AHBCLK, APBCLK dividers
 *           - The flash latency
 *           - The PLL settings (when required)
 *
 * @param  None
 * @retval None
 */
void SystemClock_Config( void )
{
  RCC_OscInitTypeDef RCC_OscInitStruct={0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct={0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct={0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV8;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1); /* ERROR */
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    while(1); /* ERROR */
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLLSAI1.PLLN = 86;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV7;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    while(1); /* ERROR */
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);

  /**
   *  Write twice the value to flush the APB-AHB bridge to ensure the  bit is written
   */
  HAL_PWR_EnableBkUpAccess(); /**< Enable access to the RTC registers */
  HAL_PWR_EnableBkUpAccess();

  /**
   * Select LSE clock
   */
  LL_RCC_LSE_Enable();
  while(!LL_RCC_LSE_IsReady());

  /**
   * Select wakeup source of BLE RF
   */
  LL_RCC_SetRFWKPClockSource(LL_RCC_RFWKP_CLKSOURCE_LSE);

  /**
   * Switch OFF LSI
   */
  LL_RCC_LSI1_Disable();

  /**
   * Set RNG on HSI48
   */
  LL_RCC_HSI48_Enable();
  while(!LL_RCC_HSI48_IsReady());
  LL_RCC_SetCLK48ClockSource(LL_RCC_CLK48_CLKSOURCE_HSI48);
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

/**
 * This function is empty to avoid starting the SysTick Timer
 */
HAL_StatusTypeDef HAL_InitTick( uint32_t TickPriority )
{
  return (HAL_OK);
}

/**
 * This function is empty as the SysTick Timer is not used
 */
void HAL_Delay(__IO uint32_t Delay)
{
  return;
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
