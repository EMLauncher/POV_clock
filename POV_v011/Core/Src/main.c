/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "PI_controller.h"
#include "draw_clock.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union {
	uint8_t b[4];
	uint32_t v;
} uint8to32_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#include "my_definitions.h"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
volatile uint8_t digit = 0;
PI_ctrl_inst_q15 omegaPI;
volatile int32_t theta_q31 = 0L, // 2^31==4pi[rad] integrated angular
		omegaTs = 0L; // 2^31==4pi[rad]
volatile int16_t theta; // 2^15==4pi[rad] dumped angular
volatile uint8_t phase_zero = 0;
volatile uint32_t dispPixels[THETA_RESL] = {0x00000000};
volatile uint8_t sw_status = NO_SW_FLAG;
clock_pixel cp;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void PLL_init(void);
static void TIM3_init(void);
static void EXTI_init(void);
static void const_delay(void);
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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  GPIOF->MODER &= GPIO_MODER_MODE2; // CubeMX does not initialize PF2. PF2:input mode.

  omegaPI.u = OMEGA_MIN; // initialize minimum omega
  omegaTs = __CALC_OMEGATS(omegaPI.u);
  PLL_init();
  EXTI_init();
  TIM3_init();
  clock_pixel_init(&cp, (uint32_t *)dispPixels);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
 	  if (phase_zero) {
		  update_PI_ctrl(&omegaPI, theta);
		  omegaTs = __CALC_OMEGATS(omegaPI.u);
		  phase_zero = 0;
	  }
 	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
 	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // You must call HAL_RTC_GetDate() after HAL_RTC_GetTime()
 	  cp.hour = sTime.Hours;
 	  cp.min = sTime.Minutes;
 	  cp.sec = sTime.Seconds;
 	  update_clock_pixel(&cp);

 	  switch(sw_status) {
 	  case MIN_INC_FLAG:
 		  if (sTime.Minutes == 59) { // if not able to ++ minutes, ++ hours
 			  sTime.Minutes = 0;
 			  sw_status = HOUR_INC_FLAG;
 		  } else {
 			  sTime.Minutes++;
 			  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
 			  HAL_Delay(300); // avoid chattering
 			 sw_status = NO_SW_FLAG;
 		  }
 		  break;
 	  case MIN_DEC_FLAG:
 		  if (sTime.Minutes == 0) { // if not able to -- minutes, -- hours
 			  sTime.Minutes = 59;
 			  sw_status = HOUR_DEC_FLAG;
 		  } else {
 			  sTime.Minutes--;
 			  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
 			  HAL_Delay(300); // avoid chattering
 			  sw_status = NO_SW_FLAG;
 		  }
 		  break;
 	  case HOUR_INC_FLAG:
 		  if (sTime.Hours == 12) {
 			  sTime.Hours = 1;
 		  } else {
 			  sTime.Hours++;
 		  }
 		  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
 		  HAL_Delay(300); // avoid chattering
 		  sw_status = NO_SW_FLAG;
 		  break;
 	 case HOUR_DEC_FLAG:
 	  	  if (sTime.Hours == 1) {
 	  		  sTime.Hours = 12;
 	  	  } else {
 	 	 	  sTime.Hours--;
 	  	  }
 	  	  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
 	  	  HAL_Delay(300); // avoid chattering
 	  	  sw_status = NO_SW_FLAG;
 	  	  break;
 	 default:
 		 break;
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

  /** Configure LSE Drive Capability
  */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 3;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// synchronize theta
static void PLL_init(void) {
	omegaPI.Kp = PLL_KP;
	omegaPI.Kp_decimal = PLL_KP_DECIMAL;
	omegaPI.TsTi = PLL_TSTI;
	omegaPI.y_target = Q15_2PI;
	omegaPI.u_max = OMEGA_MAX;
	omegaPI.u_min = OMEGA_MIN;
	init_PI_ctrl(&omegaPI, Q15_2PI);

	return;
}

// EXTI for PB7
static void EXTI_init(void) {
	// initialize EXTI
	EXTI->FTSR1 |= EXTI_FTSR1_FT7; // falling edge enable on Line 7
	EXTI->EXTICR[1] |= (0x01 << EXTI_EXTICR1_EXTI3_Pos); // select PB7
	EXTI->IMR1 |= EXTI_IMR1_IM7; // disable CPU wake-up mask Line7
	NVIC_EnableIRQ(EXTI4_15_IRQn); // enable interrupt on Line 4 to 15
	return;
}

void EXTI4_15_IRQHandler(void) {
	if (EXTI->FPR1 & EXTI_FPR1_FPIF7) {
		// Line 7 falling edge
		theta = theta_q31 >> 16;
		theta_q31 = 0L;
		phase_zero = 1;
		EXTI->FPR1 = EXTI_FPR1_FPIF7; // clear flag (cleared by writing "1" into bit)
	}
}

static void TIM3_init(void) {
	// initialize TIM3
	RCC->APBENR1 |= RCC_APBENR1_TIM3EN; // clock enable
	TIM3->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable
	TIM3->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR); // edge-aligned mode, upcounter
	TIM3->CR1 |= TIM_CR1_URS; // only overflow generate UEV
	TIM3->DIER |= TIM_DIER_UIE; // interrupt enable
	TIM3->PSC = 48 - 1; // prescaler 1/48
	TIM3->ARR = 25 - 1; // period=25us @48MHz
	NVIC_EnableIRQ(TIM3_IRQn);
	TIM3->CR1 |= TIM_CR1_CEN; // timer enable
	return;
}

// TIM3 interrupt
void TIM3_IRQHandler(void) {
	TIM3->DIER &= ~TIM_DIER_UIE;

	// update display
	theta_q31 += omegaTs;
	if (theta_q31 < 0) { // overflow theta
		theta_q31 = Q31_4PI;
	}
	uint16_t theta_idx = theta_q31 >> THETA_FIX_Pos;
	theta_idx %= THETA_RESL;
	GPIOA->ODR &= ~LED8BIT_msk;
	uint8to32_t dispPix; // これから表示するピクセルの値, 32bitに代入して8bitずつ取り出su

	// turn ON selected digit & turn OFF previous digit
	switch (digit) {
	case 0:
		if (!(GPIOF->IDR & (1UL << 2))) { // PF2 (SW pin) is 0
			sw_status = HOUR_INC_FLAG;
		}
		GPIOB->ODR &= ~DIG3_msk; // turn OFF DIG3
		dispPix.v = dispPixels[theta_idx];
		const_delay(); // not for display ghost
		GPIOA->ODR |= (LED8BIT_msk & dispPix.b[2]); // update pixel
		GPIOA->ODR |= DIG0_msk; // turn ON DIG0
		break;
	case 1:
		if (!(GPIOF->IDR & (1UL << 2))) {
			sw_status = MIN_DEC_FLAG;
		}
		GPIOA->ODR &= ~DIG0_msk; // turn OFF DIG0
		dispPix.v = dispPixels[theta_idx];
		const_delay(); // not for display ghost
		GPIOA->ODR |= (LED8BIT_msk & dispPix.b[3]);
		GPIOA->ODR |= DIG1_msk; // turn ON DIG1
		break;
	case 2:
		if (!(GPIOF->IDR & (1UL << 2))) {
			sw_status = MIN_INC_FLAG;
		}
		GPIOA->ODR &= ~DIG1_msk; // turn OFF DIG1
		dispPix.v = dispPixels[(theta_idx + (THETA_RESL >> 1)) % THETA_RESL]; // add to theta 180 deg.
		const_delay(); // not for display ghost
		GPIOA->ODR |= (LED8BIT_msk & dispPix.b[0]);
		GPIOA->ODR |= DIG2_msk; // turn ON DIG2
		break;
	case 3:
		if (!(GPIOF->IDR & (1UL << 2))) {
			sw_status = HOUR_DEC_FLAG;
		}
		GPIOA->ODR &= ~DIG2_msk; // turn OFF DIG2
		dispPix.v = dispPixels[(theta_idx + (THETA_RESL >> 1)) % THETA_RESL];
		const_delay(); // not for display ghost
		GPIOA->ODR |= (LED8BIT_msk & dispPix.b[1]);
		GPIOB->ODR |= DIG3_msk; // turn ON DIG3
		break;
	default:
		// do nothing
		break;
	}
	digit++;
	digit %= 4;

	TIM3->SR &= ~TIM_SR_UIF; // clear pending flag
	TIM3->DIER |= TIM_DIER_UIE;
}

static void __attribute__((optimize("O0"))) const_delay(void) {
	uint16_t foo = 0;

	/*
	 * memo
	 * 0 NG ghost
	 * 1 OK no ghost
	 * 10 OK no ghost
	 */
	while (foo < 1) {
		foo++;
	}

	return;
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

#ifdef  USE_FULL_ASSERT
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
