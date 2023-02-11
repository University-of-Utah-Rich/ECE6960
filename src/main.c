/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "gpio.h"
#include "pixels.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

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

/* USER CODE BEGIN PV */
// each uint32_t has 8 4bit nibbles representing

// 2 backbuffers for double buffering
static uint32_t backbuffer[2][4 * 3 * 32];
// index of the backbuffer that is currently being displayed
static volatile uint8_t backbuffer_index = 0;
// flag indicating that the backbuffer is ready to be displayed
static volatile uint8_t buffer_in_use = 0;
// flag indicating that a vsync is in progress
static volatile uint8_t vsync_in_progress = 0;
// bit angle modulation lookup table
const uint32_t bit_angle_modulation_lookup[SCAN_RATE] = {
    0x1u, 0x2u, 0x2u, 0x4u, 0x4u, 0x4u, 0x4u, 0x8u,
    0x8u, 0x8u, 0x8u, 0x8u, 0x8u, 0x8u, 0x8u};
// the current line being drawn
static uint16_t line = 0;
// the index of the current line being drawn
static uint8_t cur_linedata = 0;
// the buffer of data for all lines
static uint32_t linedata[2][6];
// the current scan being drawn
static volatile uint8_t scan = 0;
// the current frame being drawn
static volatile uint32_t frame = 0;
// flag indicating that a transfer is pending
static volatile uint8_t transfer_pending = 0;
// flag indicating that a transfer is in progress
static volatile uint16_t spi_transmit_line = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void switch_back_buffer();
void begin_vsync();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM7_Init();
  MX_USART1_Init();
  /* USER CODE BEGIN 2 */
  image_buffer = backbuffer[backbuffer_index];
#ifdef TEST_PATTERN
  for (int y = 0; y < 32; ++y)
    for (int x = 0; x < 32; ++x) {
      // layout: b - g - r
      image_buffer = backbuffer[0];
      set_pixel(x, y, x / 2, y / 2, (63 - (x + y)) / 4);
      image_buffer = backbuffer[1];
      set_pixel(x, y, x / 2, y / 2, (63 - (x + y)) / 4);
    }
#endif
  float t = 0;
  // drive the OE pin low to enable the display
  HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);
  // start the timer
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
		// wait forever for a vSync
		if ( !vsync_in_progress)
		{
			continue;
		}
		// adjust image buffer in case the backbuffer id has changed
		image_buffer = &backbuffer[backbuffer_index][0];
		for(int8_t y=0;y<32;++y)
		{
			uint32_t pr, pg, pb;
			pr = pg = pb = 0;

			// this is tricky. We will later push a u32 include all
			// 8 values for r or g or b to the image buffer.
			// most significant nibble is the offset 7.
			// We will later left-shift by for each pixel, so we
			// start at the top.
			for(int8_t x=31;x>=0;--x)
			{
				float py = y - 15.f;
				float px = x - 15.f;
				float v1 = arm_sin_f32((float)px/ 4.0f  + t);
				float v2 = arm_sin_f32((px/2.f * arm_sin_f32(t/2.f)
				+ py/2.f * arm_cos_f32(t/3.f)) + t);
				float cx = px / 5.f + 0.5f * arm_sin_f32(t/5.f);
				float cy = py / 5.f + 0.5f * arm_cos_f32(t/3.f);
				float sqrtfout;
				arm_sqrt_f32(2.f * (cx*cx + cy*cy) + 1, &sqrtfout);
				float v3 = arm_sin_f32(sqrtfout + t);

				float v = (v1 + v2 + v3) / 3.f;

				int8_t r = (int8_t)((arm_sin_f32(v*3.14f) + 1.f) * 7.5f);
				int8_t g = (int8_t)((arm_sin_f32(v*7.14f + 1.5f) + 1.f) * 7.5f);
				int8_t b = (int8_t)((arm_sin_f32(v*4.14f + 3.14f/3.f) + 1.f) * 7.5f);

				// as we start from the most significant pixel,
				// the left shift should be exactly what we need.
				pr <<=4; pr |= (r & 0xFu);
				pg <<=4; pg |= (g & 0xFu);
				pb <<=4; pb |= (b & 0xFu);
				if ( (x & 0x7u) == 0)
				{
					set_pixels_u32(x,y, pb, pg, pr);
				}
			}
		}
    // reset the buffer and the vsync flag
		vsync_in_progress = 0;
    buffer_in_use = 0;
		t += 0.2f;//2.f;//1.f/60.f;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  // Set PLL to 8x for 64MHz clock (overclocked)
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == &hspi1) {
    // drive OE high
    HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_SET);

    // GPIOC->ODR = spi_transmit_line & 0xF;
    // Assign pins A, B, C, D to the 4 LSBs of the data
    HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, (spi_transmit_line & 0x1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, (spi_transmit_line & 0x2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_GPIO_Port, C_Pin, (spi_transmit_line & 0x4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_GPIO_Port, D_Pin, (spi_transmit_line & 0x8) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    //HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)&spi_transmit_line, 1);
    // latch data in
    // GPIOB->BSRR = GPIO_PIN_1;
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_SET);

    // give the IOs some time to settle
    __asm volatile(" nop	\n"
                   " nop	\n"
                   " nop	\n"
                   " nop	\n");

    // disable latches
    // GPIOB->BSRR = GPIO_PIN_1 << 16;
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, GPIO_PIN_RESET);

    // drive OE low
    // GPIOB->BSRR = GPIO_PIN_0 << 16;
    HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);

    transfer_pending = 0;
  }
}

uint32_t bitline(const uint32_t *image_data, const uint8_t bam_pattern) {
  uint32_t v = 0;
  const uint32_t bam_pattern32 = (uint32_t)bam_pattern;
  for (uint8_t i = 0; i < 4; ++i) {
    const uint32_t id = *image_data++;
    uint32_t bam_shifter = bam_pattern32;
    for (uint8_t j = 0; j < 8; ++j) {
      v <<= 1;
      if ((id & bam_shifter) > 0)
        v |= 0x1u;
      bam_shifter <<= 4;
    }
  }

  return v;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim7) {
    if (!transfer_pending) {
      transfer_pending = 1;

      // dispatch SPI transfer in the background
      spi_transmit_line = line;
      HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)&linedata[cur_linedata][0],
                           sizeof(uint32_t) * 6);

      ++line;

      // we only have 15 slots
      if (line > 0xF) {
        line = 0;
        ++scan;
      }
      //  times oversampling
      if (scan > SCAN_RATE) {
        scan = 0;
        ++frame;

        // switch buffers
        switch_back_buffer();
        // signal vsync to main loop
        begin_vsync();
      }

      // switch to other buffer to populate SPI data
      cur_linedata ^= 1;
      /* prepare next line */
      uint32_t *ld = &(linedata[cur_linedata][0]);

      const uint32_t *display_buffer = &backbuffer[backbuffer_index ^ 1][0];
      const uint32_t *lp_top = display_buffer + line * 4 * 3;
      const uint32_t *lp_bottom = display_buffer + line * 4 * 3 + 16 * 4 * 3;
      const uint8_t bam_pattern = bit_angle_modulation_lookup[scan];

      *ld++ = __REV(bitline(lp_bottom, bam_pattern));
      *ld++ = __REV(bitline(lp_bottom + 4, bam_pattern));
      *ld++ = __REV(bitline(lp_bottom + 8, bam_pattern));
      *ld++ = __REV(bitline(lp_top, bam_pattern));
      *ld++ = __REV(bitline(lp_top + 4, bam_pattern));
      *ld = __REV(bitline(lp_top + 8, bam_pattern));
    }
  }
}

/**
 * @brief switch between the two back buffers
 *
 */
void switch_back_buffer() {
  while (buffer_in_use) {
    // block until the other buffer is not in use
  }
  buffer_in_use = 1;
  backbuffer_index ^= 1;
}

void begin_vsync() {
 while (vsync_in_progress)
 {
  // block until vsync is done
 }
 vsync_in_progress = 1;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
