/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "fonts.h"
#include <stdio.h>
#include <string.h>
#include "peanut_gb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GB_COLOR_WHITE      0xFFFF
#define GB_COLOR_LIGHT_GRAY 0xADB5
#define GB_COLOR_DARK_GRAY  0x528A
#define GB_COLOR_BLACK      0x0000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */
uint8_t sd_tx_dummy[64];
static uint8_t rom_sector_buf[512];
uint8_t sd_dma_done = 0;
static uint32_t rom_sector_index = UINT32_MAX; /* invalid */
static uint8_t rom_sector_valid = 0;           /* 0 = not valid, 1 = valid */
static DWORD rom_file_pos = 0xFFFFFFFFu;       /* last known file pointer */
struct gb_s gb_instance;
FIL rom_file;
uint16_t line_buffer[160];
static const uint16_t gb_color_map[4] = {
    GB_COLOR_WHITE, GB_COLOR_LIGHT_GRAY, GB_COLOR_DARK_GRAY, GB_COLOR_BLACK
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void Error_Handler_Blink(void) { while(1) { HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); HAL_Delay(200); } }
void SD_DMA_InitDummy(void) {
      memset(sd_tx_dummy, 0xFF, sizeof(sd_tx_dummy));
  }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static volatile uint32_t rom_cache_hits = 0;
static volatile uint32_t rom_cache_misses = 0;

static inline uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
    uint32_t sector = addr >> 9;         /* addr / 512 */
    uint16_t offset = addr & 0x1FFu;     /* addr % 512 */

    if (!rom_sector_valid || sector != rom_sector_index) {
        FRESULT res;
        UINT br;
        DWORD wanted_pos = sector * 512UL;

        rom_cache_misses++;

        /* Only call f_lseek if FatFs file pointer is not already at wanted_pos */
        if (rom_file_pos != wanted_pos) {
            res = f_lseek(&rom_file, wanted_pos);
            if (res != FR_OK) {
                rom_sector_valid = 0;
                return 0xFF;
            }
            rom_file_pos = wanted_pos;
        }

        /* Read full 512-byte sector into cache */
        res = f_read(&rom_file, rom_sector_buf, 512, &br);
        if (res != FR_OK || br != 512) {
            rom_sector_valid = 0;
            return 0xFF;
        }

        rom_sector_index = sector;
        rom_sector_valid = 1;
        rom_file_pos = wanted_pos + 512UL; /* file pointer advanced by f_read */
    } else {
        rom_cache_hits++;
    }

    return rom_sector_buf[offset];
}


void gb_lcd_draw_line(struct gb_s* gb, const uint8_t pixels[160], const uint_fast8_t line)
{
    // Your original code to fill the 160-pixel buffer. This is known to work.
    for (int i = 0; i < 160; i++) {
        line_buffer[i] = gb_color_map[pixels[i] & 3];
    }

    // Your original code to calculate the centered start coordinates. This is known to work.
    uint16_t start_x = (320 - 160) / 2;
    uint16_t start_y = (240 - 144) / 2;

    // --- NEW LOGIC: DRAW EVERYTHING TWICE ---

    // 1. Calculate the Y position for the top row of our scaled pixel.
    uint16_t y0 = start_y + (line * 2);
    // 2. Calculate the Y position for the bottom row of our scaled pixel.
    uint16_t y1 = y0 + 1;

    // 3. Draw the line at the top position (if it's on screen).
    if (y0 < 240) {
        ILI9341_SetAddress(start_x, y0, start_x + 159, y0);
        ILI9341_WriteBuffer((uint8_t*)line_buffer, 160 * 2);
    }

    // 4. Draw the EXACT SAME line again at the bottom position (if it's on screen).
    if (y1 < 240) {
        ILI9341_SetAddress(start_x, y1, start_x + 159, y1);
        ILI9341_WriteBuffer((uint8_t*)line_buffer, 160 * 2);
    }
}
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();
  ILI9341_SetRotation(SCREEN_HORIZONTAL_2);
  ILI9341_FillScreen(BLACK);
  SD_DMA_InitDummy();
  FATFS fs; FRESULT res;
  res = f_mount(&fs, "", 1);
  if (res != FR_OK) {}

  res = f_open(&rom_file, "tetris.gb", FA_READ);
  if (res != FR_OK) {}

  rom_sector_valid = 0;
  rom_file_pos = 0;
  (void)gb_rom_read(NULL, 0); /* warm sector 0 into cache */
  enum gb_init_error_e init_res;
  init_res = gb_init(&gb_instance, &gb_rom_read,NULL,NULL,NULL, NULL);
  if (init_res != GB_INIT_NO_ERROR) {}

  gb_init_lcd(&gb_instance, &gb_lcd_draw_line);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      for(int i = 0; i < 5; i++)
      {
          gb_instance.direct.joypad = 0xFF;           // Start with all released
          gb_instance.direct.joypad &= ~JOYPAD_START; // Press START
          gb_run_frame(&gb_instance);
      }

      // --- Simulate Releasing START for 30 frames ---
      for(int i = 0; i < 30; i++)
      {
          gb_instance.direct.joypad = 0xFF; // All buttons released
          gb_run_frame(&gb_instance);
      }    /* USER CODE END WHILE */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, lcd_dc_Pin|lcd_rst_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, lcd_cs_Pin|SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : lcd_dc_Pin lcd_rst_Pin lcd_cs_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = lcd_dc_Pin|lcd_rst_Pin|lcd_cs_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2) {
        sd_dma_done = 1;   // flag that DMA transfer finished
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
