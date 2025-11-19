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
#include "sd_functions.h"
#include <stdio.h> // Required for snprintf
#include <string.h> // Required for strcmp
#define PEANUT_GB_HIGH_LCD_ACCURACY 0 // Add this line
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

#define ROM_BUFFER_SIZE 70000


#define LCD_WIDTH  320
#define LCD_HEIGHT 240
#define GB_WIDTH   160
#define GB_HEIGHT  144

//Button Pin Definitions ---
#define BTN_UP_PORT     GPIOA
#define BTN_UP_PIN      GPIO_PIN_8
#define BTN_DOWN_PORT   GPIOA
#define BTN_DOWN_PIN    GPIO_PIN_9
#define BTN_LEFT_PORT   GPIOA
#define BTN_LEFT_PIN    GPIO_PIN_11
#define BTN_RIGHT_PORT  GPIOA
#define BTN_RIGHT_PIN   GPIO_PIN_10

#define BTN_A_PORT      GPIOA
#define BTN_A_PIN       GPIO_PIN_12
#define BTN_B_PORT      GPIOA
#define BTN_B_PIN       GPIO_PIN_15
#define BTN_SELECT_PORT GPIOB
#define BTN_SELECT_PIN  GPIO_PIN_3
#define BTN_START_PORT  GPIOB
#define BTN_START_PIN   GPIO_PIN_4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */
uint8_t rom_buffer[ROM_BUFFER_SIZE];
UINT rom_size = 0; // This will store the actual size of the loaded ROM file.
struct gb_s gb_instance;
uint16_t line_buffer[160];
static const uint16_t gb_color_map[4] = {
    GB_COLOR_WHITE, GB_COLOR_LIGHT_GRAY, GB_COLOR_DARK_GRAY, GB_COLOR_BLACK
};

// Two buffers for DMA Double Buffering
uint16_t dma_line_buffer[2][LCD_WIDTH];
volatile uint8_t buffer_index = 0;       // Which buffer is the CPU writing to?
volatile uint8_t spi_dma_busy = 0;       // Is DMA currently transmitting?
// Vertical Scaling Accumulator
int v_accumulator = 0;

// --- MENU DEFINITIONS ---
const char *game_list[] = {
    "mario.gb",
    "castalvania.gb",
    "tetris.gb",
    "Dr. Mario.gb"
};

// Calculate number of games automatically
const uint8_t TOTAL_GAMES = sizeof(game_list) / sizeof(game_list[0]);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void Error_Handler_LCD(const char* error_msg);
void update_joypad_state(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        spi_dma_busy = 0;
    }
}
static inline uint8_t gb_rom_read_from_ram(struct gb_s *gb, const uint_fast32_t addr)
{
    // Check that the address is within the bounds of the loaded ROM
    if (addr < rom_size)
    {
        return rom_buffer[addr];
    }

    // If the game tries to read outside the ROM area, return 0xFF.
    // This is not ideal but prevents a crash from out-of-bounds access.
    return 0xFF;
}

// LCD drawing function remains the same
void gb_lcd_draw_line(struct gb_s* gb, const uint8_t pixels[160], const uint_fast8_t line)
{
    // 1. VERTICAL SCALING LOGIC
    // We want to stretch 144 GB lines to 240 LCD lines.
    // Ratio is 1.66. We add 240 to accumulator every GB line.
    // While accumulator >= 144, we send a line to LCD and subtract 144.

    v_accumulator += LCD_HEIGHT;

    // If we don't need to draw this line yet (unlikely with upscaling, but good safety)
    if (v_accumulator < GB_HEIGHT) return;

    // 2. HORIZONTAL SCALING (160 -> 320)
    // This is the CPU intensive part, so we do it while the *previous* DMA might still be running.
    // We write to the "current" buffer (buffer_index).

    uint16_t *dest = dma_line_buffer[buffer_index];

    // Unrolling loop for speed: Pixel doubling 160 -> 320
    for (int i = 0; i < 160; i++)
    {
        uint16_t color = gb_color_map[pixels[i] & 3];
        // Write pixel twice to stretch width
        *dest++ = color;
        *dest++ = color;
    }

    // 3. DMA TRANSFER LOOP
    // Depending on the vertical scaling, we might need to send this ONE GB line
    // to the LCD multiple times (usually 1 or 2 times).

    while (v_accumulator >= GB_HEIGHT)
    {
        // Wait for previous DMA to finish if it's still busy
        while (spi_dma_busy) {};

        // Set Busy Flag
        spi_dma_busy = 1;

        // Send the current buffer via DMA
        // Note: We are sending 320 pixels * 2 bytes = 640 bytes
        HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)dma_line_buffer[buffer_index], LCD_WIDTH * 2);

        // Decrease accumulator
        v_accumulator -= GB_HEIGHT;
    }

    // Flip buffer index for the next line so CPU can write to the other buffer
    // while DMA finishes sending the current one.
    buffer_index = !buffer_index;
}
void Error_Handler_LCD(const char* error_msg)
{
    ILI9341_FillScreen(RED);
    ILI9341_DrawText("ERROR", FONT2, 10, 10, WHITE, RED);
    ILI9341_DrawText(error_msg, FONT2, 10, 40, WHITE, RED);
    while(1)
    {
        // Blink an LED or just hang
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
    }
}
int select_game_menu(void)
{
    int selected_index = 0;
    uint8_t prev_joypad = 0xFF; // Store previous state for edge detection
    uint8_t redraw = 1;         // Flag to prevent flickering

    ILI9341_FillScreen(BLACK);
    ILI9341_DrawText("SELECT GAME", FONT4, 60, 10, WHITE, BLACK);


    while (1)
    {
        // 1. Input Handling
        update_joypad_state();
        uint8_t joy = gb_instance.direct.joypad;

        // Logic 0 means pressed in PeanutGB joypad mapping
        int up_pressed    = !(joy & JOYPAD_UP);
        int down_pressed  = !(joy & JOYPAD_DOWN);
        int a_pressed     = !(joy & JOYPAD_A);
        int start_pressed = !(joy & JOYPAD_START);

        // Navigation (with simple delay for debounce)
        if (up_pressed)
        {
            selected_index--;
            if (selected_index < 0) selected_index = TOTAL_GAMES - 1;
            redraw = 1;
            HAL_Delay(150); // Slow down scrolling
        }

        if (down_pressed)
        {
            selected_index++;
            if (selected_index >= TOTAL_GAMES) selected_index = 0;
            redraw = 1;
            HAL_Delay(150);
        }

        // Selection
        if (a_pressed || start_pressed)
        {
            // Visual feedback
            ILI9341_DrawText("Loading...", FONT4, 80, 200, GREEN, BLACK);
            HAL_Delay(500);
            return selected_index;
        }

        // 2. Drawing Logic
        if (redraw)
        {
            for (int i = 0; i < TOTAL_GAMES; i++)
            {
                uint16_t color = (i == selected_index) ? YELLOW : WHITE;
                uint16_t bg    = BLACK;

                char buffer[30];
                // Add a cursor ">" for the selected item
                if (i == selected_index) {
                    snprintf(buffer, sizeof(buffer), "> %s", game_list[i]);
                } else {
                    snprintf(buffer, sizeof(buffer), "  %s", game_list[i]);
                }

                // Draw list starting at Y=60, spacing 20 pixels
                ILI9341_DrawText(buffer, FONT2, 20, 60 + (i * 20), color, bg);
            }
            redraw = 0;
        }

        HAL_Delay(10); // Small delay to save power
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();
  ILI9341_SetRotation(SCREEN_HORIZONTAL_2);
  ILI9341_FillScreen(BLACK);

  // --- NEW: Main Logic for loading ROM ---

  // 1. Mount the SD Card
  ILI9341_DrawText("Mounting SD Card...", FONT2, 10, 10, WHITE, BLACK);
  if (sd_mount() != FR_OK) {
      Error_Handler_LCD("SD Card Mount Failed!");
  }

  int game_idx = select_game_menu();
  const char* rom_filename = game_list[game_idx];

  // 2. Load the ROM file into the buffer
  //const char* rom_filename = "castalvania.gb";
  char msg_buffer[50];
  snprintf(msg_buffer, sizeof(msg_buffer), "Loading %s...", rom_filename);
  ILI9341_DrawText(msg_buffer, FONT2, 10, 30, WHITE, BLACK);

  // Use the sd_read_file function to load the entire file.
  // It will store the number of bytes actually read into the 'rom_size' variable.
  int result = sd_read_file(rom_filename, (char*)rom_buffer, ROM_BUFFER_SIZE, &rom_size);

  if (result != FR_OK) {
      snprintf(msg_buffer, sizeof(msg_buffer), "Failed to read %s", rom_filename);
      Error_Handler_LCD(msg_buffer);
  }

  if (rom_size == 0) {
      Error_Handler_LCD("ROM file is empty or not found!");
  }
   if (rom_size >= ROM_BUFFER_SIZE) {
      ILI9341_DrawText("Warning: ROM may be truncated!", FONT2, 10, 70, YELLOW, BLACK);
      HAL_Delay(2000);
  }

  snprintf(msg_buffer, sizeof(msg_buffer), "Loaded %u bytes.", (unsigned int)rom_size);
  ILI9341_DrawText(msg_buffer, FONT2, 10, 50, GREEN, BLACK);
  HAL_Delay(1000);

  // We are done with the SD card for now, so we can unmount it.
  sd_unmount();

  // 3. Initialize the Peanut-GB emulator
  ILI9341_FillScreen(BLACK);
  ILI9341_DrawText("Initializing Emulator...", FONT2, 10, 10, WHITE, BLACK);

  enum gb_init_error_e init_res;
  // Initialize the emulator, passing our new RAM-based read function
  init_res = gb_init(&gb_instance, &gb_rom_read_from_ram, NULL, NULL, NULL, NULL);

  if (init_res != GB_INIT_NO_ERROR) {
       Error_Handler_LCD("Emulator Init Failed!");
  }

  // Initialize the LCD component of the emulator
  gb_init_lcd(&gb_instance, &gb_lcd_draw_line);

  ILI9341_FillScreen(BLACK); // Clear screen for the game
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  update_joypad_state();

	        // --- RENDER FRAME ---

	        // 1. Reset Vertical Accumulator for the new frame
	        v_accumulator = 0;
	        buffer_index = 0;

	        // 2. Set LCD Window to Full Screen ONCE per frame
	        // This saves massive overhead compared to setting it per line
	        ILI9341_SetAddress(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

	        // Prepare ILI9341 for data stream (Write RAM command)
	        // You might need to expose the WriteRAM command from your library
	        // typically it is command 0x2C.
	        ILI9341_WriteCommand(0x2C);

	        // Set CS Low manually to keep stream open (if library handles CS per byte, this optimizes it)
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, PinState)

	        // 3. Run Emulator for one frame (Draws lines)
	        gb_instance.direct.frame_skip = 0; // Draw every frame we process
	        gb_run_frame(&gb_instance);

	        // 4. Wait for the very last DMA to finish before closing the frame
	        while (spi_dma_busy) {};

	        // Deselect CS
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

	        // --- FRAME SKIPPING LOGIC ---
	        // If the emulator is still too slow, we process a "blind" frame
	        // without drawing to catch up on audio/game logic.

	        update_joypad_state();
	        gb_instance.direct.frame_skip = 1; // Don't draw
	        gb_run_frame(&gb_instance);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void update_joypad_state(void)
{
    // Start with all buttons released (logic 1)
    uint8_t joypad_state = 0xFF;

    // Read each button. If a pin is LOW (GPIO_PIN_RESET), the button is pressed.
    // Clear the corresponding bit in the joypad state (set to 0).
    if (HAL_GPIO_ReadPin(BTN_RIGHT_PORT, BTN_RIGHT_PIN) == GPIO_PIN_RESET) joypad_state &= ~JOYPAD_RIGHT;
    if (HAL_GPIO_ReadPin(BTN_LEFT_PORT, BTN_LEFT_PIN) == GPIO_PIN_RESET)   joypad_state &= ~JOYPAD_LEFT;
    if (HAL_GPIO_ReadPin(BTN_UP_PORT, BTN_UP_PIN) == GPIO_PIN_RESET)       joypad_state &= ~JOYPAD_UP;
    if (HAL_GPIO_ReadPin(BTN_DOWN_PORT, BTN_DOWN_PIN) == GPIO_PIN_RESET)   joypad_state &= ~JOYPAD_DOWN;

    if (HAL_GPIO_ReadPin(BTN_A_PORT, BTN_A_PIN) == GPIO_PIN_RESET)         joypad_state &= ~JOYPAD_A;
    if (HAL_GPIO_ReadPin(BTN_B_PORT, BTN_B_PIN) == GPIO_PIN_RESET)         joypad_state &= ~JOYPAD_B;
    if (HAL_GPIO_ReadPin(BTN_SELECT_PORT, BTN_SELECT_PIN) == GPIO_PIN_RESET) joypad_state &= ~JOYPAD_SELECT;
    if (HAL_GPIO_ReadPin(BTN_START_PORT, BTN_START_PIN) == GPIO_PIN_RESET)   joypad_state &= ~JOYPAD_START;

    // Update the emulator's joypad variable
    gb_instance.direct.joypad = joypad_state;
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
