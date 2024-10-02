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
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "stdbool.h"

#define APP_START_ADDRESS 0x0800C000
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint8_t RX_Data_1 = 0;						// Буфер для считывания регистра состояния внешней флеш
uint8_t RX_Data_256 [256] = {0};			// Буфер для считывания страницы внешней флеш
uint8_t Read_Status_Register_cmd = 0x05;    // Может не нужен?
uint8_t Read_Data_cmd = 0x03;				// Команда считывания из внещней флеш
uint8_t TX_Read_Data [4];					// Команда считывания из внешней флеш + адрес (3 байта)

uint32_t firmware_size = 0;					// Размер прошивки во внешней флеш в байтах
uint32_t SectorError = 0; 					// Ошибка сектора указывает на переменную, которая содержит
											// информацию о конфигурации поврежденного сектора в случае
											// ошибки (0xFFFFFFFFu означает, что все сектора были правильно удалены).

uint32_t MAX_flash_addr = 0x03FF00;			// До этого адреса сканируем внешнюю флеш на окончание прошивки
uint32_t Pages_counter = 0;					// Сколько страниц считали/вывели в SWV
uint32_t FF_counter = 0;					//

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

//void Read_Page (uint32_t addr);
void Read_Data(uint32_t addr);				// Считывает страницу с внешней флен в RX_Data_256
int Check_BUSY_bit (void);					// Считывает бит "занято" регистра состояния внешней флеш
uint32_t Find_end_of_firmware(void);		// Находит 32 байта 0xFF идущих подряд
void Erase_MCU (void);						// Стирает внутреннюю флеш
void Write_MCU (void);						// Прошивает внутреннюю флеш

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	ITM_SendChar(ch);
	return ch;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  printf("Bootloader begin\n");														// Начало бутлодера, включаем диод
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  while(Check_BUSY_bit()){															// Проверка что внешняя флеш не занята никаким процессом
	  printf("BUSY \n");
	  HAL_Delay(50);
  }

  firmware_size = Find_end_of_firmware();
  if(firmware_size == UINT32_MAX) printf("Error! Couldn'n find end of firmware\n");
  else printf("Programm size %" PRId32 " \n", firmware_size);

  if(!HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin)) {
	  printf("Start erasing MCU \n");
	  Erase_MCU();
  } else {
	  printf("Start writing MCU \n");

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	  Write_MCU();
  }



  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);		// Выключаем диод. Бутлодер кончился, переходим на основную прошивку
  printf("Bootloader end\n");

  uint32_t appJumpAddress;
  appJumpAddress = * ((volatile uint32_t*)(APP_START_ADDRESS + 4));

  HAL_SPI_DeInit(&hspi1);
  HAL_RCC_DeInit();
  HAL_DeInit();

  void(*GoToApp)(void);
  GoToApp = (void (*)(void)) appJumpAddress;

  __disable_irq();
  __set_MSP(*((volatile uint32_t*)APP_START_ADDRESS));
  GoToApp();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blue_LED_Pin */
  GPIO_InitStruct.Pin = Blue_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Blue_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SPI_Pin */
  GPIO_InitStruct.Pin = CS_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int Check_BUSY_bit (void){
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);						// read status register
	HAL_SPI_Transmit(&hspi1, &Read_Status_Register_cmd, sizeof(Read_Status_Register_cmd), 1000);
	HAL_SPI_Receive(&hspi1, &RX_Data_1, sizeof(RX_Data_1), 1000);
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	//HAL_Delay(50);

	return (RX_Data_1 & 1);
}

void Read_Data(uint32_t addr){
	TX_Read_Data [0] = Read_Data_cmd;
	TX_Read_Data [1] = (uint8_t)(addr >> 16) & 0xFF;
	TX_Read_Data [2] = (uint8_t)(addr >> 8) & 0xFF;
	TX_Read_Data [3] = (uint8_t)(addr & 0xFF);

	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_RESET);						// read first page
	HAL_SPI_Transmit(&hspi1, TX_Read_Data, sizeof(TX_Read_Data), 1000);
	HAL_SPI_Receive(&hspi1, RX_Data_256, sizeof(RX_Data_256), 1000);
	HAL_GPIO_WritePin(CS_SPI_GPIO_Port, CS_SPI_Pin, GPIO_PIN_SET);
	//HAL_Delay(30);
}

uint32_t Find_end_of_firmware(void) {
	while(MAX_flash_addr > firmware_size) {								// Читаем и выводим постранично
		Read_Data(firmware_size);

		printf("Page: %" PRId32 " \n", Pages_counter);					// Выводим в формате 16h побайтово по странице
		for(int x = 0; x < 16; x++){
	  		for(int y = 0; y < 16; y++){				 					// Квадратом 16 на 16
	  			printf("0x%02X ", RX_Data_256[y + x*16]);
	  			if(RX_Data_256[y + x*16] == 0xFF) {							// Ищем конец файла строку 0xFF
	  				FF_counter++;

	  				if(FF_counter == 16) {
	  					firmware_size = firmware_size + (x * 16);
	  					printf("\n");
	  					return firmware_size;
	  				}
	  			}
	  	  }
	  	  FF_counter = 0;
	  	  printf("\n");
	  	}
		printf("\n");
	  	firmware_size += 0x100;
	  	Pages_counter++;
	  }
	printf("\n");
    return UINT32_MAX;
}

void Erase_MCU (void){
	__disable_irq();

	FLASH_EraseInitTypeDef eraseInitStruct;
	eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInitStruct.Banks = FLASH_BANK_1;
	eraseInitStruct.Sector = FLASH_SECTOR_3;
	eraseInitStruct.NbSectors = 1;
	eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&eraseInitStruct, &SectorError);
	HAL_FLASH_Lock();

	__enable_irq();
}

void Write_MCU (void){
	uint32_t buff_to_write = 0;					// Будем записывать словами по 32 бита
	uint32_t bytes_written = 0;					// Сколько байт записали во внутреннюю флеш

	__disable_irq();
	HAL_FLASH_Unlock();

	for(int x = 0; x < firmware_size; x += 256){
		Read_Data(x);
		for(int y = 0; y < 256; y += 4) {
			buff_to_write = (uint32_t) (RX_Data_256[y+3] << 24)|(RX_Data_256[y+2] << 16)|(RX_Data_256[y+1] << 8)|(RX_Data_256[y] << 0);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_START_ADDRESS + (bytes_written), buff_to_write);
			bytes_written += 4;
			buff_to_write = 0;
		}
		printf("bytes_written %" PRId32 " \n", bytes_written);
	}
	if(firmware_size - bytes_written > 0) {
		Read_Data(bytes_written);
		for(int y = 0; y < (firmware_size % 256); y += 4) {
			buff_to_write = (uint32_t) (RX_Data_256[y] << 0)|(RX_Data_256[y+1] << 8)|(RX_Data_256[y+2] << 16)|(RX_Data_256[y+3] << 24);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_START_ADDRESS + (bytes_written), buff_to_write);
			bytes_written += 4;
		}
	}

	HAL_FLASH_Lock();
	__enable_irq();
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
