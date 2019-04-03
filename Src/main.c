/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma2d.h"
#include "gfxsimulator.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"
#include "../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_gyroscope.h"
#include "../Drivers/BSP/STM32F429I-Discovery/Fonts/fonts.h"
#include "Hehe.h"
#include "Menu.h"
#include "Menu_kontynuuj.h"
#include "Menu_nowagra.h"
#include "Menu_poziomy.h"
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
volatile ZyroskopDane DataOld, DataNow;
volatile int32_t AngleX, AngleY, AngleZ;
float dT;
uint8_t Animacja;
volatile uint16_t X, Y;
volatile uint8_t Direction;
volatile uint8_t fMovedX;
volatile uint8_t fMovedY;
volatile uint16_t ResetTimeX;
volatile uint16_t ResetTimeY;

DMA2D_HandleTypeDef hdma2d;
I2C_HandleTypeDef hi2c3;
LTDC_HandleTypeDef hltdc;
SPI_HandleTypeDef hspi5;
SDRAM_HandleTypeDef hsdram1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t spi5_sendrecv(uint8_t byte) {
	uint8_t answer;

	HAL_SPI_TransmitReceive(&hspi5, &byte, &answer, 1, HAL_MAX_DELAY);

	return answer;
}

uint8_t SPI5_read(uint8_t address) {
	uint8_t dane;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	spi5_sendrecv(address | 0x80);
	dane = spi5_sendrecv(0xFF);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

	return dane;
}

void SPI5_write(uint8_t address, uint8_t data) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	spi5_sendrecv(address);
	data = spi5_sendrecv(data);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

}

uint8_t OurL3GD20_Init() {
	if (SPI5_read(0x0F) != 0b11010100) {

		return 1;
	}

	//Enable L3GD20 Power bit
	SPI5_write(0x20, 0xFF);

	//Set L3GD20 scale

	SPI5_write(0x23, 0x00);

	//Set high-pass filter settings
	SPI5_write(0x21, 0x00);

	//Enable high-pass filter
	SPI5_write(0x24, 0x10);



	//Everything OK
	return 0;
}

void OurL3GD20_Read() {
	float s;
	short temp1, temp2, temp3;

	// Read X axis

	temp1 = (SPI5_read(0x28) | SPI5_read(0x29) << 8);
	temp2 = (SPI5_read(0x2A) | SPI5_read(0x2B) << 8);
	temp3 = (SPI5_read(0x2C) | SPI5_read(0x2D) << 8);

	// Sensitivity at 250 range = 8.75 mdps/digit
	s = 8.75 * 0.001;

	DataNow.OsX = (short) ((float) temp1 * s);
	DataNow.OsY = (short) ((float) temp2 * s);
	DataNow.OsZ = (short) ((float) temp3 * s);

}

void send_char(char c) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &c, 1, 1000);
}

int __io_putchar(int ch) {
	send_char(ch);
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
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_GFXSIMULATOR_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);
	__HAL_SPI_ENABLE(&hspi5);

	Animacja = 0;
	Direction = 1;
	X = 120;
	Y = 170;
	dT = 0.001;
	fMovedX = 0;
	fMovedY = 0;
	ResetTimeX = 0;
	ResetTimeY = 0;

	OurL3GD20_Init();

	BSP_LCD_Init();
	//BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER+1024*1024*4);
	BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);

	BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

	BSP_LCD_DisplayOn();
	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image_data_Menu);
	HAL_Delay(2000);
	BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image_data_Menu_nowagra);
	HAL_Delay(2000);
	BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image_data_Menu_kontynuuj);
	HAL_Delay(2000);
	BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image_data_Menu_poziomy);
	//BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
	//BSP_LCD_DisplayStringAtLine(5, (uint8_t*) "Hello");

	HAL_Delay(2000);
	BSP_LCD_ClearStringLine(5);

	BSP_LCD_Clear(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(0, 0, 240, 15);
	BSP_LCD_FillRect(0, 305, 240, 15);
	BSP_LCD_FillRect(0, 15, 15, 290);
	BSP_LCD_FillRect(225, 15, 15, 290);


	Animacja = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		printf("Angle X: %li\n\r", AngleX);
		printf("Angle Y: %li\n\r", AngleY);
		printf("Angle Z: %li\n\r", AngleZ);
		printf("CzasX: %d\n\r", ResetTimeX);
		printf("CzasY: %d\n\r", ResetTimeY);
		printf("Predkosc X: %d\n\r", DataNow.OsX);
		printf("Predkosc Y: %d\n\r", DataNow.OsY);
		//printf("OsX: %d\n\r", Data.OsX);
		//printf("OsY: %d\n\r", Data.OsY);
		//printf("OsZ: %d\n\r", Data.OsZ);
		HAL_Delay(200);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM10) //Przerwanie pochodzi od timera 10
	{
		if (Animacja == 1)
		{
			if ((Y < 300) && (Direction == 1)) {

				BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

				if(X <= 25 && Y <= 25)
					BSP_LCD_FillRect(X-10, Y - 10, 40, 40);
				else if(X >= 215 && Y >= 295)
					BSP_LCD_FillRect(X-20, Y - 20, 30, 30);
				else if(X <= 25 && Y >= 295)
					BSP_LCD_FillRect(X-10, Y - 20, 40, 30);
				else if(X >= 215 && Y <= 25)
					BSP_LCD_FillRect(X-20, Y - 10, 30, 40);
				else if(X <= 25)
					BSP_LCD_FillRect(X-10, Y - 20, 40, 40);
				else if(X >= 215)
					BSP_LCD_FillRect(X-20, Y - 20, 30, 40);
				else if(Y >= 295)
					BSP_LCD_FillRect(X-20, Y - 20, 40, 30);
				else if(Y <= 25)
					BSP_LCD_FillRect(X-20, Y - 10, 40, 40);
				else
					BSP_LCD_FillRect(X-20, Y - 20, 40, 40);



				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

				//Y += 10;
				BSP_LCD_FillCircle(X, Y, 10);
				BSP_LCD_FillRect(0, 0, 240, 15);
				BSP_LCD_FillRect(0, 305, 240, 15);
				BSP_LCD_FillRect(0, 15, 15, 290);
				BSP_LCD_FillRect(225, 15, 15, 290);

			} else if ((Y >= 300) && (Direction == 1)) {
				Direction = 0;

				BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				if(X <= 25 && Y <= 25)
					BSP_LCD_FillRect(X-10, Y - 10, 40, 40);
				else if(X >= 215 && Y >= 295)
					BSP_LCD_FillRect(X-20, Y - 20, 30, 30);
				else if(X <= 25 && Y >= 295)
					BSP_LCD_FillRect(X-10, Y - 20, 40, 30);
				else if(X >= 215 && Y <= 25)
					BSP_LCD_FillRect(X-20, Y - 10, 30, 40);
				else if(X <= 25)
					BSP_LCD_FillRect(X-10, Y - 20, 40, 40);
				else if(X >= 215)
					BSP_LCD_FillRect(X-20, Y - 20, 30, 40);
				else if(Y >= 295)
					BSP_LCD_FillRect(X-20, Y - 20, 40, 30);
				else if(Y <= 25)
					BSP_LCD_FillRect(X-20, Y - 10, 40, 40);
				else
					BSP_LCD_FillRect(X-20, Y - 20, 40, 40);


				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

				//Y -= 10;
				BSP_LCD_FillCircle(X, Y, 10);
				BSP_LCD_FillRect(0, 0, 240, 15);
				BSP_LCD_FillRect(0, 305, 240, 15);
				BSP_LCD_FillRect(0, 15, 15, 290);
				BSP_LCD_FillRect(225, 15, 15, 290);

			} else if ((Y > 20) && (Direction == 0)) {

				BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				if(X <= 25 && Y <= 25)
					BSP_LCD_FillRect(X-10, Y - 10, 40, 40);
				else if(X >= 215 && Y >= 295)
					BSP_LCD_FillRect(X-20, Y - 20, 30, 30);
				else if(X <= 25 && Y >= 295)
					BSP_LCD_FillRect(X-10, Y - 20, 40, 30);
				else if(X >= 215 && Y <= 25)
					BSP_LCD_FillRect(X-20, Y - 10, 30, 40);
				else if(X <= 25)
					BSP_LCD_FillRect(X-10, Y - 20, 40, 40);
				else if(X >= 215)
					BSP_LCD_FillRect(X-20, Y - 20, 30, 40);
				else if(Y >= 295)
					BSP_LCD_FillRect(X-20, Y - 20, 40, 30);
				else if(Y <= 25)
					BSP_LCD_FillRect(X-20, Y - 10, 40, 40);
				else
					BSP_LCD_FillRect(X-20, Y - 20, 40, 40);

				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

				//Y -= 10;
				BSP_LCD_FillCircle(X, Y, 10);
				BSP_LCD_FillRect(0, 0, 240, 15);
				BSP_LCD_FillRect(0, 305, 240, 15);
				BSP_LCD_FillRect(0, 15, 15, 290);
				BSP_LCD_FillRect(225, 15, 15, 290);

			} else if ((Y <= 20) && (Direction == 0)) {
				Direction = 1;

				BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				if(X <= 25 && Y <= 25)
					BSP_LCD_FillRect(X-10, Y - 10, 40, 40);
				else if(X >= 215 && Y >= 295)
					BSP_LCD_FillRect(X-20, Y - 20, 30, 30);
				else if(X <= 25 && Y >= 295)
					BSP_LCD_FillRect(X-10, Y - 20, 40, 30);
				else if(X >= 215 && Y <= 25)
					BSP_LCD_FillRect(X-20, Y - 10, 30, 40);
				else if(X <= 25)
					BSP_LCD_FillRect(X-10, Y - 20, 40, 40);
				else if(X >= 215)
					BSP_LCD_FillRect(X-20, Y - 20, 30, 40);
				else if(Y >= 295)
					BSP_LCD_FillRect(X-20, Y - 20, 40, 30);
				else if(Y <= 25)
					BSP_LCD_FillRect(X-20, Y - 10, 40, 40);
				else
					BSP_LCD_FillRect(X-20, Y - 2co0, 40, 40);

				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

				//Y += 10;
				BSP_LCD_FillCircle(X, Y, 10);
				BSP_LCD_FillRect(0, 0, 240, 15);
				BSP_LCD_FillRect(0, 305, 240, 15);
				BSP_LCD_FillRect(0, 15, 15, 290);
				BSP_LCD_FillRect(225, 15, 15, 290);
			}

		}
	}

	if (htim->Instance == TIM11) {
		OurL3GD20_Read();
//
		if((DataNow.OsX >= 25 && DataNow.OsY <= 20) || (DataNow.OsX <= -25 && DataNow.OsY >= -20))
			AngleX += (long)(DataNow.OsX + ((DataOld.OsX - DataNow.OsX)*0.5));
		else if((DataNow.OsX >= 25 && DataNow.OsY >= 25) || (DataNow.OsX <= -25 && DataNow.OsY <= -25))
			AngleX += (long)(DataNow.OsX + ((DataOld.OsX - DataNow.OsX)*0.5));

		if((DataNow.OsY >= 25 && DataNow.OsX <= 20) || (DataNow.OsY <= -25 && DataNow.OsX >= -20))
			AngleY += (long)(DataNow.OsY + ((DataOld.OsY - DataNow.OsY)*0.5));
		else if((DataNow.OsX >= 25 && DataNow.OsY >= 25) || (DataNow.OsX <= -25 && DataNow.OsY <= -25))
			AngleY += (long)(DataNow.OsY + ((DataOld.OsY - DataNow.OsY)*0.5));

		DataOld = DataNow;
// movement of the ball
		if(AngleY > 10000)
		{
			fMovedX = 1;
			fMovedY = 1;

			if(X < 215)
				X += 1;

		}else if(AngleY < -10000)
		{
			fMovedX = 1;
			fMovedY = 1;

			if(X > 25)
				X -= 1;

		}

		if(AngleX > 10000)
		{
			fMovedX = 1;
			fMovedY = 1;

			if(Y < 295)
				Y += 1;

		}else if(AngleX < -10000)
		{
			fMovedX = 1;
			fMovedY = 1;

			if(Y > 25)
				Y -= 1;

		}

		if(fMovedY == 1 && (AngleY <= 10000 && AngleY >= -10000)||(AngleY>20000 ||AngleY<-20000))
		{
			ResetTimeY += 1;

		}else if(fMovedY == 1 && (AngleY > 10000 || AngleY < -10000))
		{
			ResetTimeY = 0;
		}


		if(fMovedX == 1 && (AngleX <= 10000 && AngleX >= -10000)||(AngleX>20000 ||AngleX<-20000))
		{
			ResetTimeX += 1;

		}else if(fMovedX == 1 && (AngleX > 10000 || AngleX < -10000))
		{
			ResetTimeX = 0;
		}

		if(ResetTimeX >= 1000)
		{
			AngleX = 0;

			ResetTimeX = 0;
			fMovedX = 0;
		}

		if(ResetTimeY >= 1000)
		{
			AngleY = 0;

			ResetTimeY = 0;
			fMovedY = 0;
		}


	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
