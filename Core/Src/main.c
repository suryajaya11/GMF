/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

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
uint8_t serial_byte;
uint8_t serial_ptr;
uint8_t serial_buf[50];
uint8_t sync = 0;

uint32_t unsync_rate = 0;
uint16_t wait_to_sync = 0;

uint16_t channels[16];
uint16_t sync_val = 26;
uint8_t frame_size;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* MCU Configu	ration--------------------------------------------------------*/

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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart4, &serial_byte, 1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(serial_ptr > 49){
		serial_ptr = 0;
		sync = 0;
		memset(serial_buf, 0, 50);
		unsync_rate++;
	}

	if(serial_byte == 0xc8 && sync == 0){
//		memset(serial_buf, 0, 50);
		serial_buf[serial_ptr] = serial_byte;
		serial_ptr = 1;
		sync = 1;
		return;
	}

	if(sync == 1){
		frame_size = serial_byte;
		serial_buf[serial_ptr] = serial_byte;
		serial_ptr++;
		sync = 2;
		return;
	}

	if(sync == 2){
		serial_buf[serial_ptr] = serial_byte;
		serial_ptr++;
		if(serial_ptr >= frame_size + 1){
			sync = 0;
			serial_ptr = 0;
//			frame_size = 0;

			//11 bit
			channels[0] = serial_buf[3] + ((serial_buf[4] & 0b111) << 8);
			channels[1] = (serial_buf[4] >> 3) + ((serial_buf[5] & 0b111111) << 5);
			channels[2] = (serial_buf[5] >> 6) + ((serial_buf[6] & 0xff) << 2) + ((serial_buf[7] & 0b1) << 10);
			channels[3] = (serial_buf[7] >> 1) + ((serial_buf[8] & 0b1111) << 7);

			channels[4] = (serial_buf[8] >> 4) + ((serial_buf[9] & 0b1111111) << 4);
			channels[5] = (serial_buf[9] >> 7) + (serial_buf[10] << 1) + ((serial_buf[11] & 0b11) << 9);
			channels[6] = (serial_buf[11] >> 2) + ((serial_buf[12] & 0b11111) << 6);
			channels[7] = (serial_buf[13] >> 5) + (serial_buf[14] << 3);

			channels[8] = serial_buf[15] + ((serial_buf[16] & 0b111) << 8);
			channels[9] = (serial_buf[16] >> 3) + ((serial_buf[17] & 0b111111) << 5);
			channels[10] = (serial_buf[17] >> 6) + ((serial_buf[18] & 0xff) << 2) + ((serial_buf[19] & 0b1) << 10);
			channels[11] = (serial_buf[19] >> 1) + ((serial_buf[20] & 0b1111) << 7);


			//10 bit
//			channels[0] = serial_buf[3] + ((serial_buf[4] & 0b11) << 8);
//			channels[1] = (serial_buf[4] >> 2) + ((serial_buf[5] & 0b1111) << 6);

			//12 bit
//			channels[0] = serial_buf[3] + ((serial_buf[4] & 0b1111) << 8);
//			channels[1] = (serial_buf[4] >> 2) + ((serial_buf[5] & 0b1111) << 4);
		}
	}



	//sync by reading sync byte -> very bad
	/*
	if(serial_byte == 0xc8){
		serial_ptr = 0;
		memset(serial_buf, 0, 50);
	}

	serial_buf[serial_ptr] = serial_byte;
	serial_ptr++;
*/


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
