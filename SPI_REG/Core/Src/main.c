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
#include <stdlib.h>

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

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void gpio_cnfg(){
	RCC->AHB2ENR |= (1 << 1); // enable GPIOB CLK
	RCC->APB1ENR1 |= (1 << 14); // enable GPIOB CLK
}

void gpio_cnfg_direct(){
	 (*(volatile uint32_t*)0x4002104C) |= (1 << 1); // enable GPIOB CLK
	 (*(volatile uint32_t*)0x4002104C) |= (1 << 0); // enable GPIOA CLK
	 (*(volatile uint32_t*)0x40021058) |= (1 << 1); // enable GPIOB CLK
}

void spi_cnfg_direct(){
	uint32_t SET_state   = 0XA8000000;
	uint32_t reset_state = 0X3FFFFFF;
	(*(volatile uint32_t*)0x48000400) &= reset_state; //GPIOB->MODER RST
	(*(volatile uint32_t*)0x48000400) |= SET_state;  // GPIOB->MODERE SET AF

	uint32_t reset_AFHR = 0xFFFFF;
	uint32_t set_AFHR = 0x55500000;
	(*(volatile uint32_t*)0x48000424) &= reset_AFHR; //GPIOB->MODER RST
	(*(volatile uint32_t*)0x48000424) |= set_AFHR;  //set AFHR->AF5 for SPI2

	//configure CR1 and CR2
	uint16_t CR1_RESET = 0x3904;
	uint16_t CR1_PRESCALER =  0x38; // prescaler 256
	uint16_t CR1_MODE = 0x4;        // MASTER mode
	(*(volatile uint32_t*)0x40003800) &= CR1_RESET; //reset SPI2->CR1
	(*(volatile uint32_t*)0x40003800) |= CR1_PRESCALER | CR1_MODE; //SPI2->CR1 CNFG

	//configure CR2
	uint16_t CR2_RESET = 0x0700;
	uint16_t CR2_SET = 0x0700;
	(*(volatile uint32_t*)0x40003804) &= CR2_RESET;
	(*(volatile uint32_t*)0x40003804) |= CR2_SET;
}



void spi_cnfg(){

	/*--------------------------------GPIOB MODE REGISTERS-------------------------------------------*/
	//GPIOB
	uint32_t PB15_SET_AF = ((uint32_t)1 << 31); //set to AF
	uint32_t PB14_SET_AF = ((uint32_t)1 << 29);
	uint32_t PB13_SET_AF = ((uint32_t)1 << 27);
	uint32_t PB1_SET_OUT = ((uint32_t)1  << 2);
	uint32_t PB15_RESET = ~(((uint32_t)1 << 31 | (uint32_t)1 << 30));
	uint32_t PB14_RESET = ~(((uint32_t)1 << 29 | (uint32_t)1 << 28));
	uint32_t PB13_RESET = ~(((uint32_t)1 << 27 | (uint32_t)1 << 26));
	uint32_t PB1_RESET  = ~(((uint32_t)1 << 3 | (uint32_t)1 << 2));

	GPIOB->MODER &= (PB15_RESET & PB14_RESET & PB13_RESET & PB1_RESET);
	GPIOB->MODER |= (PB15_SET_AF | PB14_SET_AF | PB13_SET_AF | PB1_SET_OUT);

	//AFR REG
	uint32_t PB15_AFHR_RESET = ~((uint32_t)1 << 31 | (uint32_t)1 << 30 | (uint32_t)1 << 29 | (uint32_t)1 << 28);
	uint32_t PB14_AFHR_RESET = ~((uint32_t)1 << 27 | (uint32_t)1 << 26 | (uint32_t)1 << 25 | (uint32_t)1 << 24);
	uint32_t PB13_AFHR_RESET = ~((uint32_t)1 << 23 | (uint32_t)1 << 22 | (uint32_t)1 << 21 | (uint32_t)1 << 20);
	uint32_t AF5 = ((uint32_t)1 << 0 | 1 << 2); // SET AF5

	GPIOB->AFR[1] &= (PB15_AFHR_RESET & PB14_AFHR_RESET & PB13_AFHR_RESET);
	GPIOB->AFR[1] |= (AF5 << 28 | AF5 << 24 | AF5 << 20);

	/*--------------------------------SPI CTRL REGISTERS-------------------------------------------*/

	//CR1 REG
	uint16_t CR1_RESET = ~(uint16_t)(
			(uint16_t)1 << 15 | (uint16_t)1 << 14 | (uint16_t)1 << 10|
			(uint16_t)1 << 9  | (uint16_t)1 << 7  | (uint16_t)1 << 6 |
			(uint16_t)1 << 5  | (uint16_t)1 << 4  | (uint16_t)1 << 3 |
			(uint16_t)1 << 1  | (uint16_t)1 << 0
	);

	uint16_t CR1_PRESCALER = ((uint16_t)1 << 5 | (uint16_t)1 << 4 | (uint16_t)1 << 3); // PRSCL 256
	uint16_t CR1_MODE = (1 << 2); // SPI2 MASTER mode

	SPI2->CR1 &= CR1_RESET;
	SPI2->CR1 |= CR1_PRESCALER | CR1_MODE;

	//CR2 REG
	uint16_t CR2_RESET = 0x0700;
	uint32_t SPI2_DS = (1 << 0 | 1 << 1 | 1 << 2); // data size - 8 bits
	SPI2->CR2 &= CR2_RESET;
	SPI2->CR2 |= SPI2_DS;

	//enable interrupts
	NVIC_EnableIRQ(SPI2_IRQn);
	NVIC_SetPriority(SPI2_IRQn, 1);

}

void DMA_cnfg(){
	//DMAx_Channelx->CPAR = (uint32_t)&(SPI2->DR);
}

volatile uint8_t* buffer;
volatile int16_t buffer_len;
volatile uint8_t busy = 0;
volatile uint8_t* rx_buffer;

void SPI2_IRQHandler(){
	//check if empty
	if(SPI2->SR & ((uint16_t)1 << 1) && buffer_len > 0){
		SPI2->DR = *(buffer++);
		buffer_len -= 1;
	}
	if(buffer_len <= 0){
		busy = 0;
		GPIOB->BSRR |= ((uint16_t)1 << 1); //pull CS 1 HIGH
		SPI2->CR2 &= ~(1 << 7);  //disable TEXEIE
	}
}

void transfer_function(uint8_t* tx_data, uint8_t len){
	if (busy == 1) return; //return if BUS BUSY
	GPIOB->BSRR |= ((uint16_t)1 << 17); //pull CS low
	buffer = tx_data;
	buffer_len = len;
	rx_buffer
	busy = 1;
	SPI2->CR2 |= (1 << 7); //ENABLE TEXEIE INTERUPT
	SPI2->CR1 |= (1 << 6); //ENABLE SPI2
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	gpio_cnfg();
    //spi_cnfg();
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

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
  	  	  	gpio_cnfg_direct();
// 		    uint32_t PB15_SET_AF = (1u << 31);
// 		 	uint32_t PB14_SET_AF = (1u << 29);
// 		 	uint32_t PB13_SET_AF = (1u << 27);
// 		 	uint32_t PB15_RESET = ~((1 << 31 | 1 << 30));
// 		 	uint32_t PB14_RESET = ~((1 << 29 | 1 << 28));
// 		 	uint32_t PB13_RESET = ~((1 << 27 | 1 << 26));
// 		 	GPIOB->MODER &= (PB15_RESET & PB14_RESET & PB13_RESET);
// 		 	GPIOB->MODER |= (PB15_SET_AF | PB14_SET_AF | PB13_SET_AF);

 		   spi_cnfg();

	  //now configure SPI

  uint8_t* data = (uint8_t*)malloc(sizeof(uint8_t) * 1);
  uint8_t* receive = (uint8_t*)malloc(sizeof(uint8_t) * 1);
  while (1)
  {
	  transfer_function(data, 200);
	  HAL_Delay(1);
	 // HAL_Delay(50);
//	  SPI2->CR1 |= (1 << 6);
//	  while (!(SPI2->SR & (1 << 1)));
//	  SPI2->DR = 0xAA;
//	  while (SPI2->SR & (1 << 7));
//	  SPI2->CR1 &= ~(1 << 6);
	 // GPIOB->MODER &= 0;
	  //transfer_function(7);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
	  GPIOB->MODER &= (uint32_t)0;
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
