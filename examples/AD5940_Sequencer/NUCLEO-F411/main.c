/*

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*/
#include "stdio.h"
#include "AD5940.h"
#include "stm32f4xx_hal.h"

/* Functions that used to initialize MCU platform */
uint32_t MCUPlatformInit(void *pCfg);

int main(void)
{
  void AD5940_Main(void);
  MCUPlatformInit(0);
  AD5940_MCUResourceInit(0);    /* Initialize resources that AD5940 use, like SPI/GPIO/Interrupt. */

  printf("Hello AD5940-Build Time:%s\n",__TIME__);
  AD5940_Main();
}

#define DEBUG_UART                         USART2
#define DEBUG_UART_IRQN                    USART2_IRQn
#define DEBUGUART_CLK_ENABLE()             __HAL_RCC_USART2_CLK_ENABLE()
#define DEBUGUART_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOA_CLK_ENABLE()

/* Definition for AD5940 Pins */
#define DEBUGUART_TX_PIN                   GPIO_PIN_2
#define DEBUGUART_TX_GPIO_PORT             GPIOA
#define DEBUGUART_TX_AF                    GPIO_AF7_USART2

#define DEBUGUART_RX_PIN                   GPIO_PIN_3
#define DEBUGUART_RX_GPIO_PORT             GPIOA
#define DEBUGUART_RX_AF                    GPIO_AF7_USART2

UART_HandleTypeDef UartHandle;

void Error_Handler(void){
  while(1);
}
/**
  * @brief SPI MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param husart: SPI handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *husart)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if(husart->Instance == DEBUG_UART)
  {     
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    DEBUGUART_GPIO_CLK_ENABLE();
    /* Enable UART clock */
    DEBUGUART_CLK_ENABLE();
    
    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = DEBUGUART_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = DEBUGUART_TX_AF;
    HAL_GPIO_Init(DEBUGUART_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = DEBUGUART_RX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Alternate = DEBUGUART_RX_AF;
    HAL_GPIO_Init(DEBUGUART_RX_GPIO_PORT, &GPIO_InitStruct);
  }
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

uint32_t MCUPlatformInit(void *pCfg)
{
  HAL_Init();
  SystemClock_Config();
  HAL_Init();
  /* Init UART */  
  UartHandle.Instance        = DEBUG_UART;

  UartHandle.Init.BaudRate   = 230400;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    return 0;
  }
  __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
  HAL_NVIC_EnableIRQ(DEBUG_UART_IRQN);
	return 1;
}

void USART2_IRQHandler(void)
{
  //void UARTCmd_Process(char);
  volatile char c;
  if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE))
  {
    c = USART2->DR;
    //UARTCmd_Process(c);
  }
}

#include "stdio.h"
#ifdef __ICCARM__
int putchar(int c)
#else
int fputc(int c, FILE *f)
#endif
{
  uint8_t t = c;
  HAL_UART_Transmit(&UartHandle, &t, 1, 1000); 
  return c;
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}
