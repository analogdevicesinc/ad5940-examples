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

uint32_t MCUPlatformInit(void *pCfg)
{
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
	return 1;
}

void USART2_IRQHandler(void)
{
  void UARTCmd_Process(char);
  volatile char c;
  if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE))
  {
    c = USART2->DR;
    UARTCmd_Process(c);
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

