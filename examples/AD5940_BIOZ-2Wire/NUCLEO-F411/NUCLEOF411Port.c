/**  
 * @file       NUCLEOF411Port.c
 * @brief      ST NUCLEOF411 board port file.
 * @version    V0.2.0
 * @author     ADI
 * @date       March 2019
 * @par Revision History:
 * 
 * Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
 * 
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
**/
#include "ad5940.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"

/* Definition for STM32 SPI clock resources */
#define AD5940SPI                          SPI1
#define AD5940_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define AD5940_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define AD5940_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define AD5940_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define AD5940_CS_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define AD5940_RST_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define AD5940_GP0INT_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()

#define AD5940SPI_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define AD5940SPI_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for AD5940 Pins */
#define AD5940_SCK_PIN                     GPIO_PIN_5
#define AD5940_SCK_GPIO_PORT               GPIOA
#define AD5940_SCK_AF                      GPIO_AF5_SPI1
#define AD5940_MISO_PIN                    GPIO_PIN_6
#define AD5940_MISO_GPIO_PORT              GPIOA
#define AD5940_MISO_AF                     GPIO_AF5_SPI1
#define AD5940_MOSI_PIN                    GPIO_PIN_7
#define AD5940_MOSI_GPIO_PORT              GPIOA
#define AD5940_MOSI_AF                     GPIO_AF5_SPI1

#define AD5940_CS_PIN                      GPIO_PIN_6
#define AD5940_CS_GPIO_PORT                GPIOB

#define AD5940_RST_PIN                     GPIO_PIN_0   //A3
#define AD5940_RST_GPIO_PORT               GPIOB

#define AD5940_GP0INT_PIN                  GPIO_PIN_10   //A3
#define AD5940_GP0INT_GPIO_PORT            GPIOA
#define AD5940_GP0INT_IRQn                 EXTI15_10_IRQn

SPI_HandleTypeDef  SpiHandle;

#define SYSTICK_MAXCOUNT ((1L<<24)-1) /* we use Systick to complete function Delay10uS(). This value only applies to NUCLEOF411 board. */
#define SYSTICK_CLKFREQ   100000000L  /* Systick clock frequency in Hz. This only appies to NUCLEOF411 board */
volatile static uint8_t ucInterrupted = 0;       /* Flag to indicate interrupt occurred */

/**
	@brief Using SPI to transmit N bytes and return the received bytes. This function targets to 
                     provide a more efficent way to transmit/receive data.
	@param pSendBuffer :{0 - 0xFFFFFFFF}
      - Pointer to the data to be sent.
	@param pRecvBuff :{0 - 0xFFFFFFFF}
      - Pointer to the buffer used to store received data.
	@param length :{0 - 0xFFFFFFFF}
      - Data length in SendBuffer.
	@return None.
**/
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,unsigned char *pRecvBuff,unsigned long length)
{
  HAL_SPI_TransmitReceive(&SpiHandle, pSendBuffer, pRecvBuff, length, (uint32_t)-1);
}

void AD5940_CsClr(void)
{
  HAL_GPIO_WritePin(AD5940_CS_GPIO_PORT, AD5940_CS_PIN, GPIO_PIN_RESET);
}

void AD5940_CsSet(void)
{
  HAL_GPIO_WritePin(AD5940_CS_GPIO_PORT, AD5940_CS_PIN, GPIO_PIN_SET);
}

void AD5940_RstSet(void)
{
  HAL_GPIO_WritePin(AD5940_RST_GPIO_PORT, AD5940_RST_PIN, GPIO_PIN_SET);
}

void AD5940_RstClr(void)
{
  HAL_GPIO_WritePin(AD5940_RST_GPIO_PORT, AD5940_RST_PIN, GPIO_PIN_RESET);
}

void AD5940_Delay10us(uint32_t time)
{
  time/=100;
  if(time == 0) time =1;
  HAL_Delay(time);
}

uint32_t AD5940_GetMCUIntFlag(void)
{
	return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
	ucInterrupted = 0;
	return 1;
}

uint32_t AD5940_MCUResourceInit(void *pCfg)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Step1, initialize SPI peripheral and its GPIOs for CS/RST */
  AD5940_SCK_GPIO_CLK_ENABLE();
  AD5940_MISO_GPIO_CLK_ENABLE();
  AD5940_MOSI_GPIO_CLK_ENABLE();
  AD5940_CS_GPIO_CLK_ENABLE();
  AD5940_RST_GPIO_CLK_ENABLE();
  /* Enable SPI clock */
  AD5940_CLK_ENABLE(); 
  
  GPIO_InitStruct.Pin       = AD5940_SCK_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = AD5940_SCK_AF;
  HAL_GPIO_Init(AD5940_SCK_GPIO_PORT, &GPIO_InitStruct);
  
  /* SPI MISO GPIO pin configuration  */
  GPIO_InitStruct.Pin = AD5940_MISO_PIN;
  GPIO_InitStruct.Alternate = AD5940_MISO_AF;
  HAL_GPIO_Init(AD5940_MISO_GPIO_PORT, &GPIO_InitStruct);
  
  /* SPI MOSI GPIO pin configuration  */
  GPIO_InitStruct.Pin = AD5940_MOSI_PIN;
  GPIO_InitStruct.Alternate = AD5940_MOSI_AF;
  HAL_GPIO_Init(AD5940_MOSI_GPIO_PORT, &GPIO_InitStruct);
  /* SPI CS GPIO pin configuration  */
  GPIO_InitStruct.Pin = AD5940_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(AD5940_CS_GPIO_PORT, &GPIO_InitStruct);
  
  /* SPI RST GPIO pin configuration  */
  GPIO_InitStruct.Pin = AD5940_RST_PIN;
  HAL_GPIO_Init(AD5940_RST_GPIO_PORT, &GPIO_InitStruct);
  
  AD5940_CsSet();
  AD5940_RstSet();
  
  /* Set the SPI parameters */
  SpiHandle.Instance               = AD5940SPI;
  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; //SPI clock should be < AD5940_SystemClock
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.Mode = SPI_MODE_MASTER;
  HAL_SPI_Init(&SpiHandle);
  
  /* Step 2: Configure external interrupot line */
  AD5940_GP0INT_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin       = AD5940_GP0INT_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(AD5940_GP0INT_GPIO_PORT, &GPIO_InitStruct);
  
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_EnableIRQ(AD5940_GP0INT_IRQn);
//  HAL_NVIC_SetPriority(AD5940_GP0INT_IRQn, 0, 0);
  return 0;
}

/* MCU related external line interrupt service routine */
void EXTI15_10_IRQHandler()
{
  ucInterrupted = 1;
  __HAL_GPIO_EXTI_CLEAR_IT(AD5940_GP0INT_PIN);
}

