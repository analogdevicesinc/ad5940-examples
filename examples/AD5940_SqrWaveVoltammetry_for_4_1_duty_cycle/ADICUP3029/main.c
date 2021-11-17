/*

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*/

#include "stdio.h"
#include "ADuCM3029.h"
#include "AD5940.h"

/* Functions that used to initialize MCU platform */
uint32_t MCUPlatformInit(void *pCfg);

int main(void)
{
  void AD5940_Main(void);
  MCUPlatformInit(0);
  AD5940_MCUResourceInit(0);
  printf("Hello AD5940-Build Time:%s\n",__TIME__);
  AD5940_Main();
}

/* Below functions are used to initialize MCU Platform */
uint32_t MCUPlatformInit(void *pCfg)
{
  int UrtCfg(int iBaud);

  /*Stop watch dog timer(ADuCM3029)*/
  pADI_WDT0->CTL = 0xC9;
  /* Clock Configure */
  pADI_CLKG0_OSC->KEY = 0xCB14;               // Select HFOSC as system clock.
  pADI_CLKG0_OSC->CTL =                       // Int 32khz LFOSC selected in LFMUX
    BITM_CLKG_OSC_CTL_HFOSCEN|BITM_CLKG_OSC_CTL_HFXTALEN;

  while((pADI_CLKG0_OSC->CTL&BITM_CLKG_OSC_CTL_HFXTALOK) == 0);

  pADI_CLKG0_OSC->KEY = 0xCB14; 
  pADI_CLKG0_CLK->CTL0 = 0x201;                   /* Select XTAL as system clock */
  pADI_CLKG0_CLK->CTL1 = 0;                   // ACLK,PCLK,HCLK divided by 1
  pADI_CLKG0_CLK->CTL5 = 0x00;                 // Enable clock to all peripherals - no clock gating

  UrtCfg(230400);/*Baud rate: 230400*/
  return 1;
}

/**
	@brief int UrtCfg(int iBaud, int iBits, int iFormat)
			==========Configure the UART.
	@param iBaud :{B1200,B2200,B2400,B4800,B9600,B19200,B38400,B57600,B115200,B230400,B430800}	\n
		Set iBaud to the baudrate required:
		Values usually: 1200, 2200 (for HART), 2400, 4800, 9600,
		        19200, 38400, 57600, 115200, 230400, 430800, or type in baud-rate directly
	@note
		- Powers up UART if not powered up.
		- Standard baudrates are accurate to better than 0.1% plus clock error.\n
		- Non standard baudrates are accurate to better than 1% plus clock error.
   @warning - If an external clock is used for the system the ullRtClk must be modified with \n
         the speed of the clock used.
**/

int UrtCfg(int iBaud)
{
  int iBits = 3;//8bits, 
  int iFormat = 0;//, int iBits, int iFormat
  int i1;
  int iDiv;
  int iRtC;
  int iOSR;
  int iPllMulValue;
  unsigned long long ullRtClk = 16000000;                // The root clock speed


  /*Setup P0[11:10] as UART pins*/
  pADI_GPIO0->CFG = (1<<22)|(1<<20)|(pADI_GPIO0->CFG&(~((3<<22)|(3<<20))));

  iDiv = (pADI_CLKG0_CLK->CTL1& BITM_CLKG_CLK_CTL1_PCLKDIVCNT);                 // Read UART clock as set by CLKCON1[10:8]
  iDiv = iDiv>>8;
  if (iDiv == 0)
    iDiv = 1;
  iRtC = (pADI_CLKG0_CLK->CTL0& BITM_CLKG_CLK_CTL0_CLKMUX); // Check what is the root clock

  switch (iRtC)
  {
  case 0:                                               // HFOSC selected
    ullRtClk = 26000000;
    break;

  case 1:                                               // HFXTAL selected
    if ((pADI_CLKG0_CLK->CTL0 & 0x200)==0x200)           // 26Mhz XTAL used
        ullRtClk = 26000000;
    else
        ullRtClk = 16000000;                              // Assume 16MHz XTAL
    break;

  case 2:                                               // SPLL output
    iPllMulValue = (pADI_CLKG0_CLK->CTL3 &             // Check muliplication factor in PLL settings
                    BITM_CLKG_CLK_CTL3_SPLLNSEL);      // bits[4:0]. Assume div value of 0xD in bits [14:11]
    ullRtClk = (iPllMulValue *1000000);                // Assume straight multiplication by pADI_CLKG0_CLK->CTL3[4:0]
    break;

  case 3:
    ullRtClk = 26000000;                                //External clock is assumed to be 26MhZ, if different
    break;                                             //clock speed is used, this should be changed

  default:
    break;
  }
  //   iOSR = (pADI_UART0->COMLCR2 & 0x3);
  //   iOSR = 2^(2+iOSR);
  pADI_UART0->COMLCR2 = 0x3;
  iOSR = 32;
  //i1 = (ullRtClk/(iOSR*iDiv))/iBaud;	              // UART baud rate clock source is PCLK divided by OSR
  i1 = (ullRtClk/(iOSR*iDiv))/iBaud-1;   //for bigger M and N value
  pADI_UART0->COMDIV = i1;

  pADI_UART0->COMFBR = 0x8800|(((((2048/(iOSR*iDiv))*ullRtClk)/i1)/iBaud)-2048);
  pADI_UART0->COMIEN = 0;
  pADI_UART0->COMLCR = (iFormat&0x3c)|(iBits&3);


  pADI_UART0->COMFCR = (BITM_UART_COMFCR_RFTRIG & 0/*RX_FIFO_1BYTE*/ ) |BITM_UART_COMFCR_FIFOEN;
  pADI_UART0->COMFCR |= BITM_UART_COMFCR_RFCLR|BITM_UART_COMFCR_TFCLR;                                   // Clear the UART FIFOs
  pADI_UART0->COMFCR &= ~(BITM_UART_COMFCR_RFCLR|BITM_UART_COMFCR_TFCLR);                                // Disable clearing mechanism

  NVIC_EnableIRQ(UART_EVT_IRQn);              // Enable UART interrupt source in NVIC
  pADI_UART0->COMIEN = BITM_UART_COMIEN_ERBFI|BITM_UART_COMIEN_ELSI; /* Rx Interrupt */
  return pADI_UART0->COMLSR;
}
#include "stdio.h"
#ifdef __ICCARM__
int putchar(int c)
#else
int fputc(int c, FILE *f)
#endif
{
  pADI_UART0->COMTX = c;
  while((pADI_UART0->COMLSR&0x20) == 0);// tx fifo empty
  return c;
}
