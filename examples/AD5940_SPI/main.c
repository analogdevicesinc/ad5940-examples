/*

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*/

#include "ADuCM3029.H"
#include "AD5940PORT.h"
#include "stdio.h"

int main(void)
{
   void AD5940_Main(void);
   MCUPlatformInit(0);
   MCUGpioInit(0);
   MCUExtiInit(0);
   MCUSPIInit(0);
   MCUSysTickInit(0);
   printf("Hello AD5940-Build Time:%s\n",__TIME__);
   AD5940_Main();
}

//void Host_EnterHibernate(void)
//{
//   int32_t index = 0;
//   uint32_t savedWDT;
//   savedWDT = pADI_WDT0->CTL;  //None of the watchdog timer registers are retained in hibernate mode
//   SCB->SCR = 0x04;		// sleepdeep mode - write to the Cortex-m3 System Control register bit2
//   pADI_PMG0->PWRKEY = 0x4859;	// key1
//   pADI_PMG0->PWRMOD = ENUM_PMG_PWRMOD_HIBERNATE|BITM_PMG_PWRMOD_MONVBATN;
//   for (index=0;index<2;index++);
//   __WFI();
//   for (index=0;index<2;index++);
//   pADI_WDT0->CTL = savedWDT;  //restore WDT control register.
//   UrtCfg(230400);/*Baud rate: 230400*/
//   SpiMasterInit();
//}
