/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  Neo Xu
 @brief:   Used to control specific application and further process data.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
/** 
 * @addtogroup AD5940_System_Examples
 * @{
 *  @defgroup BioElec_Example
 *  @{
  */
#include "ad5940.h"
#include "AD5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "BIOZ-2Wire.h"

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

/* It's your choice here how to do with the data. Here is just an example to print them to UART */
int32_t BIOZShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpCar_Type *pImp = (fImpCar_Type*)pData;
  AppBIOZCtrl(BIOZCTRL_GETFREQ, &freq);

  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
		printf("Freq:%.2f ", freq);
    //printf("RzMag: %f Ohm , RzPhase: %f \n",AD5940_ComplexMag(&pImp[i]), AD5940_ComplexPhase(&pImp[i])*180/MATH_PI);
		printf("Impedance:(Real,Image) = (%f,%f)\n", pImp[i].Real, pImp[i].Image);
  }
  return 0;
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_XTAL;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_XTAL;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bFALSE;
  clk_cfg.HFXTALEn = bTRUE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP4_SYNC|GP2_TRIG|GP1_SYNC|GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;

  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

/* !!Change the application parameters here if you want to change it to none-default value */
void AD5940BIOZStructInit(void)
{
  AppBIOZCfg_Type *pBIOZCfg; 
  AppBIOZGetCfg(&pBIOZCfg);
  
  pBIOZCfg->SeqStartAddr = 0;
  pBIOZCfg->MaxSeqLen = 512; 
  
	pBIOZCfg->SinFreq = 20e3;			/* 20kHz. This value is ignored if SweepEn = bTRUE */
	pBIOZCfg->RcalVal = 10000.0;	/* Value of RCAl on the evaluaiton board */
	pBIOZCfg->HstiaRtiaSel = HSTIARTIA_200;	
  
	/* Configure Switch matrix */
	pBIOZCfg->DswitchSel = SWD_CE0;
  pBIOZCfg->PswitchSel = SWP_CE0;
  pBIOZCfg->NswitchSel = SWN_AIN2;
  pBIOZCfg->TswitchSel = SWN_AIN2;
	
  /* Configure Sweep Parameters */
  pBIOZCfg->SweepCfg.SweepEn = bTRUE;
  pBIOZCfg->SweepCfg.SweepStart = 1000;
  pBIOZCfg->SweepCfg.SweepStop = 200000.0;
  pBIOZCfg->SweepCfg.SweepPoints = 40;	/* Maximum is 100 */
  pBIOZCfg->SweepCfg.SweepLog = bFALSE;
	
	pBIOZCfg->BIOZODR = 5;         /* ODR(Sample Rate) 5Hz */
	pBIOZCfg->NumOfData = -1;      	/* Never stop until you stop it manually by AppBIOZCtrl() function */
}


void AD5940_Main(void)
{
  uint32_t temp;
  
  AD5940PlatformCfg();
  
  AD5940BIOZStructInit(); /* Configure your parameters in this function */
  
  AppBIOZInit(AppBuff, APPBUFF_SIZE);    /* Initialize BIOZ application. Provide a buffer, which is used to store sequencer commands */
  AppBIOZCtrl(BIOZCTRL_START, 0);         /* Control BIOZ measurement to start. Second parameter has no meaning with this command. */
 
  while(1)
  {
    /* Check if interrupt flag which will be set when interrupt occurred. */
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp = APPBUFF_SIZE;
      AppBIOZISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
      BIOZShowResult(AppBuff, temp); /* Show the results to UART */
    }
  }
}

/**
 * @}
 * @}
 * */
 
