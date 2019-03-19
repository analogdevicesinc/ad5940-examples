/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  $Author: nxu2 $
 @brief:   Used to control specific application and futhur process data.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
/** 
 * @addtogroup AD5940_System_Examples
 * @{
 *  @defgroup Battery_Example
 *  @{
  */
#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "BATImpedance.h"

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

/* It's your choice here how to do with the data. Here is just an example to print them to UART */
int32_t BATShowResult(uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type *pImp = (fImpCar_Type*)pData;

  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("(real, image) = (%f,%f)mOhm \n",pImp[i].Real,pImp[i].Image);
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
  clk_cfg.SysClkDiv = SYSCLKDIV_2;
  clk_cfg.SysClkSrc = SYSCLKSRC_XTAL; //on battery board, there is a 32MHz crystal.
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
  fifo_cfg.FIFOThresh = 4;//AppBATCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940BATStructInit(void)
{
  AppBATCfg_Type *pBATCfg;
  AppBATGetCfg(&pBATCfg);
  pBATCfg->SeqStartAddr = 0;
  pBATCfg->MaxSeqLen = 512;
  pBATCfg->RcalVal = 50.0;  //50mOhm
  pBATCfg->ACVoltPP = 300.0f;
  pBATCfg->DCVolt = 1200.0f;
  pBATCfg->SinFreq = 1e3;
  pBATCfg->DftNum = DFTNUM_16384;
  pBATCfg->NumOfData = -1;      /* Never stop until you stop it mannually by AppBATCtrl() function */
  pBATCfg->BatODR = 1.0;         /* ODR(Sample Rate) 20Hz */
  pBATCfg->FifoThresh = 8;      /* 8 */
  pBATCfg->ADCSinc3Osr = ADCSINC3OSR_4;
}

void AD5940_Main(void)
{
  uint32_t temp;
  BoolFlag bCalibrating = bTRUE;  
  AD5940PlatformCfg();
  
  AD5940BATStructInit(); /* Configure your parameters in this function */
  
  AppBATInit(AppBuff, APPBUFF_SIZE);    /* Initialize BAT application. Provide a buffer, which is used to store sequencer commands */
  temp = 10;
  AppBATCtrl(BATCTRL_MRCAL, &temp);     /* Measur RCAL for 10 times */
 
  while(1)
  {
    /* Check if interrupt flag which will be set when interrupt occured. */
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp = APPBUFF_SIZE;
      AppBATISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
      
      if(bCalibrating)
      {
        bCalibrating = bFALSE;
        AppBATCtrl(BATCTRL_START, 0); /* Measure battery */
      }
      else
        BATShowResult(AppBuff, temp);
    }
  }
}

/**
 * @}
 * @}
 * */
 
