/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  ADI
 @brief:   Used to control specific application and process data.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
/** 
 * @addtogroup AD5940_System_Examples
 * @{
 *  @defgroup BIA_Pro_Example
 *  @{
  */
#include "ad5940.h"
#include "AD5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "BodyImpedance-HiZ_Electrodes.h"

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

#define REAL_IMAG 0 
#define MAG_PAHSE 1

#define REAL_IMAG_OR_MAG_PAHSE MAG_PAHSE  /*SKR you can chose what you want to print*/

/* It's your choice here how to do with the data. Here is just an example to print them to UART */
int32_t BIAShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpCar_Type *pImp = (fImpCar_Type*)pData;
  AppBIACtrl(BIACTRL_GETFREQ, &freq);

  printf("Freq:%.2f, ", freq);
  /*Process data*/
  
#if REAL_IMAG_OR_MAG_PAHSE == REAL_IMAG   
  for(int i=0;i<DataCount;i++)
  {
    for(int jj=0;jj<5;jj++)
    {
       printf("%.2f,%.2f,  ",pImp[(i*5)+jj].Real,pImp[(i*5)+jj].Image);
    }
    printf("\r\n");
    
  }

#elif REAL_IMAG_OR_MAG_PAHSE == MAG_PAHSE   
  for(int i=0;i<DataCount;i++)
  {
    for(int jj=0;jj<5;jj++)
    {
       printf("%f,%f,  ",sqrt(pImp[(i*5)+jj].Real*pImp[(i*5)+jj].Real+pImp[(i*5)+jj].Image*pImp[(i*5)+jj].Image),atan2(pImp[(i*5)+jj].Image,pImp[(i*5)+jj].Real)*180/MATH_PI);
    }
    printf("\r\n");
    
  }  
#else
#error wrong value
#endif
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
  clk_cfg.ADCCLkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_2;
  clk_cfg.SysClkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bTRUE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bTRUE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 12;//AppBIACfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
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
	
  return 0;
}

/* !!Change the application parameters here if you want to change it to none-default value */
void AD5940BIAStructInit(void)
{
  AppBIACfg_Type *pBIACfg;
  
  AppBIAGetCfg(&pBIACfg);
  
  pBIACfg->AdcClkFreq = 32000000.0; /*DO NOT MODIFY*/ /*High Power Mode and external Xtal.*/
  
  pBIACfg->SeqStartAddr = 0;
  pBIACfg->MaxSeqLen = 512; /** @todo add checker in function */
  
  pBIACfg->RcalVal = 10000.0;
  pBIACfg->DftNum = DFTNUM_16384;
  pBIACfg->NumOfData = -1;      /* Never stop until you stop it mannually by AppBIACtrl() function */
  pBIACfg->BiaODR = 2.5;         /* ODR(Sample Rate) 20Hz */
  pBIACfg->FifoThresh = 12; /*SKR: this parameter needs to be changed... but it should not be visible*/      /* 4 */
  pBIACfg->ADCSinc3Osr = ADCSINC3OSR_2;
  
  pBIACfg->SinFreq = 50000.0; /*50kHz */ 
  pBIACfg->SweepCfg.SweepEn = bFALSE;
  pBIACfg->SweepCfg.SweepStart = 10000;
  pBIACfg->SweepCfg.SweepStop = 150000.0;
  pBIACfg->SweepCfg.SweepPoints = 100;
  pBIACfg->SweepCfg.SweepLog = bTRUE; 
}

void AD5940_Main(void)
{
  static uint32_t IntCount;
  static uint32_t count;
  uint32_t temp;
  
  AD5940PlatformCfg();
  
  AD5940BIAStructInit(); /* Configure your parameters in this function */
  
  AppBIAInit(AppBuff, APPBUFF_SIZE);    /* Initialize BIA application. Provide a buffer, which is used to store sequencer commands */     /*SKR this needs to be invisible by the final user*/
  AppBIACtrl(BIACTRL_START, 0);         /* Control BIA measurment to start. Second parameter has no meaning with this command. */
 
  while(1)
  {
    /* Check if interrupt flag which will be set when interrupt occured. */
    if(AD5940_GetMCUIntFlag())
    {
      IntCount++;
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp = APPBUFF_SIZE;
      AppBIAISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
      BIAShowResult(AppBuff, temp); /* Show the results to UART */

      if(IntCount == 240)
      {
        IntCount = 0;
        //AppBIACtrl(BIACTRL_SHUTDOWN, 0);
      }
    }
    count++;
    if(count > 1000000)
    {
      count = 0;
      //AppBIAInit(0, 0);    /* Re-initialize BIA application. Because sequences are ready, no need to provide a buffer, which is used to store sequencer commands */
      //AppBIACtrl(BIACTRL_START, 0);          /* Control BIA measurment to start. Second parameter has no meaning with this command. */
    }
  }
}

/**
 * @}
 * @}
 * */
 
