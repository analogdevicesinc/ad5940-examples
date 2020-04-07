/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  Neo Xu
 @brief:   Used to control specific application and furfur process data.
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
#include "ElectrodermalActivity.h"

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */
uint32_t ResistorForBaseline = 0;


/* print EDA result to uart */
AD5940Err EDAShowResult(void *pData, uint32_t DataCount)
{
  float RtiaMag;
  /*Process data*/
  fImpCar_Type *pImp = (fImpCar_Type*)pData;
  AppEDACtrl(EDACTRL_GETRTIAMAG, &RtiaMag);

  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    float mag, phase;
    fImpCar_Type res;
    res = pImp[i];
    res.Real += ResistorForBaseline;    /* Show the real result of impedance under test(between F+/S+) */
    mag = AD5940_ComplexMag(&res);
    phase = AD5940_ComplexPhase(&res)*180/MATH_PI;
    printf("Rtia:%.2f,(Real,Image):(%.2f,%.2f)Ohm---Mag:%.2fOhm,Phase:%.2f`\n",RtiaMag, res.Real, res.Image, mag, phase);
  }
  return 0;
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  SEQCfg_Type seq_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppBIACfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
	fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP4_SYNC|GP2_EXTCLK|GP1_SYNC|GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
	
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Enable AFE to enter sleep mode. */

  /* Measure LFOSC frequency */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("Freq:%f\n", LFOSCFreq);
	
  return 0;
}


void AD5940EDAStructInit(void)
{
  AppEDACfg_Type *pCfg;
  
  AppEDAGetCfg(&pCfg);
  pCfg->MaxSeqLen = 512;
  
  pCfg->LfoscClkFreq = 32000;             /* Don't do LFOSC calibration now. We assume the default LFOSC is trimmed. */
  pCfg->RtiaAutoScaleEnable = bTRUE;      /* We manually select resistor value. */
  pCfg->LptiaRtiaSel = LPTIARTIA_120K;    
  pCfg->SinAmplitude = 1100*3/4;          /* Set excitation voltage to 0.75 times of full range. */
  pCfg->SinFreq = 100.0f;
  pCfg->SampleFreq = 400.0f;              /* Do not change sample frequency unless you know how it works. */
  pCfg->EDAODR = 4.0f;                    /* ODR decides how frequently to start the engine to measure impedance. */
  pCfg->FifoThresh = 4;                   /* The minimum threshold value is 4, and should always be 4*N, where N is 1,2,3... */
  pCfg->bParaChanged = bTRUE;
}


void AD5940_Main(void)
{
  uint32_t temp;
  fImpCar_Type EDABase = 
  {
    .Real = 24299.84f,
    .Image = -110778.71f,
  };
  
  AD5940PlatformCfg();
  
  AD5940EDAStructInit(); /* Configure your parameters in this function */
  
  AppEDAInit(AppBuff, APPBUFF_SIZE);    /* Initialize BIA application. Provide a buffer, which is used to store sequencer commands */
  AppEDACtrl(APPCTRL_START, 0);         /* Control BIA measurement to start. Second parameter has no meaning with this command. */
  AppEDACtrl(EDACTRL_SETBASE, &EDABase);
  ResistorForBaseline = 20000;          /* Above result is obtained using 20kOhm resistor on BioElec Rev C board. */
  while(1)
  {
    /* Check if interrupt flag which will be set when interrupt occurred. */
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp = APPBUFF_SIZE;
      AppEDAISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
      EDAShowResult(AppBuff, temp); /* Show the results to UART */
    }
  }
}

uint32_t rst_eda_base(uint32_t para1, uint32_t para2)
{
  printf("Reset EDA impedance baseline\n");
  ResistorForBaseline = 0;
  AppEDACtrl(EDACTRL_RSTBASE, 0);
  return 0;
}

uint32_t set_eda_base(uint32_t para1, uint32_t para2)
{
  fImpCar_Type ImpAVR;
  printf("Set EDA impedance baseline with current measured impedance average result\n");
  printf("Resistor used to measure baseline is %dOhm\n", para1);
  ResistorForBaseline = para1;
  AppEDACtrl(EDACTRL_GETAVR, &ImpAVR);
  AppEDACtrl(EDACTRL_SETBASE, &ImpAVR);
  return 0;
}

uint32_t get_average_imp(uint32_t para1, uint32_t para2)
{
  fImpCar_Type ImpAVR;
  printf("Measured average impedance result is:\n");
  AppEDACtrl(EDACTRL_GETAVR, &ImpAVR);
  printf("(Real,Image)=(%.2f,%.2f)Ohm\n", ImpAVR.Real, ImpAVR.Image);
  return 0;
}

uint32_t eda_start(uint32_t para1, uint32_t para2)
{
  printf("Start EDA measurement\n");
  AppEDACtrl(APPCTRL_START, 0);
  return 0;
}

uint32_t eda_stop(uint32_t para1, uint32_t para2)
{
  printf("Stop EDA measurement right now!!\n");
  AppEDACtrl(APPCTRL_STOPNOW, 0);
  return 0;
}

/**
 * @}
 * @}
 * */
 
