/*!
 *****************************************************************************
 @file:    AD5940Main.c
 @author:  Neo Xu
 @brief:   Used to control specific application and process data.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "ad5940.h"
#include "AD5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "BodyImpedance.h"
#include "Electrocardiograph.h"
#include "ElectrodermalActivity.h"

#define APP_NUM       3     /* Totally, we have 3 applications */

#define APP_ID_EDA    0
#define APP_ID_ECG    1
#define APP_ID_BIA    2

#define APP_EDA_SEQ_ADDR    0
#define APP_ECG_SEQ_ADDR    256
#define APP_BIA_SEQ_ADDR    384

#define APP_EDA_MAX_SEQLEN  256
#define APP_ECG_MAX_SEQLEN  128
#define APP_BIA_MAX_SEQLEN  128

typedef struct
{
  AD5940Err (*pAppGetCfg) (void *pCfg);
  AD5940Err (*pAppInit)   (uint32_t *pBuffer, uint32_t BufferSize);
  AD5940Err (*pAppISR)    (void *pBuff, uint32_t *pCount);
  AD5940Err (*pAppCtrl)   (int32_t BcmCtrl, void *pPara);
  AD5940Err (*pAppUserDataProc)    (void *pBuff, uint32_t pCount);
}BioElecApp_Type;

AD5940Err BIAShowResult(void *pData, uint32_t DataCount);
AD5940Err ECGShowResult(void *pData, uint32_t DataCount);
AD5940Err EDAShowResult(void *pData, uint32_t DataCount);

BioElecApp_Type BioElecAppList[APP_NUM]=
{
  /* EDA App */
  {
    .pAppGetCfg = AppEDAGetCfg,
    .pAppInit = AppEDAInit,
    .pAppISR = AppEDAISR,
    .pAppCtrl = AppEDACtrl,
    .pAppUserDataProc = EDAShowResult,
  },
  /* ECG App */
  {
    .pAppGetCfg = AppECGGetCfg,
    .pAppInit = AppECGInit,
    .pAppISR = AppECGISR,
    .pAppCtrl = AppECGCtrl,
    .pAppUserDataProc = ECGShowResult,
  },
  /* BIA App */
  {
    .pAppGetCfg = AppBIAGetCfg,
    .pAppInit = AppBIAInit,
    .pAppISR = AppBIAISR,
    .pAppCtrl = AppBIACtrl,
    .pAppUserDataProc = BIAShowResult,
  },
};
#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */

/* It's your choice here how to do with the data. Here is just an example to print them to UART */
AD5940Err BIAShowResult(void *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  AppBIACtrl(BIACTRL_GETFREQ, &freq);

  printf("Freq:%.2f ", freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("RzMag: %f Ohm , RzPhase: %f \n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI);
  }
  return 0;
}

/* print ECG result to uart */
AD5940Err ECGShowResult(void *pData, uint32_t DataCount)
{
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("i:%d, %d \n", i, ((uint32_t *)pData)[i]);
  }
  return 0;
}

/* print EDA result to uart */
AD5940Err EDAShowResult(void *pData, uint32_t DataCount)
{
  float RtiaMag;
  float mag, phase;
  /*Process data*/
  fImpCar_Type *pImp = (fImpCar_Type*)pData;
  AppEDACtrl(EDACTRL_GETRTIAMAG, &RtiaMag);

  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    mag = AD5940_ComplexMag(&pImp[i]);
    phase = AD5940_ComplexPhase(&pImp[i])*180/MATH_PI;
    printf("Rtia:%.2f, RzMag: %f Ohm , RzPhase: %f degree \n",RtiaMag, mag, phase);
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

/* !!Change the application parameters here if you want to change it to none-default value */
void AD5940BIAStructInit(void)
{
  AppBIACfg_Type *pBIACfg;
  
  AppBIAGetCfg(&pBIACfg);
  
  pBIACfg->SeqStartAddr = APP_BIA_SEQ_ADDR;
  pBIACfg->MaxSeqLen = APP_BIA_MAX_SEQLEN;  /* @todo add checker in function */
  
  pBIACfg->RcalVal = 10000.0;
  pBIACfg->DftNum = DFTNUM_8192;
  pBIACfg->NumOfData = -1;      /* Never stop until you stop it manually by AppBIACtrl() function */
  pBIACfg->BiaODR = 20;         /* ODR(Sample Rate) 20Hz */
  pBIACfg->FifoThresh = 4;      /* 4 */
  pBIACfg->ADCSinc3Osr = ADCSINC3OSR_2;
  pBIACfg->LfoscClkFreq = LFOSCFreq;

  pBIACfg->bParaChanged = bTRUE; /* Always initialize AFE. */
}

/* !!Change the application parameters here if you want to change it to none-default value */
void AD5940ECGStructInit(void)
{
  AppECGCfg_Type *pCfg;
  
  AppECGGetCfg(&pCfg);
  
  pCfg->SeqStartAddr = APP_ECG_SEQ_ADDR;
  pCfg->MaxSeqLen = APP_ECG_MAX_SEQLEN; /* @todo add checker in function */
  
  pCfg->ECGODR = 250;         /* ODR(Sample Rate) 200Hz */
  pCfg->FifoThresh = 256;      /* 4 */
  pCfg->ADCSinc3Osr = ADCSINC3OSR_2;
  pCfg->LfoscClkFreq = LFOSCFreq;
  pCfg->bParaChanged = bTRUE;   /* We always initialize AFE. */
}

void AD5940EDAStructInit(void)
{
  AppEDACfg_Type *pCfg;
  
  AppEDAGetCfg(&pCfg);
  pCfg->SeqStartAddr = APP_EDA_SEQ_ADDR;
  pCfg->MaxSeqLen = APP_EDA_MAX_SEQLEN;
  
  pCfg->LfoscClkFreq = LFOSCFreq;

  pCfg->bParaChanged = bTRUE;   /* We always initialize AFE. */
}

BioElecApp_Type *pCurrApp;
uint8_t bSwitchingApp = 1;
uint8_t toApp = APP_ID_BIA;

void AD5940_Main(void)
{
  static uint32_t IntCount;
  uint32_t temp;
  
  AD5940PlatformCfg();
  AD5940BIAStructInit();  /* Configure your parameters in this function */
  AD5940ECGStructInit();  /*  */
  AD5940EDAStructInit();  /*  */
  pCurrApp = &BioElecAppList[toApp];
  while(1)
  {
    if(bSwitchingApp)
    {
      //if the 'old' app stopped?
      BoolFlag running;
      if(pCurrApp->pAppCtrl(APPCTRL_RUNNING, &running) == AD5940ERR_OK){
        if(running == bFALSE){
          bSwitchingApp = 0;
          pCurrApp = &BioElecAppList[toApp];
          /* Initialize registers that fit to all measurements */
          AD5940PlatformCfg();
          pCurrApp->pAppInit(AppBuff, APPBUFF_SIZE);
          AD5940_ClrMCUIntFlag(); /* Clear the interrupts happened during initialization */
          pCurrApp->pAppCtrl(APPCTRL_START, 0);
        }
      }
    }
    /* Check if interrupt flag which will be set when interrupt occurred. */
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp = APPBUFF_SIZE;
      pCurrApp->pAppISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
      if(pCurrApp->pAppUserDataProc)
        pCurrApp->pAppUserDataProc(AppBuff, temp); /* Show the results to UART */

      if(IntCount++ == 10)
      {
        IntCount = 0;
        /* Control the application at any time */
        /* For example, I want to measure EDA excitation voltage periodically */
        //if(toApp == APP_ID_EDA)
        //  pCurrApp->pAppCtrl(EDACTRL_MEASVOLT, 0);  
      }
    }
  }
}


uint32_t command_start_measurement(uint32_t para1, uint32_t para2)
{
  pCurrApp->pAppCtrl(APPCTRL_START, 0);
  return 0;
}

uint32_t command_stop_measurement(uint32_t para1, uint32_t para2)
{
  pCurrApp->pAppCtrl(APPCTRL_STOPNOW, 0);
  return 0;
}

uint32_t command_switch_app(uint32_t AppID, uint32_t para2)
{
  if(AppID == APP_ID_EDA)
  {
    AD5940EDAStructInit();
    printf("Switch to EDA application\n");
  }
  else if(AppID == APP_ID_ECG)
  {
    AD5940ECGStructInit();
    printf("Switch to ECG application\n");
  }
  else if(AppID == APP_ID_BIA)
  {
    AD5940BIAStructInit();
    printf("Switch to BIA application\n");
  }
  else{
    printf("Wrong application ID.\n");
    return (uint32_t)-1;
  }

  if(pCurrApp)
    pCurrApp->pAppCtrl(APPCTRL_STOPSYNC, 0);
  bSwitchingApp = 1;
  toApp = AppID;
  return 0;
}

