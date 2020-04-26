/*!
 *****************************************************************************
 @file:    Electrocardiograph.c
 @author:  Neo Xu
 @brief:   ECG Measurement.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "Electrocardiograph.h"

/* 
  Application configuration structure. Specified by user from template.
  The variables are usable in this whole application.
  It includes basic configuration for sequencer generator and application related parameters
*/
AppECGCfg_Type AppECGCfg = 
{
  .bParaChanged = bFALSE,
  .bBioElecBoard = bTRUE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 512,

  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 512,

  .ECGODR = 1000.0,           /* 1000.0 Hz*/
  .NumOfData = -1,
  .FifoThresh = 100,

  .LfoscClkFreq = 32000.0,
  .SysClkFreq = 16000000.0,
  .AdcClkFreq = 16000000.0,
  .PwrMod = AFEPWR_LP,

  .AdcPgaGain = ADCPGA_1,
  .ADCSinc3Osr = ADCSINC3OSR_2,
  .ADCSinc2Osr = ADCSINC2OSR_22,

  .ECGInited = bFALSE,
  .StopRequired = bFALSE,
  .bRunning = bFALSE,
  .FifoDataCount = 0,
};

/**
   This function is provided for upper controllers that want to change 
   application parameters specially for user defined parameters.
*/
AD5940Err AppECGGetCfg(void *pCfg)
{
  if(pCfg){
    *(AppECGCfg_Type**)pCfg = &AppECGCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

int32_t AppECGCtrl(int32_t Command, void *pPara)
{
  
  switch (Command)
  {
    case APPCTRL_START:
    {
      WUPTCfg_Type wupt_cfg;

      if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP;  /* Wakeup Failed */
      if(AppECGCfg.ECGInited == bFALSE)
        return AD5940ERR_APPERROR;
      /* Start it */
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
      wupt_cfg.WuptOrder[0] = SEQID_0;
      wupt_cfg.SeqxSleepTime[SEQID_0] = 4-1;
      wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(AppECGCfg.LfoscClkFreq/AppECGCfg.ECGODR)-4-1;
      AD5940_WUPTCfg(&wupt_cfg);
      
      AppECGCfg.FifoDataCount = 0;  /* restart */
      AppECGCfg.bRunning = bTRUE;
      break;
    }
    case APPCTRL_STOPNOW:
    {
      if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP;  /* Wakeup Failed */
      /* Start Wupt right now */
      AD5940_WUPTCtrl(bFALSE);
      /* There is chance this operation will fail because sequencer could put AFE back 
        to hibernate mode just after waking up. Use STOPSYNC is better. */
      AD5940_WUPTCtrl(bFALSE);
      AppECGCfg.bRunning = bFALSE;
      break;
    }
    case APPCTRL_STOPSYNC:
    {
      AppECGCfg.StopRequired = bTRUE;
      break;
    }
    case APPCTRL_SHUTDOWN:
    {
      AppECGCtrl(APPCTRL_STOPNOW, 0);  /* Stop the measurement if it's running. */
      /* Turn off LPloop related blocks which are not controlled automatically by hibernate operation */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);
      AD5940_EnterSleepS();  /* Enter Hibernate */
    }
    break;
    case APPCTRL_RUNNING:
      if(pPara == NULL)
        return AD5940ERR_NULLP; /* Null pointer */
      *(BoolFlag*)pPara = AppECGCfg.bRunning;
      break;
    default: break;
  }
  return AD5940ERR_OK;
}

/* Application initialization */
static AD5940Err AppECGSeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;
  AFERefCfg_Type aferef_cfg;
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  SWMatrixCfg_Type sw_matrix;

  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);
  
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */

  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;       /* The High speed buffers are automatically turned off during hibernate */
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  /* LP reference control - turn off them to save power*/
  aferef_cfg.LpBandgapEn = bFALSE;
  aferef_cfg.LpRefBufEn = bFALSE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	

  /* Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_AIN6;
  adc_base.ADCMuxN = ADCMUXN_VSET1P1;
  adc_base.ADCPga = AppECGCfg.AdcPgaGain;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  adc_filter.ADCSinc3Osr = AppECGCfg.ADCSinc3Osr;
  adc_filter.ADCSinc2Osr = AppECGCfg.ADCSinc2Osr;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
  AD5940_ADCFilterCfgS(&adc_filter);

  sw_matrix.Dswitch = SWD_OPEN;
  sw_matrix.Pswitch = SWP_RE0|SWP_RE1|SWP_DE0;
  sw_matrix.Nswitch = SWN_AIN2|SWN_SE0;
  sw_matrix.Tswitch = SWT_AIN0|SWT_AFE3LOAD;
  AD5940_SWMatrixCfgS(&sw_matrix);

  AD5940_AFECtrlS(AFECTRL_HPREFPWR, bTRUE); /* Enable reference. It's automatically turned off during hibernate */

  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_SEQGpioCtrlS(/*AGPIO_Pin6|*/AGPIO_Pin5|AGPIO_Pin1); /* GP6 to indicate sequencer is running. GP5 to disable AD8233. GP1 to enable AD8233 RLD function. */

    /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    AppECGCfg.InitSeqInfo.SeqId = SEQID_1;
    AppECGCfg.InitSeqInfo.SeqRamAddr = AppECGCfg.SeqStartAddr;
    AppECGCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppECGCfg.InitSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppECGCfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

static AD5940Err AppECGSeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;
  
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataType = DATATYPE_SINC3;
  clks_cal.DataCount = 1;             /* Sample one data when wakeup */
  clks_cal.ADCSinc2Osr = AppECGCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppECGCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppECGCfg.SysClkFreq/AppECGCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);
  //printf("Wait clocks:%d\n", WaitClks);
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGpioCtrlS(AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1);//GP6->endSeq, GP5 -> AD8233=OFF, GP1->RLD=OFF .
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling.*/
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*50)); 
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  //wait for first data ready
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_ADCPWR, bFALSE);  /* Stop ADC convert and DFT */
  AD5940_SEQGpioCtrlS(/*AGPIO_Pin6|*/AGPIO_Pin5|AGPIO_Pin1); /* GP6 to indicate sequencer is running. GP5 to enable AD8233. GP1 to enable AD8233 RLD function. */
  AD5940_EnterSleepS();/* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  if(error == AD5940ERR_OK)
  {
    AppECGCfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppECGCfg.MeasureSeqInfo.SeqRamAddr = AppECGCfg.InitSeqInfo.SeqRamAddr + AppECGCfg.InitSeqInfo.SeqLen ;
    AppECGCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppECGCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM. The buffer 'pSeqCmd' will be used to generate next sequence  */
    AD5940_SEQCmdWrite(AppECGCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

/* This function provide application initialize. It can also enable Wupt that will automatically trigger sequence. Or it can configure  */
int32_t AppECGInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;  

  SEQCfg_Type seq_cfg;
  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */
    /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  
  /* Reconfigure FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE);									/* Disable FIFO firstly */
  AD5940_FIFOThrshSet(AppECGCfg.FifoThresh);
  AD5940_FIFOCtrlS(FIFOSRC_SINC3, bTRUE);
  
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Start sequence generator */
  /* Initialize sequencer generator */
  if((AppECGCfg.ECGInited == bFALSE)||\
       (AppECGCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;   
    AD5940_SEQGenInit(pBuffer, BufferSize);

    /* Generate initialize sequence */
    error = AppECGSeqCfgGen(); /* Application initialization sequence using either MCU or sequencer */
    if(error != AD5940ERR_OK) return error;

    /* Generate measurement sequence */
    error = AppECGSeqMeasureGen();
    if(error != AD5940ERR_OK) return error;

    AppECGCfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
  }

  /* Initialization sequencer  */
  AppECGCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppECGCfg.InitSeqInfo);
  AD5940_SEQCtrlS(bTRUE);  /* Enable sequencer */
  AD5940_SEQMmrTrig(AppECGCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  
  /* Measurement sequence  */
  AppECGCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppECGCfg.MeasureSeqInfo);

  AD5940_SEQCtrlS(bTRUE);  /* Enable sequencer, and wait for trigger. It's disabled in initialization sequence */
  AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */

  AD5940_AFEPwrBW(AppECGCfg.PwrMod, AFEBW_250KHZ);

  AppECGCfg.ECGInited = bTRUE;  /* ECG application has been initialized. */
  return AD5940ERR_OK;
}

/* Modify registers when AFE wakeup */
int32_t AppECGRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppECGCfg.NumOfData > 0)
  {
    AppECGCfg.FifoDataCount += *pDataCount/4;
    if(AppECGCfg.FifoDataCount >= AppECGCfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if(AppECGCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    AppECGCfg.StopRequired = bFALSE;
    AppECGCfg.bRunning = bFALSE;
    return AD5940ERR_OK;
  }
  return AD5940ERR_OK;
}

/* Depending on the data type, do appropriate data pre-process before return back to controller */
static int32_t AppECGDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t DataCount = *pDataCount;
  
  *pDataCount = 0;

  /* Get ADC result */
  for(uint32_t i=0; i<DataCount; i++)
  {
    pData[i] &= 0xffff;
  }
  *pDataCount = DataCount; 

  return 0;
}

/**

*/
AD5940Err AppECGISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount;
  uint32_t FifoCnt;
  BuffCount = *pCount;
  
  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);     /* We are operating registers, so we don't allow AFE enter sleep mode which is done in our sequencer */
  *pCount = 0;
  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    /* Now there should be 4 data in FIFO */
    FifoCnt = AD5940_FIFOGetCnt();
    
    if(FifoCnt > BuffCount)
    {
      ///@todo buffer is limited.
    }
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AppECGRegModify(pBuff, &FifoCnt);   /* If there is need to do AFE re-configure, do it here when AFE is in active state */
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);    /* Allow AFE to enter sleep mode. AFE will stay at active mode until sequencer trigger sleep */
    /* AD5940_EnterSleepS(); // We cannot manually put AFE to hibernate because it's possible sequencer is running to take measurements */
    /* Process data */ 
    AppECGDataProcess(pBuff,&FifoCnt); 
    *pCount = FifoCnt;
    return AD5940ERR_OK;
  }
  
  return AD5940ERR_OK;
} 


