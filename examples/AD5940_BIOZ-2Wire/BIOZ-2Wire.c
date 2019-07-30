/*!
 *****************************************************************************
 @file:    BIOZ-2Wire.c
 @author:  $Author: nxu2 $
 @brief:   BioImpedance measurement using 2-wire.
 @version: $Revision: 766 $
 @date:    $Date: 2018-06-27 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "BIOZ-2Wire.h"

/**
 * @note This example is modified from BIOZ example. This one is for 2-wire impedance measuremnt.
 *       The default pins used are CE0 and AIN2. The differnce with BIOZ is that the body voltage
 *       Measurment is replaced with excitation voltage measurment and it's only measured once.
*/

/* 
  Application configuration structure. Specified by user from template.
  The variables are usable in this whole application.
  It includes basic configuration for sequencer generator and application related parameters
*/
AppBIOZCfg_Type AppBIOZCfg = 
{
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,
  
  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 0,

  .ReDoRtiaCal = bFALSE,
  .SysClkFreq = 16000000.0,
  .WuptClkFreq = 32000.0,
  .AdcClkFreq = 16000000.0,
  .BIOZODR = 20.0, /* 20.0 Hz*/
  .NumOfData = -1,
  .RcalVal = 10000.0, /* 10kOhm */

  .PwrMod = AFEPWR_LP,
  .HstiaRtiaSel = HSTIARTIA_10K,
  .CtiaSel = 16,
  .ExcitBufGain = EXCITBUFGAIN_2,
  .HsDacGain = HSDACGAIN_1,
  .HsDacUpdateRate = 7,
  .DacVoltPP = 600.0,

  .SinFreq = 50000.0, /* 1000Hz */

  .ADCPgaGain = ADCPGA_1,
  .ADCSinc3Osr = ADCSINC3OSR_2,
  .ADCSinc2Osr = ADCSINC2OSR_22,

  .DftNum = DFTNUM_8192,
  .DftSrc = DFTSRC_SINC3,
  .HanWinEn = bTRUE,

  .SweepCfg.SweepEn = bFALSE,
  .SweepCfg.SweepStart = 10000,
  .SweepCfg.SweepStop = 150000.0,
  .SweepCfg.SweepPoints = 100,
  .SweepCfg.SweepLog = bTRUE,
  .SweepCfg.SweepIndex = 0,

  .FifoThresh = 4,
  .BIOZInited = bFALSE,
  .StopRequired = bFALSE,
};

/**
   This function is provided for upper controllers that want to change 
   application parameters specially for user defined parameters.
*/
AD5940Err AppBIOZGetCfg(void *pCfg)
{
  if(pCfg){
    *(AppBIOZCfg_Type**)pCfg = &AppBIOZCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

AD5940Err AppBIOZCtrl(int32_t BcmCtrl, void *pPara)
{
  switch (BcmCtrl)
  {
    case BIOZCTRL_START:
    {
      WUPTCfg_Type wupt_cfg;
      if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP;  /* Wakeup Failed */
      if(AppBIOZCfg.BIOZInited == bFALSE)
        return AD5940ERR_APPERROR;
      /* Start it */
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
      wupt_cfg.WuptOrder[0] = SEQID_0;
      wupt_cfg.SeqxSleepTime[SEQID_0] = (uint32_t)(AppBIOZCfg.WuptClkFreq/AppBIOZCfg.BIOZODR)-2-1;
      wupt_cfg.SeqxWakeupTime[SEQID_0] = 1; /* The minimum value is 1. Do not set it to zero. Set it to 1 will spend 2 32kHz clock. */
      AD5940_WUPTCfg(&wupt_cfg);
      
      AppBIOZCfg.FifoDataCount = 0;  /* restart */
#ifdef ADI_DEBUG
      ADI_Print("BIOZ Start...\n");
#endif
      break;
    }
    case BIOZCTRL_STOPNOW:
    {
      if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP;  /* Wakeup Failed */
      /* Start Wupt right now */
      AD5940_WUPTCtrl(bFALSE);
      AD5940_WUPTCtrl(bFALSE);
#ifdef ADI_DEBUG
      ADI_Print("BIOZ Stop Now...\n");
#endif
      break;
    }
    case BIOZCTRL_STOPSYNC:
    {
#ifdef ADI_DEBUG
      ADI_Print("BIOZ Stop SYNC...\n");
#endif
      AppBIOZCfg.StopRequired = bTRUE;
      break;
    }
    case BIOZCTRL_GETFREQ:
    if(pPara)
    {
      if(AppBIOZCfg.SweepCfg.SweepEn == bTRUE)
        *(float*)pPara = AppBIOZCfg.FreqofData;
      else
        *(float*)pPara = AppBIOZCfg.SinFreq;
    }
    break;
    case BIOZCTRL_SHUTDOWN:
    {
      AppBIOZCtrl(BIOZCTRL_STOPNOW, 0);  /* Stop the measurment if it's running. */
      /* Turn off LPloop related blocks which are not controlled automatically by sleep operation */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);
      AD5940_EnterSleepS();  /* Enter Hibernate */
#ifdef ADI_DEBUG
      ADI_Print("BIOZ Shut down...\n");
#endif
    }
    break;
    default:
    break;
  }
  return AD5940ERR_OK;
}

/* Generate init sequence */
static AD5940Err AppBIOZSeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type hs_loop;
  DSPCfg_Type dsp_cfg;
  float sin_freq;

  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);

  //AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  /* LP reference control - turn off them to save powr*/
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	
  hs_loop.HsDacCfg.ExcitBufGain = AppBIOZCfg.ExcitBufGain;
  hs_loop.HsDacCfg.HsDacGain = AppBIOZCfg.HsDacGain;
  hs_loop.HsDacCfg.HsDacUpdateRate = AppBIOZCfg.HsDacUpdateRate;

  hs_loop.HsTiaCfg.DiodeClose = bFALSE;
  hs_loop.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hs_loop.HsTiaCfg.HstiaCtia = AppBIOZCfg.CtiaSel; /* 31pF + 2pF */
  hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop.HsTiaCfg.HstiaRtiaSel = AppBIOZCfg.HstiaRtiaSel;

  hs_loop.SWMatCfg.Dswitch = SWD_OPEN;
  hs_loop.SWMatCfg.Pswitch = SWP_PL|SWP_PL2;
  hs_loop.SWMatCfg.Nswitch = SWN_NL|SWN_NL2;
  hs_loop.SWMatCfg.Tswitch = SWT_TRTIA;

  hs_loop.WgCfg.WgType = WGTYPE_SIN;
  hs_loop.WgCfg.GainCalEn = bFALSE;
  hs_loop.WgCfg.OffsetCalEn = bFALSE;
  if(AppBIOZCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppBIOZCfg.FreqofData = AppBIOZCfg.SweepCfg.SweepStart;
    AppBIOZCfg.SweepCurrFreq = AppBIOZCfg.SweepCfg.SweepStart;
    AD5940_SweepNext(&AppBIOZCfg.SweepCfg, &AppBIOZCfg.SweepNextFreq);
    sin_freq = AppBIOZCfg.SweepCurrFreq;
  }
  else
  {
    sin_freq = AppBIOZCfg.SinFreq;
    AppBIOZCfg.FreqofData = sin_freq;
  }
  hs_loop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sin_freq, AppBIOZCfg.SysClkFreq);
  hs_loop.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(AppBIOZCfg.DacVoltPP/800.0f*2047 + 0.5f);
  hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
  hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&hs_loop);

  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P;
  dsp_cfg.ADCBaseCfg.ADCPga = AppBIOZCfg.ADCPgaGain;
  
  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));
  
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* Don't care becase it's disabled */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;	/* Tell filter block clock rate of ADC*/
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppBIOZCfg.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppBIOZCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchClkEnable = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc3ClkEnable = bTRUE;
  dsp_cfg.ADCFilterCfg.WGClkEnable = bTRUE;
  dsp_cfg.ADCFilterCfg.DFTClkEnable = bTRUE;
  dsp_cfg.DftCfg.DftNum = AppBIOZCfg.DftNum;
  dsp_cfg.DftCfg.DftSrc = AppBIOZCfg.DftSrc;
  dsp_cfg.DftCfg.HanWinEn = AppBIOZCfg.HanWinEn;
  
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* Don't care about Statistic */
  AD5940_DSPCfgS(&dsp_cfg);

  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGpioCtrlS(0/*AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1*/);        //GP6->endSeq, GP5 -> AD8233=OFF, GP1->RLD=OFF .
  
  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extral command to disable sequencer for initialization sequence because we only want it to run one time. */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop seuqncer generator */
  if(error == AD5940ERR_OK)
  {
    AppBIOZCfg.InitSeqInfo.SeqId = SEQID_1;
    AppBIOZCfg.InitSeqInfo.SeqRamAddr = AppBIOZCfg.SeqStartAddr;
    AppBIOZCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppBIOZCfg.InitSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppBIOZCfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

static AD5940Err AppBIOZSeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  SWMatrixCfg_Type sw_cfg;
  ClksCalInfo_Type clks_cal;
  
  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = AppBIOZCfg.DftSrc;
  clks_cal.DataCount = 1L<<(AppBIOZCfg.DftNum+2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = AppBIOZCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppBIOZCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppBIOZCfg.SysClkFreq/AppBIOZCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);
  
  AD5940_SEQGpioCtrlS(AGPIO_Pin1/*|AGPIO_Pin5|AGPIO_Pin1*/);//GP6->endSeq, GP5 -> AD8233=OFF, GP1->RLD=OFF .
  
  sw_cfg.Dswitch = SWD_CE0;
  sw_cfg.Pswitch = SWP_CE0;
  sw_cfg.Nswitch = SWN_AIN1;
  sw_cfg.Tswitch = SWT_AIN1|SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));
  
  //AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* Enable Waveform generator, ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* wait for first data ready */  
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* Stop ADC convert and DFT */

  sw_cfg.Dswitch = SWD_OPEN;
  sw_cfg.Pswitch = SWP_PL|SWP_PL2;
  sw_cfg.Nswitch = SWN_NL|SWN_NL2;
  sw_cfg.Tswitch = SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg); /* Float switches */

  AD5940_SEQGpioCtrlS(0/*AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1*/);        //GP6->endSeq, GP5 -> AD8233=OFF, GP1->RLD=OFF .
  AD5940_EnterSleepS();/* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop seuqncer generator */

  if(error == AD5940ERR_OK)
  {
    AppBIOZCfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppBIOZCfg.MeasureSeqInfo.SeqRamAddr = AppBIOZCfg.InitSeqInfo.SeqRamAddr + AppBIOZCfg.InitSeqInfo.SeqLen ;
    AppBIOZCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppBIOZCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppBIOZCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

static AD5940Err AppBIOZRtiaCal(void)
{
  HSRTIACal_Type hsrtia_cal;

  hsrtia_cal.AdcClkFreq = AppBIOZCfg.AdcClkFreq;
  hsrtia_cal.ADCSinc2Osr = AppBIOZCfg.ADCSinc2Osr;
  hsrtia_cal.ADCSinc3Osr = AppBIOZCfg.ADCSinc3Osr;
  hsrtia_cal.bPolarResult = bTRUE; /* We need magnitude and phase here */
  hsrtia_cal.DftCfg.DftNum = AppBIOZCfg.DftNum;
  hsrtia_cal.DftCfg.DftSrc = AppBIOZCfg.DftSrc;
  hsrtia_cal.DftCfg.HanWinEn = AppBIOZCfg.HanWinEn;
  hsrtia_cal.fRcal= AppBIOZCfg.RcalVal;
  hsrtia_cal.HsTiaCfg.DiodeClose = bFALSE;
  hsrtia_cal.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hsrtia_cal.HsTiaCfg.HstiaCtia = AppBIOZCfg.CtiaSel;
  hsrtia_cal.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hsrtia_cal.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hsrtia_cal.HsTiaCfg.HstiaRtiaSel = AppBIOZCfg.HstiaRtiaSel;
  hsrtia_cal.SysClkFreq = AppBIOZCfg.SysClkFreq;

  if(AppBIOZCfg.SweepCfg.SweepEn == bTRUE)
  {
    uint32_t i;
    AppBIOZCfg.SweepCfg.SweepIndex = 0;  /* Reset index */
    for(i=0;i<AppBIOZCfg.SweepCfg.SweepPoints;i++)
    {
      AD5940_SweepNext(&AppBIOZCfg.SweepCfg, &hsrtia_cal.fFreq);
      AD5940_HSRtiaCal(&hsrtia_cal, &AppBIOZCfg.RtiaCalTable[i]);
#ifdef ADI_DEBUG
      ADI_Print("Freq:%.2f, (%f, %f)Ohm\n", hsrtia_cal.fFreq, AppBIOZCfg.RtiaCalTable[i].Real, AppBIOZCfg.RtiaCalTable[i].Image);
#endif
    }
    AppBIOZCfg.SweepCfg.SweepIndex = 0;  /* Reset index */
    AppBIOZCfg.RtiaCurrValue = AppBIOZCfg.RtiaCalTable[0];
  }
  else
  {
    hsrtia_cal.fFreq = AppBIOZCfg.SinFreq;
    AD5940_HSRtiaCal(&hsrtia_cal, &AppBIOZCfg.RtiaCurrValue);
#ifdef ADI_DEBUG
      ADI_Print("Freq:%.2f, (%f, %f)Ohm\n", hsrtia_cal.fFreq, AppBIOZCfg.RtiaCurrValue.Real, AppBIOZCfg.RtiaCurrValue.Image);
#endif
  }
  return AD5940ERR_OK;
}

/* This function provide application initialize.   */
AD5940Err AppBIOZInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;

  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */

  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* Do RTIA calibration */
  if((AppBIOZCfg.ReDoRtiaCal == bTRUE) || \
      AppBIOZCfg.BIOZInited == bFALSE)  /* Do calibration on the first initializaion */
  {
    AppBIOZRtiaCal();
    AppBIOZCfg.ReDoRtiaCal = bFALSE;
  }
  /* Reconfigure FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);									/* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = AppBIOZCfg.FifoThresh;              /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);

  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  
  /* Start sequence generator */
  /* Initialize sequencer generator */
  if((AppBIOZCfg.BIOZInited == bFALSE)||\
       (AppBIOZCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;   
    AD5940_SEQGenInit(pBuffer, BufferSize);

    /* Generate initialize sequence */
    error = AppBIOZSeqCfgGen(); /* Application initialization sequence using either MCU or sequencer */
    if(error != AD5940ERR_OK) return error;

    /* Generate measurement sequence */
    error = AppBIOZSeqMeasureGen();
    if(error != AD5940ERR_OK) return error;

    AppBIOZCfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
  }

  /* Initialization sequencer  */
  AppBIOZCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppBIOZCfg.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer */
  AD5940_SEQMmrTrig(AppBIOZCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
	/* Manually configure system bandwidth and power mode before measuring excitation voltage. */
  AD5940_AFEPwrBW(AppBIOZCfg.PwrMod, AFEBW_250KHZ);
  /* Measurment sequence  */
  AppBIOZCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppBIOZCfg.MeasureSeqInfo);
  
  /* Set ADC MUX to P/N node to measure excitation voltage firstly */
	AD5940_SleepKeyCtrlS(SLPKEY_LOCK);	/* Do not put AD5940 into sleep mode. */
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer, and wait for trigger */
  AD5940_ADCMuxCfgS(ADCMUXP_P_NODE, ADCMUXN_N_NODE);
  AD5940_SEQMmrTrig(AppBIOZCfg.MeasureSeqInfo.SeqId); /* Trigger sequencer to sample excitation voltage */
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DFTRDY) == bFALSE);
	/* Reset FIFO before clear interrupt flag. */
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);  /* Disable FIFO to reset FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bTRUE);   /* Enable FIFO */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  {
    int32_t Real, Image;
    Real = AD5940_ReadAfeResult(AFERESULT_DFTREAL)&0x3ffff;
    Image = AD5940_ReadAfeResult(AFERESULT_DFTIMAGE)&0x3ffff;
    if(Real&(1<<17)) /* Bit17 is sign bit */
      Real |= 0xfffc0000; /* Data is 18bit in two's complement, bit17 is the sign bit */
    if(Image&(1<<17)) /* Bit17 is sign bit */
      Image |= 0xfffc0000; /* Data is 18bit in two's complement, bit17 is the sign bit */
    Image = -Image;
    AppBIOZCfg.ExcitVolt.Real = Real;
    AppBIOZCfg.ExcitVolt.Image = Image;
  }
#ifdef ADI_DEBUG
  ADI_Print("ExcitVolt:(%f,%f)\n", AppBIOZCfg.ExcitVolt.Real, AppBIOZCfg.ExcitVolt.Image);
#endif
  AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);	/* Set ADC Mux back to HSTIA to measure current. */
	AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);	/* Allow AD5940 to enter sleep mode. */

  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer, and wait for trigger */
  AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */

  AppBIOZCfg.BIOZInited = bTRUE;  /* BIOZ application has been initialized. */
  return AD5940ERR_OK;
}

/* Modify registers when AFE wakeup */
static AD5940Err AppBIOZRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppBIOZCfg.NumOfData > 0)
  {
    AppBIOZCfg.FifoDataCount += *pDataCount/4;
    if(AppBIOZCfg.FifoDataCount >= AppBIOZCfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if(AppBIOZCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  if(AppBIOZCfg.SweepCfg.SweepEn) /* Need to set new frequency and set power mode */
  {
    AD5940_WGFreqCtrlS(AppBIOZCfg.SweepNextFreq, AppBIOZCfg.SysClkFreq);
  }
  return AD5940ERR_OK;
}

/* Depending on the data type, do appropriate data pre-process before return back to controller */
static AD5940Err AppBIOZDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t DataCount = *pDataCount;
  uint32_t ImpResCount = DataCount/2;

  fImpCar_Type * pOut = (fImpCar_Type*)pData;
  iImpCar_Type * pSrcData = (iImpCar_Type*)pData;

  *pDataCount = 0;

  DataCount = (DataCount/2)*2; /* One DFT result has two data in FIFO, real part and imaginary part.  */

  /* Convert DFT result to int32_t type */
  for(uint32_t i=0; i<DataCount; i++)
  {
    pData[i] &= 0x3ffff;
    if(pData[i]&(1<<17)) /* Bit17 is sign bit */
    {
      pData[i] |= 0xfffc0000; /* Data is 18bit in two's complement, bit17 is the sign bit */
    }
  }
  for(uint32_t i=0; i<ImpResCount; i++)
  {
    fImpCar_Type DftCurr;
    fImpCar_Type res;

    DftCurr.Real = (float)pSrcData[i].Real;
    DftCurr.Image = (float)pSrcData[i].Image;
    DftCurr.Image = -DftCurr.Image;
    DftCurr.Real = -DftCurr.Real;
    DftCurr.Image = -DftCurr.Image;
    res = AD5940_ComplexDivFloat(&DftCurr, &AppBIOZCfg.RtiaCurrValue);           /* I=Vrtia/Zrtia */
    res = AD5940_ComplexDivFloat(&AppBIOZCfg.ExcitVolt, &res);
    //ADI_Print("I:%f,%f ", DftCurr.Real, DftCurr.Image);
    pOut[i] = res;
  }
  *pDataCount = ImpResCount; 
  /* Calculate next frequency point */
  if(AppBIOZCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppBIOZCfg.FreqofData = AppBIOZCfg.SweepCurrFreq;
    AppBIOZCfg.SweepCurrFreq = AppBIOZCfg.SweepNextFreq;
    AD5940_SweepNext(&AppBIOZCfg.SweepCfg, &AppBIOZCfg.SweepNextFreq);
    AppBIOZCfg.RtiaCurrValue = AppBIOZCfg.RtiaCalTable[AppBIOZCfg.SweepCfg.SweepIndex];
  }
  return AD5940ERR_OK;
}

/**

*/
AD5940Err AppBIOZISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount;
  uint32_t FifoCnt;
  BuffCount = *pCount;
  if(AppBIOZCfg.BIOZInited == bFALSE)
    return AD5940ERR_APPERROR;
  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* Don't enter hibernate */
  *pCount = 0;

  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    /* Now there should be 4 data in FIFO */
    FifoCnt = (AD5940_FIFOGetCnt()/4)*4;
    
    if(FifoCnt > BuffCount)
    {
      ///@todo buffer is limited.
    }
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AppBIOZRegModify(pBuff, &FifoCnt);   /* If there is need to do AFE re-configure, do it here when AFE is in active state */
    //AD5940_EnterSleepS();  /* Manually put AFE back to hibernate mode. */
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter hibernate mode */
    /* Process data */ 
    AppBIOZDataProcess((int32_t*)pBuff,&FifoCnt); 
    *pCount = FifoCnt;
    return 0;
  }
  
  return 0;
} 

/**
  * @}
  */

