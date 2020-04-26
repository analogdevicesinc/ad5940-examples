/*!
 *****************************************************************************
 @file:    BodyComposition.c
 @author:  Neo Xu
 @brief:   BIA measurement sequences.
 -----------------------------------------------------------------------------
Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "BodyImpedance.h"

/* 
  Application configuration structure. Specified by user from template.
  The variables are usable in this whole application.
  It includes basic configuration for sequencer generator and application related parameters
*/
AppBIACfg_Type AppBIACfg = 
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
  .BiaODR = 20.0, /* 20.0 Hz*/
  .NumOfData = -1,
  .RcalVal = 10000.0, /* 10kOhm */

  .PwrMod = AFEPWR_LP,
  .HstiaRtiaSel = HSTIARTIA_1K,
  .CtiaSel = 16,
  .ExcitBufGain = EXCITBUFGAIN_2,
  .HsDacGain = HSDACGAIN_1,
  .HsDacUpdateRate = 7,
  .DacVoltPP = 800.0,

  .SinFreq = 50000.0, /* 50kHz */

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
  .BIAInited = bFALSE,
  .StopRequired = bFALSE,
  .MeasSeqCycleCount = 0,
};

/**
   This function is provided for upper controllers that want to change 
   application parameters specially for user defined parameters.
*/
AD5940Err AppBIAGetCfg(void *pCfg)
{
  if(pCfg){
    *(AppBIACfg_Type**)pCfg = &AppBIACfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

AD5940Err AppBIACtrl(int32_t BcmCtrl, void *pPara)
{
  switch (BcmCtrl)
  {
    case BIACTRL_START:
    {
      WUPTCfg_Type wupt_cfg;
      if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP;  /* Wakeup Failed */
      if(AppBIACfg.BIAInited == bFALSE)
        return AD5940ERR_APPERROR;
      /* Start it */
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
      wupt_cfg.WuptOrder[0] = SEQID_0;
      wupt_cfg.SeqxSleepTime[SEQID_0] = (uint32_t)(AppBIACfg.WuptClkFreq/AppBIACfg.BiaODR)-2-1;
      wupt_cfg.SeqxWakeupTime[SEQID_0] = 1; /* The minimum value is 1. Do not set it to zero. Set it to 1 will spend 2 32kHz clock. */
      AD5940_WUPTCfg(&wupt_cfg);
      
      AppBIACfg.FifoDataCount = 0;  /* restart */
      break;
    }
    case BIACTRL_STOPNOW:
    {
      if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP;  /* Wakeup Failed */
      /* Start Wupt right now */
      AD5940_WUPTCtrl(bFALSE);
      /* There is chance this operation will fail because sequencer could put AFE back 
        to hibernate mode just after waking up. Use STOPSYNC is better. */
      AD5940_WUPTCtrl(bFALSE);
      break;
    }
    case BIACTRL_STOPSYNC:
    {
      AppBIACfg.StopRequired = bTRUE;
      break;
    }
    case BIACTRL_GETFREQ:
    if(pPara)
    {
      if(AppBIACfg.SweepCfg.SweepEn == bTRUE)
        *(float*)pPara = AppBIACfg.FreqofData;
      else
        *(float*)pPara = AppBIACfg.SinFreq;
    }
    break;
    case BIACTRL_SHUTDOWN:
    {
      AppBIACtrl(BIACTRL_STOPNOW, 0);  /* Stop the measurement if it's running. */
      /* Turn off LPloop related blocks which are not controlled automatically by sleep operation */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);
      AD5940_EnterSleepS();  /* Enter Hibernate */
    }
    break;
    default:
    break;
  }
  return AD5940ERR_OK;
}

/* Generate init sequence */
static AD5940Err AppBIASeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type hs_loop;
  LPLoopCfg_Type lp_loop;
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
  /* LP reference control - turn off them to save power*/
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	
  hs_loop.HsDacCfg.ExcitBufGain = AppBIACfg.ExcitBufGain;
  hs_loop.HsDacCfg.HsDacGain = AppBIACfg.HsDacGain;
  hs_loop.HsDacCfg.HsDacUpdateRate = AppBIACfg.HsDacUpdateRate;

  hs_loop.HsTiaCfg.DiodeClose = bFALSE;
  hs_loop.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hs_loop.HsTiaCfg.HstiaCtia = AppBIACfg.CtiaSel; /* 31pF + 2pF */
  hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop.HsTiaCfg.HstiaRtiaSel = AppBIACfg.HstiaRtiaSel;

  hs_loop.SWMatCfg.Dswitch = SWD_OPEN;
  hs_loop.SWMatCfg.Pswitch = SWP_PL|SWP_PL2;
  hs_loop.SWMatCfg.Nswitch = SWN_NL|SWN_NL2;
  hs_loop.SWMatCfg.Tswitch = SWT_TRTIA;

  hs_loop.WgCfg.WgType = WGTYPE_SIN;
  hs_loop.WgCfg.GainCalEn = bFALSE;
  hs_loop.WgCfg.OffsetCalEn = bFALSE;
  if(AppBIACfg.SweepCfg.SweepEn == bTRUE)
  {
		AppBIACfg.SweepCfg.SweepIndex = 0;
    AppBIACfg.FreqofData = AppBIACfg.SweepCfg.SweepStart;
    AppBIACfg.SweepCurrFreq = AppBIACfg.SweepCfg.SweepStart;
    AD5940_SweepNext(&AppBIACfg.SweepCfg, &AppBIACfg.SweepNextFreq);
    sin_freq = AppBIACfg.SweepCurrFreq;
  }
  else
  {
    sin_freq = AppBIACfg.SinFreq;
    AppBIACfg.FreqofData = sin_freq;
  }
  hs_loop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sin_freq, AppBIACfg.SysClkFreq);
  hs_loop.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(AppBIACfg.DacVoltPP/800.0f*2047 + 0.5f);
  hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
  hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&hs_loop);

  lp_loop.LpDacCfg.LpdacSel = LPDAC0;
  lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lp_loop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN;
  lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_loop.LpDacCfg.DataRst = bFALSE;
  lp_loop.LpDacCfg.PowerEn = bTRUE;
  lp_loop.LpDacCfg.DacData12Bit = (uint32_t)((1100-200)/2200.0*4095);
  lp_loop.LpDacCfg.DacData6Bit = 31;

  lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaRf = LPTIARF_20K;
  lp_loop.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
  lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
  lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(5)|LPTIASW(6)|LPTIASW(7)|LPTIASW(8)|LPTIASW(9)|LPTIASW(12)|LPTIASW(13); /** @todo Optimization needed for new silicon */
  AD5940_LPLoopCfgS(&lp_loop);

  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P;
  dsp_cfg.ADCBaseCfg.ADCPga = AppBIACfg.ADCPgaGain;
  
  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));
  
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* Don't care because it's disabled */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;	/* Tell filter block clock rate of ADC*/
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppBIACfg.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppBIACfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.DftCfg.DftNum = AppBIACfg.DftNum;
  dsp_cfg.DftCfg.DftSrc = AppBIACfg.DftSrc;
  dsp_cfg.DftCfg.HanWinEn = AppBIACfg.HanWinEn;
  
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* Don't care about Statistic */
  AD5940_DSPCfgS(&dsp_cfg);
    
  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGpioCtrlS(0/*AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1*/);        //GP6->endSeq, GP5 -> AD8233=OFF, GP1->RLD=OFF .
  
  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    AppBIACfg.InitSeqInfo.SeqId = SEQID_1;
    AppBIACfg.InitSeqInfo.SeqRamAddr = AppBIACfg.SeqStartAddr;
    AppBIACfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppBIACfg.InitSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppBIACfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

static AD5940Err AppBIASeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  SWMatrixCfg_Type sw_cfg;
  ClksCalInfo_Type clks_cal;
  
  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = AppBIACfg.DftSrc;
  clks_cal.DataCount = 1L<<(AppBIACfg.DftNum+2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = AppBIACfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppBIACfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppBIACfg.SysClkFreq/AppBIACfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);
  
  AD5940_SEQGpioCtrlS(AGPIO_Pin6/*|AGPIO_Pin5|AGPIO_Pin1*/);//GP6->endSeq, GP5 -> AD8233=OFF, GP1->RLD=OFF .
  
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));  /* @todo wait 250us?? */
  sw_cfg.Dswitch = SWD_CE0;
  sw_cfg.Pswitch = SWP_CE0;
  sw_cfg.Nswitch = SWN_AIN1;
  sw_cfg.Tswitch = SWT_AIN1|SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg);
  
  AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* Enable Waveform generator, ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* wait for first data ready */  
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* Stop ADC convert and DFT */

  AD5940_ADCMuxCfgS(ADCMUXP_AIN3, ADCMUXN_AIN2);
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* Enable Waveform generator, ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));  //delay for signal settling DFT_WAIT
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
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  AppBIACfg.MeasSeqCycleCount = AD5940_SEQCycleTime();
  AppBIACfg.MaxODR = 1/(((AppBIACfg.MeasSeqCycleCount + 10) / 16.0)* 1E-6)  ;
  if(AppBIACfg.BiaODR > AppBIACfg.MaxODR)
  {
    /* We have requested a sampling rate that cannot be achieved with the time it
       takes to acquire a sample.
    */
    AppBIACfg.BiaODR = AppBIACfg.MaxODR;
  }

  if(error == AD5940ERR_OK)
  {
    AppBIACfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppBIACfg.MeasureSeqInfo.SeqRamAddr = AppBIACfg.InitSeqInfo.SeqRamAddr + AppBIACfg.InitSeqInfo.SeqLen ;
    AppBIACfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppBIACfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppBIACfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

static AD5940Err AppBIARtiaCal(void)
{
  HSRTIACal_Type hsrtia_cal;

  hsrtia_cal.AdcClkFreq = AppBIACfg.AdcClkFreq;
  hsrtia_cal.ADCSinc2Osr = AppBIACfg.ADCSinc2Osr;
  hsrtia_cal.ADCSinc3Osr = AppBIACfg.ADCSinc3Osr;
  hsrtia_cal.bPolarResult = bTRUE; /* We need magnitude and phase here */
  hsrtia_cal.DftCfg.DftNum = AppBIACfg.DftNum;
  hsrtia_cal.DftCfg.DftSrc = AppBIACfg.DftSrc;
  hsrtia_cal.DftCfg.HanWinEn = AppBIACfg.HanWinEn;
  hsrtia_cal.fRcal= AppBIACfg.RcalVal;
  hsrtia_cal.HsTiaCfg.DiodeClose = bFALSE;
  hsrtia_cal.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hsrtia_cal.HsTiaCfg.HstiaCtia = AppBIACfg.CtiaSel;
  hsrtia_cal.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hsrtia_cal.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_TODE;
  hsrtia_cal.HsTiaCfg.HstiaRtiaSel = AppBIACfg.HstiaRtiaSel;
  hsrtia_cal.SysClkFreq = AppBIACfg.SysClkFreq;
  hsrtia_cal.fFreq = AppBIACfg.SweepCfg.SweepStart;

  if(AppBIACfg.SweepCfg.SweepEn == bTRUE)
  {
    uint32_t i;
    AppBIACfg.SweepCfg.SweepIndex = 0;  /* Reset index */
    for(i=0;i<AppBIACfg.SweepCfg.SweepPoints;i++)
    {
			AD5940_HSRtiaCal(&hsrtia_cal, AppBIACfg.RtiaCalTable[i]);
#ifdef ADI_DEBUG
      ADI_Print("Freq:%.2f, RTIA: Mag:%f Ohm, Phase:%.3f\n", hsrtia_cal.fFreq, AppBIACfg.RtiaCalTable[i][0], AppBIACfg.RtiaCalTable[i][1]);
#endif
      AD5940_SweepNext(&AppBIACfg.SweepCfg, &hsrtia_cal.fFreq);     
    }
    AppBIACfg.SweepCfg.SweepIndex = 0;  /* Reset index */
    AppBIACfg.RtiaCurrValue[0] = AppBIACfg.RtiaCalTable[AppBIACfg.SweepCfg.SweepIndex][0];
    AppBIACfg.RtiaCurrValue[1] = AppBIACfg.RtiaCalTable[AppBIACfg.SweepCfg.SweepIndex][1];
    
  }
  else
  {
    hsrtia_cal.fFreq = AppBIACfg.SinFreq;
    AD5940_HSRtiaCal(&hsrtia_cal, AppBIACfg.RtiaCurrValue);
  }
  return AD5940ERR_OK;
}

/* This function provide application initialize.   */
AD5940Err AppBIAInit(uint32_t *pBuffer, uint32_t BufferSize)
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
  
  if((AppBIACfg.ReDoRtiaCal == bTRUE) || \
      AppBIACfg.BIAInited == bFALSE)  /* Do calibration on the first initializaion */
  {
    AppBIARtiaCal();
    AppBIACfg.ReDoRtiaCal = bFALSE;
  }
  /* Reconfigure FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);									/* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = AppBIACfg.FifoThresh;              /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);

  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  
  /* Start sequence generator */
  /* Initialize sequencer generator */
  if((AppBIACfg.BIAInited == bFALSE)||\
       (AppBIACfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;   
    AD5940_SEQGenInit(pBuffer, BufferSize);

    /* Generate initialize sequence */
    error = AppBIASeqCfgGen(); /* Application initialization sequence using either MCU or sequencer */
    if(error != AD5940ERR_OK) return error;

    /* Generate measurement sequence */
    error = AppBIASeqMeasureGen();
    if(error != AD5940ERR_OK) return error;

    AppBIACfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
  }

  /* Initialization sequencer  */
  AppBIACfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppBIACfg.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer */
  AD5940_SEQMmrTrig(AppBIACfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  
  /* Measurement sequence  */
  AppBIACfg.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppBIACfg.MeasureSeqInfo);

  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer, and wait for trigger */
  AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */

  AD5940_AFEPwrBW(AppBIACfg.PwrMod, AFEBW_250KHZ);
  AD5940_WriteReg(REG_AFE_SWMUX, 1<<3);
  AppBIACfg.BIAInited = bTRUE;  /* BIA application has been initialized. */
  return AD5940ERR_OK;
}

/* Modify registers when AFE wakeup */
static AD5940Err AppBIARegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppBIACfg.NumOfData > 0)
  {
    AppBIACfg.FifoDataCount += *pDataCount/4;
    if(AppBIACfg.FifoDataCount >= AppBIACfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if(AppBIACfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  if(AppBIACfg.SweepCfg.SweepEn) /* Need to set new frequency and set power mode */
  {
    AD5940_WGFreqCtrlS(AppBIACfg.SweepNextFreq, AppBIACfg.SysClkFreq);
  }
  return AD5940ERR_OK;
}

/* Depending on the data type, do appropriate data pre-process before return back to controller */
static AD5940Err AppBIADataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t DataCount = *pDataCount;
  uint32_t ImpResCount = DataCount/4;

  fImpPol_Type * const pOut = (fImpPol_Type*)pData;
  iImpCar_Type * pSrcData = (iImpCar_Type*)pData;

  *pDataCount = 0;

  DataCount = (DataCount/4)*4;/* We expect RCAL data together with Rz data. One DFT result has two data in FIFO, real part and imaginary part.  */

  /* Convert DFT result to int32_t type */
  for(uint32_t i=0; i<DataCount; i++)
  {
    pData[i] &= 0x3ffff; /* @todo option to check ECC */
    if(pData[i]&(1<<17)) /* Bit17 is sign bit */
    {
      pData[i] |= 0xfffc0000; /* Data is 18bit in two's complement, bit17 is the sign bit */
    }
  }
  for(uint32_t i=0; i<ImpResCount; i++)
  {
    iImpCar_Type *pDftVolt, *pDftCurr;

    pDftCurr = pSrcData++;
    pDftVolt = pSrcData++;
    float VoltMag,VoltPhase;
    float CurrMag, CurrPhase;

    VoltMag = sqrt((float)pDftVolt->Real*pDftVolt->Real+(float)pDftVolt->Image*pDftVolt->Image);
    VoltPhase = atan2(-pDftVolt->Image,pDftVolt->Real);
    CurrMag = sqrt((float)pDftCurr->Real*pDftCurr->Real+(float)pDftCurr->Image*pDftCurr->Image);
    CurrPhase = atan2(-pDftCurr->Image,pDftCurr->Real);

    VoltMag = VoltMag/CurrMag*AppBIACfg.RtiaCurrValue[0];
    VoltPhase = VoltPhase - CurrPhase + AppBIACfg.RtiaCurrValue[1];

    pOut[i].Magnitude = VoltMag;
    pOut[i].Phase = VoltPhase;
  }
  *pDataCount = ImpResCount; 
  /* Calculate next frequency point */
  if(AppBIACfg.SweepCfg.SweepEn == bTRUE)
  {
    AppBIACfg.FreqofData = AppBIACfg.SweepCurrFreq;
    AppBIACfg.SweepCurrFreq = AppBIACfg.SweepNextFreq;
		AppBIACfg.RtiaCurrValue[0] = AppBIACfg.RtiaCalTable[AppBIACfg.SweepCfg.SweepIndex][0];
    AppBIACfg.RtiaCurrValue[1] = AppBIACfg.RtiaCalTable[AppBIACfg.SweepCfg.SweepIndex][1];
    AD5940_SweepNext(&AppBIACfg.SweepCfg, &AppBIACfg.SweepNextFreq);
  }
  return AD5940ERR_OK;
}

/**
*/
AD5940Err AppBIAISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount;
  uint32_t FifoCnt;
  BuffCount = *pCount;
  if(AppBIACfg.BIAInited == bFALSE)
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
    AppBIARegModify(pBuff, &FifoCnt);   /* If there is need to do AFE re-configure, do it here when AFE is in active state */
    //AD5940_EnterSleepS();  /* Manually put AFE back to hibernate mode. */
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter hibernate mode */
    /* Process data */ 
    AppBIADataProcess((int32_t*)pBuff,&FifoCnt); 
    *pCount = FifoCnt;
    return 0;
  }
  
  return 0;
} 

/**
  * @}
  */
