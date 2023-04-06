/*!
 *****************************************************************************
 @file:    BATImpedance.c
 @author:  Neo Xu
 @brief:   Battery impedance measurement sequences.
 -----------------------------------------------------------------------------
Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "BATImpedance.h"

/* 
  Application configuration structure. Specified by user from template.
  The variables are usable in this whole application.
  It includes basic configuration for sequencer generator and application related parameters
*/
AppBATCfg_Type AppBATCfg = 
{
  .state = STATE_IDLE,
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,
  
  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 0,

  .SysClkFreq = 16000000.0,
  .WuptClkFreq = 32000.0,
  .AdcClkFreq = 16000000.0,
  .BatODR = 20.0, /* 20.0 Hz*/
  .NumOfData = -1,

  .PwrMod = AFEPWR_LP,
  .ACVoltPP = 800.0,
  .DCVolt = 1100.0f,
  .SinFreq = 50000.0, /* 50kHz */
  .RcalVal = 50.0, /* 50mOhm */

  .ADCSinc3Osr = ADCSINC3OSR_4,
  .ADCSinc2Osr = ADCSINC2OSR_22,

  .DftNum = DFTNUM_16384,
  .DftSrc = DFTSRC_SINC3,
  .HanWinEn = bTRUE,

  .FifoThresh = 4,
  .BATInited = bFALSE,
  .StopRequired = bFALSE,
  .MeasSeqCycleCount = 0,
	
	.SweepCfg.SweepEn = bTRUE,
  .SweepCfg.SweepStart = 1000,
  .SweepCfg.SweepStop = 100000.0,
  .SweepCfg.SweepPoints = 101,
  .SweepCfg.SweepLog = bFALSE,
  .SweepCfg.SweepIndex = 0,
};

/**
   This function is provided for upper controllers that want to change 
   application parameters specially for user defined parameters.
*/
AD5940Err AppBATGetCfg(void *pCfg)
{
  if(pCfg){
    *(AppBATCfg_Type**)pCfg = &AppBATCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}


static void PreCharge(unsigned char channel) 
{
  void Arduino_WriteDn(uint32_t Dn, BoolFlag bHigh);
  switch(channel)
  {
    case PRECHARGE_CH1: //00
    Arduino_WriteDn(1<<3, bFALSE);  //d3
    Arduino_WriteDn(1<<4, bFALSE);  //d4
    break;
    case PRECHARGE_CH2: //01
    Arduino_WriteDn(1<<3, bTRUE);
    Arduino_WriteDn(1<<4, bFALSE);
    break;
    case PRECHARGE_CH3://10
    Arduino_WriteDn(1<<3, bFALSE);
    Arduino_WriteDn(1<<4, bTRUE);
    break;
    default:
    break;
  }
  AD5940_Delay10us(PRECHARGE_WAIT_MS*100);
  Arduino_WriteDn(1<<3, bTRUE);  //d3
  Arduino_WriteDn(1<<4, bTRUE);  //d4
}

AD5940Err AppBATCtrl(int32_t BatCtrl, void *pPara)
{
  switch (BatCtrl)
  {
    case BATCTRL_START:
    {
      if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP;  /* Wakeup Failed */
      if(AppBATCfg.BATInited == bFALSE)
        return AD5940ERR_APPERROR;
      AD5940_WriteReg(REG_AFE_SWMUX, 1<<0); 				/* control ADG636 to measure battery */
			AD5940_WriteReg(REG_AFE_SYNCEXTDEVICE, 0x0);
      PreCharge(PRECHARGE_BAT);
      PreCharge(PRECHARGE_AMP);
      AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);
      AD5940_FIFOThrshSet(AppBATCfg.FifoThresh);  /* DFT result contains both real and image. */
      AD5940_FIFOCtrlS(FIFOSRC_DFT, bTRUE);
			AppBATCfg.state = STATE_BATTERY;
      /* Trigger sequence using MMR write */
			AD5940_SEQMmrTrig(SEQID_0);
      AppBATCfg.FifoDataCount = 0;  /* restart */
      
      break;
    }
    case BATCTRL_STOPNOW:
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
    case BATCTRL_STOPSYNC:
    {
      AppBATCfg.StopRequired = bTRUE;
      break;
    }
		case BATCTRL_GETFREQ:
    if(pPara)
    {
      if(AppBATCfg.SweepCfg.SweepEn == bTRUE)
        *(float*)pPara = AppBATCfg.FreqofData;
      else
        *(float*)pPara = AppBATCfg.SinFreq;
    }
		break;
    case BATCTRL_SHUTDOWN:
    {
      AppBATCtrl(BATCTRL_STOPNOW, 0);  /* Stop the measurement if it's running. */
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
    case BATCTRL_MRCAL:
    if(AD5940_WakeUp(10) > 10)
        return AD5940ERR_WAKEUP;
    //Settle input RC filter.
    AD5940_WriteReg(REG_AFE_SWMUX, 0); //control ADG636 to measure rcal
		AD5940_WriteReg(REG_AFE_SYNCEXTDEVICE, 0x4);
    PreCharge(PRECHARGE_RCAL);
    PreCharge(PRECHARGE_AMP);
    AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);
		AD5940_FIFOThrshSet(2);
    AD5940_FIFOCtrlS(FIFOSRC_DFT, bTRUE); //enable FIFO
		AppBATMeasureRCAL();
    break;
    default:
    break;
  }
  return AD5940ERR_OK;
}

/* Generate init sequence */
static AD5940Err AppBATSeqCfgGen(void)
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

  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Disable all firstly. */
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  /* LP reference control, Use LP loop to provide DC Bias voltage. */
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);
  /* Determine buffer gain according to ACVoltPP */
  hs_loop.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
  hs_loop.HsDacCfg.HsDacGain = HSDACGAIN_1;
  hs_loop.HsDacCfg.HsDacUpdateRate = 0x1B; //the maximum update rate is 16MHz/7

  hs_loop.HsTiaCfg.DiodeClose = bFALSE;
  hs_loop.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hs_loop.HsTiaCfg.HstiaCtia = 31;  //HSTIA is not used.
  hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_10K;

  hs_loop.SWMatCfg.Dswitch = SWD_CE0;
  hs_loop.SWMatCfg.Pswitch = SWP_AIN1;
  hs_loop.SWMatCfg.Nswitch = SWN_AIN0;  //AIN0 is connected to AIN4 externally by JP3.
  hs_loop.SWMatCfg.Tswitch = 0; //T switch is not used.

  hs_loop.WgCfg.WgType = WGTYPE_SIN;
  hs_loop.WgCfg.GainCalEn = bFALSE;
  hs_loop.WgCfg.OffsetCalEn = bFALSE;
	if(AppBATCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppBATCfg.FreqofData = AppBATCfg.SweepCfg.SweepStart;
    AppBATCfg.SweepCurrFreq = AppBATCfg.SweepCfg.SweepStart;
		AD5940_SweepNext(&AppBATCfg.SweepCfg, &AppBATCfg.SweepNextFreq);
		sin_freq = AppBATCfg.SweepCurrFreq;    
  }
  else
  {
    sin_freq = AppBATCfg.SinFreq;
    AppBATCfg.FreqofData = sin_freq;
  }
  hs_loop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sin_freq, AppBATCfg.SysClkFreq);
  hs_loop.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(AppBATCfg.ACVoltPP/800.0f*2047 + 0.5f);
  hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
  hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&hs_loop);
  //Use LP loop to output bias voltage on AIN4 pin, which provides voltage on AIN0(N switch).
  lp_loop.LpDacCfg.LpdacSel = LPDAC0;
  lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lp_loop.LpDacCfg.LpDacSW = LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN;
  lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_12BIT;
  lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_6BIT;
  lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_loop.LpDacCfg.DataRst = bFALSE;
  lp_loop.LpDacCfg.PowerEn = bTRUE;
  lp_loop.LpDacCfg.DacData12Bit = (uint32_t)((AppBATCfg.DCVolt-200)/2200.0f*4095);
  lp_loop.LpDacCfg.DacData6Bit = 31;  //not used. Set it to middle value.

  lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lp_loop.LpAmpCfg.LpPaPwrEn = bFALSE;
  lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaRf = LPTIARF_20K; //External cap is 1uF.
  lp_loop.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
  lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
  lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(7)|LPTIASW(5)|LPTIASW(9);
  AD5940_LPLoopCfgS(&lp_loop);

  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_AIN2;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_AIN3;
  dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_1P5;
  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* Don't care because it's disabled */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppBATCfg.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppBATCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.DftCfg.DftNum = AppBATCfg.DftNum;
  dsp_cfg.DftCfg.DftSrc = AppBATCfg.DftSrc;
  dsp_cfg.DftCfg.HanWinEn = AppBATCfg.HanWinEn;
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* Don't care about Statistic */
  AD5940_DSPCfgS(&dsp_cfg);
  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bTRUE);
  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one external command to disable sequencer for initialization sequence because we only want it to run one time. */

  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    AppBATCfg.InitSeqInfo.SeqId = SEQID_1;
    AppBATCfg.InitSeqInfo.SeqRamAddr = AppBATCfg.SeqStartAddr;
    AppBATCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppBATCfg.InitSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppBATCfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

//the sequence used to measure battery response voltage.
static AD5940Err AppBATSeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = AppBATCfg.DftSrc;
  clks_cal.DataCount = 1L<<(AppBATCfg.DftNum+2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = AppBATCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppBATCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppBATCfg.SysClkFreq/AppBATCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);
  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));  /* wait 250us for reference power up from hibernate mode. */
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);  /* Enable Waveform generator, ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* Wait for ADC ready. */
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* Start ADC convert and DFT */
  AD5940_SEQGenFetchSeq(NULL, &AppBATCfg.SeqWaitAddr[0]); /* Record the start address of the next command. */

  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks/2));
   AD5940_SEQGenInsert(SEQ_WAIT(WaitClks/2));  
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT/*|AFECTRL_WG*/|AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE);  /* Stop ADC convert and DFT */
  //AD5940_EnterSleepS();/* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

  AppBATCfg.MeasSeqCycleCount = AD5940_SEQCycleTime();
  AppBATCfg.MaxODR = 1/(((AppBATCfg.MeasSeqCycleCount + 10) / 16.0)* 1E-6)  ;
  if(AppBATCfg.BatODR > AppBATCfg.MaxODR)
  {
    /* We have requested a sampling rate that cannot be achieved with the time it
       takes to acquire a sample.
    */
    AppBATCfg.BatODR = AppBATCfg.MaxODR;
  }

  if(error == AD5940ERR_OK)
  {
    AppBATCfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppBATCfg.MeasureSeqInfo.SeqRamAddr = AppBATCfg.InitSeqInfo.SeqRamAddr + AppBATCfg.InitSeqInfo.SeqLen ;
    AppBATCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppBATCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppBATCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

/* This function provide application initialize.   */
AD5940Err AppBATInit(uint32_t *pBuffer, uint32_t BufferSize)
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

  /* Reconfigure FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);									/* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = AppBATCfg.FifoThresh;              /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);

  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  
  /* Start sequence generator */
  /* Initialize sequencer generator */
  if((AppBATCfg.BATInited == bFALSE)||\
       (AppBATCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;   
    AD5940_SEQGenInit(pBuffer, BufferSize);
    /* Generate initialize sequence */
    error = AppBATSeqCfgGen(); /* Application initialization sequence using either MCU or sequencer */
    if(error != AD5940ERR_OK) return error;
    /* Generate measurement sequence */
    error = AppBATSeqMeasureGen();
    if(error != AD5940ERR_OK) return error;
    AppBATCfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
  }
  /* Initialization sequencer  */
  AppBATCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppBATCfg.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer */
  AD5940_SEQMmrTrig(AppBATCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  
	AppBATCheckFreq(AppBATCfg.SweepCfg.SweepStart);
  /* Measurement sequence  */
  AppBATCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppBATCfg.MeasureSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer, and wait for trigger */
  AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */
  AD5940_AFEPwrBW(AppBATCfg.PwrMod, AFEBW_250KHZ);
  AD5940_WriteReg(REG_AFE_SWMUX, 1<<1);
  AppBATCfg.BATInited = bTRUE;  /* BAT application has been initialized. */
  return AD5940ERR_OK;
}

/* Depending on frequency of Sin wave set optimum filter settings */
AD5940Err AppBATCheckFreq(float freq)
{
	ADCFilterCfg_Type filter_cfg;
  DFTCfg_Type dft_cfg;
  HSDACCfg_Type hsdac_cfg;
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  FreqParams_Type freq_params;
  uint32_t SeqCmdBuff[32];
  uint32_t SRAMAddr = 0;;
  /* Step 1: Check Frequency */
  freq_params = AD5940_GetFreqParameters(freq);
  
       if(freq < 0.51)
	{
	hsdac_cfg.ExcitBufGain = EXCITBUFGAIN_2;
    hsdac_cfg.HsDacGain = HSDACGAIN_1;
    hsdac_cfg.HsDacUpdateRate = 0x1B;
    AD5940_HSDacCfgS(&hsdac_cfg);
    AD5940_HSRTIACfgS(HSTIARTIA_40K);
    
    /*Update ADC rate */
    filter_cfg.ADCRate = ADCRATE_800KHZ;
    AppBATCfg.AdcClkFreq = 16e6;
    
    /* Change clock to 16MHz oscillator */
    AD5940_HPModeEn(bFALSE);
	}
        else if(freq < 5 )
	{
	hsdac_cfg.ExcitBufGain = EXCITBUFGAIN_2;
    hsdac_cfg.HsDacGain = HSDACGAIN_1;
    hsdac_cfg.HsDacUpdateRate = 0x1B;
    AD5940_HSDacCfgS(&hsdac_cfg);
    AD5940_HSRTIACfgS(HSTIARTIA_40K);
    
    /*Update ADC rate */
    filter_cfg.ADCRate = ADCRATE_800KHZ;
    AppBATCfg.AdcClkFreq = 16e6;
    
    /* Change clock to 16MHz oscillator */
    AD5940_HPModeEn(bFALSE);
    
	}else if(freq < 450)
	{
    hsdac_cfg.ExcitBufGain = EXCITBUFGAIN_2;
    hsdac_cfg.HsDacGain = HSDACGAIN_1;
    hsdac_cfg.HsDacUpdateRate = 0x1B;
    AD5940_HSDacCfgS(&hsdac_cfg);
    AD5940_HSRTIACfgS(HSTIARTIA_5K);
    
    /*Update ADC rate */
    filter_cfg.ADCRate = ADCRATE_800KHZ;
    AppBATCfg.AdcClkFreq = 16e6;
    
    /* Change clock to 16MHz oscillator */
    AD5940_HPModeEn(bFALSE);
	}
       else if(freq<80000)
       {
	hsdac_cfg.ExcitBufGain = EXCITBUFGAIN_2;
    hsdac_cfg.HsDacGain = HSDACGAIN_1;
    hsdac_cfg.HsDacUpdateRate = 0x1B;
    AD5940_HSDacCfgS(&hsdac_cfg);
    AD5940_HSRTIACfgS(HSTIARTIA_5K);
    
    /*Update ADC rate */
    filter_cfg.ADCRate = ADCRATE_800KHZ;
    AppBATCfg.AdcClkFreq = 16e6;
    
    /* Change clock to 16MHz oscillator */
    AD5940_HPModeEn(bFALSE);
       }
        /* High power mode */
	if(freq >= 80000)
	{
	hsdac_cfg.ExcitBufGain = EXCITBUFGAIN_2;
    hsdac_cfg.HsDacGain = HSDACGAIN_1;
    hsdac_cfg.HsDacUpdateRate = 0x07;
    AD5940_HSDacCfgS(&hsdac_cfg);
    AD5940_HSRTIACfgS(HSTIARTIA_5K);
    
    /*Update ADC rate */
    filter_cfg.ADCRate = ADCRATE_1P6MHZ;
    AppBATCfg.AdcClkFreq = 32e6;
    
    /* Change clock to 32MHz oscillator */
    AD5940_HPModeEn(bTRUE);
	}
  
  /* Step 2: Adjust ADCFILTERCON and DFTCON to set optimumn SINC3, SINC2 and DFTNUM settings  */
  filter_cfg.ADCAvgNum = ADCAVGNUM_16;  /* Don't care because it's disabled */ 
  filter_cfg.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  filter_cfg.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  filter_cfg.BpSinc3 = bFALSE;
  filter_cfg.BpNotch = bTRUE;
  filter_cfg.Sinc2NotchEnable = bTRUE;
  dft_cfg.DftNum = freq_params.DftNum;
  dft_cfg.DftSrc = freq_params.DftSrc;
  dft_cfg.HanWinEn = AppBATCfg.HanWinEn;
  AD5940_ADCFilterCfgS(&filter_cfg);
  AD5940_DFTCfgS(&dft_cfg);
  
  /* Step 3: Calculate clocks needed to get result to FIFO and update sequencer wait command */
  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = freq_params.DftSrc;
  clks_cal.DataCount = 1L<<(freq_params.DftNum+2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppBATCfg.SysClkFreq/AppBATCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);		
	
	
	  SRAMAddr = AppBATCfg.MeasureSeqInfo.SeqRamAddr + AppBATCfg.SeqWaitAddr[0];
	   
           SeqCmdBuff[0] =SEQ_WAIT(WaitClks/2);
           SeqCmdBuff[1] =SEQ_WAIT(WaitClks/2);
      
		AD5940_SEQCmdWrite(SRAMAddr, SeqCmdBuff, 2);
		
  return AD5940ERR_OK;
}

/* Modify registers when AFE wakeup */
static AD5940Err AppBATRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppBATCfg.NumOfData > 0)
  {
    AppBATCfg.FifoDataCount += *pDataCount/4;
    if(AppBATCfg.FifoDataCount >= AppBATCfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if(AppBATCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
	if(AppBATCfg.SweepCfg.SweepEn) /* Need to set new frequency and set power mode */
  {
    AD5940_WGFreqCtrlS(AppBATCfg.SweepNextFreq, AppBATCfg.SysClkFreq);
		AppBATCheckFreq(AppBATCfg.SweepNextFreq);
  }
  return AD5940ERR_OK;
}

/* Depending on the data type, do appropriate data pre-process before return back to controller */
static AD5940Err AppBATDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t DataCount = *pDataCount;
  uint32_t DftResCount = DataCount/2;

  fImpCar_Type * const pOut = (fImpCar_Type*)pData;
  iImpCar_Type * pSrcData = (iImpCar_Type*)pData;

  *pDataCount = 0;
  DataCount = (DataCount/2)*2;  /* We expect both Real and imaginary result.  */

  /* Convert DFT result to int32_t type */
  for(uint32_t i=0; i<DataCount; i++)
  {
    pData[i] &= 0x3ffff;
    if(pData[i]&(1<<17)) /* Bit17 is sign bit */
    {
      pData[i] |= 0xfffc0000; /* Data is 18bit in two's complement, bit17 is the sign bit */
    }
  }
  if(AppBATCfg.state == STATE_RCAL)
  {
    /* Calculate the average voltage. */
    AppBATCfg.RcalVolt.Image = 0;
    AppBATCfg.RcalVolt.Real = 0;
    for(uint32_t i=0;i<DftResCount;i++)
    {
      AppBATCfg.RcalVolt.Real += pSrcData[i].Real;
      AppBATCfg.RcalVolt.Image += pSrcData[i].Image;
    }
    AppBATCfg.RcalVolt.Real /= DftResCount;
    AppBATCfg.RcalVolt.Image /= DftResCount;
    *pDataCount = 0;  /* Report no result to upper application */
  }
  else if(AppBATCfg.state == STATE_BATTERY)
  {
    for(uint32_t i=0; i<DftResCount; i++)
    {
      fImpCar_Type BatImp, BatVolt;
      BatVolt.Real = pSrcData->Real;
      BatVolt.Image = pSrcData->Image;
      pSrcData ++;
      BatImp = AD5940_ComplexDivFloat(&BatVolt, &AppBATCfg.RcalVolt); //ratio measurement, Zbat = Vbat/Vrcal * Rcal;
      BatImp.Image *= AppBATCfg.RcalVal;
      BatImp.Real *= AppBATCfg.RcalVal;
      pOut[i] = BatImp;
		//	printf("i: %d , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f , %.2f\n",AppBATCfg.SweepCfg.SweepIndex, AppBATCfg.SweepCurrFreq, BatImp.Real, BatImp.Image, AppBATCfg.RcalVolt.Real, AppBATCfg.RcalVolt.Image, AppBATCfg.RcalVoltTable[AppBATCfg.SweepCfg.SweepIndex][0], AppBATCfg.RcalVoltTable[AppBATCfg.SweepCfg.SweepIndex][1]);
    }
    *pDataCount = DftResCount;
  }
	/* Calculate next frequency point */
		if(AppBATCfg.SweepCfg.SweepEn == bTRUE)
		{
			AppBATCfg.FreqofData = AppBATCfg.SweepCurrFreq;
			AppBATCfg.SweepCurrFreq = AppBATCfg.SweepNextFreq;
			if(AppBATCfg.state == STATE_BATTERY)
			{
				AppBATCfg.RcalVolt.Real = AppBATCfg.RcalVoltTable[AppBATCfg.SweepCfg.SweepIndex][0];
				AppBATCfg.RcalVolt.Image = AppBATCfg.RcalVoltTable[AppBATCfg.SweepCfg.SweepIndex][1];
			}
			AD5940_SweepNext(&AppBATCfg.SweepCfg, &AppBATCfg.SweepNextFreq);		
		}
  return AD5940ERR_OK;
}

/**
*/
AD5940Err AppBATISR(void *pBuff, uint32_t *pCount)
{
  uint32_t FifoCnt;
  if(AppBATCfg.BATInited == bFALSE)
    return AD5940ERR_APPERROR;
  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* Don't enter hibernate */
  *pCount = 0;

  if(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    /* Now there should be 2 data in FIFO */
    FifoCnt = (AD5940_FIFOGetCnt()/2)*2;
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AppBATRegModify(pBuff, &FifoCnt);   /* If there is need to do AFE re-configure, do it here when AFE is in active state */
    //AD5940_EnterSleepS();  /* Manually put AFE back to hibernate mode. */
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter hibernate mode */
    /* Process data */ 
    AppBATDataProcess((int32_t*)pBuff,&FifoCnt); 
    *pCount = FifoCnt;
    return 0;
  }
  
  return 0;
}

AD5940Err AppBATMeasureRCAL(void)
{
	uint32_t buff[100];
	uint32_t temp;
	AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bFALSE); /* Disable INT0 interrupt for RCAL measurement. */
	AppBATCfg.state = STATE_RCAL;
	if(AppBATCfg.SweepCfg.SweepEn)
	{
		uint32_t i;
    for(i=0;i<AppBATCfg.SweepCfg.SweepPoints;i++)
    {
      AD5940_SEQMmrTrig(SEQID_0);
			while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DATAFIFOTHRESH) == bFALSE);
			printf("i: %d   Freq: %.2f ",AppBATCfg.SweepCfg.SweepIndex, AppBATCfg.SweepCurrFreq);
			AppBATISR(buff, &temp);
			AppBATCfg.RcalVoltTable[i][0] = AppBATCfg.RcalVolt.Real;
			AppBATCfg.RcalVoltTable[i][1] = AppBATCfg.RcalVolt.Image;
			printf(" RcalVolt:(%f,%f)\n",  AppBATCfg.RcalVoltTable[i][0], AppBATCfg.RcalVoltTable[i][1]);
			AD5940_Delay10us(10000);
    }
		AppBATCfg.RcalVolt.Real = AppBATCfg.RcalVoltTable[0][0];
		AppBATCfg.RcalVolt.Image = AppBATCfg.RcalVoltTable[0][1];
	}else
	{
		AD5940_SEQMmrTrig(SEQID_0);
		while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DATAFIFOTHRESH) == bFALSE);
		AppBATISR(buff, &temp);
	}
	AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);
	return 0;
}

/**
  * @}
  */
