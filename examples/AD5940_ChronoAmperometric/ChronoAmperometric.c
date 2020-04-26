/*!
*****************************************************************************
@file:    ChronoAmperometric.c
@author:  $Author: nxu2 $
@brief:   Chrono-amperometric measurement sequences.
@version: $Revision: 766 $
@date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
-----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "ChronoAmperometric.h"

/* 
Application configuration structure. Specified by user from template.
The variables are usable in this whole application.
It includes basic configuration for sequencer generator and application related parameters
*/
AppCHRONOAMPCfg_Type AppCHRONOAMPCfg = 
{
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,
  
  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 0,
  .FifoThresh = 1000,
  
  .SysClkFreq = 16000000.0,
  .WuptClkFreq = 32000.0,
  .AdcClkFreq = 16000000.0,
  .AmpODR = 1, 
  .NumOfData = -1,
  .RcalVal = 10000.0, /* 10kOhm */
	
  .ExtRtiaVal = 0,
  .PwrMod = AFEPWR_LP,
  /* LPTIA Configure */
  .LptiaRtiaSel = LPTIARTIA_10K,
  .LpTiaRf = LPTIARF_1M,
  .LpTiaRl = LPTIARLOAD_SHORT,
  .ReDoRtiaCal = bTRUE,
  .RtiaCalValue = 0, 
  /*LPDAC Configure */
  .Vbias = 1100,
  .Vzero = 1100,
  
  /* Waveform Configuration */
  .pulseAmplitude = 500,         /* Amplitude of step in mV */
  .pulseLength = 500,     /* Length of transient in ms*/  
  .EndSeq = bFALSE,      /* Flag to indicate sequence has finished */ 
	
  /* ADC Configuration*/
  .ADCPgaGain = ADCPGA_1P5,
  .ADCSinc3Osr = ADCSINC3OSR_4,
  .ADCSinc2Osr = ADCSINC2OSR_44,
  .ADCRefVolt = 1.82,			/* Measure voltage on ADCRefVolt pin and enter here*/
  .DataFifoSrc = DATATYPE_SINC2,        /* Data type must be SINC2 for chrono-amperometric measurement*/
  .CHRONOAMPInited = bFALSE,
  .StopRequired = bFALSE,
};

/**
This function is provided for upper controllers that want to change 
application parameters specially for user defined parameters.
*/
AD5940Err AppCHRONOAMPGetCfg(void *pCfg)
{
  if(pCfg){
    *(AppCHRONOAMPCfg_Type**)pCfg = &AppCHRONOAMPCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

AD5940Err AppCHRONOAMPCtrl(int32_t AmpCtrl, void *pPara)
{
  switch (AmpCtrl)
  {
  case CHRONOAMPCTRL_START:
    {  
      WUPTCfg_Type wupt_cfg;
			SEQCfg_Type seq_cfg;

      AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
      if(AppCHRONOAMPCfg.CHRONOAMPInited == bFALSE)
        return AD5940ERR_APPERROR;
			
			/* Configure FIFO and Sequencer for normal Amperometric Measurement */
			AD5940_FIFOThrshSet(AppCHRONOAMPCfg.FifoThresh);
			seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer, others for data FIFO */
			seq_cfg.SeqBreakEn = bFALSE;
			seq_cfg.SeqIgnoreEn = bFALSE;
			seq_cfg.SeqCntCRCClr = bTRUE;
			seq_cfg.SeqEnable = bTRUE;
			seq_cfg.SeqWrTimer = 0;
			AD5940_SEQCfg(&seq_cfg);
			
      /* Configure Wakeup Timer*/
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
      wupt_cfg.WuptOrder[0] = SEQID_0;
      wupt_cfg.SeqxSleepTime[SEQID_0] = 1; /* The minimum value is 1. Do not set it to zero. Set it to 1 will spend 2 32kHz clock. */
      wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(AppCHRONOAMPCfg.WuptClkFreq*AppCHRONOAMPCfg.AmpODR)-2-1;
      AD5940_WUPTCfg(&wupt_cfg);
      
      AppCHRONOAMPCfg.FifoDataCount = 0;  /* restart */
      break;
    }
  case CHRONOAMPCTRL_STOPNOW:
    {
      AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
      /* Start Wupt right now */
      AD5940_WUPTCtrl(bFALSE);
      /* There is chance this operation will fail because sequencer could put AFE back 
        to hibernate mode just after waking up. Use STOPSYNC is better. */
      AD5940_WUPTCtrl(bFALSE);
      break;
    }
  case CHRONOAMPCTRL_STOPSYNC:
    {
      AppCHRONOAMPCfg.StopRequired = bTRUE;
      break;
    }
  case CHRONOAMPCTRL_SHUTDOWN:
    {
      AppCHRONOAMPCtrl(CHRONOAMPCTRL_STOPNOW, 0);  /* Stop the measurement if it's running. */
      /* Turn off LPloop related blocks which are not controlled automatically by sleep operation */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);
      AD5940_EnterSleepS();  /* Enter Hibernate */
    }
	case CHRONOAMPCTRL_PULSETEST:
	{
		FIFOCfg_Type fifo_cfg;
		AD5940_WUPTCtrl(bFALSE);
		AppCHRONOAMPCfg.bMeasureTransient = bTRUE;
		/* Reconfigure FIFO for Pulse test*/
		AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE);									/* Disable FIFO firstly */
		fifo_cfg.FIFOEn = bTRUE;
		fifo_cfg.FIFOMode = FIFOMODE_FIFO;
		fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
		fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
		fifo_cfg.FIFOThresh = 1000;              
		AD5940_FIFOCfg(&fifo_cfg);
		
		/* Trigger sequence by MMR write */
		AD5940_SEQMmrTrig(AppCHRONOAMPCfg.TransientSeqInfo.SeqId); 
	}
    break;
  default:
    break;
  }
  return AD5940ERR_OK;
}

/* Generate init sequence */
static AD5940Err AppCHRONOAMPSeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;
  
  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lp_loop;
  DSPCfg_Type dsp_cfg;
  HSLoopCfg_Type hs_loop;
  
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
  aferef_cfg.Lp1V1BuffEn = bTRUE;
  aferef_cfg.Lp1V8BuffEn = bTRUE;
  /* LP reference control - turn off them to save power*/
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	
  
  lp_loop.LpDacCfg.LpdacSel = LPDAC0;
  lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lp_loop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|/*LPDACSW_VBIAS2PIN|*/LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN;
  lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lp_loop.LpDacCfg.DataRst = bFALSE;
  lp_loop.LpDacCfg.PowerEn = bTRUE;	
	lp_loop.LpDacCfg.DacData6Bit = (uint32_t)((AppCHRONOAMPCfg.Vzero-200)/DAC6BITVOLT_1LSB);
	lp_loop.LpDacCfg.DacData12Bit =(int32_t)((AppCHRONOAMPCfg.SensorBias)/DAC12BITVOLT_1LSB) + lp_loop.LpDacCfg.DacData6Bit*64;
	if(lp_loop.LpDacCfg.DacData12Bit>lp_loop.LpDacCfg.DacData6Bit*64)
		lp_loop.LpDacCfg.DacData12Bit--;
    
  lp_loop.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_loop.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lp_loop.LpAmpCfg.LpTiaRf = AppCHRONOAMPCfg.LpTiaRf;
  lp_loop.LpAmpCfg.LpTiaRload = AppCHRONOAMPCfg.LpTiaRl;	
	if(AppCHRONOAMPCfg.ExtRtia == bTRUE)
  {
    lp_loop.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(9)|LPTIASW(2)|LPTIASW(4)|LPTIASW(5)/*|LPTIASW(12)*/|LPTIASW(13); 
  }else
  {
    lp_loop.LpAmpCfg.LpTiaRtia = AppCHRONOAMPCfg.LptiaRtiaSel;
    lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(5)|LPTIASW(2)|LPTIASW(4)/*|LPTIASW(12)*/|LPTIASW(13); 
  }
  AD5940_LPLoopCfgS(&lp_loop);
  
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
  dsp_cfg.ADCBaseCfg.ADCPga = AppCHRONOAMPCfg.ADCPgaGain;
  
  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));
  
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* Don't care because it's disabled */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;	/* Tell filter block clock rate of ADC*/
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppCHRONOAMPCfg.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppCHRONOAMPCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* Don't care about Statistic */
  AD5940_DSPCfgS(&dsp_cfg);
  
  hs_loop.SWMatCfg.Dswitch = 0;
  hs_loop.SWMatCfg.Pswitch = 0;
  hs_loop.SWMatCfg.Nswitch = 0;
  hs_loop.SWMatCfg.Tswitch = 0; 
  AD5940_HSLoopCfgS(&hs_loop);
  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGpioCtrlS(0);     
  
  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */
  
  /* Stop here */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  if(error == AD5940ERR_OK)
  {
    AppCHRONOAMPCfg.InitSeqInfo.SeqId = SEQID_1;
    AppCHRONOAMPCfg.InitSeqInfo.SeqRamAddr = AppCHRONOAMPCfg.SeqStartAddr;
    AppCHRONOAMPCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppCHRONOAMPCfg.InitSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppCHRONOAMPCfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

static AD5940Err AppCHRONOAMPTransientMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;
  uint32_t VbiasCode, VzeroCode;
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  
	if(AppCHRONOAMPCfg.DataFifoSrc != DATATYPE_SINC2)
		return AD5940ERR_ERROR; /* FIFO data must be SINC2 filter for measuring transient */
	/* Calculate LPDAC codes */
	VzeroCode = (uint32_t)((AppCHRONOAMPCfg.Vzero-200)/DAC6BITVOLT_1LSB);
	VbiasCode = (int32_t)((AppCHRONOAMPCfg.pulseAmplitude + AppCHRONOAMPCfg.SensorBias)/DAC12BITVOLT_1LSB) + VzeroCode*64;
	if(VbiasCode < (VzeroCode*64))
    VbiasCode --;
  /* Truncate */
  if(VbiasCode > 4095) VbiasCode = 4095;
  if(VzeroCode >   63) VzeroCode =   63;
	
  clks_cal.DataType = AppCHRONOAMPCfg.DataFifoSrc;
  clks_cal.DataCount = AppCHRONOAMPCalcDataNum(AppCHRONOAMPCfg.pulseLength);
  clks_cal.ADCSinc2Osr = AppCHRONOAMPCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppCHRONOAMPCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppCHRONOAMPCfg.SysClkFreq/AppCHRONOAMPCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);
  
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGpioCtrlS(AGPIO_Pin1);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);                       /* Start ADC conversion before applying step to capture peak */
  AD5940_WriteReg(REG_AFE_LPDACDAT0, VzeroCode<<12|VbiasCode);
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));                      /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV, bFALSE);       /* Stop ADC */
  AD5940_WriteReg(REG_AFE_LPDACDAT0,(uint32_t)((AppCHRONOAMPCfg.Vzero-200)/DAC6BITVOLT_1LSB)<<12|(int32_t)((AppCHRONOAMPCfg.SensorBias)/DAC12BITVOLT_1LSB) + VzeroCode*64);
  AD5940_SEQGpioCtrlS(0);
  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */
  AD5940_EnterSleepS();/* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  
  if(error == AD5940ERR_OK)
  {
    AppCHRONOAMPCfg.TransientSeqInfo.SeqId = SEQID_2;
    AppCHRONOAMPCfg.TransientSeqInfo.SeqRamAddr = AppCHRONOAMPCfg.MeasureSeqInfo.SeqRamAddr + AppCHRONOAMPCfg.MeasureSeqInfo.SeqLen ;
    AppCHRONOAMPCfg.TransientSeqInfo.pSeqCmd = pSeqCmd;
    AppCHRONOAMPCfg.TransientSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppCHRONOAMPCfg.TransientSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}
static AD5940Err AppCHRONOAMPSeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;
  
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  
  clks_cal.DataType = AppCHRONOAMPCfg.DataFifoSrc;
  clks_cal.DataCount = 1;
  clks_cal.ADCSinc2Osr = AppCHRONOAMPCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppCHRONOAMPCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppCHRONOAMPCfg.SysClkFreq/AppCHRONOAMPCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);
  
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGpioCtrlS(AGPIO_Pin1);
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);  /* Start ADC convert and DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* wait for first data ready */
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV, bFALSE);  /* Stop ADC */
  AD5940_SEQGpioCtrlS(0);
  AD5940_EnterSleepS();/* Goto hibernate */
  /* Sequence end. */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  
  if(error == AD5940ERR_OK)
  {
    AppCHRONOAMPCfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppCHRONOAMPCfg.MeasureSeqInfo.SeqRamAddr = AppCHRONOAMPCfg.InitSeqInfo.SeqRamAddr + AppCHRONOAMPCfg.InitSeqInfo.SeqLen ;
    AppCHRONOAMPCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppCHRONOAMPCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* Write command to SRAM */
    AD5940_SEQCmdWrite(AppCHRONOAMPCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}
static AD5940Err AppCHRONOAMPRtiaCal(void)
{
  fImpPol_Type RtiaCalValue;  /* Calibration result */
  LPRTIACal_Type lprtia_cal;
  AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

  lprtia_cal.LpAmpSel = LPAMP0;
  lprtia_cal.bPolarResult = bTRUE;                /* Magnitude + Phase */
  lprtia_cal.AdcClkFreq = AppCHRONOAMPCfg.AdcClkFreq;
  lprtia_cal.SysClkFreq = AppCHRONOAMPCfg.SysClkFreq;
  lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22;        /* Use SINC2 data as DFT data source */
  lprtia_cal.DftCfg.DftNum = DFTNUM_2048;         /* Maximum DFT number */
  lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
  lprtia_cal.DftCfg.HanWinEn = bTRUE;
  lprtia_cal.fFreq = AppCHRONOAMPCfg.AdcClkFreq/4/22/2048*3;  /* Sample 3 period of signal, 13.317Hz here. Do not use DC method, because it needs ADC/PGA calibrated firstly(but it's faster) */
  lprtia_cal.fRcal = AppCHRONOAMPCfg.RcalVal;
  lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
  lprtia_cal.bWithCtia = bFALSE;
  lprtia_cal.LpTiaRtia = AppCHRONOAMPCfg.LptiaRtiaSel;
  AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
  AppCHRONOAMPCfg.RtiaCalValue = RtiaCalValue;
  return AD5940ERR_OK;
}
/**
* @brief Initialize the amperometric test. Call this function every time before starting amperometric test.
* @param pBuffer: the buffer for sequencer generator. Only need to provide it for the first time.
* @param BufferSize: The buffer size start from pBuffer.
* @return return error code.
*/
AD5940Err AppCHRONOAMPInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  
  AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */
  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  
	/* Do RTIA calibration */
  if(((AppCHRONOAMPCfg.ReDoRtiaCal == bTRUE) || \
      AppCHRONOAMPCfg.CHRONOAMPInited == bFALSE) && AppCHRONOAMPCfg.ExtRtia == bFALSE)
  {
    AppCHRONOAMPRtiaCal();
    AppCHRONOAMPCfg.ReDoRtiaCal = bFALSE;
  } 
  
  /* Reconfigure FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE);									/* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
  fifo_cfg.FIFOThresh = AppCHRONOAMPCfg.FifoThresh;              /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);
  
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  
  /* Start sequence generator */
  /* Initialize sequencer generator */
  if((AppCHRONOAMPCfg.CHRONOAMPInited == bFALSE)||\
    (AppCHRONOAMPCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;   
    AD5940_SEQGenInit(pBuffer, BufferSize);
    
    /* Generate initialize sequence */
    error = AppCHRONOAMPSeqCfgGen(); /* Application initialization sequence using either MCU or sequencer */
    if(error != AD5940ERR_OK) return error;
    
    /* Generate measurement sequence */
    error = AppCHRONOAMPSeqMeasureGen();
    if(error != AD5940ERR_OK) return error;
    
    /* Generate transient sequence */
    error = AppCHRONOAMPTransientMeasureGen();
    if(error != AD5940ERR_OK) return error;
    
    AppCHRONOAMPCfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
  }
  
  /* Initialization sequencer  */
  AppCHRONOAMPCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppCHRONOAMPCfg.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer */
  AD5940_SEQMmrTrig(AppCHRONOAMPCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
   
  /* Transient sequence */
  AppCHRONOAMPCfg.TransientSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppCHRONOAMPCfg.TransientSeqInfo);
  
  /* Measurement sequence  */
  AppCHRONOAMPCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppCHRONOAMPCfg.MeasureSeqInfo);
  
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer, and wait for trigger */
  AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */

  AD5940_AFEPwrBW(AppCHRONOAMPCfg.PwrMod, AFEBW_250KHZ);
  AppCHRONOAMPCfg.CHRONOAMPInited = bTRUE;  /* CHRONOAMP application has been initialized. */
  AppCHRONOAMPCfg.bMeasureTransient = bFALSE;
  return AD5940ERR_OK;
}

/* Modify registers when AFE wakeup */
static AD5940Err AppCHRONOAMPRegModify(int32_t * const pData, uint32_t *pDataCount)
{
	  FIFOCfg_Type fifo_cfg;
		SEQCfg_Type seq_cfg;
	/* Reset dtat FIFO threshold for normal amp */
	if(AppCHRONOAMPCfg.EndSeq)
	{
		AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE);									/* Disable FIFO firstly */
		fifo_cfg.FIFOEn = bTRUE;
		fifo_cfg.FIFOMode = FIFOMODE_FIFO;
		fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
		fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
		fifo_cfg.FIFOThresh = AppCHRONOAMPCfg.FifoThresh;              /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
		AD5940_FIFOCfg(&fifo_cfg);
		
		seq_cfg.SeqEnable = bTRUE;
		AD5940_SEQCfg(&seq_cfg);  /* Enable sequencer, and wait for trigger */
		AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */
	}
  if(AppCHRONOAMPCfg.NumOfData > 0)
  {
    AppCHRONOAMPCfg.FifoDataCount += *pDataCount/4;
    if(AppCHRONOAMPCfg.FifoDataCount >= AppCHRONOAMPCfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if(AppCHRONOAMPCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  return AD5940ERR_OK;
}

/* Depending on the data type, do appropriate data pre-process before return back to controller */
static AD5940Err AppCHRONOAMPDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t i, datacount;
  datacount = *pDataCount;
  float *pOut = (float *)pData;
  for(i=0;i<datacount;i++)
  {
    pData[i] &= 0xffff;
    pOut[i] = AppCHRONOAMPCalcCurrent(pData[i]);
  }
  return 0;
}

/**

*/
AD5940Err AppCHRONOAMPISR(void *pBuff, uint32_t *pCount)
{
  extern uint32_t IntCount;
  uint32_t FifoCnt;

  *pCount = 0;
  if(AppCHRONOAMPCfg.CHRONOAMPInited == bFALSE)
    return AD5940ERR_APPERROR;
  AD5940_ReadReg(REG_AFE_ADCDAT); /* Any SPI Operation can wakeup AFE */  
	AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    if(AppCHRONOAMPCfg.bMeasureTransient) /* We are measuring transient */
    {
      AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
      AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
      AppCHRONOAMPDataProcess((int32_t*)pBuff, &FifoCnt);
      *pCount = FifoCnt;
      IntCount++;
    }else/* Normal Amperometric measurement*/
    {
      AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
      AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
      AppCHRONOAMPRegModify(pBuff, &FifoCnt);   /* If there is need to do AFE re-configure, do it here when AFE is in active state */
			AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Unlock so sequencer can put AD5940 to sleep */
			AD5940_EnterSleepS();
      /* Process data */ 
      AppCHRONOAMPDataProcess((int32_t*)pBuff,&FifoCnt); 
      *pCount = FifoCnt;
    }
    return 0;
  }
  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_ENDSEQ) == bTRUE)/* End sequence interrupt fires at end of transient sequence */
  {
		AppCHRONOAMPCfg.EndSeq = bTRUE;
    FifoCnt = AD5940_FIFOGetCnt();
    IntCount++;
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
		AppCHRONOAMPRegModify(pBuff, &FifoCnt);
    /* Process data */ 
    AppCHRONOAMPDataProcess((int32_t*)pBuff, &FifoCnt);
    *pCount = FifoCnt;
  }
  
  return 0;
} 

uint32_t AppCHRONOAMPCalcDataNum(uint32_t time)
{
  uint32_t temp = 0;
  const uint32_t sinc2osr_table[] = {22,44,89,178,267,533,640,667,800,889,1067,1333,0};
  const uint32_t sinc3osr_table[] = {5,4,2,0};
  
  temp = time*800/sinc2osr_table[AppCHRONOAMPCfg.ADCSinc2Osr]/sinc3osr_table[AppCHRONOAMPCfg.ADCSinc3Osr];
  
  return temp;
}
/* Calculate voltage */
float AppCHRONOAMPCalcVoltage(uint32_t ADCcode)
{
  float kFactor = 1.835/1.82;
  float fVolt = 0.0;
  int32_t tmp = 0;
  tmp = ADCcode - 32768;
  switch(AppCHRONOAMPCfg.ADCPgaGain)
  {
    case ADCPGA_1:
      fVolt = ((float)(tmp)/32768)*(AppCHRONOAMPCfg.ADCRefVolt/1)*kFactor;
      break;
    case ADCPGA_1P5:
      fVolt = ((float)(tmp)/32768)*(AppCHRONOAMPCfg.ADCRefVolt/1.5f)*kFactor;
      break;
    case ADCPGA_2:
      fVolt = ((float)(tmp)/32768)*(AppCHRONOAMPCfg.ADCRefVolt/2)*kFactor;
      break;
    case ADCPGA_4:
      fVolt = ((float)(tmp)/32768)*(AppCHRONOAMPCfg.ADCRefVolt/4)*kFactor;
      break;
    case ADCPGA_9:
      fVolt = ((float)(tmp)/32768)*(AppCHRONOAMPCfg.ADCRefVolt/9)*kFactor;
      break;
  } 
  return fVolt;
}
/* Calculate current in uA */
float AppCHRONOAMPCalcCurrent(uint32_t ADCcode)
{
  float fCurrent, fVoltage = 0.0;
  fVoltage = AppCHRONOAMPCalcVoltage(ADCcode);
  fCurrent = fVoltage/AppCHRONOAMPCfg.RtiaCalValue.Magnitude;
  return -fCurrent*1000000;
}
