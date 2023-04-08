/*!
*****************************************************************************
@file:    SWVTest.c
@author:  $Author: mlambe $
@brief:   Square Wave Voltammetry measurement sequences.
@date:    Updated 15th May 2021
-----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

/** @addtogroup AD5940_System_Examples
* @{
*    @defgroup Ramp_Test_Example
*    @brief Using sequencer to generate ramp signal and control ADC to sample data.
*    @details
* @note Need to update code when runs at S2 silicon.
* @todo update LPDAC switch settings for S2 and LPDAC 1LSB bug.
* @todo Calibrate ADC/PGA firstly to get accurate current. (Voltage/Rtia = Current)
* @note The method to calculate LPDAC ouput voltage
*        - #define LSB_DAC12BIT (2.2V/4095)
*        - #define LSB_DAC6BIT  (2.2V/4095*64)
*        - Volt_12bit = Code12Bit*LSB_DAC12BIT + 0.2V
*        - Volt_6bit = Code6Bit*LSB_DAC6BIT + 0.2V
* 
* # Ramp Signal Parameters definition
* 
* @code
* (Vbias - Vzero):
*     RampPeakVolt   -->            /                                    
*                                  /                                      
*                                 /                                       
*                                /                                                                 
*                               /                                                               
*                              /                                      
*                             /                                         
*                            /                                            
*                           /                                                
*    RampStartVolt   -->   /                    
* 
* Vzero: If there is no limitation on Vzero, Set VzeroStart to 2.2 and VzeroPeak to 0.4V
* Voltage VzeroStart -->  ______       
*                               |                
* Voltage VzeroPeak  -->        |_____          
*            
*                                            
* Vbias: Vbias is calculated from RampPeakVolt, RampStartVolt, VzeroStart and VzeroPeak.                                     
* Voltage VbiasPeak  -->       /|    /            
*                             / |   /               
*                            /  |  /                
*                           /   | /                 
* Voltage VbiasStart -->   /    |/     
*        
* RampState define:    S0 | S1  | S2 |          
* @endcode
* 
*  # The sequencer method to do Ramp test.
* The Ramp test need to update DAC data in real time to generate required waveform, and control ADC to start sample data. \n
* We used two kinds of sequence to realize it. One is to control DAC where SEQ0 and SEQ1 are used, another sequence SEQ2 controls ADC. 
*  ## Sequence Allocation
* SEQ3 is used to initialize AD5940.\n
* SEQ0/1 is used to generate voltage step.\n
* SEQ2 is used to startup ADC to sample one point.
* 
* |SRAM allocation|||
* |------------|----------------|---------|
* |SequenceID  | AddressRange   |  Usage  |
* |SEQID_3     | 0x0000-0xzzzz  | Initialization sequence|
* |SEQID_2     | 0xzzzz-0xyyyy  | ADC control sequence, run this sequence will get one ADC result|
* |SEQID_0/1   | 0xyyyy-end     | DAC update sequence. If size if not enough for all steps, use it like a FIFO.|
* Where 0xzzzz equals to SEQ3 length, 0xyyyy equals to sum of SEQ2 and SEQ3 length.
* In one word, put SEQ2 commands right after SEQ3. Don't waste any SRAM resource.
*  ##Sequencer Running Order
* The sequencer running order is set to firstly update DAC then start ADC. Repeat this process until all waveform generated.
* Below is explanation of sequencer running order.
* @code
* DAC voltage changes with sequencer, assume each step is 0.05V start from 0.2V
* 400mV->                             _______
* 350mV->                  _____     |
* 300mV->       _____     |     |____|
* 250mV->      |     |____|                           
* 200mV->    __|
* Update DAC:  ?     ?    ?     ?    ?       ?       -No update        
*            SEQ0  SEQ1  SEQ0  SEQ1 SEQ0    SEQ1    SEQ0       
*                 |   / |   / |   / |   /   |   /   |   /  |        
*                 SEQ2  SEQ2 SEQ2  SEQ2    SEQ2    SEQ2   |The final sequence is set to disable sequencer
* WuptTrigger  ?  ?    ?  ?    ?  ?    ?  ?    ?  ?    ?  ?    ?  ?    ?  ?
* Time Spend   |t1| t2 |t1| t2 |t1| t2 |t1| t2 |t1| t2 |t1| t2 |t1| t2 |t1| t2
*                                                                 |The following triggers are ignored because sequencer is disabled 
* Wupt: Wakeup Timer   
* @endcode
* 
* The final sequence will disable sequencer thus disable the whole measurement. It could be SEQ0 or SEQ1.  \n
* SEQ2 will always follow SEQ0/SEQ1 to turn on ADC to sample data.                                         \n
* SEQ0/1 and SEQ2 is managed by wakeup timer. The time delay between SEQ0/1 and SEQ
* is set by user. Reason is that after updating DAC, signal needs some time to settle before sample it.    \n
* In above figure, the time t1 is the delay set by user which controls where ADC starts sampling.
* 
* SEQ2 commands are fixed. Function is simply turn on ADC for a while and turn off it 
*      after required number of data ready.                                                                \n
* SEQ0/1 is always changing its start address to update DAC with different voltage.                        \n
* Check above figure we can see SEQ0/SEQ1 is repeatedly trigged by Wakeuptimer, if we don't change the start
* Address of SEQ0/SEQ1, they will always update DAC with same data, thus no waveform generated.
* 
* Considering below SEQ0 command which is similar for SEQ1 on modifying SEQxINFO register.:
* 
* **Sequencer Command Block 1**
* @code
* //Total sequence command length is **4**
* SEQ_WR(REG_AFE_LPDACDAT0, 0x1234);           //update DAC with correct voltage
* SEQ_WAIT(10);                                //wait 10clocks to allow DAC update
* SEQ_WR(REG_AFE_SEQ1INFO, NextAddr|SeqLen);   //The next sequence is SEQ1, set it to correct address where stores commands.
* SEQ_SLP();                                   //Put AFE to hibernate/sleep mode.
* @endcode
* 
* It will update DAC with data 0x1234, then it wait 10 clocks to allow LPDAC update.
* The final command is to send AFE to sleep state.
* The third commands here is to allow modify sequence infomation by sequencer. Above piece of commands are running by SEQ0.
* It modify the start address of **SEQ1**. SEQ1 has same ability to update DAC data but with **different** data. 
* By the time Wakeup Timer triggers SEQ1, it will update DAC with correct data.
* 
* The last block of sequencer command is to disable sequencer.
* 
* **Sequencer Command Block 2**
* @code
* SEQ_NOP();
* SEQ_NOP();
* SEQ_NOP();
* SEQ_STOP();                                   //Put AFE to hibernate/sleep mode.
* @endcode
* 
* Total SRAM is 6kB in AD594x. In normal other application, we use 2kB for sequencer and 4kB for FIFO.
* Assume the ramp test require 128 steps, then the sequence length is 4*128 = 512, each command need 4Byte. So it costs 2kB SRAM.
* When ramp test requires hundres of voltage steps(ADC samples), 2kB SRAM is far from enough. We recommend to use 4kB for sequencer
* and 2kB for data FIFO. 
* If ramp test require more steps, then we need to update SRAM with commands dynamically, use it as a ping-pong buffer.
* 
* **Sequencer Command Block 3**
* @code
* SEQ_WR(REG_AFE_LPDACDAT0, 0x1234);           
* SEQ_WAIT(10);                                
* SEQ_WR(REG_AFE_SEQ1INFO, NextAddr|SeqLen);
* SEQ_INT0();                                  //Generate custom interrupt 0 to inform MCU to update ping-pong buffer.
* @endcode
* 
* @{
* **/

#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "SqrWaveVoltammetry.h"

/**
* @brief The ramp application paramters.
* @details Do not modify following default parameters. Use the function in AD5940Main.c to change it. 
* 
* */
static AppSWVCfg_Type AppSWVCfg = 
{
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,
  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 0,
  
  .LFOSCClkFreq = 32000.0,
  .SysClkFreq = 16000000.0,
  .AdcClkFreq = 16000000.0,
  .RcalVal = 10000.0,
  .ADCRefVolt = 1820.0f,              /* 1.8V or 1.82V? */
  /* Describe Ramp signal */
  .RampStartVolt = -1000.0f,          /* -1V */
  .RampPeakVolt = +1000.0f,           /* +1V */
  .VzeroStart = 2200.0f,              /* 2.2V */
  .VzeroPeak = 400.0f,                /* 0.4V */
  .StepNumber = 866,
  /* Receive path configuration */
  .SampleDelay = 1.0f,                /* 1ms */
  .LPTIARtiaSel = LPTIARTIA_20K,      /* Maximum current decides RTIA value */
  .ExternalRtiaValue = 20000.0f,      /* Optional external RTIA resistore value in Ohm. */
  .AdcPgaGain = ADCPGA_1,     
  .ADCSinc3Osr = ADCSINC3OSR_4,
  .FifoThresh = 4,
  /* Priviate parameters */
  .SWVInited = bFALSE,
  .StopRequired = bFALSE,
  .RampState = SWV_STATE0,
  .bFirstDACSeq = bTRUE,
  .bSqrWaveHiLevel = bFALSE,
  /* Configure Square wave */
  .Frequency = 25,              /* Frequency in Hz */
  .SqrWvAmplitude = 25,         /* Square wave amplitude in mV */
  .SqrWvRampIncrement = 5,      /* Ramp increment in mV*/
};

/**
* @todo add paramater check. 
* SampleDelay will limited by wakeup timer, check WUPT register value calculation equation below for reference.
* SampleDelay > 1.0ms is acceptable.
* ...
* */

/**
* @brief This function is provided for upper controllers that want to change
*        application parameters specially for user defined parameters.
* @param pCfg: The pointer used to store application configuration structure pointer.
* @return none.
*/
AD5940Err AppSWVGetCfg(void *pCfg)
{
  if(pCfg)
  {
    *(AppSWVCfg_Type**)pCfg = &AppSWVCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

/**
* @brief Control application like start, stop.
* @param Command: The command for this application, select from below paramters
*        - APPCTRL_START: start the measurement. Note: the ramp test need firstly call function AppSWVInit() every time before start it.
*        - APPCTRL_STOPNOW: Stop the measurement immediately.
*        - APPCTRL_STOPSYNC: Stop the measuremnt when current measured data is read back.
*        - APPCTRL_SHUTDOWN: Stop the measurement immediately and put AFE to shut down mode(turn off LP loop and enter hibernate).
* @return none.
*/
AD5940Err AppSWVCtrl(uint32_t Command, void *pPara)
{
  switch (Command)
  {
  case APPCTRL_START:
    {
      WUPTCfg_Type wupt_cfg;
      
      if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP;  /* Wakeup Failed */
      if(AppSWVCfg.SWVInited == bFALSE)
        return AD5940ERR_APPERROR;
      /**
      *  SWV example is special, because the sequence is dynamically generated. 
      *  Before 'START' ramp test, call AppSWVInit firstly. 
      */
      if(AppSWVCfg.RampState == SWV_STOP)
        return AD5940ERR_APPERROR;
      
      /* Start it */
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_D;
      wupt_cfg.WuptOrder[0] = SEQID_0;
      wupt_cfg.WuptOrder[1] = SEQID_2;
      wupt_cfg.WuptOrder[2] = SEQID_1;
      wupt_cfg.WuptOrder[3] = SEQID_2;
      wupt_cfg.SeqxSleepTime[SEQID_2] = 1;
      wupt_cfg.SeqxWakeupTime[SEQID_2] = (uint32_t)(AppSWVCfg.LFOSCClkFreq*AppSWVCfg.SampleDelay/1000.0f) - 1;
      wupt_cfg.SeqxSleepTime[SEQID_0] = 1;
      wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(AppSWVCfg.LFOSCClkFreq*((1/AppSWVCfg.Frequency*500) - AppSWVCfg.SampleDelay)/1000.0f) - 4;
      wupt_cfg.SeqxSleepTime[SEQID_1] = wupt_cfg.SeqxSleepTime[SEQID_0];
      wupt_cfg.SeqxWakeupTime[SEQID_1] = wupt_cfg.SeqxWakeupTime[SEQID_0];
      
      AD5940_WUPTCfg(&wupt_cfg);
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
      break;
    }
  case APPCTRL_STOPSYNC:
    {
      AppSWVCfg.StopRequired = bTRUE;
      break;
    }
  case APPCTRL_SHUTDOWN:
    {
      AppSWVCtrl(APPCTRL_STOPNOW, 0);  /* Stop the measurement if it's running. */
      AD5940_ShutDownS();
    }
    break;
  default:
    break;
  }
  return AD5940ERR_OK;
}

/**
* @brief Generate initialization sequence and write the commands to SRAM.
* @return return error code.
*/
static AD5940Err AppSWVSeqInitGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;
  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lploop_cfg;
  DSPCfg_Type dsp_cfg;
  /* Start sequence generator here */
  AD5940_SEQGenCtrl(bTRUE);
  
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */
  
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
  
  lploop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
  lploop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_BOOST3;
  lploop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K;
  lploop_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
  lploop_cfg.LpAmpCfg.LpTiaRtia = AppSWVCfg.LPTIARtiaSel;
  if(AppSWVCfg.LPTIARtiaSel == LPTIARTIA_OPEN) /* User want to use external RTIA */
    lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(13)|LPTIASW(2)|LPTIASW(4)|LPTIASW(5)|LPTIASW(9)/*|LPTIASW(10)*/; /* SW5/9 is closed to support external RTIA resistor */
  else
    lploop_cfg.LpAmpCfg.LpTiaSW = /*LPTIASW(13)|*/LPTIASW(2)|LPTIASW(4);
  lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
	lploop_cfg.LpDacCfg.DacData6Bit = (uint32_t)((AppSWVCfg.VzeroStart - 200.0f)/DAC6BITVOLT_1LSB);
  lploop_cfg.LpDacCfg.DacData12Bit = (int32_t)((AppSWVCfg.RampStartVolt)/DAC12BITVOLT_1LSB) + lploop_cfg.LpDacCfg.DacData6Bit*64  ;
  lploop_cfg.LpDacCfg.DataRst = bFALSE;
  lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA/*|LPDACSW_VBIAS2PIN*/|LPDACSW_VZERO2LPTIA/*|LPDACSW_VZERO2PIN*/;
  lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lploop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT; /* Step Vbias. Use 12bit DAC ouput */
  lploop_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;  /* Base is Vzero. Use 6 bit DAC ouput */
  lploop_cfg.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lploop_cfg);
  
  AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
  dsp_cfg.ADCBaseCfg.ADCPga = AppSWVCfg.AdcPgaGain;
  
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppSWVCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;  /* ADC runs at 16MHz clock in this example, sample rate is 800kHz */
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;        /* We use data from SINC3 filter */
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSINC2OSR_1067;  /* Don't care */
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;   /* Don't care because it's disabled */
  AD5940_DSPCfgS(&dsp_cfg);
  
  /* Sequence end. */
  AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */
  
  /* Stop sequence generator here */
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  if(error == AD5940ERR_OK)
  {
    AD5940_StructInit(&AppSWVCfg.InitSeqInfo, sizeof(AppSWVCfg.InitSeqInfo));
    if(SeqLen >= AppSWVCfg.MaxSeqLen)
      return AD5940ERR_SEQLEN;
    
    AppSWVCfg.InitSeqInfo.SeqId = SEQID_3;
    AppSWVCfg.InitSeqInfo.SeqRamAddr = AppSWVCfg.SeqStartAddr;
    AppSWVCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppSWVCfg.InitSeqInfo.SeqLen = SeqLen;
    AppSWVCfg.InitSeqInfo.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&AppSWVCfg.InitSeqInfo);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

/**
* @brief Generate ADC control sequence and write the commands to SRAM.
* @return return error code.
*/
static AD5940Err AppSWVSeqADCCtrlGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;
  
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  
  clks_cal.DataCount = 1; /* Sample one point everytime */
  clks_cal.DataType = DATATYPE_SINC3;
  clks_cal.ADCSinc3Osr = AppSWVCfg.ADCSinc3Osr;
  clks_cal.ADCSinc2Osr = ADCSINC2OSR_1067;  /* Don't care */
  clks_cal.ADCAvgNum = ADCAVGNUM_2; /* Don't care */
  clks_cal.RatioSys2AdcClk = AppSWVCfg.SysClkFreq/AppSWVCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);
  
  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGpioCtrlS(AGPIO_Pin2);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*100));  /* wait 250us for reference power up */
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
    AD5940_StructInit(&AppSWVCfg.ADCSeqInfo, sizeof(AppSWVCfg.ADCSeqInfo));
    if((SeqLen + AppSWVCfg.InitSeqInfo.SeqLen) >= AppSWVCfg.MaxSeqLen)
      return AD5940ERR_SEQLEN;
    AppSWVCfg.ADCSeqInfo.SeqId = SEQID_2;
    AppSWVCfg.ADCSeqInfo.SeqRamAddr = AppSWVCfg.InitSeqInfo.SeqRamAddr + AppSWVCfg.InitSeqInfo.SeqLen ;
    AppSWVCfg.ADCSeqInfo.pSeqCmd = pSeqCmd;
    AppSWVCfg.ADCSeqInfo.SeqLen = SeqLen;
    AppSWVCfg.ADCSeqInfo.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&AppSWVCfg.ADCSeqInfo);
  }
  else
    return error; /* Error */
  return AD5940ERR_OK;
}

/**
* @brief Calculate DAC code step by step. 
* @details The calculation is based on following variables.
*          - RampStartVolt
*          - RampPeakVolt
*          - VzeroStart
*          - VzeroPeak
*          - StepNumber
*          Below variables must be initialzed before call this function. It's done in function @ref AppSWVInit
*          - RampState
*          - CurrStepPos
*          - bDACCodeInc
*          - CurrRampCode
* @return return error code.
*/
static AD5940Err RampDacRegUpdate(uint32_t *pDACData)
{
  uint32_t VbiasCode, VzeroCode;
  
  if (AppSWVCfg.bRampOneDir)
  { 
	if(AppSWVCfg.RampStartVolt > AppSWVCfg.RampPeakVolt)
		AppSWVCfg.bDACCodeInc = bFALSE;
    switch(AppSWVCfg.RampState)
    {
    case SWV_STATE0: /* Begin of Ramp  */
      AppSWVCfg.CurrVzeroCode = (uint32_t)((AppSWVCfg.VzeroStart - 200.0f)/DAC6BITVOLT_1LSB);
      AppSWVCfg.RampState = SWV_STATE1;
      break;
    case SWV_STATE1:
      if(AppSWVCfg.CurrStepPos >= AppSWVCfg.StepNumber/2)
      {
        AppSWVCfg.RampState = SWV_STATE2;   /* Enter State2 */
        AppSWVCfg.CurrVzeroCode = (uint32_t)((AppSWVCfg.VzeroPeak - 200.0f)/DAC6BITVOLT_1LSB);
      }
      break;
    case SWV_STATE2:
      if(AppSWVCfg.CurrStepPos >= AppSWVCfg.StepNumber)
        AppSWVCfg.RampState = SWV_STOP;     /* Enter Stop */
      break;
    case SWV_STOP:
      break;
    }
  }
  
  else
  {
    switch(AppSWVCfg.RampState)
    {
    case SWV_STATE0: /* Begin of Ramp  */
      AppSWVCfg.CurrVzeroCode = (uint32_t)((AppSWVCfg.VzeroStart - 200.0f)/DAC6BITVOLT_1LSB);
      AppSWVCfg.RampState = SWV_STATE1;
      break;
      
    case SWV_STATE1:
      if(AppSWVCfg.CurrStepPos >= AppSWVCfg.StepNumber/4)
      {
        AppSWVCfg.RampState = SWV_STATE2;   /* Enter State2 */
        AppSWVCfg.CurrVzeroCode = (uint32_t)((AppSWVCfg.VzeroPeak - 200.0f)/DAC6BITVOLT_1LSB);
      }
      break;
      
    case SWV_STATE2:
      if(AppSWVCfg.CurrStepPos >= (AppSWVCfg.StepNumber*2)/4)
      {
        AppSWVCfg.RampState = SWV_STATE3;   /* Enter State2 */
        AppSWVCfg.bDACCodeInc = AppSWVCfg.bDACCodeInc ? bFALSE : bTRUE;
      }
      break;
    case SWV_STATE3:
      if(AppSWVCfg.CurrStepPos >= (AppSWVCfg.StepNumber*3)/4)
      {
        AppSWVCfg.RampState = SWV_STATE4;   /* Enter State2 */
        AppSWVCfg.CurrVzeroCode = (uint32_t)((AppSWVCfg.VzeroPeak - 200.0f)/DAC6BITVOLT_1LSB);
      }
      break;    
    case SWV_STATE4:
      if(AppSWVCfg.CurrStepPos >= (AppSWVCfg.StepNumber))
        AppSWVCfg.RampState = SWV_STOP;     /* Enter Stop */
      break;
    case SWV_STOP:
      break;
    }
    
  }
  AppSWVCfg.CurrStepPos++;  
  if(AppSWVCfg.bSqrWaveHiLevel)
  {
    if(AppSWVCfg.bDACCodeInc)
      AppSWVCfg.CurrRampCode -= (AppSWVCfg.DACCodePerStep - AppSWVCfg.DACCodePerRamp);
    else
      AppSWVCfg.CurrRampCode -= AppSWVCfg.DACCodePerStep;
    AppSWVCfg.bSqrWaveHiLevel = bFALSE;
  }else
  {
    
    if(AppSWVCfg.bDACCodeInc)
      AppSWVCfg.CurrRampCode += AppSWVCfg.DACCodePerStep;
    else
      AppSWVCfg.CurrRampCode += (AppSWVCfg.DACCodePerStep - AppSWVCfg.DACCodePerRamp);
    AppSWVCfg.bSqrWaveHiLevel = bTRUE;
  }
  VzeroCode = AppSWVCfg.CurrVzeroCode;
  VbiasCode = (uint32_t)(VzeroCode*64 + AppSWVCfg.CurrRampCode);
  
  if(VbiasCode < (VzeroCode*64))
    VbiasCode --;
  /* Truncate */
  if(VbiasCode > 4095) VbiasCode = 4095;
  if(VzeroCode >   63) VzeroCode =   63;
  *pDACData = (VzeroCode<<12)|VbiasCode;
  return AD5940ERR_OK;
}

/* Geneate sequence(s) to update DAC step by step */
/* Note: this function doesn't need sequencer generator */

/**
* @brief Update DAC sequence in SRAM in real time.  
* @details This function generates sequences to update DAC code step by step. It's also called in interrupt
*          function when half commands in SRAM has been completed. We don't use sequence generator to save memory.
*          Check more details from documentation of this example. @ref Ramp_Test_Example
* @return return error code
* 
* */
static AD5940Err AppSWVSeqDACCtrlGen(void)
{  
#define SEQLEN_ONESTEP    4L  /* How many sequence commands are needed to update LPDAC. */
#define CURRBLK_BLK0      0   /* Current block is BLOCK0 */
#define CURRBLK_BLK1      1   /* Current block is BLOCK1 */
  AD5940Err error = AD5940ERR_OK;
  uint32_t BlockStartSRAMAddr;
  uint32_t DACData, SRAMAddr;
  uint32_t i;
  uint32_t StepsThisBlock;
  BoolFlag bIsFinalBlk;
  uint32_t SeqCmdBuff[SEQLEN_ONESTEP];
  
  /* All below static variables are inited in below 'if' block. They are only used in this function */
  static BoolFlag bCmdForSeq0 = bTRUE;
  static uint32_t DACSeqBlk0Addr, DACSeqBlk1Addr;
  static uint32_t StepsRemainning, StepsPerBlock, DACSeqCurrBlk;
  
  AppSWVCfg.StepNumber = (uint32_t)(2*(AppSWVCfg.RampPeakVolt - AppSWVCfg.RampStartVolt)/AppSWVCfg.SqrWvRampIncrement);
  if(AppSWVCfg.bRampOneDir == bFALSE)
  {
    AppSWVCfg.StepNumber*=2;
		AppSWVCfg.StepNumber-=2;
  }
  //AppSWVCfg.FifoThresh = AppSWVCfg.StepNumber;
  
  if(AppSWVCfg.StepNumber >1020)
  {
    printf("Error: Selected Increment, StartVolt and PeakVolt exceed accepted limits \n");
    while(1){}
  }
  /* Do some math calculations */
  if(AppSWVCfg.bFirstDACSeq == bTRUE)
  {
    /* Reset bIsFirstRun at end of function. */
    int32_t DACSeqLenMax;
    StepsRemainning = AppSWVCfg.StepNumber;
    DACSeqLenMax = (int32_t)AppSWVCfg.MaxSeqLen - (int32_t)AppSWVCfg.InitSeqInfo.SeqLen - (int32_t)AppSWVCfg.ADCSeqInfo.SeqLen;
    if(DACSeqLenMax < SEQLEN_ONESTEP*4)
      return AD5940ERR_SEQLEN;  /* No enough sequencer SRAM available */
    DACSeqLenMax -= SEQLEN_ONESTEP*2;  /* Reserve commands each block */
    StepsPerBlock = DACSeqLenMax/SEQLEN_ONESTEP/2;
    DACSeqBlk0Addr = AppSWVCfg.ADCSeqInfo.SeqRamAddr + AppSWVCfg.ADCSeqInfo.SeqLen;
    DACSeqBlk1Addr = DACSeqBlk0Addr + StepsPerBlock*SEQLEN_ONESTEP;
    DACSeqCurrBlk = CURRBLK_BLK0;    
    
    /* Analog part */
    AppSWVCfg.DACCodePerStep = AppSWVCfg.SqrWvAmplitude/DAC12BITVOLT_1LSB;
    AppSWVCfg.DACCodePerRamp = AppSWVCfg.SqrWvRampIncrement/DAC12BITVOLT_1LSB;
    
#if ALIGIN_VOLT2LSB
    AppSWVCfg.DACCodePerStep = (int32_t)AppSWVCfg.DACCodePerStep;
    AppSWVCfg.DACCodePerRamp = (int32_t)AppSWVCfg.DACCodePerRamp;
#endif
    if(AppSWVCfg.DACCodePerStep > 0)
      AppSWVCfg.bDACCodeInc = bTRUE;
    else
      AppSWVCfg.bDACCodeInc = bFALSE;
    AppSWVCfg.CurrRampCode = AppSWVCfg.RampStartVolt/DAC12BITVOLT_1LSB;
    AppSWVCfg.RampState = SWV_STATE0;   /* Init state to STATE0 */
    AppSWVCfg.CurrStepPos = 0;
    
    bCmdForSeq0 = bTRUE;      /* Start with SEQ0 */
  }
  
  if(StepsRemainning == 0) return AD5940ERR_OK; /* Done. */
  bIsFinalBlk = StepsRemainning <= StepsPerBlock?bTRUE:bFALSE;
  if(bIsFinalBlk)
    StepsThisBlock = StepsRemainning;
  else
    StepsThisBlock = StepsPerBlock;
  StepsRemainning -= StepsThisBlock;
  
  BlockStartSRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0)?\
DACSeqBlk0Addr:DACSeqBlk1Addr;
SRAMAddr = BlockStartSRAMAddr;

for(i=0; i<StepsThisBlock - 1; i++)
{
  uint32_t CurrAddr = SRAMAddr;
  SRAMAddr += SEQLEN_ONESTEP;  /* Jump to next sequence */
  RampDacRegUpdate(&DACData);
  SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
  SeqCmdBuff[1] = SEQ_WAIT(10); /* !!!NOTE LPDAC need 10 clocks to update data. Before send AFE to sleep state, wait 10 extra clocks */
  SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0?REG_AFE_SEQ1INFO:REG_AFE_SEQ0INFO,\
    (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR)|(SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
  SeqCmdBuff[3] = SEQ_SLP();
  AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
  bCmdForSeq0 = bCmdForSeq0?bFALSE:bTRUE;
}
/* Add final DAC update */
if(bIsFinalBlk)/* This is the final block */
{
  uint32_t CurrAddr = SRAMAddr;
  SRAMAddr += SEQLEN_ONESTEP;  /* Jump to next sequence */
  /* After update LPDAC with final data, we let sequencer to run 'final final' command, to disable sequencer.  */
  RampDacRegUpdate(&DACData);
  SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
  SeqCmdBuff[1] = SEQ_WAIT(10); /* !!!NOTE LPDAC need 10 clocks to update data. Before send AFE to sleep state, wait 10 extra clocks */
  SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0?REG_AFE_SEQ1INFO:REG_AFE_SEQ0INFO,\
    (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR)|(SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
  SeqCmdBuff[3] = SEQ_SLP();
  AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
  CurrAddr += SEQLEN_ONESTEP;
  /* The final final command is to disable sequencer. */
  SeqCmdBuff[0] = SEQ_NOP();    /* Do nothing */
  SeqCmdBuff[1] = SEQ_NOP();
  SeqCmdBuff[2] = SEQ_NOP();
  SeqCmdBuff[3] = SEQ_STOP();   /* Stop sequencer. */
  /* Disable sequencer, END of sequencer interrupt is generated. */
  AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
}
else /* This is not the final block */
{
  /* Jump to next block. */
  uint32_t CurrAddr = SRAMAddr;
  SRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0)?\
DACSeqBlk1Addr:DACSeqBlk0Addr;
RampDacRegUpdate(&DACData);          
SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
SeqCmdBuff[1] = SEQ_WAIT(10);
SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0?REG_AFE_SEQ1INFO:REG_AFE_SEQ0INFO, 
                       (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR)|(SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
SeqCmdBuff[3] = SEQ_INT0(); /* Generate Custom interrupt 0. */ 
AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
bCmdForSeq0 = bCmdForSeq0?bFALSE:bTRUE;
}

DACSeqCurrBlk = (DACSeqCurrBlk == CURRBLK_BLK0)?\
CURRBLK_BLK1:CURRBLK_BLK0; /* Switch between Block0 and block1 */
if(AppSWVCfg.bFirstDACSeq)
{
  AppSWVCfg.bFirstDACSeq = bFALSE;
  if(bIsFinalBlk == bFALSE)
  { /* Otherwise there is no need to init block1 sequence */
    error = AppSWVSeqDACCtrlGen();
    if(error != AD5940ERR_OK)
      return error;
  }
  /* This is the first DAC sequence. */
  AppSWVCfg.DACSeqInfo.SeqId = SEQID_0;
  AppSWVCfg.DACSeqInfo.SeqLen = SEQLEN_ONESTEP;
  AppSWVCfg.DACSeqInfo.SeqRamAddr = BlockStartSRAMAddr;
  AppSWVCfg.DACSeqInfo.WriteSRAM = bFALSE; /* No need to write to SRAM. We already write them above. */
  AD5940_SEQInfoCfg(&AppSWVCfg.DACSeqInfo);
}
return AD5940ERR_OK;
}


/**
* @brief Calibrate LPTIA internal RTIA resistor(s).
* @details This function will do calibration using parameters stored in @ref AppEDACfg structure.
* @return return error code.
*/
static AD5940Err AppSWVRtiaCal(void)
{
  fImpPol_Type RtiaCalValue;  /* Calibration result */
  LPRTIACal_Type lprtia_cal;
  AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));
  
  lprtia_cal.LpAmpSel = LPAMP0;
  lprtia_cal.bPolarResult = bTRUE;                /* Magnitude + Phase */
  lprtia_cal.AdcClkFreq = AppSWVCfg.AdcClkFreq;
  lprtia_cal.SysClkFreq = AppSWVCfg.SysClkFreq;
  lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22;        /* Use SINC2 data as DFT data source */
  lprtia_cal.DftCfg.DftNum = DFTNUM_2048;         /* Maximum DFT number */
  lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;        /* @todo For frequency under 12Hz, need to optimize DFT source. Use SINC3 data as DFT source */
  lprtia_cal.DftCfg.HanWinEn = bTRUE;
  lprtia_cal.fFreq = AppSWVCfg.AdcClkFreq/4/22/2048*3;  /* Sample 3 period of signal, 13.317Hz here. Do not use DC method, because it needs ADC/PGA calibrated firstly(but it's faster) */
  lprtia_cal.fRcal = AppSWVCfg.RcalVal;
  lprtia_cal.LpTiaRtia = AppSWVCfg.LPTIARtiaSel;
  lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
  lprtia_cal.bWithCtia = bFALSE;
  AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
  AppSWVCfg.RtiaValue = RtiaCalValue;
  printf("Rtia,%f,%f\n", RtiaCalValue.Magnitude, RtiaCalValue.Phase);
  return AD5940ERR_OK;
}

/**
* @brief Initialize the ramp test. Call this functions every time before start ramp test.
* @param pBuffer: the buffer for sequencer generator. Only need to provide it for the first time.
* @param BufferSize: The buffer size start from pBuffer.
* @return return error code.
*/
AD5940Err AppSWVInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;  
  FIFOCfg_Type fifo_cfg;
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
  
  /* Start sequence generator */
  /* Initialize sequencer generator */
  if((AppSWVCfg.SWVInited == bFALSE)||\
    (AppSWVCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;
    
    if(AppSWVCfg.LPTIARtiaSel == LPTIARTIA_OPEN) /* Internal RTIA is opened. User wants to use external RTIA resistor */
    {
      AppSWVCfg.RtiaValue.Magnitude = AppSWVCfg.ExternalRtiaValue;
      AppSWVCfg.RtiaValue.Phase = 0;
    }
    else  
      AppSWVRtiaCal();   
    
    AppSWVCfg.SWVInited = bFALSE;    
    AD5940_SEQGenInit(pBuffer, BufferSize);
    /* Generate sequence and write them to SRAM start from address AppSWVCfg.SeqStartAddr */
    error = AppSWVSeqInitGen();      /* Application initialization sequence */
    if(error != AD5940ERR_OK) return error;
    error = AppSWVSeqADCCtrlGen();   /* ADC control sequence */
    if(error != AD5940ERR_OK) return error;
    AppSWVCfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
  }
  
  /* Reconfigure FIFO, The Rtia calibration function may generate data that stored to FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE);        /* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
  fifo_cfg.FIFOThresh = AppSWVCfg.FifoThresh;    /* Change FIFO paramters */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;
  AD5940_FIFOCfg(&fifo_cfg);
  
  /* Clear all interrupts */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Generate DAC sequence */
  AppSWVCfg.bFirstDACSeq = bTRUE;
  error = AppSWVSeqDACCtrlGen();
  if(error != AD5940ERR_OK) return error;
  
  /* Configure sequence info. */
  AppSWVCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppSWVCfg.InitSeqInfo);
  
  AD5940_SEQCtrlS(bTRUE); /* Enable sequencer */
  AD5940_SEQMmrTrig(AppSWVCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
  
  AppSWVCfg.ADCSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppSWVCfg.ADCSeqInfo);
  
  AppSWVCfg.DACSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppSWVCfg.DACSeqInfo);
  
  AD5940_SEQCtrlS(bFALSE);
  AD5940_WriteReg(REG_AFE_SEQCNT, 0);
  AD5940_SEQCtrlS(bTRUE);   /* Enable sequencer, and wait for trigger */
  AD5940_ClrMCUIntFlag();   /* Clear interrupt flag generated before */
  
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ); /* Set to low power mode */
  
  AppSWVCfg.SWVInited = bTRUE;  /* SWV application has been initialized. */
  return AD5940ERR_OK;
}

/**
* @brief This function is called in ISR when AFE has been wakeup and we can access registers.
* @param pData: the buffer points to data read back from FIFO. Not needed for this application-SWV
* @param pDataCount: The data count in pData buffer.
* @return return error code.
*/
static int32_t AppSWVRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppSWVCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }  
  return AD5940ERR_OK;
}

/**
* @brief Depending on the data type, do appropriate data pre-process before return back to controller
* @param pData: the buffer points to data read back from FIFO. Not needed for this application-SWV
* @param pDataCount: The data count in pData buffer.
* @return return error code.
*/
static int32_t AppSWVDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t i, datacount;
  datacount = *pDataCount;
  float *pOut = (float *)pData;
  
  for(i=0;i<datacount;i++)
  {
    pData[i] &= 0xffff;
    pOut[i] = AD5940_ADCCode2Volt(pData[i], AppSWVCfg.AdcPgaGain, AppSWVCfg.ADCRefVolt)/AppSWVCfg.RtiaValue.Magnitude * 1e3f;  /* Result unit is uA. */
  }
  return 0;
}

/**
* @brief The interrupt service routine for SWV test.
* @param pBuff: The buffer provides by host, used to store data read back from FIFO.
* @param pCount: The available buffer size starts from pBuff.
* @return return error code.
*/
AD5940Err AppSWVISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount;
  uint32_t FifoCnt;
  BuffCount = *pCount;
  uint32_t IntFlag;
  
  if(AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
    return AD5940ERR_WAKEUP;  /* Wakeup Failed */
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
  *pCount = 0;
  IntFlag = AD5940_INTCGetFlag(AFEINTC_0);
  if(IntFlag & AFEINTSRC_CUSTOMINT0)          /* High priority. */
  {
    AD5940Err error;
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
		//AD5940_McuSetHigh();
    error = AppSWVSeqDACCtrlGen();
		//AD5940_McuSetLow();
    if(error != AD5940ERR_OK) return error;
   // AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    //AD5940_EnterSleepS(); /* If there is need to do AFE re-configure, do it here when AFE is in active state */
  }
  if(IntFlag&AFEINTSRC_DATAFIFOTHRESH)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    
    if(FifoCnt > BuffCount)
    {
      ///@todo buffer is limited.
    }
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AppSWVRegModify(pBuff, &FifoCnt);   
  //  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    //AD5940_EnterSleepS();
    /* Process data */ 
    AppSWVDataProcess((int32_t*)pBuff,&FifoCnt);
    *pCount = FifoCnt;
    return 0;
  }
  if(IntFlag & AFEINTSRC_ENDSEQ)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    /* Process data */ 
    AppSWVDataProcess((int32_t*)pBuff,&FifoCnt);
    *pCount = FifoCnt;
    AppSWVCtrl(APPCTRL_STOPNOW, 0);    /* Stop the Wakeup Timer. */

  }
  return 0;
} 

/**
* @}
* @}
*/
