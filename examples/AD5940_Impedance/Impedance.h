/*!
 *****************************************************************************
 @file:    Impedance.h
 @author:  Neo XU
 @brief:   4-wire/2-wire impedance measurement header file.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#ifndef _IMPEDANCESEQUENCES_H_
#define _IMPEDANCESEQUENCES_H_
#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"

typedef struct
{
/* Common configurations for all kinds of Application. */
  BoolFlag bParaChanged;        /* Indicate to generate sequence again. It's auto cleared by AppBIAInit */
  uint32_t SeqStartAddr;        /* Initialaztion sequence start address in SRAM of AD5940  */
  uint32_t MaxSeqLen;           /* Limit the maximum sequence.   */
  uint32_t SeqStartAddrCal;     /* Measurement sequence start address in SRAM of AD5940 */
  uint32_t MaxSeqLenCal;
/* Application related parameters */ 
  float ImpODR;                 /*  */
  int32_t NumOfData;            /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value here. Otherwise, set it to '-1' which means never stop. */
  float WuptClkFreq;            /* The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float SysClkFreq;             /* The real frequency of system clock */
  float AdcClkFreq;             /* The real frequency of ADC clock */
  float RcalVal;                /* Rcal value in Ohm */
  /* Switch Configuration */
  uint32_t DswitchSel;
  uint32_t PswitchSel;
  uint32_t NswitchSel;
  uint32_t TswitchSel;
  uint32_t PwrMod;              /* Control Chip power mode(LP/HP) */
  uint32_t HstiaRtiaSel;        /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
  uint32_t ExcitBufGain;        /* Select from  EXCTBUFGAIN_2, EXCTBUFGAIN_0P25 */     
  uint32_t HsDacGain;           /* Select from  HSDACGAIN_1, HSDACGAIN_0P2 */
  uint32_t HsDacUpdateRate;
  float DacVoltPP;              /* DAC output voltage in mV peak to peak. Maximum value is 800mVpp. Peak to peak voltage  */
  float BiasVolt;               /* The excitation signal is DC+AC. This parameter decides the DC value in mV unit. 0.0mV means no DC bias.*/
  float SinFreq;                /* Frequency of excitation signal */
  uint32_t DftNum;              /* DFT number */
  uint32_t DftSrc;              /* DFT Source */
  BoolFlag HanWinEn;            /* Enable Hanning window */
  uint32_t AdcPgaGain;          /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint8_t ADCSinc3Osr;
  uint8_t ADCSinc2Osr;  
  uint8_t ADCAvgNum;
  /* Sweep Function Control */
  SoftSweepCfg_Type SweepCfg;
  uint32_t FifoThresh;           /* FIFO threshold. Should be N*4 */
/* Private variables for internal usage */
/* Private variables for internal usage */
  float SweepCurrFreq;
  float SweepNextFreq;
  float FreqofData;                         /* The frequency of latest data sampled */
  BoolFlag IMPInited;                       /* If the program run firstly, generated sequence commands */
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type MeasureSeqInfo;
  BoolFlag StopRequired;          /* After FIFO is ready, stop the measurement sequence */
  uint32_t FifoDataCount;         /* Count how many times impedance have been measured */
}AppIMPCfg_Type;

#define IMPCTRL_START          0
#define IMPCTRL_STOPNOW        1
#define IMPCTRL_STOPSYNC       2
#define IMPCTRL_GETFREQ        3   /* Get Current frequency of returned data from ISR */
#define IMPCTRL_SHUTDOWN       4   /* Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */


int32_t AppIMPInit(uint32_t *pBuffer, uint32_t BufferSize);
int32_t AppIMPGetCfg(void *pCfg);
int32_t AppIMPISR(void *pBuff, uint32_t *pCount);
int32_t AppIMPCtrl(uint32_t Command, void *pPara);

#endif
