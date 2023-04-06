/*!
 @file:    BATImpedance.h
 @author:  $Author: nxu2 $
 @brief:   Battery impedance measurement header file.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef _BAT_IMPEDANCE_H_
#define _BAT_IMPEDANCE_H_
#include "ad5940.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#define PRECHARGE_WAIT_MS   4000    //precharge time in ms.

#define PRECHARGE_CH1       1
#define PRECHARGE_CH2       2
#define PRECHARGE_CH3       3

#define PRECHARGE_RCAL      PRECHARGE_CH1
#define PRECHARGE_BAT       PRECHARGE_CH2
#define PRECHARGE_AMP       PRECHARGE_CH3
/* 
  Note: this example will use SEQID_0 as measurement sequence, and use SEQID_1 as init sequence. 
  SEQID_3 is used for calibration.
*/

#define STATE_IDLE        0   /**< Initial state. */
#define STATE_RCAL        1   /**< Measure Rcal response voltage. */
#define STATE_BATTERY     2   /**< Measure battery response voltage. */
typedef struct
{
/* Common configurations for all kinds of Application. */
  uint32_t state;               /* 0: Init, 1: measure Rcal, 2: Measure Battery. */
  BoolFlag bParaChanged;        /* Indicate to generate sequence again. It's auto cleared by AppBATInit */
  BoolFlag bDoCal;              /* Need to do calibration. */
  uint32_t SeqStartAddr;        /* Initialaztion sequence start address in SRAM of AD5940  */
  uint32_t MaxSeqLen;           /* Limit the maximum sequence.   */
  uint32_t SeqStartAddrCal;     /* Measurement sequence start address in SRAM of AD5940 */
  uint32_t MaxSeqLenCal;
/* Application related parameters */ 
  float SysClkFreq;             /* The real frequency of system clock */
  float WuptClkFreq;            /* The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float AdcClkFreq;             /* The real frequency of ADC clock */
  uint32_t FifoThresh;           /* FIFO threshold. Should be N*4 */   
  float BatODR;                 /* in Hz. ODR decides the period of WakeupTimer who will trigger sequencer periodically. DFT number and sample frequency decides the maxim ODR. */
  int32_t NumOfData;            /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value here. Otherwise, set it to '-1' which means never stop. */
  uint32_t PwrMod;              /* Control Chip power mode(LP/HP) */
  float ACVoltPP;               /* Final AC excitation voltage on pin AIN1 in mV peak to peak unit. */
  float DCVolt;                 /* The DC bias voltage on AIN1 pin. Unit is mV. */
  float RcalVal;                /* Rcal value in mOhm */
  float SinFreq;                /* Frequency of excitation signal */
  uint8_t ADCSinc3Osr;          /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
  uint8_t ADCSinc2Osr;          /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
  uint32_t DftNum;              /* DFT number */
  uint32_t DftSrc;              /* DFT Source */
  BoolFlag HanWinEn;            /* Enable Hanning window */
  uint32_t SeqWaitAddr[3];    
/* Sweep Function Control */
  SoftSweepCfg_Type SweepCfg;
/* Private variables for internal usage */
  float SweepCurrFreq;
  float SweepNextFreq;
  float FreqofData;  
  BoolFlag BATInited;           /* If the program run firstly, generated sequence commands */
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type MeasureSeqInfo;
  BoolFlag StopRequired;        /* After FIFO is ready, stop the measurement sequence */
  uint32_t FifoDataCount;       /* Count how many times impedance have been measured */
  uint32_t MeasSeqCycleCount;   /* How long the measurement sequence will take */
  float MaxODR;                 /* Max ODR for sampling in this config */
  fImpCar_Type RcalVolt;        /* The measured Rcal resistor(R1) response voltage. */
  float RcalVoltTable[100][2];    
/* End */
}AppBATCfg_Type;

#define BATCTRL_START          0
#define BATCTRL_STOPNOW        1
#define BATCTRL_STOPSYNC       2
#define BATCTRL_SHUTDOWN       4   /* Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */
#define BATCTRL_MRCAL          5   /* Measure RCAL response voltage */
#define BATCTRL_GETFREQ				 6

AD5940Err AppBATGetCfg(void *pCfg);
AD5940Err AppBATInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppBATISR(void *pBuff, uint32_t *pCount);
AD5940Err AppBATCtrl(int32_t BatCtrl, void *pPara);
AD5940Err AppBATCheckFreq(float freq);
AD5940Err AppBATMeasureRCAL(void);

#endif
