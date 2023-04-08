/*!
 *****************************************************************************
 @file:    RampTest.H
 @author:  $Author: nxu2 $
 @brief:   Ramp Test header file.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
#ifndef _SWVTEST_H_
#define _SWVTEST_H_
#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"

/* Do not modify following parameters */
#define ALIGIN_VOLT2LSB     0   /* Set it to 1 to align each voltage step to 1LSB of DAC. 0: step code is fractional. */
#define DAC12BITVOLT_1LSB   (2200.0f/4095)  //mV
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64)  //mV

/**
 * The Ramp application related paramter structure
*/
typedef struct
{
/* Common configurations for all kinds of Application. */
  BoolFlag  bParaChanged;         /**< Indicate to generate sequence again. It's auto cleared by AppBIAInit */
  uint32_t  SeqStartAddr;         /**< Initialaztion sequence start address in SRAM of AD5940  */
  uint32_t  MaxSeqLen;            /**< Limit the maximum sequence.   */
  uint32_t  SeqStartAddrCal;      /**< Not used for Ramp.Calibration sequence start address in SRAM of AD5940 */
  uint32_t  MaxSeqLenCal;         /**< Not used for Ramp. */
/* Application related parameters */ 
  float     LFOSCClkFreq;         /**< The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float     SysClkFreq;           /**< The real frequency of system clock */
  float     AdcClkFreq;           /**< The real frequency of ADC clock */
  float     RcalVal;              /**< Rcal value in Ohm */
  float     ADCRefVolt;           /**< The real ADC voltage in mV. */
  /* Describe Ramp signal */
  float     RampStartVolt;        /**< The start voltage of ramp signal in mV */
  float     RampPeakVolt;         /**< The maximum or minimum voltage of ramp in mV */
  float     VzeroStart;           /**< The start voltage of Vzero in mV. Set it to 2400mV by default */
  float     VzeroPeak;            /**< The peak voltage of Vzero in mV. Set it to 200mV by default */
  uint32_t  StepNumber;           /**< Total number of steps. Limited to 4095. */
  /* Receive path configuration */
  float     SampleDelay;          /**< The time delay between update DAC and start ADC */
  uint32_t  LPTIARtiaSel;         /**< Select RTIA */
  float     ExternalRtiaValue;    /**< The optional external RTIA value in Ohm. Disconnect internal RTIA to use external RTIA. When using internal RTIA, this value is ignored. */
  uint32_t  AdcPgaGain;           /**< PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint8_t   ADCSinc3Osr;          /**< We use data from SINC3 filter. */
  /* Digital related */
  uint32_t  FifoThresh;           /**< FIFO Threshold value */
/* Private variables for internal usage */
  BoolFlag  SWVInited;           /**< If the program run firstly, generated initialization sequence commands */
  fImpPol_Type  RtiaValue;        /**< Calibrated Rtia value */
  SEQInfo_Type  InitSeqInfo;
  SEQInfo_Type  ADCSeqInfo;
  BoolFlag      bFirstDACSeq;     /**< Init DAC sequence */
  SEQInfo_Type  DACSeqInfo;       /**< The first DAC update sequence info */
  uint32_t  CurrStepPos;          /**< Current position */     
  float     DACCodePerStep;       /**< DAC codes in square waveform */
  float     DACCodePerRamp;       /**< DAC codes needed to ramp increment */
  float     CurrRampCode;         /**<  */   
  float     Frequency;						/**< Frequency of square wave */
  float     SqrWvAmplitude;       /**< Set amplitude of square wave */
  float     SqrWvRampIncrement;   /**< Ramp increase in mV */
  uint32_t  CurrVzeroCode;        
  BoolFlag  bDACCodeInc;          /**< Increase DAC code.  */
  BoolFlag  bSqrWaveHiLevel;			/**< Flag to indicate square wave high level */
	BoolFlag bRampOneDir;						/**< Ramp in one direction only */
  BoolFlag  StopRequired;         /**< After FIFO is ready, stop the measurement sequence */
  enum _RampState{SWV_STATE0 = 0, SWV_STATE1, SWV_STATE2, SWV_STATE3, SWV_STATE4, SWV_STOP} RampState;
}AppSWVCfg_Type;

#define APPCTRL_START          0
#define APPCTRL_STOPNOW        1
#define APPCTRL_STOPSYNC       2
#define APPCTRL_SHUTDOWN       3   /**< Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */

AD5940Err AppSWVInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppSWVGetCfg(void *pCfg);
AD5940Err AppSWVISR(void *pBuff, uint32_t *pCount);
AD5940Err AppSWVCtrl(uint32_t Command, void *pPara);
void AD5940_McuSetLow(void);
void AD5940_McuSetHigh(void);

#endif
