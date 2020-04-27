/*!
 *****************************************************************************
 @file:    RampTest.h
 @author:  Neo Xu
 @brief:   Ramp Test header file.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
#ifndef _RAMPTEST_H_
#define _RAMPTEST_H_
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
	BoolFlag bTestFinished;			/**< Variable to indicate ramt test has finished >*/
  /* Describe Ramp signal */
  float     RampStartVolt;        /**< The start voltage of ramp signal in mV */
  float     RampPeakVolt;         /**< The maximum or minimum voltage of ramp in mV */
  float     VzeroStart;           /**< The start voltage of Vzero in mV. Set it to 2400mV by default */
  float     VzeroPeak;            /**< The peak voltage of Vzero in mV. Set it to 200mV by default */
  uint32_t  StepNumber;           /**< Total number of steps. Limited to 4095. */
  uint32_t  RampDuration;         /**< Ramp signal duration(total time) in ms */
  /* Receive path configuration */
  float     SampleDelay;          /**< The time delay between update DAC and start ADC */
  uint32_t  LPTIARtiaSel;         /**< Select RTIA */
	uint32_t LPTIARloadSel;				/**< Select Rload */
  float     ExternalRtiaValue;    /**< The optional external RTIA value in Ohm. Disconnect internal RTIA to use external RTIA. When using internal RTIA, this value is ignored. */
  uint32_t  AdcPgaGain;           /**< PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint8_t   ADCSinc3Osr;          /**< We use data from SINC3 filter. */
  /* Digital related */
  uint32_t  FifoThresh;           /**< FIFO Threshold value */
/* Private variables for internal usage */
  BoolFlag  RAMPInited;           /**< If the program run firstly, generated initialization sequence commands */
  fImpPol_Type  RtiaValue;        /**< Calibrated Rtia value */
  SEQInfo_Type  InitSeqInfo;
  SEQInfo_Type  ADCSeqInfo;
  BoolFlag      bFirstDACSeq;     /**< Init DAC sequence */
  SEQInfo_Type  DACSeqInfo;       /**< The first DAC update sequence info */
  uint32_t  CurrStepPos;          /**< Current position */     
  float     DACCodePerStep;       /**<  */
  float     CurrRampCode;         /**<  */   
  uint32_t  CurrVzeroCode;        
  BoolFlag  bDACCodeInc;          /**< Increase DAC code.  */
  BoolFlag  StopRequired;         /**< After FIFO is ready, stop the measurement sequence */
  enum _RampState{RAMP_STATE0 = 0, RAMP_STATE1, RAMP_STATE2, RAMP_STATE3, RAMP_STATE4, RAMP_STOP} RampState;
  BoolFlag  bRampOneDir;          /**< Ramp in a single direction, no return to start */
}AppRAMPCfg_Type;

#define APPCTRL_START          0
#define APPCTRL_STOPNOW        1
#define APPCTRL_STOPSYNC       2
#define APPCTRL_SHUTDOWN       3   /**< Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */

AD5940Err AppRAMPInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppRAMPGetCfg(void *pCfg);
AD5940Err AppRAMPISR(void *pBuff, uint32_t *pCount);
AD5940Err AppRAMPCtrl(uint32_t Command, void *pPara);

#endif
