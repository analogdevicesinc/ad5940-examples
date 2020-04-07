/*!
 *****************************************************************************
 @file:    Amperometric.h
 @author:  $Author: mlambe $
 @brief:   Amperometric measurement header file.
 @version: $Revision: 766 $
 @date:    $Date: 2018-03-21 14:09:35 +0100 (Wed, 21 Mar 2018) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
#ifndef _AMPEROMETRIC_H_
#define _AMPEROMETRIC_H_
#include "AD5940.H"
#include "stdio.h"
#include "string.h"
#include "math.h"

#define DAC12BITVOLT_1LSB   (2200.0f/4095)  //mV
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64)  //mV
/* 
  Note: this example will use SEQID_0 as measurement sequence, and use SEQID_1 as init sequence. 
  SEQID_3 is used for calibration.
*/

typedef struct
{
/* Common configurations for all kinds of Application. */
  BoolFlag bParaChanged;        /* Indicate to generate sequence again. It's auto cleared by AppAMPInit */
  uint32_t SeqStartAddr;        /* Initialaztion sequence start address in SRAM of AD5940  */
  uint32_t MaxSeqLen;           /* Limit the maximum sequence.   */
  uint32_t SeqStartAddrCal;     /* Measurement sequence start address in SRAM of AD5940 */
  uint32_t MaxSeqLenCal;
  
/* Application related parameters */ 
  BoolFlag ReDoRtiaCal;         /* Set this flag to bTRUE when there is need to do calibration. */
  float SysClkFreq;             /* The real frequency of system clock */
  float WuptClkFreq;            /* The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float AdcClkFreq;             /* The real frequency of ADC clock */
  uint32_t FifoThresh;           /* FIFO threshold. Should be N*4 */   
  float AmpODR;                 /* in Hz. ODR decides the period of WakeupTimer who will trigger sequencer periodically.*/
  int32_t NumOfData;            /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value here. Otherwise, set it to '-1' which means never stop. */
  float RcalVal;                /* Rcal value in Ohm */
  float ADCRefVolt;               /* Measured 1.82 V reference*/
  uint32_t PwrMod;              /* Control Chip power mode(LP/HP) */
  uint32_t ADCPgaGain;          /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint8_t ADCSinc3Osr;          /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
  uint8_t ADCSinc2Osr;          /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
  uint32_t DataFifoSrc;         /* DataFIFO source. FIFOSRC_SINC3, FIFOSRC_DFT, FIFOSRC_SINC2NOTCH, FIFOSRC_VAR, FIFOSRC_MEAN*/
  uint32_t LptiaRtiaSel;        /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
  uint32_t LpTiaRf;             /* Rfilter select */
  uint32_t LpTiaRl;             /* SE0 Rload select */
  fImpPol_Type RtiaCalValue;           /* Calibrated Rtia value */
  float Vzero;                  /* Voltage on SE0 pin and Vzero, optimumly 1100mV*/
  float SensorBias;             /* Sensor bias voltage = VRE0 - VSE0 */
  BoolFlag ExtRtia;             /* Use internal or external Rtia */
  float ExtRtiaVal;							/* External Rtia value if using one */
  BoolFlag AMPInited;           /* If the program run firstly, generated sequence commands */
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type MeasureSeqInfo;
  BoolFlag StopRequired;          /* After FIFO is ready, stop the measurement sequence */
  uint32_t FifoDataCount;         /* Count how many times impedance have been measured */
/* End */
}AppAMPCfg_Type;

/**
 * int32_t type Impedance result in Cartesian coordinate 
*/
typedef struct
{
  float Current;
  float Voltage;
}fAmpRes_Type;



#define AMPCTRL_START          0
#define AMPCTRL_STOPNOW        1
#define AMPCTRL_STOPSYNC       2
#define AMPCTRL_SHUTDOWN       4   /* Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */

AD5940Err AppAMPGetCfg(void *pCfg);
AD5940Err AppAMPInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppAMPISR(void *pBuff, uint32_t *pCount);
AD5940Err AppAMPCtrl(int32_t AmpCtrl, void *pPara);
float AppAMPCalcVoltage(uint32_t ADCcode);
float AppAMPCalcCurrent(uint32_t ADCcode);

#endif
