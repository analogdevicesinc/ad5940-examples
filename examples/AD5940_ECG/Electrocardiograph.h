/*!
 *****************************************************************************
 @file:    Electrocardiograph.h
 @author:  Neo Xu
 @brief:   ECG measurement.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#ifndef _ELETROCARDIOAGRAPH_H_
#define _ELETROCARDIOAGRAPH_H_
#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"

typedef struct
{
/* Common configurations for all kinds of Application. */
  BoolFlag bParaChanged;        /* Indicate to generate sequence again. It's auto cleared by AppBIAInit */
  BoolFlag bBioElecBoard;       /* Indicate if the board is Bioelec board. 0: AD5941Sens1 board, 1: AD5940-BioElec */
  uint32_t SeqStartAddr;        /* Initialaztion sequence start address in SRAM of AD5940  */
  uint32_t MaxSeqLen;           /* Limit the maximum sequence.   */
  uint32_t SeqStartAddrCal;     /* Calibration sequence start address in SRAM of AD5940 */
  uint32_t MaxSeqLenCal;
/* Application related parameters */ 
  float ECGODR;                 /* Must be less than 1500Hz. Sample frequency in Hz, this value is used to set Sleep Wakeup Timer period */
  int32_t NumOfData;            /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value here. Otherwise, set it to '-1' which means never stop. */
  uint32_t FifoThresh;          /* FIFO threshold. Should be N*4 */

  float LfoscClkFreq;           /* The clock frequency of internal LFOSC in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float SysClkFreq;             /* The real frequency of system clock */
  float AdcClkFreq;             /* The real frequency of ADC clock */
  uint32_t PwrMod;              /* Control Chip power mode(LP/HP) */

  uint32_t AdcPgaGain;          /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint32_t ADCSinc3Osr;
  uint32_t ADCSinc2Osr;  
/* Private variables for internal usage */
  BoolFlag ECGInited;           /* If the program run firstly, generated sequence commands */
  BoolFlag StopRequired;        /* After FIFO is ready, stop the measurement sequence */
  uint32_t FifoDataCount;       /* How many data we have got from start. */
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type MeasureSeqInfo;
}AppECGCfg_Type;

#define APPCTRL_START          0
#define APPCTRL_STOPNOW        1
#define APPCTRL_STOPSYNC       2
//#define APPCTRL_GETFREQ        3 
#define APPCTRL_SHUTDOWN       4   /* Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */


AD5940Err AppECGGetCfg(void *pCfg);
AD5940Err AppECGInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppECGISR(void *pBuff, uint32_t *pCount);
AD5940Err AppECGCtrl(int32_t Command, void *pPara);

#endif
