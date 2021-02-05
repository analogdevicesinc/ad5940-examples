/*!
 @file:    BIOZ-2Wire.h
 @author:  Neo Xu
 @brief:   4-wire BIOZ measurement header file.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

#ifndef _BODYCOMPOSITION_H_
#define _BODYCOMPOSITION_H_
#include "ad5940.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#define MAXSWEEP_POINTS   100           /* Need to know how much buffer is needed to save RTIA calibration result */

/* 
  Note: this example will use SEQID_0 as measurement sequence, and use SEQID_1 as init sequence. 
  SEQID_3 is used for calibration.
*/

typedef struct
{
/* Common configurations for all kinds of Application. */
  BoolFlag bParaChanged;        /* Indicate to generate sequence again. It's auto cleared by AppBIOZInit */
  uint32_t SeqStartAddr;        /* Initialaztion sequence start address in SRAM of AD5940  */
  uint32_t MaxSeqLen;           /* Limit the maximum sequence.   */
  uint32_t SeqStartAddrCal;     /* Measurement sequence start address in SRAM of AD5940 */
  uint32_t MaxSeqLenCal;
/* Application related parameters */ 
  //BoolFlag bBioElecBoard;     /* The code is same for BioElec board and AD5941Sens1 board. No changes are needed */
  BoolFlag ReDoRtiaCal;         /* Set this flag to bTRUE when there is need to do calibration. */
  float SysClkFreq;             /* The real frequency of system clock */
  float WuptClkFreq;            /* The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float AdcClkFreq;             /* The real frequency of ADC clock */
  uint32_t FifoThresh;           /* FIFO threshold. Should be N*2 */   
  float BIOZODR;                 /* in Hz. ODR decides the period of WakeupTimer who will trigger sequencer periodically. DFT number and sample frequency decides the maxim ODR. */
  int32_t NumOfData;            /* By default it's '-1'. If you want the engine stops after get NumofData, then set the value here. Otherwise, set it to '-1' which means never stop. */
  float SinFreq;                /* Frequency of excitation signal */
  float RcalVal;                /* Rcal value in Ohm */
  uint32_t PwrMod;              /* Control Chip power mode(LP/HP) */
  float DacVoltPP;              /* Final excitation voltage is DacVoltPP*DAC_PGA*EXCIT_GAIN, DAC_PGA= 1 or 0.2, EXCIT_GAIN=2 or 0.25. DAC output voltage in mV peak to peak. Maximum value is 800mVpp. Peak to peak voltage  */
  uint32_t ExcitBufGain;        /* Select from  EXCITBUFGAIN_2, EXCITBUFGAIN_0P25 */     
  uint32_t HsDacGain;           /* Select from  HSDACGAIN_1, HSDACGAIN_0P2 */  
  uint32_t HsDacUpdateRate;     /* DAC update rate is SystemCLoock/Divider. The available value is 7 to 255. Set to 7 for better performance */
  uint32_t ADCPgaGain;          /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint8_t ADCSinc3Osr;          /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
  uint8_t ADCSinc2Osr;          /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
  uint32_t HstiaRtiaSel;        /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
  uint32_t CtiaSel;             /* Select CTIA in pF unit from 0 to 31pF */

  uint32_t DftNum;              /* DFT number */
  uint32_t DftSrc;              /* DFT Source */
  BoolFlag HanWinEn;            /* Enable Hanning window */
	
	/* Switch Configuration */
  uint32_t DswitchSel;
  uint32_t PswitchSel;
  uint32_t NswitchSel;
  uint32_t TswitchSel;

  /* Sweep Function Control */
  SoftSweepCfg_Type SweepCfg;
/* Private variables for internal usage */
  float SweepCurrFreq;
  float SweepNextFreq;
  fImpCar_Type RtiaCurrValue;                   /* Calibrated Rtia value at current frequency */
  fImpCar_Type RtiaCalTable[MAXSWEEP_POINTS];   /* Calibrated Rtia Value table */
  float FreqofData;                             /* The frequency of latest data sampled */
  BoolFlag BIOZInited;                          /* If the program run firstly, generated sequence commands */
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type MeasureSeqInfo;
  BoolFlag StopRequired;          /* After FIFO is ready, stop the measurement sequence */
  uint32_t FifoDataCount;         /* Count how many times impedance have been measured */
/* End */
}AppBIOZCfg_Type;

#define BIOZCTRL_START          0
#define BIOZCTRL_STOPNOW        1
#define BIOZCTRL_STOPSYNC       2
#define BIOZCTRL_GETFREQ        3   /* Get Current frequency of returned data from ISR */
#define BIOZCTRL_SHUTDOWN       4   /* Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */

AD5940Err AppBIOZGetCfg(void *pCfg);
AD5940Err AppBIOZInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppBIOZISR(void *pBuff, uint32_t *pCount);
AD5940Err AppBIOZCtrl(int32_t BcmCtrl, void *pPara);
AD5940Err AppBIOZCheckFreq(float freq);

#endif
