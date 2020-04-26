/*!
 *****************************************************************************
 @file:    AD5940_ADCMeanFIFO.c
 @author:  $Author: nxu2 $
 @brief:   Use FIFO to read statistic block mean result.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
/** @addtogroup AD5940_Standard_Examples
  * @{
    @defgroup ADC_MEAN_FIFO_Example
    @{
  */

#include "ad5940.h"
#include <stdio.h>

uint32_t ADCBuff[256];
void AD5940_Main(void)
{
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  StatCfg_Type stat_cfg;
  FIFOCfg_Type fifo_cfg;
  
  /* Use hardware reset */
  AD5940_HWReset();
  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize();
  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  /* Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_AVDD_2;
  adc_base.ADCMuxN = ADCMUXN_VSET1P1;
  adc_base.ADCPga = ADCPGA_1;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH-->StatisticBlock */
  adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
  adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */ 
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */ 
  AD5940_ADCFilterCfgS(&adc_filter);
  
  /**
   * Statistic block receive data from SINC2+Notch block. Note the diagram in datasheet page 51 PrM. 
   * The SINC3 can be bypassed optionally. SINC2 cannot be bypassed.
   * */
  stat_cfg.StatDev = STATDEV_1;               /* Not used. */
  stat_cfg.StatEnable = bTRUE;
  stat_cfg.StatSample = STATSAMPLE_128;       /* Sample 128 points and calculate mean. */
  AD5940_StatisticCfgS(&stat_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;
  fifo_cfg.FIFOSrc = FIFOSRC_MEAN;
  fifo_cfg.FIFOThresh = 2;
  AD5940_FIFOCfg(&fifo_cfg);

  /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Enable FIFO threshold interrupt. */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_ClrMCUIntFlag(); /* Clear the MCU interrupt flag which will be set in ISR. */

  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);  
  while(1)
  {
    uint32_t FifoCnt;
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
      {
        FifoCnt = AD5940_FIFOGetCnt();
        AD5940_FIFORd((uint32_t *)ADCBuff, FifoCnt);
        AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
        printf("Get %d data, ADC Code[0]:%d\n",FifoCnt, ADCBuff[0]&0xffff);
        /*!!!!!NOTE!!!!!*/
        /* The mean result already removed 32768. So to calculate the voltage, assume mean result is n, use below equation.
          Voltage = n/32768*Vref
         */
      }
    }
  }
}

/**
 * @}
 * @}
 * */
