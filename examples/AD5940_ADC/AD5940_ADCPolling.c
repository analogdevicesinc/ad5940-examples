/*!
 *****************************************************************************
 @file:    AD5940_ADCPolling.c
 @author:  $Author: nxu2 $
 @brief:   ADC Polling mode example
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
    @defgroup ADC_Polling_Example
    @{
  */

#include "ad5940.h"
#include <stdio.h>

#define ADCPGA_GAIN_SEL   ADCPGA_1P5
static void AD5940_PGA_Calibration(void){
  AD5940Err err;
  ADCPGACal_Type pgacal;
  pgacal.AdcClkFreq = 16e6;
  pgacal.ADCSinc2Osr = ADCSINC2OSR_178;
  pgacal.ADCSinc3Osr = ADCSINC3OSR_4;
  pgacal.SysClkFreq = 16e6;
  pgacal.TimeOut10us = 1000;
  pgacal.VRef1p11 = 1.11f;
  pgacal.VRef1p82 = 1.82f;
  pgacal.PGACalType = PGACALTYPE_OFFSETGAIN;
  pgacal.ADCPga = ADCPGA_GAIN_SEL;
  err = AD5940_ADCPGACal(&pgacal);
  if(err != AD5940ERR_OK){
    printf("AD5940 PGA calibration failed.");
  }
}

void AD5940_Main(void)
{
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  
  /* Use hardware reset */
  AD5940_HWReset();

  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize();
  
  AD5940_PGA_Calibration();
  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  
  /* Initialize ADC basic function */
  AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE); //We are going to measure DAC 1.82V reference.
  adc_base.ADCMuxP = ADCMUXP_VREF1P8DAC;
  adc_base.ADCMuxN = ADCMUXN_VSET1P1;
  adc_base.ADCPga = ADCPGA_GAIN_SEL;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
  adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */   
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */ 
  AD5940_ADCFilterCfgS(&adc_filter);
  
  //AD5940_ADCMuxCfgS(ADCMUXP_AIN2, ADCMUXN_VSET1P1);   /* Optionally, you can change ADC MUX with this function */

  /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 

  //AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  //AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_ADCPowerCtrlS(bTRUE);
  AD5940_ADCConvtCtrlS(bTRUE);
  
  while(1)
  {
    uint32_t rd;
    if(AD5940_INTCTestFlag(AFEINTC_1,AFEINTSRC_SINC2RDY))  
    {
      static uint32_t count;
      AD5940_INTCClrFlag(AFEINTSRC_SINC2RDY);
      rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
      count ++;
      /* ADC Sample rate is 800kSPS. SINC3 OSR is 4, SINC2 OSR is 1333. So the final output data rate is 800kSPS/4/1333 = 150.0375Hz */
      if(count == 150) /* Print data @1Hz */
      {
        count = 0;
        float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
        printf("ADC Code:%d, diff-volt: %.4f, volt:%.4f\n",rd, diff_volt, diff_volt+1.11);
      }
    }
  }
}

/**
 * @}
 * @}
 * */
