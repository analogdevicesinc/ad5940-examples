/*!
 *****************************************************************************
 @file:    AD5940_DFTPolling.c
 @author:  Neo Xu
 @brief:   DFT Polling mode example.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/
#include "ad5940.h"
#include "stdio.h"
#include "math.h"

/**
 * Note: In order to use on-chip DFT engine, WG must be set to SIN wave generator and enable it.
*/

void AD5940_Main(void)
{
  DSPCfg_Type dsp_cfg;
  WGCfg_Type wg_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize();
  
  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  
  AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
  /* Initialize ADC basic function */
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_VCE0;
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_VSET1P1;
  dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_1;
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = ADCSINC3OSR_4;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSINC2OSR_1333;
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;                /* We use SINC3 filter. */
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
  dsp_cfg.DftCfg.DftNum = DFTNUM_16384;
  dsp_cfg.DftCfg.DftSrc = DFTSRC_SINC3;
  AD5940_DSPCfgS(&dsp_cfg);

  AD5940_StructInit(&wg_cfg, sizeof(wg_cfg));
  wg_cfg.WgType = WGTYPE_SIN;
  wg_cfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(1000.0, 16000000.0);  /* 10kHz */
  AD5940_WGCfgS(&wg_cfg);

  /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH|AFECTRL_WG, bTRUE);
  AD5940_AFECtrlS(AFECTRL_DFT, bTRUE);
  AD5940_ADCConvtCtrlS(bTRUE);
  
  while(1)
  {
    int32_t real, image;
    if(AD5940_INTCTestFlag(AFEINTC_1,AFEINTSRC_DFTRDY))
    {
      AD5940_INTCClrFlag(AFEINTSRC_DFTRDY);
      real = AD5940_ReadAfeResult(AFERESULT_DFTREAL);
      if(real&(1<<17))
        real |= 0xfffc0000; /* Data is 18bit in two's complement, bit17 is the sign bit */
      printf("DFT: %d,", real);      
      image = AD5940_ReadAfeResult(AFERESULT_DFTIMAGE);
      if(image&(1<<17))
        image |= 0xfffc0000; /* Data is 18bit in two's complement, bit17 is the sign bit */
      printf("%d,", image);      
      printf("Mag:%f\n", sqrt((float)real*real + (float)image*image));
    }
  }
}

