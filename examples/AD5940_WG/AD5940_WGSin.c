/*!
 *****************************************************************************
 @file:    AD5940_WGSin.c
 @author:  Neo Xu
 @brief:   Waveform generator(sin wave) example include switch matrix.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/


#include "ad5940.h"
#include "AD5940.h"
#include <stdio.h>
#include "string.h"


#define SIN_FREQ      25000     /* 25kHz */

#define SYS_CLOCK_HZ 16000000.0 /* System clock frequency */


void AD5940_Main(void)
{
  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type HpLoopCfg;
  CLKCfg_Type clk_cfg;
  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  /* LP reference control */
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	

  HpLoopCfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
  HpLoopCfg.HsDacCfg.HsDacGain = HSDACGAIN_1;
  HpLoopCfg.HsDacCfg.HsDacUpdateRate = 7;

  HpLoopCfg.HsTiaCfg.DiodeClose = bFALSE;
  HpLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  HpLoopCfg.HsTiaCfg.HstiaCtia = 16; /* 16pF */
  HpLoopCfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  HpLoopCfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_TODE;
  HpLoopCfg.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_160K;

  HpLoopCfg.SWMatCfg.Dswitch = SWD_CE0;
  HpLoopCfg.SWMatCfg.Pswitch = SWP_CE0;
  HpLoopCfg.SWMatCfg.Nswitch = SWN_SE0LOAD;
  HpLoopCfg.SWMatCfg.Tswitch = SWT_TRTIA|SWT_SE0LOAD;

  HpLoopCfg.WgCfg.WgType = WGTYPE_SIN;
  HpLoopCfg.WgCfg.GainCalEn = bFALSE;
  HpLoopCfg.WgCfg.OffsetCalEn = bFALSE;
  HpLoopCfg.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(SIN_FREQ,SYS_CLOCK_HZ);
  HpLoopCfg.WgCfg.SinCfg.SinAmplitudeWord = 2047;
  HpLoopCfg.WgCfg.SinCfg.SinOffsetWord = 0;
  HpLoopCfg.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&HpLoopCfg);

  AD5940_AFECtrlS(AFECTRL_DACREFPWR, bTRUE);
  AD5940_AFECtrlS(AFECTRL_EXTBUFPWR|AFECTRL_INAMPPWR|AFECTRL_HSTIAPWR|AFECTRL_HSDACPWR, bTRUE);
  AD5940_AFECtrlS(AFECTRL_WG, bTRUE);
	
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  while(1);
}

