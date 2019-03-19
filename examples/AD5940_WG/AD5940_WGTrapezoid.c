/*!
 *****************************************************************************
 @file:    AD5940_WGTrapezoid.c
 @author:  $Author: nxu2 $
 @brief:   Waveform generator example include switch matrix.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
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


#define SIN_FREQ      200000.0   /* 25kHz */

#define SYS_CLOCK_HZ 16000000.0 /* System clock frequency */


void AD5940_Main(void)
{
  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type HsloopCfg;
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

  HsloopCfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
  HsloopCfg.HsDacCfg.HsDacGain = HSDACGAIN_1;
  HsloopCfg.HsDacCfg.HsDacUpdateRate = 7;

  HsloopCfg.HsTiaCfg.DiodeClose = bFALSE;
  HsloopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  HsloopCfg.HsTiaCfg.HstiaCtia = 16; /* 16pF */
  HsloopCfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  HsloopCfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_TODE;
  HsloopCfg.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_160K;

  HsloopCfg.SWMatCfg.Dswitch = SWD_CE0;
  HsloopCfg.SWMatCfg.Pswitch = SWP_CE0;
  HsloopCfg.SWMatCfg.Nswitch = SWN_SE0LOAD;
  HsloopCfg.SWMatCfg.Tswitch = SWT_TRTIA|SWT_SE0LOAD;

  HsloopCfg.WgCfg.WgType = WGTYPE_TRAPZ;
  HsloopCfg.WgCfg.GainCalEn = bFALSE;
  HsloopCfg.WgCfg.OffsetCalEn = bFALSE;
  HsloopCfg.WgCfg.TrapzCfg.WGTrapzDCLevel1 = 0x200;
  HsloopCfg.WgCfg.TrapzCfg.WGTrapzDCLevel2 = 0xa00;
  HsloopCfg.WgCfg.TrapzCfg.WGTrapzDelay1 = 50;
  HsloopCfg.WgCfg.TrapzCfg.WGTrapzDelay2 = 100;
  HsloopCfg.WgCfg.TrapzCfg.WGTrapzSlope1 = 200;
  HsloopCfg.WgCfg.TrapzCfg.WGTrapzSlope2 = 300;
  AD5940_HSLoopCfgS(&HsloopCfg);

  AD5940_AFECtrlS(AFECTRL_DACREFPWR, bTRUE);
  AD5940_AFECtrlS(AFECTRL_EXTBUFPWR|AFECTRL_INAMPPWR|AFECTRL_HSTIAPWR|AFECTRL_HSDACPWR, bTRUE);
  AD5940_AFECtrlS(AFECTRL_WG, bTRUE);
	
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  while(1);
}

