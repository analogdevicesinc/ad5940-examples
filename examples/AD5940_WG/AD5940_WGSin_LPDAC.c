/*!
 *****************************************************************************
 @file:    AD5940_WGSin_LPDAC.c
 @author:  $Author: nxu2 $
 @brief:   Waveform generator(sin wave) example using LPDAC.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
#include "ad5940.h"
#include <stdio.h>
#include "string.h"
/**
 * This example is to generate sin wave on pin CE0 using waveform generator and LPDAC.
 * Signal generator simpley generates digital codes. The code can route to both
 * HSDAC and LPDAC. So, we can generate sin wave using both DAC.
 *
 * @note: LPDAC has limited bandwidth, do not use it to generate signal above 300Hz,
 * otherwise, you will see significant performance drop.
 * The DAC update rate parameter is decided by register HSDACCON.Rate. This also true
 * when using LPDAC as data sink.
*/
#define SIN_AMPLITUDE 1100.0    /**< Signal amplitude in mV.*/
#define SIN_FREQ      100.0     /**< 100Hz. Max is 300Hz */
#define WG_CLOCK_HZ   32e3f     /**< Waveform generator clock frequency. Equal to system clock. */

void AD5940_Main(void)
{
  AFERefCfg_Type aferef_cfg;
  CLKCfg_Type clk_cfg;
  LPDACCfg_Type lpdac_cfg;
  WGCfg_Type WgCfg;
  LPAmpCfg_Type lpamp_cfg;
  HSDACCfg_Type HsDacCfg;

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

 /* Configure LPDAC*/
  lpdac_cfg.LpdacSel = LPDAC0;
  lpdac_cfg.DataRst = bFALSE;
  lpdac_cfg.LpDacSW = LPDACSW_VBIAS2LPPA/*|LPDACSW_VBIAS2PIN*/|LPDACSW_VZERO2LPTIA/*|LPDACSW_VZERO2PIN*/;
  lpdac_cfg.LpDacRef = LPDACREF_2P5;           /* Use internal 2.5V reference */
  lpdac_cfg.LpDacSrc = LPDACSRC_WG;            /* Use data from waveform generator */
  lpdac_cfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lpdac_cfg.LpDacVzeroMux = LPDACVZERO_6BIT;   /* Use 6bit LPDAC for Vzero */
  lpdac_cfg.PowerEn = bTRUE;                   /* Enable LPDAC */
  lpdac_cfg.DacData12Bit = 0;                  /* Don't care, 12bit DAC data is from WG */
  lpdac_cfg.DacData6Bit = 32;
  AD5940_LPDACCfgS(&lpdac_cfg);

  /* Configure low power amplifiers */
  lpamp_cfg.LpAmpSel = LPAMP0;
  lpamp_cfg.LpAmpPwrMod = LPAMPPWR_NORM;       /* Use normal power mode is enough */
  lpamp_cfg.LpPaPwrEn = bTRUE;                 /* Enable Potential amplifier */
  lpamp_cfg.LpTiaPwrEn = bFALSE;               /* TIA is not used in this example */
  lpamp_cfg.LpTiaRf = LPTIARF_1M;
  lpamp_cfg.LpTiaRload = LPTIARLOAD_100R;      /* don't care */
  lpamp_cfg.LpTiaRtia = LPTIARTIA_1K;          /* don't care */
  lpamp_cfg.LpTiaSW = 0;                       /* don't care */
  AD5940_LPAMPCfgS(&lpamp_cfg);

  HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
  HsDacCfg.HsDacGain = HSDACGAIN_1;
  HsDacCfg.HsDacUpdateRate = 7;                /* DAC update rate equals to WG_CLK/HsDacUpdateRate */
  AD5940_HSDacCfgS(&HsDacCfg);
  /* Configure Waveform Generator */
  WgCfg.WgType = WGTYPE_SIN;
  WgCfg.GainCalEn = bFALSE;
  WgCfg.OffsetCalEn = bFALSE;
  WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(SIN_FREQ, WG_CLOCK_HZ);
  WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(SIN_AMPLITUDE/1100.0f*2047); 
  WgCfg.SinCfg.SinOffsetWord = 0;
  WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_WGCfgS(&WgCfg);

  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_AUTOSET);
  AD5940_AFECtrlS(AFECTRL_WG, bTRUE);

  /* Change to 32kHz clock. LPDAC needs 32kHz clock for waveform generator */
  AD5940_LPModeEnS(bTRUE);  /* Enter LP control mode. The registers are summarized to LPMODECON, so we can control some blocks conveniently */
  AD5940_LPModeClkS(LPMODECLK_LFOSC);     /* Trigger switching system clock to 32kHz */
  AD5940_LPModeCtrlS(LPMODECTRL_NONE);    /* Disable all */ 
  AD5940_LPModeCtrlS(LPMODECTRL_GLBBIASZ|LPMODECTRL_GLBBIASP|LPMODECTRL_HPREFPWR|LPMODECTRL_BUFHP1P8V|LPMODECTRL_BUFHP1P1V|LPMODECTRL_HFOSCEN); 
  while(1);
}

