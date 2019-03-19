/*!
 *****************************************************************************
 @file:    AD5940_WGSin.c
 @author:  $Author: nxu2 $
 @brief:   Waveform generator(sin wave) example include switch matrix.
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


#define SIN_FREQ      100.0   /* 100Hz. Max is 300Hz */

#define SYS_CLOCK_HZ 16000000.0 /* System clock frequency */


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
  lpdac_cfg.DacData6Bit = 32;                  /* Set it to mid-scale of LPDAC. Vzero is the bias voltage of LPTIA amplifire */ 
  AD5940_LPDACCfgS(&lpdac_cfg);
	
	/* Configure low power amplifiers */
	lpamp_cfg.LpAmpSel = LPAMP0;
  lpamp_cfg.LpAmpPwrMod = LPAMPPWR_NORM;       /* Use normal power mode is enough */
  lpamp_cfg.LpPaPwrEn = bTRUE;                 /* Enable Potential amplifier */
  lpamp_cfg.LpTiaPwrEn = bTRUE;                /* Enable TIA amplifier */
  lpamp_cfg.LpTiaRf = LPTIARF_1M;                  /* Rf resistor controls cut off frequency. */
  lpamp_cfg.LpTiaRload = LPTIARLOAD_100R;      /** @note Use 100Ohm Rload. */
  lpamp_cfg.LpTiaRtia = LPTIARTIA_OPEN;  /* If autoscaling is enabled, use seleted value. */
  lpamp_cfg.LpTiaSW = LPTIASW(2)|LPTIASW(8)|LPTIASW(7)|LPTIASW(1);            /* Swtich settings for voltage measurement */
  AD5940_LPAMPCfgS(&lpamp_cfg);
	
	
  HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
  HsDacCfg.HsDacGain = HSDACGAIN_1;
  HsDacCfg.HsDacUpdateRate = 7;
	AD5940_HSDacCfgS(&HsDacCfg);
	/* Configure Waveform Generator */
  WgCfg.WgType = WGTYPE_SIN;
  WgCfg.GainCalEn = bFALSE;
  WgCfg.OffsetCalEn = bFALSE;
  WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(SIN_FREQ, 32768);
  WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)((1100.0f/2)/1100.0f*2047); 
  WgCfg.SinCfg.SinOffsetWord = 0;
  WgCfg.SinCfg.SinPhaseWord = 0;
	AD5940_WGCfgS(&WgCfg);
	
	AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_AUTOSET);
	AD5940_AFECtrlS(AFECTRL_WG, bTRUE);
	
	/* Change to 32kHz clock. LPDAC needs 32kHz clock for waveform generator */
	AD5940_LPModeEnS(bTRUE);  /* Enter LP control mode. The registers are summarized to LPMODECON, so we can control some blocks convenniently */
  AD5940_LPModeClkS(LPMODECLK_LFOSC); /* Trigger switching system clock to 32kHz */
  AD5940_LPModeCtrlS(LPMODECTRL_NONE);    /* Disable all */ 
	AD5940_LPModeCtrlS(LPMODECTRL_GLBBIASZ|LPMODECTRL_GLBBIASP|LPMODECTRL_HPREFPWR|LPMODECTRL_BUFHP1P8V|LPMODECTRL_BUFHP1P1V|LPMODECTRL_HFOSCEN); 
  while(1);
}

