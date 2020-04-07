/*!
 *****************************************************************************
 @file:    AD5940_LPDAC.c
 @author:  Neo Xu
 @brief:   Low power DAC example.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.
 
*****************************************************************************/

/**
 * Note: This example will use LP loop to output voltage on RE0 pin.
 * LPDAC reference: internal 2.5V
 * LP PA(potentialstat amplifier) is used to buffer voltage from Vbias which connects to 12bit LPDAC output
**/

#include "ad5940.h"
#include "AD5940.h"
#include <stdio.h>
#include "string.h"

void AD5940_Main(void)
{
  AFERefCfg_Type ref_cfg;
  LPLoopCfg_Type lp_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();

  /* Initialize everything to zero(false/OFF/PowerDown), only turn on what we need */
  AD5940_StructInit(&ref_cfg, sizeof(ref_cfg));
  ref_cfg.LpBandgapEn = bTRUE;                        /* Enable low power bandgap */
  ref_cfg.LpRefBufEn = bTRUE;                         /* Enable the low power reference buffer - 2.5V output */
  AD5940_REFCfgS(&ref_cfg);                           /* Call reference configuration function */

  AD5940_StructInit(&lp_cfg, sizeof(lp_cfg));         /* Reset everything to zero(OFF) */
  /* Configure what we need below */
  lp_cfg.LpDacCfg.LpdacSel = LPDAC0;                  /* Select LPDAC0. Note LPDAC1 is available on ADuCM355 */
  lp_cfg.LpDacCfg.DacData12Bit = 0x800;               /* Output midscale voltage (0.2V + 2.4V)/2 = 1.3V */
  lp_cfg.LpDacCfg.DacData6Bit = 0;                    /* 6Bit DAC data */
  lp_cfg.LpDacCfg.DataRst =bFALSE;                    /* Do not keep DATA registers at reset status */
  lp_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN;
  lp_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;            /* Select internal 2.5V reference */
  lp_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;            /* The LPDAC data comes from MMR not WG in this case */
  lp_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;   /* Connect Vbias signal to 12Bit LPDAC output */
  lp_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;    /* Connect Vzero signal to 6bit LPDAC output */
  lp_cfg.LpDacCfg.PowerEn = bTRUE;                    /* Power up LPDAC */
  lp_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
  lp_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;           /* Set low power amplifiers to normal power mode */
  lp_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;                  /* Enable LP PA(potentialstat amplifier) power */
  lp_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;                /* Leave LPTIA power off */
  lp_cfg.LpAmpCfg.LpTiaSW = LPTIASW(12)|LPTIASW(13)|LPTIASW(2)|LPTIASW(10)\
          |LPTIASW(5)|LPTIASW(9); /* Close these switches to make sure LP PA amplifier is closed loop */
  lp_cfg.LpAmpCfg.LpTiaRf = LPTIARF_SHORT;
  lp_cfg.LpAmpCfg.LpTiaRtia = LPTIARTIA_200R;
  lp_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_100R;

  AD5940_LPLoopCfgS(&lp_cfg); 
  while(1);
}

