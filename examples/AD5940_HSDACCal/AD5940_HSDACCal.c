/*!
*****************************************************************************
@file:    AD5940_HSDACCal.c
@author:  $Author: nxu2 $
@brief:   HSDAC Offset Calibration Demo calibration demo.
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

/* The HSDAC has a number of different gain settings shown in table below. 
The HSDAC needs to be calibrated seperately for each gain setting. HSDAC has two powe
modes. There are seperate offset registers for both, DACOFFSET and DACOFFSETHP. The 
HSDAC needs to be calibrated for each mode.

HSDACCON[12] | HSDACCON[0] | Output Range | 
0				|			0				|		+-607mV		 |
1				|			0				|   +-75mV     |
1				|			1				|		+-15.14mV	 |
0				|			1				|		+-121.2mV  |

*/
void AD5940_Main(void){
  HSDACCal_Type hsdac_cal;
  ADCPGACal_Type adcpga_cal;
  CLKCfg_Type clk_cfg;
  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();
  
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bTRUE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  
  
  printf("ADC Offset Cal\n");
  adcpga_cal.AdcClkFreq=16000000;
  adcpga_cal.ADCPga = ADCPGA_1;
  adcpga_cal.ADCSinc2Osr = ADCSINC2OSR_1333;
  adcpga_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  adcpga_cal.PGACalType = PGACALTYPE_OFFSET;
  adcpga_cal.TimeOut10us = 1000;
  adcpga_cal.VRef1p11 = 1.11;
  adcpga_cal.VRef1p82 = 1.82;
  AD5940_ADCPGACal(&adcpga_cal);
  
  printf("\n 607mV Range Cal\n");
  hsdac_cal.ExcitBufGain = EXCITBUFGAIN_2;	/**< Select from  EXCITBUFGAIN_2, EXCITBUFGAIN_0P25 */ 
  hsdac_cal.HsDacGain = HSDACGAIN_1; 				/**< Select from  HSDACGAIN_1, HSDACGAIN_0P2 */ 
  hsdac_cal.AfePwrMode = AFEPWR_LP;
  hsdac_cal.ADCSinc2Osr = ADCSINC2OSR_1333;
  hsdac_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  AD5940_HSDACCal(&hsdac_cal);
  
  printf("\nADC GN 4 Offset Cal\n");
  adcpga_cal.ADCPga = ADCPGA_4;
  AD5940_ADCPGACal(&adcpga_cal);
  
  printf("\n 125mV Range Cal\n");
  hsdac_cal.ExcitBufGain = EXCITBUFGAIN_2;	/**< Select from  EXCITBUFGAIN_2, EXCITBUFGAIN_0P25 */ 
  hsdac_cal.HsDacGain = HSDACGAIN_0P2; 				/**< Select from  HSDACGAIN_1, HSDACGAIN_0P2 */ 
  AD5940_HSDACCal(&hsdac_cal);
  
  printf("\n 75mV Range Cal\n");
  hsdac_cal.ExcitBufGain = EXCITBUFGAIN_0P25;	/**< Select from  EXCITBUFGAIN_2, EXCITBUFGAIN_0P25 */ 
  hsdac_cal.HsDacGain = HSDACGAIN_1; 				/**< Select from  HSDACGAIN_1, HSDACGAIN_0P2 */ 
  AD5940_HSDACCal(&hsdac_cal);
  
  printf("\n 15mV Range Cal\n");
  hsdac_cal.ExcitBufGain = EXCITBUFGAIN_0P25;	/**< Select from  EXCITBUFGAIN_2, EXCITBUFGAIN_0P25 */ 
  hsdac_cal.HsDacGain = HSDACGAIN_0P2; 				/**< Select from  HSDACGAIN_1, HSDACGAIN_0P2 */ 
  AD5940_HSDACCal(&hsdac_cal);
  
  printf("HSDAC Cal Complete!\n");
}

