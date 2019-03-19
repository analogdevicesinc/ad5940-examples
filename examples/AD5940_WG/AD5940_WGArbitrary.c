/*!
 *****************************************************************************
 @file:    AD5940_WGArbitrary.c
 @author:  $Author: nxu2 $
 @brief:   Arbitrary Waveform Genertor using sequencer.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/


#include "ad5940.h"
#include "stdio.h"
#include "string.h"

#define SIN_FREQ      200000.0f      /* 20kHz */
#define SIN_AMPLITUDE 4095           /* unit is DAC peak to peak code. Maximum is 4095 */
#define SAMPLE_RATE   2000000.0f     /* 200kHz */
#define SAMPLE_POINTS 200            /* 100 Points */
#define SYSCLK_FREQ   16000000.0f    /* System clock frequency is 16MHz for this example. */

#define APPBUFF_SIZE  1024
static uint32_t AppBuff[APPBUFF_SIZE];  /* We use 2kB SRAM for sequencer in this example, maximum sequence length is 512 */

/**
 * Write the method to generate Arbitrary Waveform. You can use expression or looup table.
 * In this example, we use sin wave expression.
**/
static uint32_t GetNextDacPoint(float Freq, float SampleRate, uint32_t Index)
{
  static uint32_t index;
  float fRes;
  int32_t iRes;
  uint32_t Amplitude = SIN_AMPLITUDE;
  if(Amplitude > 4095)  Amplitude = 4095; 
  fRes = Amplitude/2.0f*sin(2*MATH_PI*Freq*Index/SampleRate);
  iRes = (int32_t)(fRes) + 0x800;
  printf("index:%d, fRes:%f, iRes:%d\n", index++, fRes, iRes);
  if(iRes < 0)  iRes = 0;
  if(iRes > 0xfff) iRes = 0xfff;
  return iRes;
}

static AD5940Err BuildSequence(void)
{
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seqcfg;
  SEQInfo_Type seqinfo;
  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type HpLoopCfg;
  AD5940_SEQGenInit(AppBuff, APPBUFF_SIZE);
  AD5940_SEQGenCtrl(bTRUE);   /* Start sequencer generator here */
  
  /* sequence starts here */
  AD5940_SEQGpioCtrlS(AGPIO_Pin1);        /* Pull high GPIO to indicate sequencer is running */
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);
  /* Step1. Init reference system */
  AD5940_StructInit(&aferef_cfg, sizeof(aferef_cfg));  /* Disable everything and only enable below functions */
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  /* LP reference control */
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  AD5940_REFCfgS(&aferef_cfg);	

  /* Step2: Configure HSLoop: HSDAC, HSTIA, SWMatrix and WG(MMR type) */
  AD5940_StructInit(&HpLoopCfg, sizeof(HpLoopCfg));
  HpLoopCfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
  HpLoopCfg.HsDacCfg.HsDacGain = HSDACGAIN_1;
  HpLoopCfg.HsDacCfg.HsDacUpdateRate = 7;

  HpLoopCfg.HsTiaCfg.DiodeClose = bFALSE;
  HpLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  HpLoopCfg.HsTiaCfg.HstiaCtia = 16; /* 16pF */
  HpLoopCfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  HpLoopCfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_TODE;    /* Connect HSTIA output to DE0 pin */
  HpLoopCfg.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_200;

  HpLoopCfg.SWMatCfg.Dswitch = SWD_CE0;
  HpLoopCfg.SWMatCfg.Pswitch = SWP_CE0;
  HpLoopCfg.SWMatCfg.Nswitch = SWN_SE0LOAD;
  HpLoopCfg.SWMatCfg.Tswitch = SWT_TRTIA|SWT_SE0LOAD;

  HpLoopCfg.WgCfg.WgType = WGTYPE_MMR;    /* We use sequencer to update DAC data point by point. */
  HpLoopCfg.WgCfg.GainCalEn = bFALSE;
  HpLoopCfg.WgCfg.OffsetCalEn = bFALSE;
  HpLoopCfg.WgCfg.WgCode = 0x800;         /* Init to mid-scale */
  AD5940_HSLoopCfgS(&HpLoopCfg);

  AD5940_AFECtrlS(AFECTRL_DACREFPWR, bTRUE);
  AD5940_AFECtrlS(AFECTRL_EXTBUFPWR|AFECTRL_INAMPPWR|AFECTRL_HSTIAPWR|AFECTRL_HSDACPWR, bTRUE);
  AD5940_AFECtrlS(AFECTRL_WG, bTRUE);
  
  AD5940_StructInit(&seqcfg, sizeof(seqcfg)); /* Disable everything */
  seqcfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer, others for data FIFO */
  seqcfg.SeqEnable = bTRUE;   /* Keep sequencer enabled */
  seqcfg.SeqWrTimer = ((uint32_t)(SYSCLK_FREQ/SAMPLE_RATE + 0.5f)-1);      /* Run next command after write timer and timer is set to update rate */
  if(seqcfg.SeqWrTimer > 255)
    return AD5940ERR_PARA;
  AD5940_SEQCfg(&seqcfg);
  for(uint32_t i=0; i<SAMPLE_POINTS; i++)
  {
    uint32_t daccode;
    daccode = GetNextDacPoint(SIN_FREQ, SAMPLE_RATE, i);
    AD5940_WGDACCodeS(daccode);
  }
  AD5940_SEQGpioCtrlS(0);       /* Pull low GPIO to indicate sequencer stopped */
  seqcfg.SeqEnable = bFALSE;  /* Stop sequencer */
  seqcfg.SeqWrTimer = 0;      /* Reset write timer to 0 */
  AD5940_SEQCfg(&seqcfg);
  /* End of sequence */
  error = AD5940_SEQGenFetchSeq(&seqinfo.pSeqCmd, &seqinfo.SeqLen);
  AD5940_SEQGenCtrl(bFALSE);    /* Stop sequencer generator here */
  if(error != AD5940ERR_OK)
    return error;
  if(seqinfo.SeqLen > 512)
    return AD5940ERR_SEQLEN;
  seqinfo.SeqId = SEQID_0;
  seqinfo.SeqRamAddr = 0;
  seqinfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&seqinfo);

  return AD5940ERR_OK;
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppBIACfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
	fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP2_TRIG|GP1_SYNC|GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
	
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  return 0;
}

void AD5940_Main(void)
{
  AD5940Err error;
  SEQCfg_Type seqcfg;
  AD5940PlatformCfg();
  AD5940_StructInit(&seqcfg, sizeof(seqcfg));
  seqcfg.SeqEnable = bTRUE;
  seqcfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer, others for data FIFO */
  AD5940_SEQCfg(&seqcfg);   /* Enable Sequencer here. */
  error = BuildSequence();          /* Generate sequencer commands and load it to SRAM */
  if(error != AD5940ERR_OK)
  {
    printf("Build Sequence error, errorno:%d. \n", error);
    while(1);
  }
	AD5940_SEQMmrTrig(SEQID_0); /* Trigger sequence0 */
  while(1)
  {
    while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
    AD5940_SEQCtrlS(bTRUE);
	  AD5940_SEQMmrTrig(SEQID_0);
  }
}

