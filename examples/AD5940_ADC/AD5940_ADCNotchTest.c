/*!
 *****************************************************************************
 @file:    AD5940_ADCNotchTest.c
 @author:  Neo Xu
 @brief:   Notch filter test.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/
/** @addtogroup AD5940_Standard_Examples
  * @{
    @defgroup ADC_Notch_Test_Example
    @{
  */

#include "ad5940.h"
#include <stdio.h>

#ifdef ADI_DEBUG
#undef ADI_DEBUG
#endif

void ad5940_sequencer_init(uint8_t adc_rate, uint8_t sinc3osr, uint8_t sinc2osr){
  FIFOCfg_Type fifo_cfg;
  SEQCfg_Type seq_cfg;
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;   /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
  fifo_cfg.FIFOThresh = 1;
  AD5940_FIFOCfg(&fifo_cfg);          /* Disable to reset FIFO. */
	fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);          /* Enable FIFO here */
  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
		
  uint32_t WaitClks;
  uint8_t dl_60, dl_50, dl=0;
  ADCFilterCfg_Type filter;
  filter.ADCSinc3Osr = sinc3osr;
  filter.ADCSinc2Osr = sinc2osr;
  filter.ADCRate = adc_rate;
  const uint32_t sinc2osr_table[] = {22,44,89,178,267,533,640,667,800,889,1067,1333,0};
  const uint32_t sinc3osr_table[] = {5,4,2,0};
  printf("SINC3:OSR%d, SINC2:OSR%d, ", sinc3osr_table[sinc3osr], sinc2osr_table[sinc2osr]);
  if(AD5940_Notch50HzAvailable(&filter, &dl_50)){
    printf("NOTCH50:  DL:%d, %dHz ", dl_50, (uint32_t)((adc_rate==ADCRATE_1P6MHZ?1.6e6:800000.0)/sinc3osr_table[sinc3osr]/sinc2osr_table[sinc2osr]/dl_50 + .5));
    dl += dl_50-1;
  }
  if(AD5940_Notch60HzAvailable(&filter, &dl_60)){
    printf("NOTCH60:  DL:%d, %dHz ", dl_60, (uint32_t)((adc_rate==ADCRATE_1P6MHZ?1.6e6:800000.0)/sinc3osr_table[sinc3osr]/sinc2osr_table[sinc2osr]/dl_60 + .5));
    dl += dl_60-1;
  }
  ClksCalInfo_Type clks_cal;
  clks_cal.DataType = DATATYPE_NOTCH;
  clks_cal.ADCRate = adc_rate;
  clks_cal.DataCount = 1;             /* Sample one data when wakeup */
  clks_cal.ADCSinc2Osr = sinc2osr;
  clks_cal.ADCSinc3Osr = sinc3osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = adc_rate==ADCRATE_1P6MHZ?.5:1;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);
  
	static uint32_t buff[128];
	AD5940_SEQGenInit(buff, 128);
  AD5940_SEQGenCtrl(bTRUE);
	
  AD5940_SEQGpioCtrlS(AGPIO_Pin1);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  //wait for first data ready
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bFALSE);  /* Stop ADC convert and DFT */
  AD5940_SEQGpioCtrlS(0);
  AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
	
	SEQInfo_Type seqinfo;
	AD5940_SEQGenFetchSeq(&seqinfo.pSeqCmd, &seqinfo.SeqLen);
	seqinfo.SeqId = SEQID_0;
	seqinfo.SeqRamAddr = 0;
	seqinfo.WriteSRAM = bTRUE;
	AD5940_SEQInfoCfg(&seqinfo);
  
  AGPIOCfg_Type gpio_cfg;
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP2_TRIG|GP1_SYNC|GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = AGPIO_Pin2;
  AD5940_AGPIOCfg(&gpio_cfg);
}

uint32_t ad5940_notch_test(uint8_t adc_rate, uint8_t sinc3osr, uint8_t sinc2osr){
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  
  /* Use hardware reset */
  AD5940_HWReset();

  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize();
  
  CLKCfg_Type clk_cfg;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = adc_rate==ADCRATE_1P6MHZ?SYSCLKDIV_2:SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = adc_rate==ADCRATE_1P6MHZ?bTRUE:bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  
  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  
  /* Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_VREF1P8DAC;
  adc_base.ADCMuxN = ADCMUXN_VSET1P1;
  adc_base.ADCPga = ADCPGA_1P5;
  AD5940_ADCBaseCfgS(&adc_base);
  AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE);
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  adc_filter.ADCSinc3Osr = sinc3osr;
  adc_filter.ADCSinc2Osr = sinc2osr;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = adc_rate;              /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bFALSE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */ 
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
  AD5940_ADCFilterCfgS(&adc_filter);

  /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); 

	ad5940_sequencer_init(adc_rate, sinc3osr, sinc2osr);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_SEQCtrlS(bTRUE);
  AD5940_ClrMCUIntFlag();
  AD5940_SEQMmrTrig(SEQID_0);
  while(1)
  {
    {int32_t i = 1000000;while(i--);}
    if(AD5940_GetMCUIntFlag())  
    {
      AD5940_ClrMCUIntFlag();
			uint32_t fifo_count = AD5940_FIFOGetCnt();
      if(fifo_count == 1){
        int32_t rd;
        AD5940_FIFORd((uint32_t*)&rd, 1);
        rd &= 0xffff;
        float volt = AD5940_ADCCode2Volt(rd, ADCPGA_1P5, 1.82)+1.11;
        volt -= 1.82;
        if(volt < 0) volt = -volt;
        if(volt > 0.0005){
          printf("FAILED:CODE:rd:%d\n", rd);
          return 1;
        }
        printf("PASS\n");
        return 0;
      }
      else{
        printf("FAILED:%d\n", fifo_count);
        return 1;
      }
    }
  }
}

void AD5940_Main(void)
{
  uint32_t failed = 0;
  uint8_t sinc3=0, sinc2=0;
  uint8_t adc_rate=0;
  for(;adc_rate<=1;adc_rate++){
    sinc3=0;
    for(;sinc3<=ADCSINC3OSR_2;sinc3++){
      sinc2=0;
      for(;sinc2<=ADCSINC2OSR_1333;sinc2++){
        failed |= ad5940_notch_test(adc_rate, sinc3, sinc2);
      }
    }
  }
  printf("Test Done with status: %s\n",failed?"FAILED":"SUCCEED");
  while(1);
}

/**
 * @}
 * @}
 * */
