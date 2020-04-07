/*!
 *****************************************************************************
 @file:    AD5940_Sequencer.c
 @author:  Neo Xu
 @brief:   Basic usage of sequencer.
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

/**
 * Sequencer is used to control the AFE automatically. It can execute commands that
 * is pre-loaded to SRAM. There are 6kB SRAM available while you can choose to use
 * 2kB or 4kB of it and use reset of SRAM for data FIFO.
 * There are 3 commands available. We mainly use only two commands:
 *  - Write register
 *  - Wait
 * We control the AFE by registers, so with sequencer, we can do almost everything.
 * 
 * Once sequencer is enabled, it starts to wait valid trigger signal. Sequencer can
 * manage 4sequences at same time. You can choose which sequence you want to trigger.
 * To make the AFE can manage measurement automatically, there are three method to
 * trigger sequence.
 *  - MMR. You can trigger any sequence by register write. Or call function @ref AD5940_SEQMmrTrig
 *  - GPIO. You can trigger any sequence by GPIO. To use this, you must firstly set
 *          GPIO function to GPx_TRIG. Where x is the GPIO number. GPIO0 is used to trigger
 *          Sequence0 and GPIO3 is used to trigger Sequence3. Check the macro definition to 
 *          Check the details (or below table). 
 *          |GPIO|WhichSequence|
 *          |GP0|SEQUENCE0|
 *          |GP1|SEQUENCE1|
 *          |GP2|SEQUENCE2|
 *          |GP3|SEQUENCE3|
 *          |GP4|SEQUENCE0|
 *          |GP5|SEQUENCE1|
 *          |GP6|SEQUENCE2|
 *          |GP7|SEQUENCE3|
 *  - WakeupTimer. Wakeuptimer can automatically wakeup AFE from hibernate state and trigger selected
 *                 sequence in register SEQORDER. This register defines the order of sequence that 
 *                 Wakeuptimer will trigger. There are 8 slots in this register. You can fill in any
 *                 of the four sequences. Also, you can choose not to use all these 8 slots, just simply
 *                 specify the end slot. We call the 8 slots are A/B/C/D/E/F/G/H. For example you can 
 *                 choose the end slot as C. So wakeup timer will trigger the sequence in below order:
 *                 A->B->C->A->B->C->A->B->C->... until you stop Wakeuptimer.
 *                 If you fill in slot A with sequence0, B with Sequence3, C with sequence1, the sequence
 *                 will be executed in the order defined above(A-B-C-A-B-C...)
 *                 SEQ0->SEQ3->SEQ1->SEQ0->SEQ3->SEQ1->...
 *                 For each sequence, there is a sleep timer and a wakeup timer. The timer will automatically
 *                 load corresponding value.
 *                 The structure @ref WUPTCfg_Type can be used to initialize all above settings.
 * 
 *  In this example, we use both three kinds of trigger source.
 *  We firstly use Wakeup Timer to trigger sequence 0/1/2. The sequence is used to write registers and
 *  generate a custom-interrupt. We detect the interrupt to identify which sequence is running.
 *  Finally, we use GPIO to trigger sequence3.
 *  
 *  When there is conflict between trigger signals, for example, GPIO triggered one sequence that is running,
 *  current strategy is ignore this trigger.
 *  Use @reg SEQCfg_Type to configure sequencer.
 * 
 *  @note: connect GP2 and GP1 together. This demo show how to use GPIO to trigger sequencer. GP2 is the trigger input.
 *         We use GP1 to generate the trigger signal, while in real case, it should be the MCU's GPIO.
*/
#include "ad5940.h"
#include <stdio.h>
#include "string.h"

int32_t SeqISR(void);
BoolFlag bSeqEnd = bFALSE;
static const uint32_t Seq0Commands[]=
{
  SEQ_WR(REG_AFE_SWCON, 0x0000),
  SEQ_INT0(),   /* generate custom-interrupt 0. We can generate any custom interrupt(SEQ_INT0/1/2/3()) by sequencer.  */
};

static const uint32_t Seq1Commands[]=
{
  SEQ_WR(REG_AFE_SWCON, 0x1111),
  SEQ_INT1(),   /* generate custom-interrupt 0 */
  SEQ_STOP(),   /* Disable sequencer */
};

static const uint32_t Seq2Commands[]=
{
  SEQ_WR(REG_AFE_SWCON, 0x2222),
  SEQ_INT2(),   /* generate custom-interrupt 1 */
};

static const uint32_t Seq3Commands[]=
{
  SEQ_WR(REG_AFE_SWCON, 0x3333),
  SEQ_INT3(),   /* generate custom-interrupt 1 */
};

static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();    /* Call this right after AFE reset */
  /* Platform configuration */
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
  fifo_cfg.FIFOThresh = 4;//AppIMPCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);

  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0|AFEINTSRC_CUSTOMINT1|AFEINTSRC_CUSTOMINT2|AFEINTSRC_CUSTOMINT3, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  /*  GP0: the interrupt output.
      GP1: normal GPIO
      GP2: used as trigger to sequence2. If valid trigger signal detected, sequencer will try to run sequence2.
      GP3: not used.
      GP4: controlled by sequencer.
      Others: not used. The default function is mode0.
   */
  gpio_cfg.FuncSet = GP0_INT|GP1_GPIO|GP2_TRIG|GP4_SYNC;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2|AGPIO_Pin4;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  return 0;
}

#define SEQ0ADDR  0
#define SEQ1ADDR  16
#define SEQ2ADDR  32
#define SEQ3ADDR  48

void AD5940_Main(void)
{
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  WUPTCfg_Type wupt_cfg;
  SEQInfo_Type seqinfo0, seqinfo1, seqinfo2, seqinfo3;
  SeqGpioTrig_Cfg seqgpiotrig_cfg;
  AD5940PlatformCfg();

  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bTRUE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  
  /* Reconfigure FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);                  /* Disable FIFO firstly */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
  fifo_cfg.FIFOThresh = 4;      
  AD5940_FIFOCfg(&fifo_cfg);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  seqinfo0.pSeqCmd = Seq0Commands;
  seqinfo0.SeqId = SEQID_0;
  seqinfo0.SeqLen = SEQ_LEN(Seq0Commands);
  seqinfo0.SeqRamAddr = SEQ0ADDR;
  seqinfo0.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&seqinfo0); /* Configure sequence0 info and write commands to SRAM */

  seqinfo1.pSeqCmd = Seq1Commands;
  seqinfo1.SeqId = SEQID_1;
  seqinfo1.SeqLen = SEQ_LEN(Seq1Commands);
  seqinfo1.SeqRamAddr = SEQ1ADDR;
  seqinfo1.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&seqinfo1);

  seqinfo2.pSeqCmd = Seq2Commands;
  seqinfo2.SeqId = SEQID_2;
  seqinfo2.SeqLen = SEQ_LEN(Seq2Commands);
  seqinfo2.SeqRamAddr = SEQ2ADDR;
  seqinfo2.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&seqinfo2);

  seqinfo3.pSeqCmd = Seq3Commands;
  seqinfo3.SeqId = SEQID_3;
  seqinfo3.SeqLen = SEQ_LEN(Seq3Commands);
  seqinfo3.SeqRamAddr = SEQ3ADDR;
  seqinfo3.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&seqinfo3);
  
  /* Configure wakeup timer */
  wupt_cfg.WuptEn = bFALSE;     /* Don't start it right now. */
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_C; /* A->B->C->A->B-C */
  wupt_cfg.WuptOrder[0] = SEQID_0;    /* Put SEQ0 to slotA */
  wupt_cfg.WuptOrder[1] = SEQID_3;    /* Put SEQ3 to slotB */
  wupt_cfg.WuptOrder[2] = SEQID_1;    /* Put SEQ1 to slotC */
  /* There is no need to init slot DEFGH, that's WuptOrder[3] to WuptOrder[7], becaue we don't use it. EndofSeq is C.*/
  wupt_cfg.SeqxSleepTime[SEQID_0] = 10;
  wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(32000.0f*500/1000.0f) - 10 - 2; /* 500ms after, wakeup and trigger seq0 */
  wupt_cfg.SeqxSleepTime[SEQID_3] = 10;
  wupt_cfg.SeqxWakeupTime[SEQID_3] = (uint32_t)(32000.0f*1000/1000.0f)- 10 -2;  /* 1000ms after, trigger seq2 */
  wupt_cfg.SeqxSleepTime[SEQID_1] = 10;
  wupt_cfg.SeqxWakeupTime[SEQID_1] = (uint32_t)(32000.0f*2000/1000.0f)- 10 -2;  /* 2000ms after, trigger seq2 */
  AD5940_WUPTCfg(&wupt_cfg);
 
  printf("Test0: trigger sequencer by wakeup timer.\n");
  AD5940_WUPTCtrl(bTRUE); /* Enable wakeup timer. */
  while(1)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      SeqISR();
      if(bSeqEnd)
        break;
    }
  }
  AD5940_WUPTCtrl(bFALSE);  /* Wakeup timer is still running and triggering. Trigger is not accepted because sequencer 
                              is disabled in last sequence(SEQ1) command. */
  AD5940_SEQCtrlS(bTRUE);   /* Enable sequencer again, because we disabled it in seq3 last command. */

  /* Test MMR trigger */
  printf("\nTest1: trigger sequence2 manually by register write.\n");
  AD5940_SEQMmrTrig(SEQID_2); /* Trigger sequence2 manually. */
  /* Wait until CUSTMINT2 is set. We generate this interrupt in SEQ2 */
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_CUSTOMINT2) == bFALSE);  /* Test INTC1, we enabled all interrupts in INTC1. */
  AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT2);
  printf("sequence2 has been executed\n");
  printf("SWCON:0x%08x\n", AD5940_ReadReg(REG_AFE_SWCON));
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Toggle GPIO to trigger sequencer2 */
  printf("\nTest2: trigger sequence2 manually by GPIO\n");
  printf("Please connect GP2 and GP1 together. We will set GP2 function to TRIG.\n"
        "GP1 is set to GPIO function and is in output state. We use GP1 to toggle GP2.\n");
  AD5940_Delay10us(100*1000*2);
  printf("Toggle GPIO now\n");

  /* Allow GP2 falling edge to trigger sequence2 */
  seqgpiotrig_cfg.bEnable = bTRUE;
  seqgpiotrig_cfg.PinSel = AGPIO_Pin2;
  seqgpiotrig_cfg.SeqPinTrigMode = SEQPINTRIGMODE_FALLING;
  AD5940_SEQGpioTrigCfg(&seqgpiotrig_cfg);
  /* GP2 is connected to GP1 by user.
    We generate falling edge on GP1(gpio, output) to control GP2(trigger, input).
   */
  AD5940_AGPIOSet(AGPIO_Pin1);
  AD5940_AGPIOClr(AGPIO_Pin1);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_CUSTOMINT2) == bFALSE);  /* Test INTC1, we enabled all interrupts in INTC1. */

  printf("Trigger received and sequence2 has been executed\n\n");
  printf("Sequencer test done!\n");
  while(1);
}

int32_t SeqISR(void)
{
  uint32_t IntFlag, temp;
  
  IntFlag = AD5940_INTCGetFlag(AFEINTC_0);
  
  if(IntFlag & AFEINTSRC_CUSTOMINT0)
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
    printf("Custom INT0!\n");
    temp = AD5940_ReadReg(REG_AFE_SWCON);
    printf("SWCON:0x%08x\n", temp);
  }
  if(IntFlag & AFEINTSRC_CUSTOMINT1)
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT1);
    printf("Custom INT1!\n");
    temp = AD5940_ReadReg(REG_AFE_SWCON);
    printf("SWCON:0x%08x\n", temp);
  }
  if(IntFlag & AFEINTSRC_CUSTOMINT2)
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT2);
    printf("Custom INT2!\n");
    temp = AD5940_ReadReg(REG_AFE_SWCON);
    printf("SWCON:0x%08x\n", temp);
  }
  if(IntFlag & AFEINTSRC_CUSTOMINT3)
  {
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT3);
    printf("Custom INT3!\n");
    temp = AD5940_ReadReg(REG_AFE_SWCON);
    printf("SWCON:0x%08x\n", temp);
  }
  if(IntFlag & AFEINTSRC_ENDSEQ)  /* This interrupt is generated when Sequencer is disabled. */
  {
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
    printf("End of Sequence\n");
    bSeqEnd = bTRUE;
  }
	return AD5940ERR_OK;
}

