/**  
 * @file       ad5940.c
 * @brief      AD5940 library. This file contains all AD5940 library functions. 
 * @author     ADI
 * @date       March 2019
 * @par Revision History:
 * 
 * Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
 * 
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
**/
#include "ad5940.h"

/*! \mainpage AD5940 Library Introduction
 * 
 * ![AD5940 EVAL Board](https://www.analog.com/-/media/analog/en/evaluation-board-images/images/eval-ad5940elcztop-web.gif?h=500&thn=1&hash=1F38F7CC1002894616F74D316365C0A2631C432B "ADI logo") 
 * 
 * # Introduction
 *
 * The documentation is for AD594x library and examples.
 * 
 * # Manual Structure
 *
 * @ref AD5940_Library                                                      
 *  - @ref AD5940_Functions                                                 
 *  - @ref TypeDefinitions                                                    
 * @ref AD5940_Standard_Examples                                            
 * @ref AD5940_System_Examples	                                            
 * 
 * # How to Use It
 *  We provide examples that can directly run out of box.
 *  The files can generally be separated to three parts:
 *    - AD5940 Library files. ad5940.c and ad5940.h specifically. These two files are shared among all examples.
 *    - AD5940 System Examples. The system examples mean system level application like measuring impedance.
 *    - Standard examples. These include basic block level examples like ADC. It shows how to setup and use one specific block.
 * 
 * ## Requirements to run these examples
 *  ### Hardware
 *  - Use EVAL_AD5940 or EVAL_AD5941. The default MCU board we used is ADICUP3029. We also provide project for ST NUCLEO board.
 *  - Or use EVAL_ADuCM355
 *  ### Software
 *  - Pull all the source file from [GitHub](https://github.com/analogdevicesinc/ad5940-examples.git)
 *  - CMSIS pack that related to specific MCU. This normally is done by IDE you use.
 * 
 * ## Materials
 *      Please use this library together with following materials.
 *      - [AD5940 Data Sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/AD5940.pdf)
 *      - [AD5940 Eval Board](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD5940.html)
 *
 */

/* Remove below variables after AD594x is released. */
static BoolFlag bIsS2silicon = bFALSE;

/* Declare of SPI functions used to read/write registers */
#ifndef CHIPSEL_M355
static uint32_t AD5940_SPIReadReg(uint16_t RegAddr);
static void AD5940_SPIWriteReg(uint16_t RegAddr, uint32_t RegData);
#else
static uint32_t AD5940_D2DReadReg(uint16_t RegAddr);
static void AD5940_D2DWriteReg(uint16_t RegAddr, uint32_t RegData);
#endif

/** 
 * @addtogroup AD5940_Library
 *  The library functions, structures and constants.
 * @{
 *    @defgroup AD5940_Functions
 *    @{
 *        @defgroup Function_Helpers
 *        @brief The functions with no hardware access. They are helpers.
 *        @{
 *            @defgroup Sequencer_Generator_Functions
 *            @brief The set of function used to track all register read and write once it's enabled. It can translate register write operation to sequencer commands. 
 *            @{
*/

#define SEQUENCE_GENERATOR  /*!< Build sequence generator part in to lib. Comment this line to remove this feature  */

#ifdef SEQUENCE_GENERATOR
/**
 * Structure used to store register information(address and its data) 
 * */
typedef struct
{
  uint32_t RegAddr  :8;   /**< 8bit address is enough for sequencer */
  uint32_t RegValue :24;  /**< Reg data is limited to 24bit by sequencer  */
}SEQGenRegInfo_Type;

/**
 * Sequencer generator data base.
*/
struct
{
  BoolFlag EngineStart;         /**< Flag to mark start of the generator */
  uint32_t BufferSize;          /**< Total buffer size */

  uint32_t *pSeqBuff;           /**< The buffer for sequence generator(both sequences and RegInfo) */
  uint32_t SeqLen;              /**< Generated sequence length till now */
  SEQGenRegInfo_Type *pRegInfo; /**< Pointer to buffer where stores register info */
  uint32_t RegCount;            /**< The count of register info available in buffer *pRegInfo. */
  AD5940Err LastError;          /**< The last error message. */
}SeqGenDB;  /* Data base of Seq Generator */

/**
 * @brief Manually input a command to sequencer generator.
 * @param CmdWord: The 32-bit width sequencer command word. @ref Sequencer_Helper can be used to generate commands.
 * @return None;
*/
void AD5940_SEQGenInsert(uint32_t CmdWord)
{
  uint32_t temp;
  temp  = SeqGenDB.RegCount + SeqGenDB.SeqLen;
  /* Generate Sequence command */
  if(temp < SeqGenDB.BufferSize)
  {
    SeqGenDB.pSeqBuff[SeqGenDB.SeqLen] = CmdWord;
    SeqGenDB.SeqLen ++;
  }
  else  /* There is no buffer */
    SeqGenDB.LastError = AD5940ERR_BUFF;
}

/**
 * @brief Search data-base to get current register value.
 * @param RegAddr: The register address.
 * @param pIndex: Pointer to a variable that used to store index of found register-info.
 * @return Return AD5940ERR_OK if register found in data-base. Otherwise return AD5940ERR_SEQREG.
*/
static AD5940Err AD5940_SEQGenSearchReg(uint32_t RegAddr, uint32_t *pIndex)
{
  uint32_t i;

  RegAddr = (RegAddr>>2)&0xff;
  for(i=0;i<SeqGenDB.RegCount;i++)
  {
    if(RegAddr == SeqGenDB.pRegInfo[i].RegAddr)
    {
      *pIndex = i;
      return AD5940ERR_OK;
    }
  }
  return AD5940ERR_SEQREG;
}

/**
 * @brief Get the register default value by SPI read. This function requires AD5940 is in active state, otherwise we cannot get the default register value.
 * @param RegAddr: The register address.
 * @param pRegData: Pointer to a variable to store register default value.
 * @return Return AD5940ERR_OK.
*/
static AD5940Err AD5940_SEQGenGetRegDefault(uint32_t RegAddr, uint32_t *pRegData)
{
#ifdef CHIPSEL_M355
  *pRegData = AD5940_D2DReadReg(RegAddr);
#else
  *pRegData = AD5940_SPIReadReg(RegAddr);
#endif
  return AD5940ERR_OK;
}

/**
 * @brief Record the current register info to data-base. Update LastError if there is error.
 * @param RegAddr: The register address.
 * @param RegData: The register data
 * @return Return None.
*/
static void AD5940_SEQRegInfoInsert(uint16_t RegAddr, uint32_t RegData)
{
  uint32_t temp;
  temp = SeqGenDB.RegCount + SeqGenDB.SeqLen;
  
  if(temp < SeqGenDB.BufferSize)
  {
    SeqGenDB.pRegInfo --; /* Move back */
    SeqGenDB.pRegInfo[0].RegAddr = (RegAddr>>2)&0xff;
    SeqGenDB.pRegInfo[0].RegValue = RegData&0x00ffffff;
    SeqGenDB.RegCount ++;
  }
  else  /* There is no more buffer  */
  {
    SeqGenDB.LastError = AD5940ERR_BUFF;
  }
}

/**
 * @brief Get current register value. If we have record in data-base, read it. Otherwise, return the register default value.
 * @param RegAddr: The register address.
 * @return Return register value.
*/
static uint32_t AD5940_SEQReadReg(uint16_t RegAddr)
{
  uint32_t RegIndex, RegData;
  
  if(AD5940_SEQGenSearchReg(RegAddr, &RegIndex) != AD5940ERR_OK)
  {
    /* There is no record in data-base, read the default value. */
    AD5940_SEQGenGetRegDefault(RegAddr, &RegData);
    AD5940_SEQRegInfoInsert(RegAddr, RegData);
  }
  else
  {
    /* return the current register value stored in data-base */
    RegData = SeqGenDB.pRegInfo[RegIndex].RegValue;
  }

  return RegData;
}

/**
 * @brief Generate a sequencer command to write register. If the register address is out of range, it won't generate a command.
 *        This function will also update the register-info in data-base to record current register value.
 * @param RegAddr: The register address.
 * @param RegData: The register value.
 * @return Return None.
*/
static void AD5940_SEQWriteReg(uint16_t RegAddr, uint32_t RegData)
{
  uint32_t RegIndex;
  
  if(RegAddr > 0x21ff)
  {
    SeqGenDB.LastError = AD5940ERR_ADDROR;  /* address out of range  */
    return;
  }

  if(AD5940_SEQGenSearchReg(RegAddr, &RegIndex) == AD5940ERR_OK)
  {
    /* Store register value */
    SeqGenDB.pRegInfo[RegIndex].RegValue = RegData;
    /* Generate Sequence command */
    AD5940_SEQGenInsert(SEQ_WR(RegAddr, RegData));
  }
  else
  {
    AD5940_SEQRegInfoInsert(RegAddr, RegData);
    /* Generate Sequence command */
    AD5940_SEQGenInsert(SEQ_WR(RegAddr, RegData));
  }
}

/**
 * @brief Initialize sequencer generator with specified buffer.
 *        The buffer is used to store sequencer generated and record register value changes.
 *        The command is stored from start address of buffer while register value is stored from end of buffer.
 *    Buffer[0] : First sequencer command;
 *    Buffer[1] : Second Sequencer command;
 *    ...
 *    Buffer[Last-1]: The second register value record.
 *    Buffer[Last]: The first register value record.
 * @param pBuffer: Pointer to the buffer.
 * @param BufferSize: The buffer length.
 * @return Return None.
*/
void AD5940_SEQGenInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  if(BufferSize < 2) return;
  SeqGenDB.BufferSize = BufferSize;
  SeqGenDB.pSeqBuff = pBuffer;
  SeqGenDB.pRegInfo = (SEQGenRegInfo_Type*)pBuffer + BufferSize - 1; /* Point to the last element in buffer */
  SeqGenDB.SeqLen = 0;

  SeqGenDB.RegCount = 0;
  SeqGenDB.LastError = AD5940ERR_OK;
  SeqGenDB.EngineStart = bFALSE;
}

/**
 * @brief Get sequencer command generated.
 * @param ppSeqCmd: Pointer to a variable(pointer) used to store the pointer to generated sequencer command.
 * @param pSeqLen: Pointer to a variable that used to store how many commands available in buffer.
 * @return Return lasterror.
*/
AD5940Err AD5940_SEQGenFetchSeq(const uint32_t **ppSeqCmd, uint32_t *pSeqLen)
{
  AD5940Err lasterror;

  if(ppSeqCmd)
    *ppSeqCmd = SeqGenDB.pSeqBuff;  
  if(pSeqLen)
    *pSeqLen = SeqGenDB.SeqLen;

  //SeqGenDB.SeqLen = 0;  /* Start a new sequence */
  lasterror = SeqGenDB.LastError;
  //SeqGenDB.LastError = AD5940ERR_OK;  /* Clear error message */
  return lasterror;
}

/**
 * @brief Start or stop the sequencer generator. Once started, the register write will be recorded to sequencer generator.
 *        Once it's disabled, the register write is written to AD5940 directly by SPI bus.
 * @param bFlag: Enable or disable sequencer generator.
 * @return Return None.
*/
void AD5940_SEQGenCtrl(BoolFlag bFlag)
{
  if(bFlag == bFALSE) /* Disable sequence generator */
  {
    SeqGenDB.EngineStart = bFALSE;
  }
  else
  {
    SeqGenDB.SeqLen = 0;
    SeqGenDB.LastError = AD5940ERR_OK;  /* Clear error message */
    SeqGenDB.EngineStart = bTRUE;
  }
}

/**
 * @brief Calculate the number of cycles in the sequence
 * @return Return Number of ACLK Cycles that a generated sequence will take.
*/
uint32_t AD5940_SEQCycleTime(void)
{
  uint32_t i, Cycles, Cmd;  
  Cycles = 0;
  for(i=0;i<SeqGenDB.RegCount;i++)
  {
    Cmd = (SeqGenDB.pSeqBuff[i]  >> 30) & 0x3;
    if (Cmd & 0x2)
    {
      /* A write command */
      Cycles += 1;
    }
    else
    {
      if (Cmd & 0x1)
      {
        /* Timeout Command */    
        Cycles += 1;
      }
      else
        {
          /* Wait command */
          Cycles += SeqGenDB.pSeqBuff[i] & 0x3FFFFFFF;
        }
    }
  } 
  return Cycles;  
}
#endif
/**
 * @} Sequencer_Generator_Functions
*/

/**
 * Check if an uint8_t value exist in table.
*/
static int32_t _is_value_in_table(uint8_t value, const uint8_t *table, uint8_t len, uint8_t *index)
{
  for(int i=0; i<len; i++)
  {
    if(value == table[i])
    {
      *index = i;
      return bTRUE;
    }
  }
  return bFALSE;
}

/**
 * @brief return if the SINC3/SINC2 combination is available for notch 50Hz filter.
 *        If it's not availabe, hardware automatically bypass Notch even if it's enabled.
 * @param pFilterInfo the filter configuration, only need sinc2/sinc3 osr and adc data rate information.
 * @return return bTRUE if notch 50Hz filter is available.
*/
BoolFlag AD5940_Notch50HzAvailable(ADCFilterCfg_Type *pFilterInfo, uint8_t *dl)
{
  if((pFilterInfo->ADCRate == ADCRATE_800KHZ && pFilterInfo->ADCSinc3Osr == ADCSINC3OSR_2)||\
      (pFilterInfo->ADCRate == ADCRATE_1P6MHZ && pFilterInfo->ADCSinc3Osr != ADCSINC3OSR_2))
  {
    //this combination suits for filter:
    //SINC3 OSR2, for 800kSPS
    //and SINC3 OSR4 and OSR5 for 1.6MSPS,
    const uint8_t available_sinc2_osr[] = {ADCSINC2OSR_533, ADCSINC2OSR_667,ADCSINC2OSR_800, ADCSINC2OSR_889, ADCSINC2OSR_1333};
    const uint8_t dl_50Hz[] = {15,12,10,9,6};
    uint8_t index;
    if(_is_value_in_table(pFilterInfo->ADCSinc2Osr, available_sinc2_osr, sizeof(available_sinc2_osr), &index))
    {
      *dl = dl_50Hz[index];
      return bTRUE;
    }
  }
  else if(pFilterInfo->ADCRate == ADCRATE_1P6MHZ && pFilterInfo->ADCSinc3Osr == ADCSINC3OSR_2)
  {
    //this combination suits for filter:
    //SINC3 OSR2 for 1.6MSPS
    const uint8_t available_sinc2_osr[] = {ADCSINC2OSR_889, ADCSINC2OSR_1067, ADCSINC2OSR_1333};
    const uint8_t dl_50Hz[] = {18,15,12};
    uint8_t index;
    if(_is_value_in_table(pFilterInfo->ADCSinc2Osr, available_sinc2_osr, sizeof(available_sinc2_osr), &index))
    {
      *dl = dl_50Hz[index];
      return bTRUE;
    }
  }
  else if(pFilterInfo->ADCRate == ADCRATE_800KHZ && pFilterInfo->ADCSinc3Osr != ADCSINC3OSR_2)
  {
    //this combination suits for filter:
    //SINC3 OSR4 and OSR5 for 800kSPS,
    const uint8_t available_sinc2_osr[] = {ADCSINC2OSR_178, ADCSINC2OSR_267, ADCSINC2OSR_533, ADCSINC2OSR_640,\
                                    ADCSINC2OSR_800, ADCSINC2OSR_1067};
    const uint8_t dl_50Hz[] = {18,12,6,5,4,3};
    uint8_t index;
    if(_is_value_in_table(pFilterInfo->ADCSinc2Osr, available_sinc2_osr, sizeof(available_sinc2_osr), &index))
    {
      *dl = dl_50Hz[index];
      return bTRUE;
    }
  }
  *dl = 0;
  return bFALSE;
}

/**
 * @brief return if the SINC3/SINC2 combination is available for notch 60Hz filter.
 *        If it's not availabe, hardware automatically bypass Notch even if it's enabled.
 * @param pFilterInfo the filter configuration, need sinc2/sinc3 osr and adc data rate information.
 * @return return bTRUE if notch 60Hz filter is available.
*/
BoolFlag AD5940_Notch60HzAvailable(ADCFilterCfg_Type *pFilterInfo, uint8_t *dl)
{
  if((pFilterInfo->ADCRate == ADCRATE_800KHZ && pFilterInfo->ADCSinc3Osr == ADCSINC3OSR_2)||\
      (pFilterInfo->ADCRate == ADCRATE_1P6MHZ && pFilterInfo->ADCSinc3Osr != ADCSINC3OSR_2))
  {
    //this combination suits for filter:
    //SINC3 OSR2, for 800kSPS
    //and SINC3 OSR4 and OSR5 for 1.6MSPS,
    const uint8_t available_sinc2_osr[] = {ADCSINC2OSR_667, ADCSINC2OSR_1333};
    const uint8_t dl_60Hz[] = {10,5};
    uint8_t index;
    if(_is_value_in_table(pFilterInfo->ADCSinc2Osr, available_sinc2_osr, sizeof(available_sinc2_osr), &index))
    {
      *dl = dl_60Hz[index];
      return bTRUE;
    }
  }
  else if(pFilterInfo->ADCRate == ADCRATE_1P6MHZ && pFilterInfo->ADCSinc3Osr == ADCSINC3OSR_2)
  {
    //this combination suits for filter:
    //SINC3 OSR2 for 1.6MSPS
    const uint8_t available_sinc2_osr[] = {ADCSINC2OSR_889, ADCSINC2OSR_1333};
    const uint8_t dl_60Hz[] = {15,10};
    uint8_t index;
    if(_is_value_in_table(pFilterInfo->ADCSinc2Osr, available_sinc2_osr, sizeof(available_sinc2_osr), &index))
    {
      *dl = dl_60Hz[index];
      return bTRUE;
    }
  }
  else if(pFilterInfo->ADCRate == ADCRATE_800KHZ && pFilterInfo->ADCSinc3Osr != ADCSINC3OSR_2)
  {
    //this combination suits for filter:
    //SINC3 OSR4 and OSR5 for 800kSPS,
    const uint8_t available_sinc2_osr[] = {ADCSINC2OSR_178, ADCSINC2OSR_267, ADCSINC2OSR_533, ADCSINC2OSR_667,\
                                    ADCSINC2OSR_889, ADCSINC2OSR_1333};
    const uint8_t dl_60Hz[] = {15,10,5,4,3,2};
    uint8_t index;
    if(_is_value_in_table(pFilterInfo->ADCSinc2Osr, available_sinc2_osr, sizeof(available_sinc2_osr), &index))
    {
      *dl = dl_60Hz[index];
      return bTRUE;
    }
  }
  *dl = 0;
  return bFALSE;
}

/**
 * @brief Calculate how many clocks are needed in sequencer wait command to generate required number of data from filter output.
 * @note When measurement is done, it's recommend to disable blocks like ADCPWR, ADCCNV, SINC2, DFT etc. If blocks remain powered up,
 *       they may need less clocks to generate required number of output. Use function @ref AD5940_AFECtrlS to control these blocks.
 * @param pFilterInfo: Pointer to configuration structure. 
 * @param pClocks: pointer used to store results.         
 * @return return none.
*/
void AD5940_ClksCalculate(ClksCalInfo_Type *pFilterInfo, uint32_t *pClocks)
{
  uint32_t temp = 0;
  const uint32_t sinc2osr_table[] = {22,44,89,178,267,533,640,667,800,889,1067,1333,0};
  const uint32_t sinc3osr_table[] = {5,4,2,0};

  *pClocks = 0;
  if(pFilterInfo == NULL) return;
  if(pClocks == NULL) return;
  if(pFilterInfo->ADCSinc2Osr > ADCSINC2OSR_1333) return;
  if(pFilterInfo->ADCSinc3Osr > 2)  return; /* 0: OSR5, 1:OSR4, 2:OSR2 */
  if(pFilterInfo->ADCAvgNum > ADCAVGNUM_16) return; /* Average number index:0,1,2,3 */
  switch(pFilterInfo->DataType)
  {
    case DATATYPE_ADCRAW:
      temp = (uint32_t)(20*pFilterInfo->DataCount*pFilterInfo->RatioSys2AdcClk);
      break;
    case DATATYPE_SINC3:
      temp = (uint32_t)(((pFilterInfo->DataCount+2)*sinc3osr_table[pFilterInfo->ADCSinc3Osr]+1)*20*pFilterInfo->RatioSys2AdcClk + 0.5f);
      break;
    case DATATYPE_SINC2: 
      temp = (pFilterInfo->DataCount+1)*sinc2osr_table[pFilterInfo->ADCSinc2Osr] + 1;
      pFilterInfo->DataType = DATATYPE_SINC3;
      pFilterInfo->DataCount = temp;
      AD5940_ClksCalculate(pFilterInfo, &temp);
      pFilterInfo->DataType = DATATYPE_SINC2;
      temp += 15;   /* Need extra 15 clocks for FIFO etc. Just to be safe. */
      break;
    case DATATYPE_NOTCH:
    {
      ADCFilterCfg_Type filter;
      filter.ADCRate = pFilterInfo->ADCRate;
      filter.ADCSinc3Osr = pFilterInfo->ADCSinc3Osr;
      filter.ADCSinc2Osr = pFilterInfo->ADCSinc2Osr;
      uint8_t dl=0, dl_50, dl_60;
      if(AD5940_Notch50HzAvailable(&filter, &dl_50)){
        dl += dl_50 - 1;
      }
      if(AD5940_Notch60HzAvailable(&filter, &dl_60)){
        dl += dl_60 - 1;
      }
      pFilterInfo->DataType = DATATYPE_SINC2;
      pFilterInfo->DataCount += dl; //DL is the extra data input needed for filter to output first data.
      AD5940_ClksCalculate(pFilterInfo,&temp);
      //restore the filter info.
      pFilterInfo->DataType = DATATYPE_NOTCH;
      pFilterInfo->DataCount -= dl;
      break;
    }
    case DATATYPE_DFT:
      switch(pFilterInfo->DftSrc)
      {
        case DFTSRC_ADCRAW:
          pFilterInfo->DataType = DATATYPE_ADCRAW;
          AD5940_ClksCalculate(pFilterInfo, &temp);
          break;
        case DFTSRC_SINC3:
          pFilterInfo->DataType = DATATYPE_SINC3;
          AD5940_ClksCalculate(pFilterInfo, &temp);
          break;
        case DFTSRC_SINC2NOTCH:
          if(pFilterInfo->BpNotch)
            pFilterInfo->DataType = DATATYPE_SINC2;
          else
            pFilterInfo->DataType = DATATYPE_NOTCH;
          AD5940_ClksCalculate(pFilterInfo, &temp);
          break;
        case DFTSRC_AVG:
          pFilterInfo->DataType = DATATYPE_SINC3;
          pFilterInfo->DataCount *= 1L<<(pFilterInfo->ADCAvgNum+1); /* 0: average2, 1: average4, 2: average8, 3: average16 */
          AD5940_ClksCalculate(pFilterInfo, &temp);
          break;
        default:
          break;
      }
      pFilterInfo->DataType = DATATYPE_DFT;
      temp += 25; /* add margin */
      break;
    default:
    break;
  }
  *pClocks = temp;
}

/**
   @brief void AD5940_SweepNext(SoftSweepCfg_Type *pSweepCfg, float *pNextFreq)
          For sweep function, calculate next frequency point according to pSweepCfg info.
   @return Return next frequency point in Hz.
*/
void AD5940_SweepNext(SoftSweepCfg_Type *pSweepCfg, float *pNextFreq)
{
   float frequency;

   if(pSweepCfg->SweepLog)/* Log step */
   {
      if(pSweepCfg->SweepStart<pSweepCfg->SweepStop) /* Normal */
      {
         if(++pSweepCfg->SweepIndex == pSweepCfg->SweepPoints)
            pSweepCfg->SweepIndex = 0;
         frequency = pSweepCfg->SweepStart*pow(10,pSweepCfg->SweepIndex*log10(pSweepCfg->SweepStop/pSweepCfg->SweepStart)/(pSweepCfg->SweepPoints-1));
      }
      else
      {
         pSweepCfg->SweepIndex --;
         if(pSweepCfg->SweepIndex >= pSweepCfg->SweepPoints)
            pSweepCfg->SweepIndex = pSweepCfg->SweepPoints-1;
         frequency = pSweepCfg->SweepStop*pow(10,pSweepCfg->SweepIndex*
                                     (log10(pSweepCfg->SweepStart/pSweepCfg->SweepStop)/(pSweepCfg->SweepPoints-1)));
      }
   }
   else/* Linear step */
   {
      if(pSweepCfg->SweepStart<pSweepCfg->SweepStop) /* Normal */
      {
         if(++pSweepCfg->SweepIndex == pSweepCfg->SweepPoints)
            pSweepCfg->SweepIndex = 0;
         frequency = pSweepCfg->SweepStart + pSweepCfg->SweepIndex*(double)(pSweepCfg->SweepStop-pSweepCfg->SweepStart)/(pSweepCfg->SweepPoints-1);
      }
      else
      {
         pSweepCfg->SweepIndex --;
         if(pSweepCfg->SweepIndex >= pSweepCfg->SweepPoints)
            pSweepCfg->SweepIndex = pSweepCfg->SweepPoints-1;
         frequency = pSweepCfg->SweepStop + pSweepCfg->SweepIndex*(double)(pSweepCfg->SweepStart - pSweepCfg->SweepStop)/(pSweepCfg->SweepPoints-1);
      }
   }
   
   *pNextFreq = frequency;
}

/**
  @brief Initialize Structure members to zero
  @param pStruct: Pointer to the structure. 
  @param StructSize: The structure size in Byte.
  @return Return None.
**/
void AD5940_StructInit(void *pStruct, uint32_t StructSize)
{
  memset(pStruct, 0, StructSize);
}

/**
  @brief Convert ADC Code to voltage. 
  @param ADCPga: The ADC PGA used for this result.
  @param code: ADC code.
  @param VRef1p82: the actual 1.82V reference voltage.
  @return Voltage in volt.
**/
float AD5940_ADCCode2Volt(uint32_t code, uint32_t ADCPga, float VRef1p82)
{
  float kFactor = 1.835/1.82;
  float fVolt = 0.0;
  float tmp = 0;
  tmp = (int32_t)code - 32768;
  switch(ADCPga)
  {
  case ADCPGA_1:
    break;
  case ADCPGA_1P5:
    tmp /= 1.5f;
    break;
  case ADCPGA_2:
    tmp /= 2.0f;
    break;
  case ADCPGA_4:
    tmp /= 4.0f;
    break;
  case ADCPGA_9:
    tmp /= 9.0f;
    break;
  default:break;
  }
  fVolt = tmp*VRef1p82/32768*kFactor;
  return fVolt;
}

/**
 * @brief Do complex number division.
 * @param a: The dividend.
 * @param b: The divisor.
 * @return Return result.
**/
fImpCar_Type AD5940_ComplexDivFloat(fImpCar_Type *a, fImpCar_Type *b)
{
  fImpCar_Type res;
  float temp;
  temp = b->Real*b->Real + b->Image*b->Image;
  res.Real = a->Real*b->Real + a->Image*b->Image;
  res.Real /= temp;
  res.Image = a->Image*b->Real - a->Real*b->Image;
  res.Image /= temp;
  return res;
}

/**
 * @brief Do complex number multiplication.
 * @param a: The multiplicand.
 * @param b: The multiplier .
 * @return Return result.
**/
fImpCar_Type AD5940_ComplexMulFloat(fImpCar_Type *a, fImpCar_Type *b)
{
  fImpCar_Type res;
  
  res.Real = a->Real*b->Real - a->Image*b->Image;
  res.Image = a->Image*b->Real + a->Real*b->Image;

  return res;
}
/**
 * @brief Do complex number addition.
 * @param a: The addend.
 * @param b: The addend .
 * @return Return result.
**/
fImpCar_Type AD5940_ComplexAddFloat(fImpCar_Type *a, fImpCar_Type *b)
{
  fImpCar_Type res;
  
  res.Real = a->Real + b->Real;
  res.Image = a->Image + b->Image;

  return res;
}

/**
 * @brief Do complex number subtraction.
 * @param a: The minuend.
 * @param b: The subtrahend .
 * @return Return result.
**/
fImpCar_Type AD5940_ComplexSubFloat(fImpCar_Type *a, fImpCar_Type *b)
{
  fImpCar_Type res;
  
  res.Real = a->Real - b->Real;
  res.Image = a->Image - b->Image;

  return res;
}

/**
 * @brief Do complex number division.
 * @param a: The dividend.
 * @param b: The divisor.
 * @return Return result.
**/
fImpCar_Type AD5940_ComplexDivInt(iImpCar_Type *a, iImpCar_Type *b)
{
  fImpCar_Type res;
  float temp;
  temp = (float)b->Real*b->Real + (float)b->Image*b->Image;
  res.Real = (float)a->Real*b->Real + (float)a->Image*b->Image;
  res.Real /= temp;
  res.Image = (float)a->Image*b->Real - (float)a->Real*b->Image;
  res.Image /= temp;
  return res;
}

/**
 * @brief Do complex number multiplication.
 * @param a: The multiplicand.
 * @param b: The multiplier .
 * @return Return result.
**/
fImpCar_Type AD5940_ComplexMulInt(iImpCar_Type *a, iImpCar_Type *b)
{
  fImpCar_Type res;
  
  res.Real = (float)a->Real*b->Real - (float)a->Image*b->Image;
  res.Image = (float)a->Image*b->Real + (float)a->Real*b->Image;

  return res;
}

/**
 * @brief Calculate the complex number magnitude.
 * @param a: The complex number.
 * @return Return magnitude.
**/
float AD5940_ComplexMag(fImpCar_Type *a)
{
  return sqrt(a->Real*a->Real + a->Image*a->Image);
}

/**
 * @brief Calculate the complex number phase.
 * @param a: The complex number.
 * @return Return phase.
**/
float AD5940_ComplexPhase(fImpCar_Type *a)
{
  return atan2(a->Image, a->Real);
}

/**
 * @brief Calculate the optimum filter settings based on signal frequency.
 * @param freq: Frequency of signalr.
 * @return Return FreqParams.
**/
FreqParams_Type AD5940_GetFreqParameters(float freq)
{
	const uint32_t dft_table[] = {4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384};
	const uint32_t sinc2osr_table[] = {1, 22,44,89,178,267,533,640,667,800,889,1067,1333};
  const uint32_t sinc3osr_table[] = {2, 4, 5};
	float AdcRate = 800000;
	uint32_t n1 = 0;	// Sample rate after ADC filters
	uint32_t n2 = 0; // Sample rate after DFT block
	uint32_t iCycle = 0;
	FreqParams_Type freq_params;
	/* High power mode */
	if(freq >= 20000)
	{
		freq_params. DftSrc = DFTSRC_SINC3;
		freq_params.ADCSinc2Osr = 0;
		freq_params.ADCSinc3Osr = 2;
		freq_params.DftNum = DFTNUM_8192;
		freq_params.NumClks = 0;
		freq_params.HighPwrMode = bTRUE;
		return freq_params;		
	}
	
	if(freq < 0.51)
	{
		freq_params. DftSrc = DFTSRC_SINC2NOTCH;
		freq_params.ADCSinc2Osr = 6;
		freq_params.ADCSinc3Osr = 1;
		freq_params.DftNum = DFTNUM_8192;
		freq_params.NumClks = 0;
		freq_params.HighPwrMode = bTRUE;
		return freq_params;		
	}
	
	/* Start with SINC2 setting */
	for(uint8_t i = 0; i<sizeof(sinc2osr_table) / sizeof(uint32_t); i++)
	{
		n1 = sinc2osr_table[i] * sinc3osr_table[1];
		if(((AdcRate/n1) < freq * 10) && (freq<20e3))
			continue;
		
		/* Try DFT number */
		for(uint32_t j = 8; j<sizeof(dft_table) / sizeof(uint32_t); j++)
		{
			n2 = dft_table[j];
			iCycle = (uint32_t)(n1 * n2 * freq)/AdcRate;
			if(iCycle < 8)
				continue;
			freq_params. DftSrc = DFTSRC_SINC2NOTCH;
			freq_params.ADCSinc2Osr = i-1;
			freq_params.ADCSinc3Osr = 1;
			freq_params.DftNum = j;
			freq_params.NumClks = 0;
			freq_params.HighPwrMode = bFALSE;
			if(n1 == 4)
			{
				freq_params. DftSrc = DFTSRC_SINC3;
				freq_params.ADCSinc2Osr = 0;
			}
			return freq_params;
		}
	}
		
	return freq_params;
}

/**
 * @} Function_Helpers
*/

#ifdef CHIPSEL_M355
static void AD5940_D2DWriteReg(uint16_t RegAddr, uint32_t RegData)
{
  if(((RegAddr>=0x1000)&&(RegAddr<=0x3014)))  /* 32bit register */
    *(volatile uint32_t *)(RegAddr+0x400c0000) = RegData;
  else                                        /* 16bit register */
    *(volatile uint16_t *)(RegAddr+0x400c0000) = RegData;
}

static uint32_t AD5940_D2DReadReg(uint16_t RegAddr)
{
  if(((RegAddr>=0x1000)&&(RegAddr<=0x3014)))  /* 32bit register */
    return *(volatile uint32_t *)(RegAddr+0x400c0000);
  else                                        /* 16bit register */
    return *(volatile uint16_t *)(RegAddr+0x400c0000);
}

void AD5940_FIFORd(uint32_t *pBuffer, uint32_t uiReadCount)   
{
  while(uiReadCount--)
    *pBuffer++ = *(volatile uint32_t *)(0x400c206C);
}
#else
/**
 * @defgroup SPI_Block
 * @brief Functions to communicate with AD5940 registers following AD5940 SPI protocols
 * @{
 * 
 * @defgroup SPI_Block_Functions
 * @brief The basic SPI protocols. All functions are basic on AD5940_ReadWriteNBytes which
 *        provided by user.
 *        
 *  ##SPI basic protocol
 *        All SPI protocol starts with one-byte command word. Following are data(16B or 32B)
 *        There are four SPI commands available @ref SPI_Block_Const.
 * @{
*/

/**
  @brief Using SPI to transmit one byte and return the received byte. 
  @param data: The 8-bit data SPI will transmit.
  @return received data.
**/
static unsigned char AD5940_ReadWrite8B(unsigned char data)
{
   uint8_t tx[1], rx[1];
   tx[0] = data;
   AD5940_ReadWriteNBytes(tx,rx,1);
   return rx[0];
}

/**
  @brief Using SPI to transmit two bytes and return the received bytes. 
  @param data: The 16-bit data SPI will transmit.
  @return received data.
**/
static uint16_t AD5940_ReadWrite16B(uint16_t data)
{
   uint8_t SendBuffer[2];
   uint8_t RecvBuffer[2];
   SendBuffer[0] = data>>8;
   SendBuffer[1] = data&0xff;
   AD5940_ReadWriteNBytes(SendBuffer,RecvBuffer,2);
   return (((uint16_t)RecvBuffer[0])<<8)|RecvBuffer[1];
}

/**
 * @brief Using SPI to transmit four bytes and return the received bytes. 
 * @param data: The 32-bit data SPI will transmit.
 * @return received data.
**/
static uint32_t AD5940_ReadWrite32B(uint32_t data)
{
   uint8_t SendBuffer[4];
   uint8_t RecvBuffer[4];
  
   SendBuffer[0] = (data>>24)&0xff;
   SendBuffer[1] = (data>>16)&0xff;
   SendBuffer[2] = (data>> 8)&0xff;
   SendBuffer[3] = (data    )&0xff;
   AD5940_ReadWriteNBytes(SendBuffer,RecvBuffer,4);
   return (((uint32_t)RecvBuffer[0])<<24)|(((uint32_t)RecvBuffer[1])<<16)|(((uint32_t)RecvBuffer[2])<<8)|RecvBuffer[3];
}

/**
 * @brief Write register through SPI.
 * @param RegAddr: The register address.
 * @param RegData: The register data.
 * @return Return None.
**/
static void AD5940_SPIWriteReg(uint16_t RegAddr, uint32_t RegData)
{  
  /* Set register address */
  AD5940_CsClr();
  AD5940_ReadWrite8B(SPICMD_SETADDR);
  AD5940_ReadWrite16B(RegAddr);
  AD5940_CsSet();
  /* Add delay here to meet the SPI timing. */
  AD5940_CsClr();
  AD5940_ReadWrite8B(SPICMD_WRITEREG);
  if(((RegAddr>=0x1000)&&(RegAddr<=0x3014)))
    AD5940_ReadWrite32B(RegData);
  else
    AD5940_ReadWrite16B(RegData);
  AD5940_CsSet();
}

/**
 * @brief Read register through SPI.
 * @param RegAddr: The register address.
 * @return Return register data.
**/
static uint32_t AD5940_SPIReadReg(uint16_t RegAddr)
{  
  uint32_t Data = 0;
  /* Set register address that we want to read */
  AD5940_CsClr();
  AD5940_ReadWrite8B(SPICMD_SETADDR);
  AD5940_ReadWrite16B(RegAddr);
  AD5940_CsSet();
  /* Read it */
  AD5940_CsClr();
  AD5940_ReadWrite8B(SPICMD_READREG);
  AD5940_ReadWrite8B(0);  //Dummy read
  /* The real data is coming */
  if((RegAddr>=0x1000)&&(RegAddr<=0x3014))
    Data = AD5940_ReadWrite32B(0);
  else
    Data = AD5940_ReadWrite16B(0);
  AD5940_CsSet();
  return Data;
}

/**
  @brief Read specific number of data from FIFO with optimized SPI access.
  @param pBuffer: Pointer to a buffer that used to store data read back.
  @param uiReadCount: How much data to be read.
  @return none.
**/
void AD5940_FIFORd(uint32_t *pBuffer, uint32_t uiReadCount)   
{
  /* Use function AD5940_SPIReadReg to read REG_AFE_DATAFIFORD is also one method. */
   uint32_t i;
   
   if(uiReadCount < 3)
   {
      /* This method is more efficient when readcount < 3 */
      uint32_t i;
      AD5940_CsClr();
      AD5940_ReadWrite8B(SPICMD_SETADDR);
      AD5940_ReadWrite16B(REG_AFE_DATAFIFORD);
      AD5940_CsSet();
      for(i=0;i<uiReadCount;i++)
      {
         AD5940_CsClr();
         AD5940_ReadWrite8B(SPICMD_READREG);
         AD5940_ReadWrite8B(0);//Write Host status/Don't care
         pBuffer[i] = AD5940_ReadWrite32B(0);
         AD5940_CsSet();
      }
   }
   else
   {
      AD5940_CsClr();
      AD5940_ReadWrite8B(SPICMD_READFIFO);
      /* 6 dummy write before valid data read back */
      for(i=0;i<6;i++)
         AD5940_ReadWrite8B(0);
      /* Continuously read DATAFIFORD register with offset 0 */
      for(i=0;i<uiReadCount-2;i++)
      {
         pBuffer[i] = AD5940_ReadWrite32B(0); /*Offset is 0, so we always read DATAFIFORD register */
      }
      /* Read back last two FIFO data with none-zero offset*/
      pBuffer[i++] = AD5940_ReadWrite32B(0x44444444);
      pBuffer[i] = AD5940_ReadWrite32B(0x44444444);
      AD5940_CsSet();
   }
}

/**
 * @} SPI_Block_Functions
 * @} SPI_Block
*/
#endif

/**
 * @brief Write register. If sequencer generator is enabled, the register write is recorded. 
 *        Otherwise, the data is written to AD5940 by SPI.
 * @param RegAddr: The register address.
 * @param RegData: The register data.
 * @return Return None.
**/
void AD5940_WriteReg(uint16_t RegAddr, uint32_t RegData)
{
#ifdef SEQUENCE_GENERATOR
  if(SeqGenDB.EngineStart == bTRUE)
    AD5940_SEQWriteReg(RegAddr, RegData);
  else
#endif
#ifdef CHIPSEL_M355
    AD5940_D2DWriteReg(RegAddr, RegData);
#else
    AD5940_SPIWriteReg(RegAddr, RegData);
#endif
}

/**
 * @brief Read register. If sequencer generator is enabled, read current register value from data-base. 
 *        Otherwise, read register value by SPI.
 * @param RegAddr: The register address.
 * @return Return register value.
**/
uint32_t AD5940_ReadReg(uint16_t RegAddr)
{
#ifdef SEQUENCE_GENERATOR
  if(SeqGenDB.EngineStart == bTRUE)
    return AD5940_SEQReadReg(RegAddr);
  else
#endif
#ifdef CHIPSEL_M355
    return AD5940_D2DReadReg(RegAddr);
#else
    return AD5940_SPIReadReg(RegAddr);
#endif
}


/**
 * @defgroup AFE_Control 
 * @brief Some functions to control the whole AFE. They are top level switches.
 * @{
 *    @defgroup AFE_Control_Functions
 *    The top-level control functions for whole AFE perspective. 
 *    @details  This function set is used to control the whole AFE block by block. It's a top-level configuration.
 *              It's convenient when do initialization work with the functions called BLOCK**Cfg**. You can tune the parameters at run-time using more detailed
 *              functions from each block. rather than top-level functions where you need to configure all parameters.
 *    @{
*/

/**
 * @brief Initialize AD5940. This function must be called whenever there is reset(Software Reset or Hardware reset or Power up) happened.
 *        This function is used to put AD5940 to correct state.
 * @return return None
**/
void AD5940_Initialize(void)
{
  int i;
  /* Write following registers with its data sequentially whenever there is a reset happened. */
  const struct
  {
    uint16_t reg_addr;
    uint32_t reg_data;
  }RegTable[]=
  {
    {0x0908, 0x02c9},
    {0x0c08, 0x206C},
    {0x21F0, 0x0010},
#ifndef CHIPSEL_M355
    /* This is AD5940 */
    {0x0410, 0x02c9},
    {0x0A28, 0x0009},
#else
    /* This is ADuCM355 */
    {0x0410, 0x001a},
    {0x0A28, 0x0008},
#endif
    {0x238c, 0x0104},
    {0x0a04, 0x4859},
    {0x0a04, 0xF27B},
    {0x0a00, 0x8009},
    {0x22F0, 0x0000},
    //
    {0x2230, 0xDE87A5AF},
    {0x2250, 0x103F},
    {0x22B0, 0x203C},
    {0x2230, 0xDE87A5A0},
  };
  //initialize global variables
  SeqGenDB.SeqLen = 0;
  SeqGenDB.RegCount = 0;
  SeqGenDB.LastError = AD5940ERR_OK;
  SeqGenDB.EngineStart = bFALSE;
#ifndef CHIPSEL_M355
  AD5940_CsSet(); /* Pull high CS in case it's low */
#endif
  for(i=0; i<sizeof(RegTable)/sizeof(RegTable[0]); i++)
    AD5940_WriteReg(RegTable[i].reg_addr, RegTable[i].reg_data);
  i = AD5940_ReadReg(REG_AFECON_CHIPID);  
  if(i == 0x5501)
    bIsS2silicon = bTRUE;
  else if(i == 0x5502)  /* S3 chip-id is 0x5502. The is no difference with S2. */
    bIsS2silicon = bTRUE;
  else if(i == 0x5500)
    bIsS2silicon = bFALSE;
#ifdef ADI_DEBUG
  else
  {
    printf("CHIPID read error:0x%04x. AD5940 is not present?\n", i);
    while(1);
  }
#ifdef CHIPSEL_M355
  ADI_Print("This ADuCM355!\n");
#else
  ADI_Print("This AD594x!\n");
#endif
  ADI_Print("Note: Current Silicon is %s\n", bIsS2silicon?"S2":"S1");
  ADI_Print("AD5940LIB Version:v%d.%d.%d\n", AD5940LIB_VER_MAJOR, AD5940LIB_VER_MINOR, AD5940LIB_VER_PATCH);
#endif
}

/**
 * @brief Control most AFE digital and analog block within one register access.
 * @param AfeCtrlSet: A set of blocks that will be controlled select it from @ref AFECTRL_Const Below is two examples to use it.
 *        - AFECTRL_HPREFPWR: Control high power reference(bandgap).
 *        - AFECTRL_WG|AFECTRL_ADCPWR: The OR'ed control set. Control Waveform generator and ADC power.
 * @param State: Enable or disable selected control set signal. Select from @BoolFlag
 *        - bFALSE: Disable or power down selected block(s).
 *        - bTRUE:  Enable all selected block(s).
   @return return none.
*/
void AD5940_AFECtrlS(uint32_t AfeCtrlSet, BoolFlag State)
{
  /* Check parameters */
  uint32_t tempreg;
  tempreg = AD5940_ReadReg(REG_AFE_AFECON);
  if (State == bTRUE) {
    /* Clear bits to enable HPREF and ALDOLimit*/
    if (AfeCtrlSet & AFECTRL_HPREFPWR) {
        tempreg &= ~BITM_AFE_AFECON_HPREFDIS;
        AfeCtrlSet &= ~AFECTRL_HPREFPWR;
    }
    if(AfeCtrlSet & AFECTRL_ALDOLIMIT)
    {
      tempreg &= ~BITM_AFE_AFECON_ALDOILIMITEN;
      AfeCtrlSet &= ~AFECTRL_ALDOLIMIT;
    }
    tempreg |= AfeCtrlSet;
  }
  else
  {
    /* Set bits to Disable HPREF and ALDOLimit*/
    if(AfeCtrlSet & AFECTRL_HPREFPWR)
    {
        tempreg |= BITM_AFE_AFECON_HPREFDIS;
        AfeCtrlSet &= ~AFECTRL_HPREFPWR;
    }
    if(AfeCtrlSet & AFECTRL_ALDOLIMIT)
    {
      tempreg |= BITM_AFE_AFECON_ALDOILIMITEN;
      AfeCtrlSet &= ~AFECTRL_ALDOLIMIT;
    }
    tempreg &= ~AfeCtrlSet;
  }
  AD5940_WriteReg(REG_AFE_AFECON, tempreg);
}
/** When LP mode is enabled, some functions are under control of LPMODECON, rather than original registers.  */
/** @warning LPMODE is key protected, this function only takes effect after AD5940_LPModeEnS(bTRUE) */
/**
 * @brief For LP mode, use one register to control most AFE digital and analog block.
 * @details The parameter means the blocks. The selected block will be enabled. All others will be disabled.
 *          The method to enable/disable blocks are defined by register LPMODECON, either by clearing or setting bits.
 * @param EnSet: A set of blocks that will be enabled. Select it from @ref LPMODECTRL_Const. All others not selected in EnSet will be disabled.
 *        - LPMODECTRL_ALDOPWR|LPMODECTRL_HFOSCEN: Turn on ALDO and HFOSC, disable all others.
 *        - LPMODECTRL_ALL: Enable all blocks.
   @return return none.
*/
AD5940Err AD5940_LPModeCtrlS(uint32_t EnSet)
{
  /* Check parameters */
  uint32_t tempreg;
  uint32_t DisSet;    /* The blocks to be disabled */
  DisSet = LPMODECTRL_ALL & (~EnSet);
  tempreg = AD5940_ReadReg(REG_AFE_LPMODECON);
  /* Enable selected set */
  {
    /* Clear bits to enable HFOSC, HPREF, ALDO */
    if (EnSet & LPMODECTRL_HFOSCEN) {
        tempreg &= ~BITM_AFE_LPMODECON_HFOSCPD;
        EnSet &= ~LPMODECTRL_HFOSCEN;
    }
    if(EnSet & LPMODECTRL_HPREFPWR)
    {
      tempreg &= ~BITM_AFE_LPMODECON_HPREFDIS;
      EnSet &= ~LPMODECTRL_HPREFPWR;
    }
    if(EnSet & LPMODECTRL_ALDOPWR)
    {
      tempreg &= ~BITM_AFE_LPMODECON_ALDOEN;
      EnSet &= ~LPMODECTRL_ALDOPWR;
    }
    tempreg |= EnSet; /* Set other bits to enable function */
  }
  /* Disable other blocks */
  {
    /* Set bits to disable HFOSC, HPREF, ALDO */
    if (DisSet & LPMODECTRL_HFOSCEN) {
        tempreg |= BITM_AFE_LPMODECON_HFOSCPD;
        DisSet &= ~LPMODECTRL_HFOSCEN;
    }
    if(DisSet & LPMODECTRL_HPREFPWR)
    {
      tempreg |= BITM_AFE_LPMODECON_HPREFDIS;
      DisSet &= ~LPMODECTRL_HPREFPWR;
    }
    if(DisSet & LPMODECTRL_ALDOPWR)
    {
      tempreg |= BITM_AFE_LPMODECON_ALDOEN;
      DisSet &= ~LPMODECTRL_ALDOPWR;
    }
    tempreg &= ~DisSet; /* Clear other bits to disable function */
  }
  AD5940_WriteReg(REG_AFE_LPMODECON, tempreg);

  return AD5940ERR_OK;
}

/**
   @brief Set AFE power mode and system bandwidth include HSDAC, Excitation-buffer, HSTIA and ADC etc.
   @param AfePwr : {AFEPWR_LP, AFEPWR_HP}
          Select parameters from @ref AFEPWR_Const
          - AFEPWR_LP: Set AFE to low power mode
          - AFEPWR_HP: Set AFE to High speed mode to support 200kHz.
   @param AfeBw : {AFEBW_AUTOSET, AFEBW_50KHZ, AFEBW_100KHZ, AFEBW_250KHZ}
          - AFEBW_AUTOSET: Set the bandwidth automatically based on WGFCW frequency word.
          - AFEBW_50KHZ: Set system bandwidth to 50kHz.
          - AFEBW_100KHZ: Set system bandwidth to 100kHz.
          - AFEBW_250KHZ: Set system bandwidth to 250kHz.
   @return return none.
*/
void AD5940_AFEPwrBW(uint32_t AfePwr, uint32_t AfeBw)
{
  //check parameters
  uint32_t tempreg;
  tempreg = AfePwr;
  tempreg |= AfeBw << BITP_AFE_PMBW_SYSBW;
  AD5940_WriteReg(REG_AFE_PMBW, tempreg);
}

/**
   @brief Configure reference buffer include 1.8V/1.1V high/low power buffers.
   @param pBufCfg :Pointer to buffer configure structure;
   @return return none.
*/
void AD5940_REFCfgS(AFERefCfg_Type *pBufCfg)
{
  uint32_t tempreg;
  
  /* HP Reference(bandgap) */
  tempreg = AD5940_ReadReg(REG_AFE_AFECON);
  tempreg &= ~BITM_AFE_AFECON_HPREFDIS;
  if(pBufCfg->HpBandgapEn == bFALSE)
    tempreg |= BITM_AFE_AFECON_HPREFDIS;
  AD5940_WriteReg(REG_AFE_AFECON, tempreg);
  /* Reference buffer configure */
  tempreg = AD5940_ReadReg(REG_AFE_BUFSENCON);
  if(pBufCfg->Hp1V8BuffEn == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8HPADCEN;
  if(pBufCfg->Hp1V1BuffEn == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P1HPADCEN;
  if(pBufCfg->Lp1V8BuffEn == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8LPADCEN;
  if(pBufCfg->Lp1V1BuffEn == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P1LPADCEN;
  if(pBufCfg->Hp1V8ThemBuff == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8THERMSTEN;
  if(pBufCfg->Hp1V8Ilimit == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8HPADCILIMITEN;
  if(pBufCfg->Disc1V8Cap == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P8HPADCCHGDIS;
  if(pBufCfg->Disc1V1Cap == bTRUE)
    tempreg |= BITM_AFE_BUFSENCON_V1P1LPADCCHGDIS;
  AD5940_WriteReg(REG_AFE_BUFSENCON, tempreg);

  /* LPREFBUFCON */
  tempreg = 0;
  if(pBufCfg->LpRefBufEn == bFALSE)
    tempreg |= BITM_AFE_LPREFBUFCON_LPBUF2P5DIS;
  if(pBufCfg->LpBandgapEn == bFALSE)
    tempreg |= BITM_AFE_LPREFBUFCON_LPREFDIS;
  if(pBufCfg->LpRefBoostEn == bTRUE)
    tempreg |= BITM_AFE_LPREFBUFCON_BOOSTCURRENT;
  AD5940_WriteReg(REG_AFE_LPREFBUFCON, tempreg);
}
/**
 * @} End of AFE_Control_Functions
 * @} End of AFE_Control
 * */

/**
 * @defgroup High_Speed_Loop
 * @brief The high speed loop
 * @{
 *    @defgroup High_Speed_Loop_Functions
 *    @{
*/

/**
   @brief Configure High speed loop(high bandwidth loop or 
          called excitation loop). This configuration includes HSDAC, HSTIA and Switch matrix. 
   @param pHsLoopCfg : Pointer to configure structure;
   @return return none.
*/
void AD5940_HSLoopCfgS(HSLoopCfg_Type *pHsLoopCfg)
{
  AD5940_HSDacCfgS(&pHsLoopCfg->HsDacCfg);
  AD5940_HSTIACfgS(&pHsLoopCfg->HsTiaCfg);
  AD5940_SWMatrixCfgS(&pHsLoopCfg->SWMatCfg);
  AD5940_WGCfgS(&pHsLoopCfg->WgCfg);
}

/**
   @brief Initialize switch matrix
   @param pSwMatrix: Pointer to configuration structure
   @return return none.
*/
void AD5940_SWMatrixCfgS(SWMatrixCfg_Type *pSwMatrix)
{
  AD5940_WriteReg(REG_AFE_DSWFULLCON, pSwMatrix->Dswitch);
  AD5940_WriteReg(REG_AFE_PSWFULLCON, pSwMatrix->Pswitch);
  AD5940_WriteReg(REG_AFE_NSWFULLCON, pSwMatrix->Nswitch);
  AD5940_WriteReg(REG_AFE_TSWFULLCON, pSwMatrix->Tswitch);
  AD5940_WriteReg(REG_AFE_SWCON, BITM_AFE_SWCON_SWSOURCESEL); /* Update switch configuration */
}

/**
   @brief Initialize HSDAC
   @param pHsDacCfg: Pointer to configuration structure
   @return return none.
*/
void AD5940_HSDacCfgS(HSDACCfg_Type *pHsDacCfg)
{
  uint32_t tempreg;
  //Check parameters
  tempreg = 0;
  if(pHsDacCfg->ExcitBufGain == EXCITBUFGAIN_0P25)
    tempreg |= BITM_AFE_HSDACCON_INAMPGNMDE; /* Enable attenuator */
  if(pHsDacCfg->HsDacGain == HSDACGAIN_0P2)
    tempreg |= BITM_AFE_HSDACCON_ATTENEN; /* Enable attenuator */
  tempreg |= (pHsDacCfg->HsDacUpdateRate&0xff)<<BITP_AFE_HSDACCON_RATE;
  AD5940_WriteReg(REG_AFE_HSDACCON, tempreg);
}


static void __AD5940_SetDExRTIA(uint32_t DExPin, uint32_t DeRtia, uint32_t DeRload)
{
  uint32_t tempreg;
  /* deal with HSTIA DE RTIA */
  if(DeRtia >= HSTIADERTIA_OPEN)
    tempreg = 0x1f << 3;  /* bit field HPTIRES03CON[7:3] */
  else if(DeRtia >= HSTIADERTIA_1K)
  {
    tempreg = (DeRtia - 3 + 11) << 3;
  }
  else  /* DERTIA 50/100/200Ohm */
  {
    const uint8_t DeRtiaTable[3][5] = 
    {
//Rload  0      10    30    50    100 
			{0x00, 0x01, 0x02, 0x03, 0x06}, /* RTIA 50Ohm */
			{0x03, 0x04, 0x05, 0x06, 0x07}, /* RTIA 100Ohm */
			{0x07, 0x07, 0x09, 0x09, 0x0a}, /* RTIA 200Ohm */
    };
    if(DeRload < HSTIADERLOAD_OPEN)
      tempreg = (uint32_t)(DeRtiaTable[DeRtia][DeRload])<<3;
    else
      tempreg = (0x1f)<<3;  /* Set it to HSTIADERTIA_OPEN. This setting is illegal */
  }
  /* deal with HSTIA Rload */
  tempreg |= DeRload;
  if(DExPin) //DE1
    AD5940_WriteReg(REG_AFE_DE1RESCON, tempreg);
  else  //DE0
    AD5940_WriteReg(REG_AFE_DE0RESCON, tempreg);
}

/**
   @brief Initialize High speed TIA amplifier
   @param pHsTiaCfg: Pointer to configuration structure
   @return return none.
*/
AD5940Err AD5940_HSTIACfgS(HSTIACfg_Type *pHsTiaCfg)
{
  uint32_t tempreg;
  //Check parameters
  if(pHsTiaCfg == NULL) return AD5940ERR_NULLP;
    /* Available parameter is 1k, 5k,...,160k, short, OPEN */
  if(pHsTiaCfg->HstiaDeRtia < HSTIADERTIA_1K)
    return AD5940ERR_PARA;
  if(pHsTiaCfg->HstiaDeRtia > HSTIADERTIA_OPEN)
    return AD5940ERR_PARA;  /* Parameter is invalid */

  if(pHsTiaCfg->HstiaDeRload > HSTIADERLOAD_OPEN)
    return AD5940ERR_PARA;  /* Available parameter is OPEN, 0R,..., 100R */

  tempreg = 0;
  tempreg |= pHsTiaCfg->HstiaBias;
  AD5940_WriteReg(REG_AFE_HSTIACON, tempreg);
  /* HSRTIACON */
  /* Calculate CTIA value */
  tempreg = pHsTiaCfg->HstiaCtia << BITP_AFE_HSRTIACON_CTIACON;
  tempreg |= pHsTiaCfg->HstiaRtiaSel;
  if(pHsTiaCfg->DiodeClose == bTRUE)
    tempreg |= BITM_AFE_HSRTIACON_TIASW6CON; /* Close switch 6 */
  AD5940_WriteReg(REG_AFE_HSRTIACON, tempreg);
  /* DExRESCON */
  __AD5940_SetDExRTIA(0, pHsTiaCfg->HstiaDeRtia, pHsTiaCfg->HstiaDeRload);
#ifdef CHIPSEL_M355
  __AD5940_SetDExRTIA(1, pHsTiaCfg->HstiaDe1Rtia, pHsTiaCfg->HstiaDe1Rload);
#endif

  /* Done */
  return AD5940ERR_OK;
}
/**
 * @brief Configure HSTIA RTIA resistor and keep other parameters unchanged.
 * @param HSTIARtia: The RTIA setting, select it from @ref HSTIARTIA_Const
 * @return return none.
*/
void AD5940_HSRTIACfgS(uint32_t HSTIARtia)
{
  uint32_t tempreg;
  tempreg = AD5940_ReadReg(REG_AFE_HSRTIACON);
  tempreg &= ~BITM_AFE_HSRTIACON_RTIACON;
  HSTIARtia &= BITM_AFE_HSRTIACON_RTIACON;
  tempreg |= HSTIARtia<<BITP_AFE_HSRTIACON_RTIACON;
  AD5940_WriteReg(REG_AFE_HSRTIACON, tempreg);
}

/**
 * @defgroup Waveform_Generator_Functions
 * @{
*/
/**
 * @brief Initialize waveform generator
 * @param pWGInit: Pointer to configuration structure
 * @return return none.
*/
void AD5940_WGCfgS(WGCfg_Type *pWGInit)
{
  //Check parameters
  uint32_t tempreg;
  if(pWGInit->WgType == WGTYPE_SIN)
  {
    /* Configure Sine wave Generator */
    AD5940_WriteReg(REG_AFE_WGFCW, pWGInit->SinCfg.SinFreqWord);
    AD5940_WriteReg(REG_AFE_WGAMPLITUDE, pWGInit->SinCfg.SinAmplitudeWord);
    AD5940_WriteReg(REG_AFE_WGOFFSET, pWGInit->SinCfg.SinOffsetWord);
    AD5940_WriteReg(REG_AFE_WGPHASE, pWGInit->SinCfg.SinPhaseWord);
  }
  else if(pWGInit->WgType == WGTYPE_TRAPZ)
  {
    /* Configure Trapezoid Generator */
    AD5940_WriteReg(REG_AFE_WGDCLEVEL1, pWGInit->TrapzCfg.WGTrapzDCLevel1);
    AD5940_WriteReg(REG_AFE_WGDCLEVEL2, pWGInit->TrapzCfg.WGTrapzDCLevel2);
    AD5940_WriteReg(REG_AFE_WGDELAY1, pWGInit->TrapzCfg.WGTrapzDelay1);
    AD5940_WriteReg(REG_AFE_WGDELAY2, pWGInit->TrapzCfg.WGTrapzDelay2);
    AD5940_WriteReg(REG_AFE_WGSLOPE1, pWGInit->TrapzCfg.WGTrapzSlope1);
    AD5940_WriteReg(REG_AFE_WGSLOPE2, pWGInit->TrapzCfg.WGTrapzSlope2);
  }
  else
  {
    /* Write DAC data. It's only have effect when WgType set to WGTYPE_MMR */ 
    AD5940_WriteReg(REG_AFE_HSDACDAT, pWGInit->WgCode);
  }
  tempreg = 0;
  
  if(pWGInit->GainCalEn == bTRUE)
    tempreg |= BITM_AFE_WGCON_DACGAINCAL;
  if(pWGInit->OffsetCalEn == bTRUE)
    tempreg |= BITM_AFE_WGCON_DACOFFSETCAL;
  tempreg |= (pWGInit->WgType) << BITP_AFE_WGCON_TYPESEL;
  AD5940_WriteReg(REG_AFE_WGCON, tempreg);
}

/**
 * @brief Write HSDAC code directly when WG configured to MMR type
 * @param code: The 12-bit HSDAC code.
 * @return return none.
*/
AD5940Err AD5940_WGDACCodeS(uint32_t code)
{
  code &= 0xfff;
  AD5940_WriteReg(REG_AFE_HSDACDAT, code);
  return AD5940ERR_OK;
}

/**
 * @brief Update WG SIN wave frequency in Hz.
 * @param SinFreqHz: The desired frequency in Hz.
 * @param WGClock: The clock for WG. It's same as system clock and the default value is internal 16MHz HSOSC.
 * @return return none.
*/
void AD5940_WGFreqCtrlS(float SinFreqHz, float WGClock)
{
  uint32_t freq_word;
  freq_word = AD5940_WGFreqWordCal(SinFreqHz, WGClock);
  AD5940_WriteReg(REG_AFE_WGFCW, freq_word);
}

/**
   @brief Calculate sine wave generator frequency word. The maxim frequency is 250kHz-1LSB
   @param SinFreqHz : Target frequency in Hz unit.
   @param WGClock: Waveform generator clock frequency in Hz unit. The clock is sourced from system clock, default value is 16MHz HFOSC.
   @return return none.
*/
uint32_t AD5940_WGFreqWordCal(float SinFreqHz, float WGClock)
{
  uint32_t temp;
  uint32_t __BITWIDTH_WGFCW = 26;
  if(bIsS2silicon == bTRUE)
    __BITWIDTH_WGFCW = 30;
  if(WGClock == 0) return 0;
  temp = (uint32_t)(SinFreqHz*(1LL<<__BITWIDTH_WGFCW)/WGClock + 0.5f);
  if(temp > ((__BITWIDTH_WGFCW == 26)?0xfffff:0xffffff))
    temp = (__BITWIDTH_WGFCW == 26)?0xfffff:0xffffff;
  
  return temp;
}

/**
 * @} Waveform_Generator_Functions
 * @} High_Speed_Loop_Functions
 * @} High_Speed_Loop
*/


/**
 * @defgroup Low_Power_Loop
 * @brief The low power loop.
 * @{
 *    @defgroup Low_Power_Loop_Functions
 *    @{
*/

/**
   @brief Configure low power loop include LPDAC LPAmp(PA and TIA)
   @param pLpLoopCfg: Pointer to configure structure;
   @return return none.
*/
void AD5940_LPLoopCfgS(LPLoopCfg_Type *pLpLoopCfg)
{
  AD5940_LPDACCfgS(&pLpLoopCfg->LpDacCfg);
  AD5940_LPAMPCfgS(&pLpLoopCfg->LpAmpCfg);
}

/**
   @brief Initialize LPDAC
   @param pLpDacCfg: Pointer to configuration structure
   @return return none.
*/
void AD5940_LPDACCfgS(LPDACCfg_Type *pLpDacCfg)
{
  uint32_t tempreg;
  tempreg = 0;
  tempreg = (pLpDacCfg->LpDacSrc)<<BITP_AFE_LPDACCON0_WAVETYPE;
  tempreg |= (pLpDacCfg->LpDacVzeroMux)<<BITP_AFE_LPDACCON0_VZEROMUX;
  tempreg |= (pLpDacCfg->LpDacVbiasMux)<<BITP_AFE_LPDACCON0_VBIASMUX;
  tempreg |= (pLpDacCfg->LpDacRef)<<BITP_AFE_LPDACCON0_REFSEL;
  if(pLpDacCfg->DataRst == bFALSE)
    tempreg |= BITM_AFE_LPDACCON0_RSTEN;
  if(pLpDacCfg->PowerEn == bFALSE)
    tempreg |= BITM_AFE_LPDACCON0_PWDEN;
  if(pLpDacCfg->LpdacSel == LPDAC0)
  {
    AD5940_WriteReg(REG_AFE_LPDACCON0, tempreg);
    AD5940_LPDAC0WriteS(pLpDacCfg->DacData12Bit, pLpDacCfg->DacData6Bit);
    AD5940_WriteReg(REG_AFE_LPDACSW0, pLpDacCfg->LpDacSW|BITM_AFE_LPDACSW0_LPMODEDIS);  /* Overwrite LPDACSW settings. On Si1, this register is not accessible. */
  }
  else
  {
    AD5940_WriteReg(REG_AFE_LPDACCON1, tempreg);
    AD5940_LPDAC1WriteS(pLpDacCfg->DacData12Bit, pLpDacCfg->DacData6Bit);
    AD5940_WriteReg(REG_AFE_LPDACSW1, pLpDacCfg->LpDacSW|BITM_AFE_LPDACSW0_LPMODEDIS);  /* Overwrite LPDACSW settings. On Si1, this register is not accessible. */
  }
}

/**
   @brief Write LPDAC data
   @param Data12Bit: 12Bit DAC data
   @param Data6Bit: 6Bit DAC data
   @return return none.
*/
void AD5940_LPDACWriteS(uint16_t Data12Bit, uint8_t Data6Bit)
{
  /* Check parameter */
  Data6Bit &= 0x3f;
  Data12Bit &= 0xfff;
  AD5940_WriteReg(REG_AFE_LPDACDAT0, ((uint32_t)Data6Bit<<12)|Data12Bit);
}

/**
   @brief Write LPDAC0 data
   @param Data12Bit: 12Bit DAC data
   @param Data6Bit: 6Bit DAC data
   @return return none.
*/
void AD5940_LPDAC0WriteS(uint16_t Data12Bit, uint8_t Data6Bit)
{
  /* Check parameter */
  Data6Bit &= 0x3f;
  Data12Bit &= 0xfff;
  AD5940_WriteReg(REG_AFE_LPDACDAT0, ((uint32_t)Data6Bit<<12)|Data12Bit);
}

/**
   @brief Write LPDAC1 data
   @param Data12Bit: 12Bit DAC data
   @param Data6Bit: 6Bit DAC data
   @return return none.
*/
void AD5940_LPDAC1WriteS(uint16_t Data12Bit, uint8_t Data6Bit)
{
  /* Check parameter */
  Data6Bit &= 0x3f;
  Data12Bit &= 0xfff;
  AD5940_WriteReg(REG_AFE_LPDACDAT1, ((uint32_t)Data6Bit<<12)|Data12Bit);
}

/**
   @brief Initialize LP TIA and PA
   @param pLpAmpCfg: Pointer to configuration structure
   @return return none.
*/
void AD5940_LPAMPCfgS(LPAmpCfg_Type *pLpAmpCfg)
{
  //check parameters
  uint32_t tempreg;

  tempreg = 0;
  if(pLpAmpCfg->LpPaPwrEn == bFALSE)
    tempreg |= BITM_AFE_LPTIACON0_PAPDEN; 
  if(pLpAmpCfg->LpTiaPwrEn == bFALSE)
    tempreg |= BITM_AFE_LPTIACON0_TIAPDEN;
  if(pLpAmpCfg->LpAmpPwrMod == LPAMPPWR_HALF) 
    tempreg |= BITM_AFE_LPTIACON0_HALFPWR;
  else
  {
    tempreg |= pLpAmpCfg->LpAmpPwrMod<<BITP_AFE_LPTIACON0_IBOOST;
  }
  tempreg |= pLpAmpCfg->LpTiaRtia<<BITP_AFE_LPTIACON0_TIAGAIN;
  tempreg |= pLpAmpCfg->LpTiaRload<<BITP_AFE_LPTIACON0_TIARL;
  tempreg |= pLpAmpCfg->LpTiaRf<<BITP_AFE_LPTIACON0_TIARF;
  if(pLpAmpCfg->LpAmpSel == LPAMP0)
  {
    AD5940_WriteReg(REG_AFE_LPTIACON0, tempreg);
    AD5940_WriteReg(REG_AFE_LPTIASW0, pLpAmpCfg->LpTiaSW);
  }
  else
  {
    AD5940_WriteReg(REG_AFE_LPTIACON1, tempreg);
    AD5940_WriteReg(REG_AFE_LPTIASW1, pLpAmpCfg->LpTiaSW);
  }
}
/**
 * @} Low_Power_Loop_Functions
 * @} Low_Power_Loop
*/


/**
 * @defgroup DSP_Block
 * @brief DSP block includes ADC, filters, DFT and statistic functions. 
 * @{
 *    @defgroup DSP_Block_Functions
 *    @{
 * */

/**
   @brief Configure low power loop include LPDAC LPAmp(PA and TIA)
   @param pDSPCfg: Pointer to configure structure;
   @return return none.
*/
void AD5940_DSPCfgS(DSPCfg_Type *pDSPCfg)
{
  AD5940_ADCBaseCfgS(&pDSPCfg->ADCBaseCfg);
  AD5940_ADCFilterCfgS(&pDSPCfg->ADCFilterCfg);
  AD5940_ADCDigCompCfgS(&pDSPCfg->ADCDigCompCfg);
  AD5940_DFTCfgS(&pDSPCfg->DftCfg);
  AD5940_StatisticCfgS(&pDSPCfg->StatCfg);
}

/**
   @brief Read AD5940 generated data like ADC and DFT etc.
   @param AfeResultSel: available parameters are @ref AFERESULT_Const
          - AFERESULT_SINC3: Read SINC3 filter data result
          - AFERESULT_SINC2: Read SINC2+NOTCH filter result, when Notch filter is bypassed, the result is SINC2
          - AFERESULT_STATSVAR: Statistic variance result
   @return return data read back.
*/
uint32_t AD5940_ReadAfeResult(uint32_t AfeResultSel)
{
  uint32_t rd = 0;
  //PARA_CHECK((AfeResultSel));
  switch (AfeResultSel)
  {
    case AFERESULT_SINC3:
      rd = AD5940_ReadReg(REG_AFE_ADCDAT);
      break;
    case AFERESULT_SINC2:
      rd = AD5940_ReadReg(REG_AFE_SINC2DAT);
      break;
    case AFERESULT_TEMPSENSOR:
      rd = AD5940_ReadReg(REG_AFE_TEMPSENSDAT);
      break;
    case AFERESULT_DFTREAL:
      rd = AD5940_ReadReg(REG_AFE_DFTREAL);
      break;
    case AFERESULT_DFTIMAGE:
      rd = AD5940_ReadReg(REG_AFE_DFTIMAG);
      break;
    case AFERESULT_STATSMEAN:
      rd = AD5940_ReadReg(REG_AFE_STATSMEAN);
      break;
    case AFERESULT_STATSVAR:
      rd = AD5940_ReadReg(REG_AFE_STATSVAR);
      break;
  }
  
  return rd;
}

/**
 *  @defgroup ADC_Block_Functions
 *  @{
*/

/**
   @brief Initializes ADC peripheral according to the specified parameters in the pADCInit.
   @param pADCInit: Pointer to ADC initialize structure.
   @return return none.
*/
void AD5940_ADCBaseCfgS(ADCBaseCfg_Type *pADCInit)
{
  uint32_t tempreg = 0;
  //PARA_CHECK(IS_ADCMUXP(pADCInit->ADCMuxP));
  //PARA_CHECK(IS_ADCMUXN(pADCInit->ADCMuxN));
  PARA_CHECK(IS_ADCPGA(pADCInit->ADCPga));
  PARA_CHECK(IS_ADCAAF(pADCInit->ADCAAF));

  tempreg = pADCInit->ADCMuxP;
  tempreg |= (uint32_t)(pADCInit->ADCMuxN)<<BITP_AFE_ADCCON_MUXSELN;
  //if(pADCInit->OffCancEnable == bTRUE)
  //  tempreg |= BITM_AFE_ADCCON_GNOFSELPGA;
  tempreg |= (uint32_t)(pADCInit->ADCPga)<<BITP_AFE_ADCCON_GNPGA;

  AD5940_WriteReg(REG_AFE_ADCCON, tempreg);
}

/**
   @brief Initializes ADC filter according to the specified parameters in the pFiltCfg.
   @param pFiltCfg: Pointer to filter initialize structure.
   @return return none.
*/
void AD5940_ADCFilterCfgS(ADCFilterCfg_Type *pFiltCfg)
{
  uint32_t tempreg;
  PARA_CHECK(IS_ADCSINC3OSR(pFiltCfg->ADCSinc3Osr));
  PARA_CHECK(IS_ADCSINC2OSR(pFiltCfg->ADCSinc2Osr));
  PARA_CHECK(IS_ADCAVGNUM(pFiltCfg->ADCAvgNum));
  PARA_CHECK(IS_ADCRATE(pFiltCfg->ADCRate));

  tempreg = AD5940_ReadReg(REG_AFE_ADCFILTERCON);
  tempreg &= BITM_AFE_ADCFILTERCON_AVRGEN; /* Keep this bit setting. */

  tempreg |= pFiltCfg->ADCRate;
  if(pFiltCfg->BpNotch == bTRUE)
    tempreg |= BITM_AFE_ADCFILTERCON_LPFBYPEN;
  if(pFiltCfg->BpSinc3 == bTRUE)
    tempreg |= BITM_AFE_ADCFILTERCON_SINC3BYP;
  /**
   * Average filter is enabled when DFT source is @ref DFTSRC_AVG in function @ref AD5940_DFTCfgS.
   * Once average function is enabled, it's automatically set as DFT source, register DFTCON.DFTINSEL is ignored.
   */
  //if(pFiltCfg->AverageEnable == bTRUE)
  //  tempreg |= BITM_AFE_ADCFILTERCON_AVRGEN;
  tempreg |= (uint32_t)(pFiltCfg->ADCSinc2Osr)<<BITP_AFE_ADCFILTERCON_SINC2OSR;
  tempreg |= (uint32_t)(pFiltCfg->ADCSinc3Osr)<<BITP_AFE_ADCFILTERCON_SINC3OSR;
  tempreg |= (uint32_t)(pFiltCfg->ADCAvgNum)<<BITP_AFE_ADCFILTERCON_AVRGNUM;

  AD5940_WriteReg(REG_AFE_ADCFILTERCON, tempreg);

  /* SINC2+Notch has a block enable/disable bit in AFECON register */
  if(pFiltCfg->Sinc2NotchEnable)
  {
    AD5940_AFECtrlS(AFECTRL_SINC2NOTCH,bTRUE);
  }
}

/**
   @brief Power up or power down ADC block(including ADC PGA and FRONTBUF).
   @param State : {bTRUE, bFALSE}
          - bTRUE: Power up ADC
          - bFALSE: Power down ADC
   @return return none.
*/
void AD5940_ADCPowerCtrlS(BoolFlag State)
{
  uint32_t tempreg;
  tempreg = AD5940_ReadReg(REG_AFE_AFECON);
  if(State == bTRUE)
  {
    tempreg |= BITM_AFE_AFECON_ADCEN;
  }
  else
  {
    tempreg &= ~BITM_AFE_AFECON_ADCEN;
  }
  AD5940_WriteReg(REG_AFE_AFECON,tempreg);
}

/**
   @brief Start or stop ADC convert.
   @param State : {bTRUE, bFALSE}
          - bTRUE: Start ADC convert
          - bFALSE: Stop ADC convert
   @return return none.
*/
void AD5940_ADCConvtCtrlS(BoolFlag State)
{
  uint32_t tempreg;
  tempreg = AD5940_ReadReg(REG_AFE_AFECON);
  if(State == bTRUE)
  {
    tempreg |= BITM_AFE_AFECON_ADCCONVEN;
  }
  else
  {
    tempreg &= ~BITM_AFE_AFECON_ADCCONVEN;
  }
  AD5940_WriteReg(REG_AFE_AFECON,tempreg);
}

/**
   @brief Configure ADC input MUX
   @param ADCMuxP : {ADCMUXP_FLOAT, ADCMUXP_HSTIA_P, ,,, ,ADCMUXP_P_NODE}
          - ADCMUXP_FLOAT: float ADC MUX positive input
          - ADCMUXP_HSTIA_P: High speed TIA output sense terminal
          - ADCMUXP_P_NODE: Excitation loop P node
   @param ADCMuxN : {ADCMUXP_FLOAT, ADCMUXP_HSTIA_P, ,,, ,ADCMUXP_P_NODE}
          - ADCMUXP_FLOAT: float ADC MUX positive input
          - ADCMUXP_HSTIA_P: High speed TIA output sense terminal
          - ADCMUXP_P_NODE: Excitation loop P node

   @return return none.
*/
void AD5940_ADCMuxCfgS(uint32_t ADCMuxP, uint32_t ADCMuxN)
{
  uint32_t tempreg;
  //PARA_CHECK(IS_ADCMUXP(ADCMuxP));
  //PARA_CHECK(IS_ADCMUXN(ADCMuxN));
  
  tempreg = AD5940_ReadReg(REG_AFE_ADCCON);
  tempreg &= ~(BITM_AFE_ADCCON_MUXSELN|BITM_AFE_ADCCON_MUXSELP);
  tempreg |= ADCMuxP<<BITP_AFE_ADCCON_MUXSELP;
  tempreg |= ADCMuxN<<BITP_AFE_ADCCON_MUXSELN;
  AD5940_WriteReg(REG_AFE_ADCCON, tempreg);
}

/**
   @brief Set ADC digital comparator function
   @param pCompCfg: Pointer to configuration structure
   @return return none.
*/
void AD5940_ADCDigCompCfgS(ADCDigComp_Type *pCompCfg)
{
  //PARA_CHECK((AfeResultSel));
  AD5940_WriteReg(REG_AFE_ADCMIN, pCompCfg->ADCMin);
  AD5940_WriteReg(REG_AFE_ADCMINSM, pCompCfg->ADCMinHys);
  AD5940_WriteReg(REG_AFE_ADCMAX, pCompCfg->ADCMax);
  AD5940_WriteReg(REG_AFE_ADCMAXSMEN, pCompCfg->ADCMaxHys);
}
/** @} ADC_Block_Functions */

/**
   @brief Configure statistic functions
   @param pStatCfg: Pointer to configuration structure
   @return return none.
*/
void AD5940_StatisticCfgS(StatCfg_Type *pStatCfg)
{
  uint32_t tempreg;
  //check parameters
  tempreg = 0;
  if(pStatCfg->StatEnable == bTRUE)
    tempreg |= BITM_AFE_STATSCON_STATSEN;
  tempreg |= (pStatCfg->StatSample) << BITP_AFE_STATSCON_SAMPLENUM;
  tempreg |= (pStatCfg->StatDev) << BITP_AFE_STATSCON_STDDEV;
  AD5940_WriteReg(REG_AFE_STATSCON, tempreg);
}

/**
 * @brief Set ADC Repeat convert function number. Turn off ADC automatically after Number samples of ADC raw data are ready
 * @param Number: Specify after how much ADC raw data need to sample before shutdown ADC
 * @return return none.
*/
void AD5940_ADCRepeatCfgS(uint32_t Number)
{
  //check parameter if(number<255)
  AD5940_WriteReg(REG_AFE_REPEATADCCNV, Number<<BITP_AFE_REPEATADCCNV_NUM);
}

/**
   @brief Configure DFT number and source and hanning window
   @param pDftCfg: Pointer to configuration structure
   @return return none.
*/
void AD5940_DFTCfgS(DFTCfg_Type *pDftCfg)
{
  uint32_t reg_dftcon, reg_adcfilter;

  reg_dftcon = 0;
  /* Deal with DFTSRC_AVG. Once average function is enabled, it's automatically set as DFT source */
  reg_adcfilter = AD5940_ReadReg(REG_AFE_ADCFILTERCON);
  if(pDftCfg->DftSrc == DFTSRC_AVG)
  {
    reg_adcfilter |= BITM_AFE_ADCFILTERCON_AVRGEN;
    AD5940_WriteReg(REG_AFE_ADCFILTERCON, reg_adcfilter);
  }
  else
  {
    /* Disable Average function and set correct DFT source */
    reg_adcfilter &= ~BITM_AFE_ADCFILTERCON_AVRGEN;
    AD5940_WriteReg(REG_AFE_ADCFILTERCON, reg_adcfilter);

    /* Set new DFT source */
    reg_dftcon |= (pDftCfg->DftSrc) << BITP_AFE_DFTCON_DFTINSEL;
  }
  /* Set DFT number */
  reg_dftcon |= (pDftCfg->DftNum) << BITP_AFE_DFTCON_DFTNUM;
  
  if(pDftCfg->HanWinEn == bTRUE)
    reg_dftcon |= BITM_AFE_DFTCON_HANNINGEN;
  AD5940_WriteReg(REG_AFE_DFTCON, reg_dftcon);
}

/**
 * @} DSP_Block_Functions
 * @} DSP_Block
*/

/**
 * @defgroup Sequencer_FIFO
 * @brief Sequencer and FIFO.
 * @{
 *    @defgroup Sequencer_FIFO_Functions
 *    @{
*/

/**
   @brief Configure AD5940 FIFO
   @param pFifoCfg: Pointer to configuration structure.
   @return return none.
*/
void AD5940_FIFOCfg(FIFOCfg_Type *pFifoCfg)
{
  uint32_t tempreg;
  //check parameters
  AD5940_WriteReg(REG_AFE_FIFOCON, 0);  /* Disable FIFO firstly! */
  /* CMDDATACON register. Configure this firstly */
  tempreg = AD5940_ReadReg(REG_AFE_CMDDATACON);
  tempreg &= BITM_AFE_CMDDATACON_CMD_MEM_SEL|BITM_AFE_CMDDATACON_CMDMEMMDE; /* Keep sequencer memory settings */
  tempreg |= pFifoCfg->FIFOMode << BITP_AFE_CMDDATACON_DATAMEMMDE; 				  /* Data FIFO mode: stream or FIFO */
  tempreg |= pFifoCfg->FIFOSize << BITP_AFE_CMDDATACON_DATA_MEM_SEL;  		  /* Data FIFO memory size */
  /* The reset memory can be used for sequencer, configure it by function AD5940_SEQCfg() */
  AD5940_WriteReg(REG_AFE_CMDDATACON, tempreg);

  /* FIFO Threshold */
  AD5940_WriteReg(REG_AFE_DATAFIFOTHRES, pFifoCfg->FIFOThresh << BITP_AFE_DATAFIFOTHRES_HIGHTHRES);
  /* FIFOCON register. Final step is to enable FIFO */
  tempreg = 0;
  if(pFifoCfg->FIFOEn == bTRUE)
    tempreg |= BITM_AFE_FIFOCON_DATAFIFOEN;																/* Enable FIFO after everything set. */
  tempreg |= pFifoCfg->FIFOSrc << BITP_AFE_FIFOCON_DATAFIFOSRCSEL;
  AD5940_WriteReg(REG_AFE_FIFOCON, tempreg);
}

/**
   @brief Read current FIFO configuration.
   @param pFifoCfg: Pointer to a buffer that used to store FIFO configuration.
   @return return AD5940ERR_OK if succeed.
*/
AD5940Err AD5940_FIFOGetCfg(FIFOCfg_Type *pFifoCfg)
{
  uint32_t tempreg;
  //check parameters
  if(pFifoCfg == NULL) return AD5940ERR_NULLP;
  /* CMDDATACON register. */
  tempreg = AD5940_ReadReg(REG_AFE_CMDDATACON);
  pFifoCfg->FIFOMode = (tempreg&BITM_AFE_CMDDATACON_DATAMEMMDE)>>BITP_AFE_CMDDATACON_DATAMEMMDE;
  pFifoCfg->FIFOSize = (tempreg&BITM_AFE_CMDDATACON_DATA_MEM_SEL)>>BITP_AFE_CMDDATACON_DATA_MEM_SEL;

  /* FIFO Threshold */
  tempreg = AD5940_ReadReg(REG_AFE_DATAFIFOTHRES);
  pFifoCfg->FIFOThresh = (tempreg&BITM_AFE_DATAFIFOTHRES_HIGHTHRES)>>BITP_AFE_DATAFIFOTHRES_HIGHTHRES;
  /* FIFOCON register. */
  tempreg = AD5940_ReadReg(REG_AFE_FIFOCON);
  pFifoCfg->FIFOEn = (tempreg&BITM_AFE_FIFOCON_DATAFIFOEN)?bTRUE:bFALSE;
  pFifoCfg->FIFOSrc = (tempreg&BITM_AFE_FIFOCON_DATAFIFOSRCSEL)>>BITP_AFE_FIFOCON_DATAFIFOSRCSEL;

  return AD5940ERR_OK;
}

/**
 * @brief Configure AD5940 FIFO Source and enable or disable FIFO.
 * @param FifoSrc : available choices are @ref FIFOSRC_Const 
 *      - FIFOSRC_SINC3       SINC3 data
 *      - FIFOSRC_DFT         DFT real and imaginary part 
 *      - FIFOSRC_SINC2NOTCH  SINC2+NOTCH block. Notch can be bypassed, so SINC2 data can be feed to FIFO 
 *      - FIFOSRC_VAR         Statistic variance output 
 *      - FIFOSRC_MEAN        Statistic mean output
 * @param FifoEn: enable or disable the FIFO.
 * @return return none.
*/
void AD5940_FIFOCtrlS(uint32_t FifoSrc, BoolFlag FifoEn)
{
  uint32_t tempreg;

  tempreg = 0;
  if(FifoEn == bTRUE)
    tempreg |= BITM_AFE_FIFOCON_DATAFIFOEN;
  tempreg |= FifoSrc << BITP_AFE_FIFOCON_DATAFIFOSRCSEL;
  AD5940_WriteReg(REG_AFE_FIFOCON, tempreg);
}

/**
 * @brief Configure AD5940 Data FIFO threshold value
   @param FIFOThresh: FIFO threshold value
   @return return none.
*/
void AD5940_FIFOThrshSet(uint32_t FIFOThresh)
{
  /* FIFO Threshold */
  AD5940_WriteReg(REG_AFE_DATAFIFOTHRES, FIFOThresh << BITP_AFE_DATAFIFOTHRES_HIGHTHRES);
}

/**
 * @brief Get Data count in FIFO
 * @return return none.
*/
uint32_t AD5940_FIFOGetCnt(void)
{
  return AD5940_ReadReg(REG_AFE_FIFOCNTSTA) >> BITP_AFE_FIFOCNTSTA_DATAFIFOCNTSTA;
}


/* Sequencer */
/**
 * @brief Initialize Sequencer
 * @param pSeqCfg: Pointer to configuration structure
   @return return none.
*/
void AD5940_SEQCfg(SEQCfg_Type *pSeqCfg)
{
  /* check parameters */
  uint32_t tempreg, fifocon;
  
  fifocon = AD5940_ReadReg(REG_AFE_FIFOCON);
  AD5940_WriteReg(REG_AFE_FIFOCON, 0);  /* Disable FIFO before changing memory configuration */
  /* Configure CMDDATACON register */
  tempreg = AD5940_ReadReg(REG_AFE_CMDDATACON);
  tempreg &= ~(BITM_AFE_CMDDATACON_CMDMEMMDE|BITM_AFE_CMDDATACON_CMD_MEM_SEL);  /* Clear settings for sequencer memory */
  tempreg |= (1L) << BITP_AFE_CMDDATACON_CMDMEMMDE;    										  /* Sequencer is always in memory mode */ 
  tempreg |= (pSeqCfg->SeqMemSize) << BITP_AFE_CMDDATACON_CMD_MEM_SEL; 	
  AD5940_WriteReg(REG_AFE_CMDDATACON, tempreg);

  if(pSeqCfg->SeqCntCRCClr)
  {
    AD5940_WriteReg(REG_AFE_SEQCON, 0);  /* Disable sequencer firstly */
    AD5940_WriteReg(REG_AFE_SEQCNT, 0);  /* When sequencer is disabled, any write to SEQCNT will clear CNT and CRC register */  
  }
  tempreg = 0;
  if(pSeqCfg->SeqEnable == bTRUE)
    tempreg |= BITM_AFE_SEQCON_SEQEN;
  tempreg |= (pSeqCfg->SeqWrTimer) << BITP_AFE_SEQCON_SEQWRTMR;
  AD5940_WriteReg(REG_AFE_SEQCON, tempreg);
  AD5940_WriteReg(REG_AFE_FIFOCON, fifocon);  /* restore FIFO configuration */

  // tempreg = 0;
  // if(pSeqCfg->SeqBreakEn)
  //   tempreg |= 0x01;  // add register definition? bitm_afe_
  // if(pSeqCfg->SeqIgnoreEn)
  //   tempreg |= 0x02;  
  // AD5940_WriteReg(0x21dc, tempreg);
}
/**
 * @brief Read back current sequencer configuration and store it to pSeqCfg
 * @param pSeqCfg: Pointer to structure
 * @return return AD5940ERR_OK if succeed.
*/
AD5940Err AD5940_SEQGetCfg(SEQCfg_Type *pSeqCfg)
{
  /* check parameters */
  uint32_t tempreg;
  if(pSeqCfg == NULL)
    return AD5940ERR_NULLP;
  /* Read CMDDATACON register */
  tempreg = AD5940_ReadReg(REG_AFE_CMDDATACON);
  pSeqCfg->SeqMemSize = (tempreg&BITM_AFE_CMDDATACON_CMD_MEM_SEL) >> BITP_AFE_CMDDATACON_CMD_MEM_SEL;
  pSeqCfg->SeqCntCRCClr = bFALSE; /* Has no meaning */
  /* SEQCON register */
  tempreg = AD5940_ReadReg(REG_AFE_SEQCON);
  pSeqCfg->SeqEnable = (tempreg&BITM_AFE_SEQCON_SEQEN)?bTRUE:bFALSE;
  pSeqCfg->SeqWrTimer = (tempreg&BITM_AFE_SEQCON_SEQWRTMR) >> BITP_AFE_SEQCON_SEQWRTMR;
  return AD5940ERR_OK;
}

/**
 * @brief Enable or Disable sequencer. 
 * @note Only after valid trigger signal, sequencer can run.
 * @return return none.
*/
void AD5940_SEQCtrlS(BoolFlag SeqEn)
{
  uint32_t tempreg = AD5940_ReadReg(REG_AFE_SEQCON);
  if(SeqEn == bTRUE)
    tempreg |= BITM_AFE_SEQCON_SEQEN;
  else
    tempreg &= ~BITM_AFE_SEQCON_SEQEN;

  AD5940_WriteReg(REG_AFE_SEQCON, tempreg);
}

/**
 * @brief Halt sequencer immediately. Use this to debug. In normal application, there is no situation that can use this function.
 * @return return none.
*/
void AD5940_SEQHaltS(void)
{
  AD5940_WriteReg(REG_AFE_SEQCON, BITM_AFE_SEQCON_SEQHALT|BITM_AFE_SEQCON_SEQEN);
}

/**
 * @brief Trigger sequencer by register write.
 * @return return none.
**/
void AD5940_SEQMmrTrig(uint32_t SeqId)
{
  if(SeqId > SEQID_3)
    return;
  AD5940_WriteReg(REG_AFECON_TRIGSEQ, 1L<<SeqId);
}

/**
 * @brief Write sequencer commands to AD5940 SRAM.
 * @return return none.
**/
void AD5940_SEQCmdWrite(uint32_t StartAddr, const uint32_t *pCommand, uint32_t CmdCnt)
{
  while(CmdCnt--)
  {
    AD5940_WriteReg(REG_AFE_CMDFIFOWADDR, StartAddr++);
    AD5940_WriteReg(REG_AFE_CMDFIFOWRITE, *pCommand++);
  }
}

/**
   @brief Initialize Sequence INFO. 
   @details There are four set of registers that record sequence information. 
          The info contains command start address in SRAM and sequence length. 
          Hardware can automatically manage these four sequences. If the application 
          requires more than 4 sequences, user should manually record the sequence 
          Info(address and length) in MCU.
   @param pSeq: Pointer to configuration structure. Specify sequence start address in SRAM and sequence length.
   @return return none.
*/
void AD5940_SEQInfoCfg(SEQInfo_Type *pSeq)
{
  switch(pSeq->SeqId)
  {
    case SEQID_0:
    /* Configure SEQINFO register */
    AD5940_WriteReg(REG_AFE_SEQ0INFO, (pSeq->SeqLen<< 16) | pSeq->SeqRamAddr);
    break;
    case SEQID_1:
    AD5940_WriteReg(REG_AFE_SEQ1INFO, (pSeq->SeqLen<< 16) | pSeq->SeqRamAddr);
    break;
    case SEQID_2:
    AD5940_WriteReg(REG_AFE_SEQ2INFO, (pSeq->SeqLen<< 16) | pSeq->SeqRamAddr);
    break;
    case SEQID_3:
    AD5940_WriteReg(REG_AFE_SEQ3INFO, (pSeq->SeqLen<< 16) | pSeq->SeqRamAddr);
    break;
    default:
    break;
  }
  if(pSeq->WriteSRAM == bTRUE)
  {
    AD5940_SEQCmdWrite(pSeq->SeqRamAddr, pSeq->pSeqCmd, pSeq->SeqLen);
  }
}

/**
 * @brief Get sequence info: start address and sequence length.
 * @param SeqId: Select from {SEQID_0, SEQID_1, SEQID_2, SEQID_3}
          - Select which sequence we want to get the information. 
   @param pSeqInfo: Pointer to sequence info structure. 
   @return return AD5940ERR_OK when succeed.
*/
AD5940Err AD5940_SEQInfoGet(uint32_t SeqId, SEQInfo_Type *pSeqInfo)
{
  uint32_t tempreg;
  if(pSeqInfo == NULL) return AD5940ERR_NULLP;
  switch(SeqId)
  {
    case SEQID_0:
    tempreg = AD5940_ReadReg(REG_AFE_SEQ0INFO);
    break;
    case SEQID_1:
    tempreg = AD5940_ReadReg(REG_AFE_SEQ1INFO);
    break;
    case SEQID_2:
    tempreg = AD5940_ReadReg(REG_AFE_SEQ2INFO);
    break;
    case SEQID_3:
    tempreg = AD5940_ReadReg(REG_AFE_SEQ3INFO);
    break;
    default:
			return AD5940ERR_PARA;
  }
  pSeqInfo->pSeqCmd = 0;    /* We don't know where you store the sequence in MCU SRAM */
  pSeqInfo->SeqId = SeqId;
  pSeqInfo->SeqLen = (tempreg>>16)&0x7ff;
  pSeqInfo->SeqRamAddr = tempreg&0x7ff;
  pSeqInfo->WriteSRAM = bFALSE;  /* Don't care */

  return AD5940ERR_OK;
}


/**
   @brief Control GPIO with register SYNCEXTDEVICE. Because sequencer have no ability to access register GPIOOUT,
         so we use this register for sequencer.
   @param Gpio : Select from {AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2|AGPIO_Pin3|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6|AGPIO_Pin7}
          - The combination of GPIO pins. The selected pins will be set to High. Others will be pulled low.
   @return return None.

**/
void AD5940_SEQGpioCtrlS(uint32_t Gpio)
{
  AD5940_WriteReg(REG_AFE_SYNCEXTDEVICE, Gpio);
}

/**
 * @brief Read back current count down timer value for Sequencer Timer Out command.
 * @return return register value of Sequencer Timer out value.
**/
uint32_t AD5940_SEQTimeOutRd(void)
{
  return AD5940_ReadReg(REG_AFE_SEQTIMEOUT);
}

/**
 * @brief Configure GPIO to allow it to trigger corresponding sequence(SEQ0/1/2/3).
 * @details There are four sequences. We can use GPIO to trigger each sequence. For example,
 *          GP0 or GP4 can be used to trigger sequence0 and GP3 or GP7 to trigger sequence3.
 *          There are five mode available to detect pin action: Rising edge, falling edge, both rising and falling
 *          edge, low level or high level.
 *          Be careful to use level detection. The trigger signal is always available if the pin level is matched.
 *          Once the sequence is done, it will immediately run again if the pin level is still matched.
 * @return return AD5940ERR_OK if succeed.
**/
AD5940Err AD5940_SEQGpioTrigCfg(SeqGpioTrig_Cfg *pSeqGpioTrigCfg)
{
  uint32_t reg_ei0con, reg_ei1con;
  uint32_t pin_count, pin_mask;
  uint32_t mode, en;
  if(pSeqGpioTrigCfg == NULL)
    return AD5940ERR_NULLP;
  reg_ei0con = AD5940_ReadReg(REG_ALLON_EI0CON);
  reg_ei1con = AD5940_ReadReg(REG_ALLON_EI1CON);

  pin_count = 0;    /* Start from pin0 */
  pin_mask = 0x01;  /* start from pin0, mask 0x01 */
  pSeqGpioTrigCfg->SeqPinTrigMode &= 0x07;  /* 3bit width */

  mode = pSeqGpioTrigCfg->SeqPinTrigMode;
  en = pSeqGpioTrigCfg->bEnable?1:0;
  for(;;)
  {
    uint32_t bit_position;
    if(pSeqGpioTrigCfg->PinSel&pin_mask)
    {
      if(pin_count < 4) /* EI0CON register */
      {
        bit_position = pin_count*4;
        reg_ei1con &= ~(0xfL<<bit_position); /* Clear bits */
        reg_ei1con |= mode << bit_position;
        reg_ei1con |= en << (bit_position + 3); /* bit offset 3 is the EN bit. */
      }
      else
      {
        bit_position = (pin_count-4)*4;
        reg_ei1con &= ~(0xfL<<bit_position); /* Clear bits */
        reg_ei1con |= mode << bit_position;
        reg_ei1con |= en << (bit_position + 3); /* bit offset 3 is the EN bit. */
      }
    }
    pin_count ++;
    pin_mask <<= 1;
    if(pin_count == 8)
      break;
  }
  AD5940_WriteReg(REG_ALLON_EI0CON, reg_ei0con);
  AD5940_WriteReg(REG_ALLON_EI0CON, reg_ei1con);
  return AD5940ERR_OK;
}

/**
 * @brief Configure Wakeup Timer
 * @param pWuptCfg: Pointer to configuration structure.
 * @return return none.
*/
void AD5940_WUPTCfg(WUPTCfg_Type *pWuptCfg)
{
  uint32_t tempreg;
  //check parameters
  /* Sleep and Wakeup time */
  AD5940_WriteReg(REG_WUPTMR_SEQ0WUPL, (pWuptCfg->SeqxWakeupTime[0] & 0xFFFF));    
  AD5940_WriteReg(REG_WUPTMR_SEQ0WUPH, (pWuptCfg->SeqxWakeupTime[0] & 0xF0000)>>16);
  AD5940_WriteReg(REG_WUPTMR_SEQ0SLEEPL, (pWuptCfg->SeqxSleepTime[0] & 0xFFFF));    
  AD5940_WriteReg(REG_WUPTMR_SEQ0SLEEPH, (pWuptCfg->SeqxSleepTime[0] & 0xF0000)>>16);

  AD5940_WriteReg(REG_WUPTMR_SEQ1WUPL, (pWuptCfg->SeqxWakeupTime[1] & 0xFFFF));    
  AD5940_WriteReg(REG_WUPTMR_SEQ1WUPH, (pWuptCfg->SeqxWakeupTime[1] & 0xF0000)>>16);
  AD5940_WriteReg(REG_WUPTMR_SEQ1SLEEPL, (pWuptCfg->SeqxSleepTime[1] & 0xFFFF));    
  AD5940_WriteReg(REG_WUPTMR_SEQ1SLEEPH, (pWuptCfg->SeqxSleepTime[1] & 0xF0000)>>16);

  AD5940_WriteReg(REG_WUPTMR_SEQ2WUPL, (pWuptCfg->SeqxWakeupTime[2] & 0xFFFF));    
  AD5940_WriteReg(REG_WUPTMR_SEQ2WUPH, (pWuptCfg->SeqxWakeupTime[2] & 0xF0000)>>16);
  AD5940_WriteReg(REG_WUPTMR_SEQ2SLEEPL, (pWuptCfg->SeqxSleepTime[2] & 0xFFFF));    
  AD5940_WriteReg(REG_WUPTMR_SEQ2SLEEPH, (pWuptCfg->SeqxSleepTime[2] & 0xF0000)>>16);

  AD5940_WriteReg(REG_WUPTMR_SEQ3WUPL, (pWuptCfg->SeqxWakeupTime[3] & 0xFFFF));    
  AD5940_WriteReg(REG_WUPTMR_SEQ3WUPH, (pWuptCfg->SeqxWakeupTime[3] & 0xF0000)>>16);
  AD5940_WriteReg(REG_WUPTMR_SEQ3SLEEPL, (pWuptCfg->SeqxSleepTime[3] & 0xFFFF));    
  AD5940_WriteReg(REG_WUPTMR_SEQ3SLEEPH, (pWuptCfg->SeqxSleepTime[3] & 0xF0000)>>16);
  
  /* TMRCON register */
  //if(pWuptCfg->WakeupEn == bTRUE)  /* enable use Wupt to wakeup AFE */
  /* We always allow Wupt to wakeup AFE automatically. */
  AD5940_WriteReg(REG_ALLON_TMRCON, BITM_ALLON_TMRCON_TMRINTEN);
  /* Wupt order */
  tempreg = 0;
  tempreg |= (pWuptCfg->WuptOrder[0]&0x03) << BITP_WUPTMR_SEQORDER_SEQA; /* position A */
  tempreg |= (pWuptCfg->WuptOrder[1]&0x03) << BITP_WUPTMR_SEQORDER_SEQB; /* position B */
  tempreg |= (pWuptCfg->WuptOrder[2]&0x03) << BITP_WUPTMR_SEQORDER_SEQC; /* position C */
  tempreg |= (pWuptCfg->WuptOrder[3]&0x03) << BITP_WUPTMR_SEQORDER_SEQD; /* position D */
  tempreg |= (pWuptCfg->WuptOrder[4]&0x03) << BITP_WUPTMR_SEQORDER_SEQE; /* position E */
  tempreg |= (pWuptCfg->WuptOrder[5]&0x03) << BITP_WUPTMR_SEQORDER_SEQF; /* position F */
  tempreg |= (pWuptCfg->WuptOrder[6]&0x03) << BITP_WUPTMR_SEQORDER_SEQG; /* position G */
  tempreg |= (pWuptCfg->WuptOrder[7]&0x03) << BITP_WUPTMR_SEQORDER_SEQH; /* position H */
  AD5940_WriteReg(REG_WUPTMR_SEQORDER, tempreg);

  tempreg = 0;
  if(pWuptCfg->WuptEn == bTRUE)
    tempreg |= BITM_WUPTMR_CON_EN;
  /* We always allow Wupt to trigger sequencer */
  tempreg |= pWuptCfg->WuptEndSeq << BITP_WUPTMR_CON_ENDSEQ;
  //tempreg |= 1L<<4;
  AD5940_WriteReg(REG_WUPTMR_CON, tempreg);
}

/**
 * @brief Enable or disable wakeup timer
 * @param Enable : {bTRUE, bFALSE}
 *        - bTRUE: enable wakeup timer
 *        - bFALSE: Disable wakeup timer
 * @return return none.
*/
void AD5940_WUPTCtrl(BoolFlag Enable)
{
  uint16_t tempreg;
  tempreg = AD5940_ReadReg(REG_WUPTMR_CON);
  tempreg &= ~BITM_WUPTMR_CON_EN;

  if(Enable == bTRUE)
    tempreg |= BITM_WUPTMR_CON_EN;
  
  AD5940_WriteReg(REG_WUPTMR_CON, tempreg);
}

/**
 * @brief Configure WakeupTimer.
 * @param SeqId: Select from SEQID_0/1/2/3. The wakeup timer will load corresponding value from four sets of registers.
 * @param SleepTime: After how much time, AFE will try to enter hibernate. We disabled this feature in AD59840_Initialize. After this timer expired, nothing will happen.
 * @param WakeupTime: After how much time, AFE will wakeup and trigger corresponding sequencer.
 * @note By SleepTime and WakeupTime, the sequencer is triggered periodically and period is (SleepTime+WakeupTime)
 * @return return none.
*/
AD5940Err AD5940_WUPTTime(uint32_t SeqId, uint32_t SleepTime, uint32_t WakeupTime)
{
  switch (SeqId)
  {
    case SEQID_0:
    {
      AD5940_WriteReg(REG_WUPTMR_SEQ0WUPL, (WakeupTime & 0xFFFF));    
      AD5940_WriteReg(REG_WUPTMR_SEQ0WUPH, (WakeupTime & 0xF0000)>>16);
      AD5940_WriteReg(REG_WUPTMR_SEQ0SLEEPL, (SleepTime & 0xFFFF));    
      AD5940_WriteReg(REG_WUPTMR_SEQ0SLEEPH, (SleepTime & 0xF0000)>>16);
      break;
    }
    case SEQID_1:
    {
      AD5940_WriteReg(REG_WUPTMR_SEQ1WUPL, (WakeupTime & 0xFFFF));    
      AD5940_WriteReg(REG_WUPTMR_SEQ1WUPH, (WakeupTime & 0xF0000)>>16);
      AD5940_WriteReg(REG_WUPTMR_SEQ1SLEEPL, (SleepTime & 0xFFFF));    
      AD5940_WriteReg(REG_WUPTMR_SEQ1SLEEPH, (SleepTime & 0xF0000)>>16);
      break;
    }
    case SEQID_2:
    {
      AD5940_WriteReg(REG_WUPTMR_SEQ2WUPL, (WakeupTime & 0xFFFF));    
      AD5940_WriteReg(REG_WUPTMR_SEQ2WUPH, (WakeupTime & 0xF0000)>>16);
      AD5940_WriteReg(REG_WUPTMR_SEQ2SLEEPL, (SleepTime & 0xFFFF));    
      AD5940_WriteReg(REG_WUPTMR_SEQ2SLEEPH, (SleepTime & 0xF0000)>>16);
      break;
    }
    case SEQID_3:
    {
      AD5940_WriteReg(REG_WUPTMR_SEQ3WUPL, (WakeupTime & 0xFFFF));    
      AD5940_WriteReg(REG_WUPTMR_SEQ3WUPH, (WakeupTime & 0xF0000)>>16);
      AD5940_WriteReg(REG_WUPTMR_SEQ3SLEEPL, (SleepTime & 0xFFFF));    
      AD5940_WriteReg(REG_WUPTMR_SEQ3SLEEPH, (SleepTime & 0xF0000)>>16);
      break;
    }
    default:
    return AD5940ERR_PARA;
  }
  return AD5940ERR_OK;
}

/**
 * @} end-of Sequencer_FIFO_Functions
 * @} end-of Sequencer_FIFO
*/

/**
 * @defgroup MISC_Block
 * @brief Other functions not included in above blocks. Clock, GPIO, INTC etc.
 * @{
 *    @defgroup MISC_Block_Functions
 *    @{
*/

/**
 * @brief Configure AD5940 clock
 * @param pClkCfg: Pointer to configuration structure.
 * @return return none.
*/
void AD5940_CLKCfg(CLKCfg_Type *pClkCfg)
{
  uint32_t tempreg, reg_osccon;

  reg_osccon = AD5940_ReadReg(REG_ALLON_OSCCON);
  /* Enable clocks */
  if(pClkCfg->HFXTALEn == bTRUE)
  {
    reg_osccon |= BITM_ALLON_OSCCON_HFXTALEN;
    AD5940_WriteReg(REG_ALLON_OSCKEY,KEY_OSCCON); /* Write Key */
    AD5940_WriteReg(REG_ALLON_OSCCON, reg_osccon); /* Enable HFXTAL */
    while((AD5940_ReadReg(REG_ALLON_OSCCON)&BITM_ALLON_OSCCON_HFXTALOK) == 0); /* Wait for clock ready */
  }

  if(pClkCfg->HFOSCEn == bTRUE)
  {
    reg_osccon |= BITM_ALLON_OSCCON_HFOSCEN;
    AD5940_WriteReg(REG_ALLON_OSCKEY,KEY_OSCCON); /* Write Key */
    AD5940_WriteReg(REG_ALLON_OSCCON, reg_osccon); /* Enable HFOSC */
    while((AD5940_ReadReg(REG_ALLON_OSCCON)&BITM_ALLON_OSCCON_HFOSCOK) == 0); /* Wait for clock ready */
    /* Configure HFOSC mode if it's enabled. */
    if(pClkCfg->HfOSC32MHzMode  == bTRUE)
      AD5940_HFOSC32MHzCtrl(bTRUE);
    else
      AD5940_HFOSC32MHzCtrl(bFALSE);
  }

  if(pClkCfg->LFOSCEn == bTRUE)
  {
    reg_osccon |= BITM_ALLON_OSCCON_LFOSCEN;  
    AD5940_WriteReg(REG_ALLON_OSCKEY,KEY_OSCCON); /* Write Key */  
    AD5940_WriteReg(REG_ALLON_OSCCON, reg_osccon); /* Enable LFOSC */
    while((AD5940_ReadReg(REG_ALLON_OSCCON)&BITM_ALLON_OSCCON_LFOSCOK) == 0); /* Wait for clock ready */
  }

  /* Switch clocks */
  /* step1. Set clock divider */
  tempreg = pClkCfg->SysClkDiv&0x3f;
  tempreg |= (pClkCfg->SysClkDiv&0x3f) << BITP_AFECON_CLKCON0_SYSCLKDIV;
  tempreg |= (pClkCfg->ADCClkDiv&0xf) << BITP_AFECON_CLKCON0_ADCCLKDIV;
  AD5940_WriteReg(REG_AFECON_CLKCON0, tempreg);
  AD5940_Delay10us(10);
  /* Step2. set clock source */
  tempreg = pClkCfg->SysClkSrc;
  tempreg |= pClkCfg->ADCCLkSrc << BITP_AFECON_CLKSEL_ADCCLKSEL;
  AD5940_WriteReg(REG_AFECON_CLKSEL, tempreg);

  /* Disable clocks */
  if(pClkCfg->HFXTALEn == bFALSE)
    reg_osccon &= ~BITM_ALLON_OSCCON_HFXTALEN;
  if(pClkCfg->HFOSCEn == bFALSE)
    reg_osccon &= ~BITM_ALLON_OSCCON_HFOSCEN;
  if(pClkCfg->LFOSCEn == bFALSE)
    reg_osccon &= ~BITM_ALLON_OSCCON_LFOSCEN;
  AD5940_WriteReg(REG_ALLON_OSCKEY, KEY_OSCCON); /* Write Key */
  AD5940_WriteReg(REG_ALLON_OSCCON, reg_osccon);
}

/**
 * @brief Configure Internal HFOSC to output 32MHz or 16MHz.
 * @param Mode32MHz : {bTRUE, bFALSE}
 *        - bTRUE: HFOSC 32MHz mode.
 *        - bFALSE: HFOSC 16MHz mode.
 * @return return none.
*/
void AD5940_HFOSC32MHzCtrl(BoolFlag Mode32MHz)
{
  uint32_t RdCLKEN1;
  uint32_t RdHPOSCCON;   

  uint32_t bit8,bit9;
    
  RdCLKEN1 = AD5940_ReadReg(REG_AFECON_CLKEN1);
  bit8 = (RdCLKEN1>>9)&0x01;
  bit9 = (RdCLKEN1>>8)&0x01;  /* Fix bug in silicon, bit8 and bit9 is swapped when read back. */
  RdCLKEN1 = RdCLKEN1&0xff;
  RdCLKEN1 |= (bit8<<8)|(bit9<<9);
  AD5940_WriteReg(REG_AFECON_CLKEN1,RdCLKEN1|BITM_AFECON_CLKEN1_ACLKDIS); /* Disable ACLK during clock changing */

  RdHPOSCCON = AD5940_ReadReg(REG_AFE_HPOSCCON); 
  if(Mode32MHz == bTRUE)
  {
    AD5940_WriteReg(REG_AFE_HPOSCCON,RdHPOSCCON&(~BITM_AFE_HPOSCCON_CLK32MHZEN)); /* Enable 32MHz output(bit definition-0: 32MHz, 1: 16MHz) */  
    while((AD5940_ReadReg(REG_ALLON_OSCCON)&BITM_ALLON_OSCCON_HFOSCOK) == 0); /* Wait for clock ready */
  }
  else
  {
    AD5940_WriteReg(REG_AFE_HPOSCCON,RdHPOSCCON|BITM_AFE_HPOSCCON_CLK32MHZEN); /* Enable 16MHz output(bit definition-0: 32MHz, 1: 16MHz) */       
    while((AD5940_ReadReg(REG_ALLON_OSCCON)&BITM_ALLON_OSCCON_HFOSCOK) == 0); /* Wait for clock ready */
  }

  AD5940_WriteReg(REG_AFECON_CLKEN1,RdCLKEN1&(~BITM_AFECON_CLKEN1_ACLKDIS)); /* Enable ACLK */
}
/**
 * @brief Enable high power mode for high frequency EIS
 * @param Mode32MHz : {bTRUE, bFALSE}
 *        - bTRUE: HFOSC 32MHz mode.
 *        - bFALSE: HFOSC 16MHz mode.
 * @return return none.
*/
void 			AD5940_HPModeEn(BoolFlag Enable)
{
	CLKCfg_Type clk_cfg;
	uint32_t temp_reg = 0;
	
	/* Check what the system clock is */
	temp_reg = AD5940_ReadReg(REG_AFECON_CLKSEL);
	clk_cfg.ADCCLkSrc = (temp_reg>>2)&0x3; 
  clk_cfg.SysClkSrc = temp_reg & 0x3; 
	if(Enable == bTRUE)
	{
		clk_cfg.SysClkDiv = SYSCLKDIV_2;
		clk_cfg.HfOSC32MHzMode = bTRUE;
		AD5940_AFEPwrBW(AFEPWR_HP, AFEBW_250KHZ);
	}
	else
	{
		clk_cfg.SysClkDiv = SYSCLKDIV_1;
		clk_cfg.HfOSC32MHzMode = bFALSE;
		AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_100KHZ);
	}
    clk_cfg.ADCClkDiv = ADCCLKDIV_1;       
    clk_cfg.HFOSCEn = (temp_reg & 0x3) == 0x1? bFALSE : bTRUE;;
	clk_cfg.HFXTALEn = (temp_reg & 0x3) == 0x1? bTRUE : bFALSE;
    clk_cfg.LFOSCEn = bTRUE;
    AD5940_CLKCfg(&clk_cfg);
}

/**
 * @defgroup Interrupt_Controller_Functions
 * @{
*/
/* AFE Interrupt Controller */
/**
 * @brief Enable or Disable selected interrupt source(s)
 * @param AfeIntcSel : {AFEINTC_0, AFEINTC_1}
 *        - AFEINTC_0: Configure Interrupt Controller 0
 *        - AFEINTC_1: Configure Interrupt Controller 1
 * @param AFEIntSrc: select from @ref AFEINTC_SRC_Const
 *       - AFEINTSRC_ADCRDY        : Bit0 ADC Result Ready Status
 *       - AFEINTSRC_DFTRDY        : Bit1 DFT Result Ready Status
 *       - AFEINTSRC_SUPPLYFILTRDY : Bit2 Low Pass Filter Result Status
 *       - AFEINTSRC_TEMPRDY       : Bit3, Temp Sensor Result Ready
 *       - AFEINTSRC_ADCMINERR     : Bit4, ADC Minimum Value
 *       - AFEINTSRC_ADCMAXERR     : Bit5, ADC Maximum Value
 *       - AFEINTSRC_ADCDIFFERR    : Bit6, ADC Delta Ready
 *       - AFEINTSRC_MEANRDY       : Bit7, Mean Result Ready
 *       - AFEINTSRC_VARRDY       : Bit8, Variance Result Ready 
 *       - AFEINTSRC_DLYCMDDONE   : Bit9, User controlled interrupt by writing AFEGENINTSTA. Provides an Early Indication for the End of the Test _Block.
 *       - AFEINTSRC_HWSETUPDONE  : Bit10, User controlled interrupt by writing AFEGENINTSTA. Indicates the MMR Setup for the Measurement Step Finished 
 *       - AFEINTSRC_BRKSEQ       : Bit11, User controlled interrupt by writing AFEGENINTSTA.
 *       - AFEINTSRC_CUSTOMINS    : Bit12, User controlled interrupt by writing AFEGENINTSTA. General Purpose Custom Interrupt. 
 *       - AFEINTSRC_BOOTLDDONE   : Bit13, OTP Boot Loading Done 
 *       - AFEINTSRC_WAKEUP       : Bit14, AFE Woken up
 *       - AFEINTSRC_ENDSEQ       : Bit15, End of Sequence Interrupt. 
 *       - AFEINTSRC_SEQTIMEOUT   : Bit16, Sequencer Timeout Command Finished. 
 *       - AFEINTSRC_SEQTIMEOUTERR : Bit17, Sequencer Timeout Command Error. 
 *       - AFEINTSRC_CMDFIFOFULL  : Bit18, Command FIFO Full Interrupt. 
 *       - AFEINTSRC_CMDFIFOEMPTY : Bit19, Command FIFO Empty 
 *       - AFEINTSRC_CMDFIFOTHRESH: Bit20, Command FIFO Threshold Interrupt. 
 *       - AFEINTSRC_CMDFIFOOF    : Bit21, Command FIFO Overflow Interrupt. 
 *       - AFEINTSRC_CMDFIFOUF    : Bit22, Command FIFO Underflow Interrupt. 
 *       - AFEINTSRC_DATAFIFOFULL : Bit23, Data FIFO Full Interrupt. 
 *       - AFEINTSRC_DATAFIFOEMPTY: Bit24, Data FIFO Empty 
 *       - AFEINTSRC_DATAFIFOTHRESH: Bit25, Data FIFO Threshold Interrupt. 
 *       - AFEINTSRC_DATAFIFOOF   : Bit26, Data FIFO Overflow Interrupt. 
 *       - AFEINTSRC_DATAFIFOUF   : Bit27, Data FIFO Underflow Interrupt. 
 *       - AFEINTSRC_WDTIRQ       : Bit28, WDT Timeout Interrupt. 
 *       - AFEINTSRC_CRC_OUTLIER  : Bit29, CRC interrupt for M355, Outliers Int for AD5940  
 *       - AFEINTSRC_GPT0INT_SLPWUT: Bit30, General Purpose Timer0 IRQ for M355. Sleep or Wakeup Timer timeout for AD5940
 *       - AFEINTSRC_GPT1INT_TRYBRK: Bit31, General Purpose Timer1 IRQ for M355. Tried to Break IRQ for AD5940
 *       - AFE_INTC_ALLINT        : All interrupts
 * @param State : {bTRUE, bFALSE}
 *      - bTRUE: Enable these interrupt source(s)
 *      - bFALSE: Disable interrupt source(s)
 * @return return none.
*/
void AD5940_INTCCfg(uint32_t AfeIntcSel, uint32_t AFEIntSrc, BoolFlag State)
{
  uint32_t tempreg;
  uint32_t regaddr = REG_INTC_INTCSEL0;
  
  if(AfeIntcSel == AFEINTC_1)
    regaddr = REG_INTC_INTCSEL1;
  
  tempreg = AD5940_ReadReg(regaddr);
  if(State == bTRUE)
    tempreg |= AFEIntSrc;    /* Enable this interrupt */
  else
    tempreg &= ~(AFEIntSrc); /* Disable this interrupt  */
  AD5940_WriteReg(regaddr,tempreg);
}

/**
 * @brief Check if current interrupt configuration.
 * @param AfeIntcSel : {AFEINTC_0, AFEINTC_1}
 *        - AFEINTC_0: Configure Interrupt Controller 0
 *        - AFEINTC_1: Configure Interrupt Controller 1
*/
uint32_t AD5940_INTCGetCfg(uint32_t AfeIntcSel)
{
  uint32_t tempreg;
  if(AfeIntcSel == AFEINTC_0)
    tempreg = AD5940_ReadReg(REG_INTC_INTCSEL0);
  else
    tempreg = AD5940_ReadReg(REG_INTC_INTCSEL1);
  return tempreg;
}

/**
 * @brief Clear selected interrupt(s) flag(INTC0Flag and INTC1Flag are both cleared).
 * @param AfeIntSrcSel: Select from @ref AFEINTC_SRC_Const
 * @return return none.
**/
void AD5940_INTCClrFlag(uint32_t AfeIntSrcSel)
{
  AD5940_WriteReg(REG_INTC_INTCCLR,AfeIntSrcSel);
}

/**
 * @brief Test if selected interrupt source(s) is(are) bTRUE.
 * @param AfeIntcSel : {AFEINTC_0, AFEINTC_1}
 *        - AFEINTC_0: Read Interrupt Controller 0 flag
 *        - AFEINTC_1: Read Interrupt Controller 1 flag
 * @param AfeIntSrcSel: Select from @ref AFEINTC_SRC_Const
 * @return If selected interrupt source(s) are all cleared, return bFALSE. Otherwise return bTRUE.
**/
BoolFlag AD5940_INTCTestFlag(uint32_t AfeIntcSel, uint32_t AfeIntSrcSel)
{
  uint32_t tempreg;
  uint32_t regaddr = (AfeIntcSel == AFEINTC_0)? REG_INTC_INTCFLAG0: REG_INTC_INTCFLAG1;
  
  tempreg = AD5940_ReadReg(regaddr);
  if(tempreg & AfeIntSrcSel)
    return bTRUE;
  else
    return bFALSE;
}

/**
 * @brief return register value of REG_INTC_INTCFLAGx
 * @param AfeIntcSel : {AFEINTC_0, AFEINTC_1}
 *        - AFEINTC_0: Read Interrupt Controller 0 flag
 *        - AFEINTC_1: Read Interrupt Controller 1 flag     
 * @return register value of REG_INTC_INTCFLAGx.
**/
uint32_t AD5940_INTCGetFlag(uint32_t AfeIntcSel)
{
  uint32_t tempreg;
  uint32_t regaddr = (AfeIntcSel == AFEINTC_0)? REG_INTC_INTCFLAG0: REG_INTC_INTCFLAG1;
  
  tempreg = AD5940_ReadReg(regaddr);
  return tempreg;
}

/**
 * @} Interrupt_Controller_Functions
*/

/**
 * @defgroup GPIO_Block_Functions
 * @{
*/

/**
 * @brief Initialize AFE GPIO
 * @param pAgpioCfg: Pointer to configuration structure
 * @return return none.
*/
void AD5940_AGPIOCfg(AGPIOCfg_Type *pAgpioCfg)
{
  AD5940_AGPIOFuncCfg(pAgpioCfg->FuncSet);
  AD5940_AGPIOOen(pAgpioCfg->OutputEnSet);
  AD5940_AGPIOIen(pAgpioCfg->InputEnSet);
  AD5940_AGPIOPen(pAgpioCfg->PullEnSet);
  AD5940_WriteReg(REG_AGPIO_GP0OUT, pAgpioCfg->OutVal);
}

/**
 * @brief Configure the function of GP0 to GP7.
 * @param uiCfgSet :{GP0_INT,GP0_TRIG,GP0_SYNC,GP0_GPIO|
 *               GP1_GPIO,GP1_TRIG,GP1_SYNC,GP1_SLEEP|
 *                GP2_PORB,GP2_TRIG,GP2_SYNC,GP2_EXTCLK|
 *                GP3_GPIO,GP3_TRIG,GP3_SYNC,GP3_INT0|\
 *                GP4_GPIO,GP4_TRIG,GP4_SYNC,GP4_INT1|
 *                GP5_GPIO,GP5_TRIG,GP5_SYNC,GP5_EXTCLK|
 *                GP6_GPIO,GP6_TRIG,GP6_SYNC,GP6_INT0|
 *                GP7_GPIO,GP7_TRIG,GP7_SYNC,GP7_INT}
 * @return return none.
**/
void AD5940_AGPIOFuncCfg(uint32_t uiCfgSet)
{
   AD5940_WriteReg(REG_AGPIO_GP0CON,uiCfgSet);
}

/**
 * @brief Enable GPIO output mode on selected pins. Disable output on non-selected pins.
 * @param uiPinSet :Select from {AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2|AGPIO_Pin3|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6|AGPIO_Pin7}
 * @return return none
**/
void AD5940_AGPIOOen(uint32_t uiPinSet)
{
   AD5940_WriteReg(REG_AGPIO_GP0OEN,uiPinSet);
}

/**
 * @brief Enable input on selected pins while disable others.
 * @param uiPinSet: Select from {AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2|AGPIO_Pin3|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6|AGPIO_Pin7}
 * @return return none
**/
void AD5940_AGPIOIen(uint32_t uiPinSet)
{
   AD5940_WriteReg(REG_AGPIO_GP0IEN,uiPinSet);
}

/**
 * @brief Read the GPIO status.
 * @return return GP0IN register which is the GPIO status.
**/
uint32_t AD5940_AGPIOIn(void)
{
  return AD5940_ReadReg(REG_AGPIO_GP0IN);
}

/**
 * @brief Enable pull-up or down on selected pins while disable other pins.
 * @param uiPinSet: Select from: {AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2|AGPIO_Pin3|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6|AGPIO_Pin7}
 * @return return none
**/
void AD5940_AGPIOPen(uint32_t uiPinSet)
{
   AD5940_WriteReg(REG_AGPIO_GP0PE,uiPinSet);
}

/**
 * @brief Put selected GPIOs to high level.
 * @param uiPinSet: Select from: {AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2|AGPIO_Pin3|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6|AGPIO_Pin7}
 * @return return none
**/
void AD5940_AGPIOSet(uint32_t uiPinSet)
{
   AD5940_WriteReg(REG_AGPIO_GP0SET,uiPinSet);
}

/**
 * @brief Put selected GPIOs to low level.
 * @param uiPinSet: Select from: {AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2|AGPIO_Pin3|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6|AGPIO_Pin7}
 * @return return none
**/
void AD5940_AGPIOClr(uint32_t uiPinSet)
{
   AD5940_WriteReg(REG_AGPIO_GP0CLR,uiPinSet);
}

/**
 * @brief Toggle selected GPIOs.
 * @param uiPinSet: Select from: {AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2|AGPIO_Pin3|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6|AGPIO_Pin7}
 * @return return none
**/
void AD5940_AGPIOToggle(uint32_t uiPinSet)
{
   AD5940_WriteReg(REG_AGPIO_GP0TGL,uiPinSet);
}

/**
 * @} GPIO_Block_Functions
*/

/**
 * @defgroup LPMode_Block_Functions
 * @{
*/
/**
 * @brief Enter or leave LPMODE. 
 * @details Once enter this mode, some registers are collected together to a new register so we can
 *          Control most blocks with in one register. The so called LPMODE has nothing to do with AD5940 power.
 * @return return AD5940ERR_OK
**/
AD5940Err AD5940_LPModeEnS(BoolFlag LPModeEn)
{
  if(LPModeEn == bTRUE)
    AD5940_WriteReg(REG_AFE_LPMODEKEY, KEY_LPMODEKEY);  /* Enter LP mode by right key. */
  else
    AD5940_WriteReg(REG_AFE_LPMODEKEY, 0); /* Write wrong key to exit LP mode */
  return AD5940ERR_OK;
}

/**
 * @brief Select system clock source for LPMODE. 
 * @note Only in LP Mode, this operation takes effect. Enter LPMODE by function @ref AD5940_LPModeEnS.
 * @param LPModeClk: Select from @ref LPMODECLK_Const
 *       - LPMODECLK_LFOSC: Select LFOSC 32kHz for system clock
 *       - LPMODECLK_HFOSC: Select HFOSC 16MHz/32MHz for system clock
 * @return none.
*/
void AD5940_LPModeClkS(uint32_t LPModeClk)
{
  AD5940_WriteReg(REG_AFE_LPMODECLKSEL, LPModeClk);
}

/**
 * @} LPMode_Block_Functions
*/

/**
 * @brief Enter sleep mode key to unlock it or enter incorrect key to lock it. \
 *        Once key is unlocked, it will always be effect until manually lock it
 * @param SlpKey : {SLPKEY_UNLOCK, SLPKEY_LOCK}
          - SLPKEY_UNLOCK Unlock Key so we can enter sleep(or called hibernate) mode.
          - SLPKEY_LOCK Lock key so AD5940 is prohibited to enter sleep mode.
   @return return none.
*/
void AD5940_SleepKeyCtrlS(uint32_t SlpKey)
{
  AD5940_WriteReg(REG_AFE_SEQSLPLOCK, SlpKey);
}

/**
 * @brief Put AFE to hibernate. 
 * @details This will only take effect when SLP_KEY has been unlocked. Use function @ref AD5940_SleepKeyCtrlS to enter correct key.
 * @return return none.
*/
void AD5940_EnterSleepS(void)
{
  AD5940_WriteReg(REG_AFE_SEQTRGSLP, 0);
  AD5940_WriteReg(REG_AFE_SEQTRGSLP, 1);
}

/**
 * @brief Turn off LP-Loop and put AFE to hibernate mode;
 * @details By function @ref AD5940_EnterSleepS, we can put most blocks to hibernate mode except LP block.
 *          This function will shut down LP block and then enter sleep mode.
 * @return return none.
*/
void AD5940_ShutDownS(void)
{
  /* Turn off LPloop related blocks which are not controlled automatically by hibernate operation */
  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lp_loop;
  /* Turn off LP-loop manually because it's not affected by sleep/hibernate mode */
  AD5940_StructInit(&aferef_cfg, sizeof(aferef_cfg));
  AD5940_StructInit(&lp_loop, sizeof(lp_loop));
  AD5940_REFCfgS(&aferef_cfg);
  AD5940_LPLoopCfgS(&lp_loop);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Unlock the key */
  AD5940_EnterSleepS();  /* Enter Hibernate */
}

/**
 * @brief Try to wakeup AD5940 by read register.
 * @details Any SPI operation can wakeup AD5940. AD5940_Initialize must be called to enable this function.
 * @param TryCount Specify how many times we will read register. Zero or negative number means always waiting here.
 * @return How many times register is read. If returned value is bigger than TryCount, it means wakeup failed.
*/
uint32_t  AD5940_WakeUp(int32_t TryCount)
{
  uint32_t count = 0;
  while(1)
  {
    count++;
    if(AD5940_ReadReg(REG_AFECON_ADIID) == AD5940_ADIID)
      break;    /* Succeed */
    if(TryCount<=0) 
      continue; /* Always try to wakeup AFE */

    if(count > TryCount)
      break;    /* Failed */
  }
  return count;
}

/**
 * @brief Read ADIID register, the value for current version is @ref AD5940_ADIID
 * @return return none.
*/
uint32_t AD5940_GetADIID(void)
{
  return AD5940_ReadReg(REG_AFECON_ADIID);
}

/**
 * @brief Read CHIPID register, the value for current version is 0x5501.
 * @return return none.
*/
uint32_t AD5940_GetChipID(void)
{
  return AD5940_ReadReg(REG_AFECON_CHIPID);
}
/**
 * @brief Reset AD5940 by register.
 * @note AD5940 must be in active state so we can access registers.
 *       If AD5940 system clock is too low, we consider to use hardware reset, or 
 *       we need to make sure register write is successfully.
 * @return return none.
*/
AD5940Err  AD5940_SoftRst(void)
{
  AD5940_WriteReg(REG_AFECON_SWRSTCON, AD5940_SWRST);
  AD5940_Delay10us(20); /* AD5940 need some time to exit reset status. 200us looks good. */
  /* We can check RSTSTA register to make sure software reset happened. */
  return AD5940ERR_OK;
}

/**
 * @brief Reset AD5940 with RESET pin.
 * @note This will call function AD5940_RstClr which locates in file XXXPort.C
 * @return return none.
*/
void AD5940_HWReset(void)
{
#ifndef CHIPSEL_M355
  AD5940_RstClr();
  AD5940_Delay10us(200); /* Delay some time */
  AD5940_RstSet();
  AD5940_Delay10us(500); /* AD5940 need some time to exit reset status. 200us looks good. */
#else
  //There is no method to reset AFE only for M355.
#endif
}

/**
 * @} MISC_Block_Functions
 * @} MISC_Block
*/

/**
 * @defgroup Calibration_Block
 * @brief The non-factory calibration routines.
 * @{
 *    @defgroup Calibration_Functions
 *    @{
 *  
 * 
 */
/**
 * @brief Turn on High power 1.8V/1.1V reference and 2.5V LP reference.
 * @return return none.
*/
static void __AD5940_ReferenceON(void)
{
  AFERefCfg_Type ref_cfg;
  /* Turn ON ADC/DAC and LPDAC reference */
  ref_cfg.Hp1V1BuffEn = bTRUE;
  ref_cfg.Hp1V8BuffEn = bTRUE;
  ref_cfg.HpBandgapEn = bTRUE;
  ref_cfg.HSDACRefEn = bTRUE;
  ref_cfg.LpBandgapEn = bTRUE;
  ref_cfg.LpRefBufEn = bTRUE;

  ref_cfg.Disc1V1Cap = bFALSE;
  ref_cfg.Disc1V8Cap = bFALSE;
  ref_cfg.Hp1V8Ilimit = bFALSE;
  ref_cfg.Hp1V8ThemBuff = bFALSE;
  ref_cfg.Lp1V1BuffEn = bFALSE;
  ref_cfg.Lp1V8BuffEn = bFALSE;
  ref_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&ref_cfg);
}

/**
 * @brief Turn on ADC to sample one SINC2 data.
 * @return return ADCCode.
*/
static uint32_t __AD5940_TakeMeasurement(int32_t *time_out)
{
  uint32_t ADCCode = 0;
  AD5940_INTCClrFlag(AFEINTSRC_SINC2RDY);
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bTRUE);/* Start conversion */
  do
  {
    AD5940_Delay10us(1);  /* Delay 10us */
    if(AD5940_INTCTestFlag(AFEINTC_1,AFEINTSRC_SINC2RDY))
    {
        ADCCode = AD5940_ReadAfeResult(AFERESULT_SINC2);
        break;
    }
    if(*time_out != -1)
      (*time_out)--;	
  }while(*time_out != 0);
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_SINC2NOTCH, bFALSE);/* Stop conversion */
  return ADCCode;
}

/**
  @brief Calibrate ADC PGA
  @param pADCPGACal: PGA calibration parameters include filter setup and PGA gain.
  @return AD5940ERR_OK.
**/
AD5940Err AD5940_ADCPGACal(ADCPGACal_Type *pADCPGACal)
{
  const float kFactor = 1.835f/1.82f;
  ADCBaseCfg_Type adc_base;

  int32_t time_out;
  uint32_t INTCCfg;
  int32_t ADCCode;
  BoolFlag bADCClk32MHzMode;

  uint32_t regaddr_gain, regaddr_offset;
  
  if(pADCPGACal == NULL) return AD5940ERR_NULLP;
  if(pADCPGACal->ADCPga > ADCPGA_9) return AD5940ERR_PARA;  /* Parameter Error */
  
  if(pADCPGACal->AdcClkFreq > (32000000*0.8))
    bADCClk32MHzMode = bTRUE; 

  /**
   *  Determine Gain calibration method according to different gain value...
   *  and calibration register 
   * */
  static const struct _cal_registers
  {
    uint16_t gain_reg;
    uint16_t offset_reg;
  }cal_registers[] = {
    {REG_AFE_ADCGAINGN1,REG_AFE_ADCOFFSETGN1},
    {REG_AFE_ADCGAINGN1P5,REG_AFE_ADCOFFSETGN1P5},
    {REG_AFE_ADCGAINGN2,REG_AFE_ADCOFFSETGN2},
    {REG_AFE_ADCGAINGN4,REG_AFE_ADCOFFSETGN4},
    {REG_AFE_ADCGAINGN9,REG_AFE_ADCOFFSETGN9},
  };
  regaddr_gain = cal_registers[pADCPGACal->ADCPga].gain_reg;
  regaddr_offset = cal_registers[pADCPGACal->ADCPga].offset_reg;

  /* Do initialization */
  __AD5940_ReferenceON();
  ADCFilterCfg_Type adc_filter;
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH. Use SIN2 data for calibration-->Lower noise */
  adc_filter.ADCSinc3Osr = pADCPGACal->ADCSinc3Osr;
  adc_filter.ADCSinc2Osr = pADCPGACal->ADCSinc2Osr;  /* 800KSPS/4/1333 = 150SPS */
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = bADCClk32MHzMode?ADCRATE_1P6MHZ:ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
  AD5940_ADCFilterCfgS(&adc_filter);
  /* Turn ON reference and ADC power, and DAC reference. We use DAC 1.8V reference to calibrate ADC because of the ADC reference bug. */
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); /* Disable all */
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_HPREFPWR|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_Delay10us(25);   /* Wait 250us for reference power up */
  /* INTC configure and open calibration lock */
  INTCCfg = AD5940_INTCGetCfg(AFEINTC_1);
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_SINC2RDY, bTRUE); /* Enable SINC2 Interrupt in INTC1 */
  AD5940_WriteReg(REG_AFE_CALDATLOCK, KEY_CALDATLOCK);  /* Unlock KEY */

  /* Do offset calibration. */
  if(pADCPGACal->PGACalType != PGACALTYPE_GAIN){  /* Need offset calibration */
    int32_t ExpectedCode = 0x8000;        /* Ideal ADC output */
    AD5940_WriteReg(regaddr_offset, 0);   /* Reset offset register */

    adc_base.ADCMuxP = ADCMUXP_VSET1P1;
    adc_base.ADCMuxN = ADCMUXN_VSET1P1;   /* Short input with common voltage set to 1.11v */
    adc_base.ADCPga = pADCPGACal->ADCPga; /* Set correct Gain value. */
    AD5940_ADCBaseCfgS(&adc_base);
    AD5940_Delay10us(5);                  /* Wait for sometime */
    ADCCode = 0;
    for(int i=0; i<8; i++)
    { /* ADC offset calibration register has resolution of 0.25LSB. take full use of it. */
      time_out = pADCPGACal->TimeOut10us;   /* Reset time out counter */
      ADCCode += __AD5940_TakeMeasurement(&time_out);  /* Turn on ADC to get one valid data and then turn off ADC. */
      if(time_out == 0) goto ADCPGACALERROR_TIMEOUT;  /* Time out error. */
    }
    /* Calculate and write the result to registers before gain calibration */
    ADCCode = (ExpectedCode<<3) - ADCCode;  /* We will shift back 1bit below */
    /**
     * AD5940 use formular Output = gain*(input + offset) for calibration.
     * So, the measured results should be divided by gain to get value for offset register.
    */
    uint32_t gain = AD5940_ReadReg(regaddr_gain);
    ADCCode = (ADCCode*0x4000)/gain;
    ADCCode = ((ADCCode+1)>>1)&0x7fff;      /* Round 0.5 */
    AD5940_WriteReg(regaddr_offset, ADCCode);
  }
  
  /* Do gain calibration */
  if(pADCPGACal->PGACalType != PGACALTYPE_OFFSET)  /* Need gain calibration */
  {
    int32_t ExpectedGainCode;
    static const float ideal_pga_gain[]={1,1.5,2,4,9};
    AD5940_WriteReg(regaddr_gain, 0x4000);  /* Reset gain register */
    if(pADCPGACal->ADCPga <= ADCPGA_2)
    {
      //gain1,1.5,2 could use reference directly
      adc_base.ADCMuxP = ADCMUXP_VREF1P8DAC;
      adc_base.ADCMuxN = ADCMUXN_VSET1P1;
      ExpectedGainCode = (int32_t)((pADCPGACal->VRef1p82 - pADCPGACal->VRef1p11)*ideal_pga_gain[pADCPGACal->ADCPga]/\
                                    pADCPGACal->VRef1p82*32768/kFactor)\
                                    + 0x8000;
    }
    else
    {
      //gain4,9 use DAC generated voltage
      adc_base.ADCMuxP = ADCMUXP_P_NODE;
      adc_base.ADCMuxN = ADCMUXN_N_NODE;
      /* Setup HSLOOP to generate voltage for GAIN4/9 calibration. */
      AD5940_AFECtrlS(AFECTRL_EXTBUFPWR|AFECTRL_INAMPPWR|AFECTRL_HSTIAPWR|AFECTRL_WG, bTRUE);
      HSLoopCfg_Type hsloop_cfg;
      hsloop_cfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
      hsloop_cfg.HsDacCfg.HsDacGain = HSDACGAIN_1;
      hsloop_cfg.HsDacCfg.HsDacUpdateRate = 7;
      hsloop_cfg.HsTiaCfg.DiodeClose = bFALSE;
      hsloop_cfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
      hsloop_cfg.HsTiaCfg.HstiaCtia = 31;
      hsloop_cfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
      hsloop_cfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
      hsloop_cfg.HsTiaCfg.HstiaDe1Rload = HSTIADERLOAD_OPEN;
      hsloop_cfg.HsTiaCfg.HstiaDe1Rtia = HSTIADERTIA_OPEN;
      hsloop_cfg.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_200;
      hsloop_cfg.SWMatCfg.Dswitch = SWD_OPEN;
      hsloop_cfg.SWMatCfg.Pswitch = SWP_PL;
      hsloop_cfg.SWMatCfg.Nswitch = SWN_NL;
      hsloop_cfg.SWMatCfg.Tswitch = SWT_TRTIA;
      hsloop_cfg.WgCfg.GainCalEn = bTRUE;
      hsloop_cfg.WgCfg.OffsetCalEn = bTRUE;
      hsloop_cfg.WgCfg.WgType = WGTYPE_MMR;
      uint32_t HSDACCode;
      if(pADCPGACal->ADCPga == ADCPGA_4)
        HSDACCode = 0x800 + 0x300;  /* 0x300--> 0x300/0x1000*0.8*BUFFERGAIN2 = 0.3V. */
      else if(pADCPGACal->ADCPga == ADCPGA_9)
        HSDACCode = 0x800 + 0x155;  /* 0x155--> 0x155/0x1000*0.8*BUFFERGAIN2 = 0.133V. */
      hsloop_cfg.WgCfg.WgCode = HSDACCode;
      AD5940_HSLoopCfgS(&hsloop_cfg);

      //measure expected code
      adc_base.ADCPga = ADCPGA_1P5;
      AD5940_ADCBaseCfgS(&adc_base);  
      AD5940_Delay10us(5);
      time_out = pADCPGACal->TimeOut10us;   /* Reset time out counter */
      ExpectedGainCode = 0x8000 + (int32_t)((__AD5940_TakeMeasurement(&time_out) - 0x8000)/1.5f\
                                            *ideal_pga_gain[pADCPGACal->ADCPga]);
      if(time_out == 0) goto ADCPGACALERROR_TIMEOUT;
    }
    adc_base.ADCPga = pADCPGACal->ADCPga;    /* Set to gain under calibration */
    AD5940_ADCBaseCfgS(&adc_base);
    AD5940_Delay10us(5);
    time_out = pADCPGACal->TimeOut10us;      /* Reset time out counter */
    ADCCode = __AD5940_TakeMeasurement(&time_out);
    if(time_out == 0) goto ADCPGACALERROR_TIMEOUT;
    /* Calculate and write the result to registers */
    ADCCode = (ExpectedGainCode - 0x8000)*0x4000/(ADCCode-0x8000);
    ADCCode &= 0x7fff;
    AD5940_WriteReg(regaddr_gain, ADCCode);
  }

  /* Restore INTC1 SINC2 configure */
  if(INTCCfg&AFEINTSRC_SINC2RDY);
  else
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_SINC2RDY, bFALSE); /* Disable SINC2 Interrupt */

  AD5940_WriteReg(REG_AFE_CALDATLOCK, 0);  /* Lock KEY */
  /* Done */
  return AD5940ERR_OK;

ADCPGACALERROR_TIMEOUT:
  AD5940_ADCConvtCtrlS(bFALSE);  /* Stop conversion */
  AD5940_WriteReg(REG_AFE_CALDATLOCK, 0);  /* Lock KEY */
  return AD5940ERR_TIMEOUT;
}

/**
 * @brief Calibrate LPTIA offset
 * @param pLPTIAOffsetCal Pointer to LPTIA offset calibration settings.
 * @return AD5940ERR_OK.
**/
AD5940Err AD5940_LPTIAOffsetCal(LPTIAOffsetCal_Type *pLPTIAOffsetCal)
{
  AD5940Err error = AD5940ERR_OK;
  LPLoopCfg_Type lploop_cfg;
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;

  int32_t time_out;
  uint32_t INTCCfg;
  int32_t ADCCode;
  BoolFlag bADCClk32MHzMode;
  
  if(pLPTIAOffsetCal == NULL) return AD5940ERR_NULLP;  
  if(pLPTIAOffsetCal->AdcClkFreq > (32000000*0.8))
    bADCClk32MHzMode = bTRUE;

  /* Step0: Do initialization */
  /* Turn on AD5940 references in case it's disabled. */
  __AD5940_ReferenceON();
  lploop_cfg.LpAmpCfg.LpAmpSel = pLPTIAOffsetCal->LpAmpSel;
  lploop_cfg.LpAmpCfg.LpAmpPwrMod = pLPTIAOffsetCal->LpAmpPwrMod;  /* Power mode will affect amp offset. */
  lploop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_OPEN;
  lploop_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_100R;
  lploop_cfg.LpAmpCfg.LpTiaRtia = pLPTIAOffsetCal->LpTiaRtia;
  lploop_cfg.LpAmpCfg.LpTiaSW = pLPTIAOffsetCal->LpTiaSW;  /* Disconnect capacitors so it settles quickly */
  lploop_cfg.LpDacCfg.LpdacSel = (pLPTIAOffsetCal->LpAmpSel == LPAMP0)?LPDAC0:LPDAC1;
  lploop_cfg.LpDacCfg.DacData12Bit = pLPTIAOffsetCal->DacData12Bit;
  lploop_cfg.LpDacCfg.DacData6Bit = pLPTIAOffsetCal->DacData6Bit;  
  lploop_cfg.LpDacCfg.DataRst = bFALSE;
  lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lploop_cfg.LpDacCfg.LpDacVzeroMux = pLPTIAOffsetCal->LpDacVzeroMux;
  lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VZERO2LPTIA;
  lploop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lploop_cfg.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lploop_cfg);

  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH. Use SIN2 data for calibration-->Lower noise */
  adc_filter.ADCSinc3Osr = pLPTIAOffsetCal->ADCSinc3Osr;
  adc_filter.ADCSinc2Osr = pLPTIAOffsetCal->ADCSinc2Osr;  /* 800KSPS/4/1333 = 150SPS */
  adc_filter.ADCAvgNum = ADCAVGNUM_2;               /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = bADCClk32MHzMode?ADCRATE_1P6MHZ:ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                       /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                      /* We use SINC3 filter. */
  adc_filter.Sinc2NotchEnable = bTRUE;              /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
  AD5940_ADCFilterCfgS(&adc_filter);
  /* Initialize ADC MUx and PGA */
  if(pLPTIAOffsetCal->LpAmpSel == LPAMP0)
  {
    adc_base.ADCMuxP = ADCMUXP_LPTIA0_P;      
    adc_base.ADCMuxN = ADCMUXN_LPTIA0_N;
  }
  else
  {
    adc_base.ADCMuxP = ADCMUXP_LPTIA1_P;      
    adc_base.ADCMuxN = ADCMUXN_LPTIA1_N;
  }
  adc_base.ADCPga = pLPTIAOffsetCal->ADCPga;                 /* Set correct Gain value. */
  AD5940_ADCBaseCfgS(&adc_base);
  /* Turn ON ADC and its reference. And SINC2. */
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); /* Disable all firstly, we only enable things we use */
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_HPREFPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_Delay10us(25);                     /* Wait 250us for reference power up */
  /* INTC configure and open calibration lock */
  INTCCfg = AD5940_INTCGetCfg(AFEINTC_1);
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_SINC2RDY, bTRUE); /* Enable SINC2 Interrupt in INTC1 */
  AD5940_WriteReg(REG_AFE_CALDATLOCK, KEY_CALDATLOCK);  /* Unlock KEY */

  /* Do offset calibration. */
  {
    int32_t ExpectedCode = 0x8000;        /* Ideal ADC output */
    AD5940_WriteReg(REG_AFE_ADCOFFSETLPTIA0, 0);   /* Reset offset register */

    if(pLPTIAOffsetCal->SettleTime10us > 0)
      AD5940_Delay10us(pLPTIAOffsetCal->SettleTime10us);  /* Delay 10us */
    time_out = pLPTIAOffsetCal->TimeOut10us;   /* Reset time out counter */
    ADCCode = __AD5940_TakeMeasurement(&time_out);  /* Turn on ADC to get one valid data and then turn off ADC. */
    if(time_out == 0)
    {
      error = AD5940ERR_TIMEOUT;
      goto LPTIAOFFSETCALERROR;
    }  /* Time out error. */
    /* Calculate and write the result to registers before gain calibration */
    ADCCode = ((ExpectedCode - ADCCode)<<3);  /* We will shift back 1bit below */
    ADCCode = ((ADCCode+1)>>1); /* Round 0.5 */
    if((ADCCode > 0x3fff) ||
        (ADCCode < -0x4000))    /* The register used for offset calibration is limited to -0x4000 to 0x3fff */
    {
      error = AD5940ERR_CALOR;
      goto LPTIAOFFSETCALERROR;
    }
    ADCCode &= 0x7fff;
    if(pLPTIAOffsetCal->LpAmpSel == LPAMP0)
      AD5940_WriteReg(REG_AFE_ADCOFFSETLPTIA0, ADCCode);
    else
      AD5940_WriteReg(REG_AFE_ADCOFFSETLPTIA1, ADCCode);
  }
  /* Restore INTC1 SINC2 configure */
  if(INTCCfg&AFEINTSRC_SINC2RDY);
  else
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_SINC2RDY, bFALSE); /* Disable SINC2 Interrupt */
  
  AD5940_WriteReg(REG_AFE_CALDATLOCK, 0);  /* Lock KEY */
  /* Done */
  return AD5940ERR_OK;

LPTIAOFFSETCALERROR:
  AD5940_ADCConvtCtrlS(bFALSE);  /* Stop conversion */
  AD5940_WriteReg(REG_AFE_CALDATLOCK, 0);  /* Lock KEY */
  return error;
}

/**
 * @brief Calibrate HSTIA offset-ongoing.
 * @param pHSTIAOffsetCal: pointer to configuration.
 * @return AD5940ERR_OK.
**/
AD5940Err AD5940_HSTIAOffsetCal(LPTIAOffsetCal_Type *pHSTIAOffsetCal)
{
  return AD5940ERR_OK;
}

/**
 * @brief Measure HSTIA internal RTIA impedance.
 * @param pCalCfg: pointer to calibration structure.
 * @param pResult:  Pointer to a variable that used to store result. 
 *                  If bPolarResult in structure is set, then use type fImpPol_Type otherwise use fImpCar_Type. 
 * @return AD5940ERR_OK if succeed.
**/
AD5940Err AD5940_HSRtiaCal(HSRTIACal_Type *pCalCfg, void *pResult)
{
  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type hs_loop;
  DSPCfg_Type dsp_cfg;
  uint32_t INTCCfg;
  
  BoolFlag bADCClk32MHzMode = bFALSE;
  uint32_t ExcitBuffGain = EXCITBUFGAIN_2;
  uint32_t HsDacGain = HSDACGAIN_1;

  float ExcitVolt; /* Excitation voltage, unit is mV */
  uint32_t RtiaVal;
  uint32_t const HpRtiaTable[]={200,1000,5000,10000,20000,40000,80000,160000,0};
  uint32_t WgAmpWord;

  iImpCar_Type DftRcal, DftRtia;

  if(pCalCfg == NULL) return AD5940ERR_NULLP;
  if(pCalCfg->fRcal == 0)
    return AD5940ERR_PARA;
  if(pCalCfg->HsTiaCfg.HstiaRtiaSel > HSTIARTIA_160K)
    return AD5940ERR_PARA;
  if(pCalCfg->HsTiaCfg.HstiaRtiaSel == HSTIARTIA_OPEN)
    return AD5940ERR_PARA; /* Do not support calibrating DE0-RTIA */
  if(pResult == NULL)
      return AD5940ERR_NULLP;

  if(pCalCfg->AdcClkFreq > (32000000*0.8))
    bADCClk32MHzMode = bTRUE; 

  /* Calculate the excitation voltage we should use based on RCAL/Rtia */
  RtiaVal = HpRtiaTable[pCalCfg->HsTiaCfg.HstiaRtiaSel];
  /*
    DAC output voltage calculation
    Note: RCAL value should be similar to RTIA so the accuracy is best.
    HSTIA output voltage should be limited to 0.2V to AVDD-0.2V, with 1.1V bias. We use 80% of this range for safe. 
    Because the bias voltage is fixed to 1.1V, so for AC signal maximum amplitude is 1.1V-0.2V = 0.9Vp. That's 1.8Vpp.
    Formula is:    ExcitVolt(in mVpp) = (1800mVpp*80% / RTIA) * RCAL
    ADC input range is +-1.5V which is enough for calibration.
    
  */
  ExcitVolt = 1800*0.8*pCalCfg->fRcal/RtiaVal;

  if(ExcitVolt <= 800*0.05) /* Voltage is so small that we can enable the attenuator of DAC(1/5) and Excitation buffer(1/4). 800mVpp is the DAC output voltage */
  {
    ExcitBuffGain = EXCITBUFGAIN_0P25;
    HsDacGain = HSDACGAIN_0P2;
    /* Excitation buffer voltage full range is 800mVpp*0.05 = 40mVpp */
    WgAmpWord = ((uint32_t)(ExcitVolt/40*2047*2)+1)>>1; /* Assign value with rounding (0.5 LSB error) */
  }
  else if(ExcitVolt <= 800*0.25) /* Enable Excitation buffer attenuator */
  {
    ExcitBuffGain = EXCITBUFGAIN_0P25;
    HsDacGain = HSDACGAIN_1;
    /* Excitation buffer voltage full range is 800mVpp*0.25 = 200mVpp */
    WgAmpWord = ((uint32_t)(ExcitVolt/200*2047*2)+1)>>1; /* Assign value with rounding (0.5 LSB error) */
  }
  else if(ExcitVolt <= 800*0.4) /* Enable DAC attenuator */
  {
    ExcitBuffGain = EXCITBUFGAIN_2;
    HsDacGain = HSDACGAIN_0P2;
    /* Excitation buffer voltage full range is 800mVpp*0.4 = 320mV */
    WgAmpWord = ((uint32_t)(ExcitVolt/320*2047*2)+1)>>1; /* Assign value with rounding (0.5 LSB error) */
  }
  else /* No attenuator is needed. This is the best condition which means RTIA is close to RCAL */
  {
    ExcitBuffGain = EXCITBUFGAIN_2;
    HsDacGain = HSDACGAIN_1;
    /* Excitation buffer voltage full range is 800mVpp*2=1600mVpp */
    WgAmpWord = ((uint32_t)(ExcitVolt/1600*2047*2)+1)>>1; /* Assign value with rounding (0.5 LSB error) */
  }

  if(WgAmpWord > 0x7ff)
  WgAmpWord = 0x7ff;
  
  /*INTC configuration */
  INTCCfg = AD5940_INTCGetCfg(AFEINTC_1);
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_DFTRDY, bTRUE); /* Enable SINC2 Interrupt in INTC1 */
  
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */
  /* Configure reference system */
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  aferef_cfg.LpBandgapEn = bFALSE;
  aferef_cfg.LpRefBufEn = bFALSE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	
  /* Configure HP Loop */
  hs_loop.HsDacCfg.ExcitBufGain = ExcitBuffGain;
  hs_loop.HsDacCfg.HsDacGain = HsDacGain;
  hs_loop.HsDacCfg.HsDacUpdateRate = 7; /* Set it to highest update rate */
  memcpy(&hs_loop.HsTiaCfg, &pCalCfg->HsTiaCfg, sizeof(pCalCfg->HsTiaCfg));
  hs_loop.SWMatCfg.Dswitch = SWD_RCAL0;
  hs_loop.SWMatCfg.Pswitch = SWP_RCAL0;
  hs_loop.SWMatCfg.Nswitch = SWN_RCAL1;
  hs_loop.SWMatCfg.Tswitch = SWT_RCAL1|SWT_TRTIA;
  hs_loop.WgCfg.WgType = WGTYPE_SIN;
  hs_loop.WgCfg.GainCalEn = bTRUE;
  hs_loop.WgCfg.OffsetCalEn = bTRUE;
  hs_loop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(pCalCfg->fFreq, pCalCfg->SysClkFreq);
  hs_loop.WgCfg.SinCfg.SinAmplitudeWord = WgAmpWord;
  hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
  hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&hs_loop);
  /* Configure DSP */
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_N_NODE;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_P_NODE;
  dsp_cfg.ADCBaseCfg.ADCPga = ADCPGA_1P5;
  AD5940_StructInit(&dsp_cfg.ADCDigCompCfg, sizeof(dsp_cfg.ADCDigCompCfg));
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* Don't care because it's disabled */
  dsp_cfg.ADCFilterCfg.ADCRate = bADCClk32MHzMode?ADCRATE_1P6MHZ:ADCRATE_800KHZ;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = pCalCfg->ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = pCalCfg->ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  
  memcpy(&dsp_cfg.DftCfg, &pCalCfg->DftCfg, sizeof(pCalCfg->DftCfg));
  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg));
  AD5940_DSPCfgS(&dsp_cfg);

  /* Enable all of them. They are automatically turned off during hibernate mode to save power */
  AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                /*AFECTRL_WG|*/AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bTRUE);
  
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* Enable Waveform generator, ADC power */
  //wait for sometime.
  AD5940_Delay10us(25);
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* Start ADC convert and DFT */
  /* Wait until DFT ready */
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DFTRDY) == bFALSE);  
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* Stop ADC convert and DFT */
  AD5940_INTCClrFlag(AFEINTSRC_DFTRDY);
  
  DftRcal.Real = AD5940_ReadAfeResult(AFERESULT_DFTREAL);
  DftRcal.Image = AD5940_ReadAfeResult(AFERESULT_DFTIMAGE);

  AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* Enable Waveform generator, ADC power */
  //wait for sometime.
  AD5940_Delay10us(25);
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* Start ADC convert and DFT */
  /* Wait until DFT ready */
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DFTRDY) == bFALSE);  
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* Stop ADC convert and DFT */
  AD5940_INTCClrFlag(AFEINTSRC_DFTRDY);

  DftRtia.Real = AD5940_ReadAfeResult(AFERESULT_DFTREAL);
  DftRtia.Image = AD5940_ReadAfeResult(AFERESULT_DFTIMAGE);
  
  if(DftRcal.Real&(1L<<17))
    DftRcal.Real |= 0xfffc0000;
  if(DftRcal.Image&(1L<<17))
    DftRcal.Image |= 0xfffc0000;
  if(DftRtia.Real&(1L<<17))
    DftRtia.Real |= 0xfffc0000;
  if(DftRtia.Image&(1L<<17))
    DftRtia.Image |= 0xfffc0000;
  /* 
    ADC MUX is set to HSTIA_P and HSTIA_N.
    While the current flow through RCAL and then into RTIA, the current direction should be from HSTIA_N to HSTIA_P if we 
    measure the voltage across RCAL by MUXSELP_P_NODE and MUXSELN_N_NODE.
    So here, we add a negative sign to results
  */
  DftRtia.Image = -DftRtia.Image;
  DftRtia.Real = -DftRtia.Real; /* Current is measured by MUX HSTIA_P-HSTIA_N. It should be  */
   /*
      The impedance engine inside of AD594x give us Real part and Imaginary part of DFT. Due to technology used, the Imaginary 
      part in register is the opposite number. So we add a negative sign on the Imaginary part of results. 
   */
  DftRtia.Image = -DftRtia.Image;
  DftRcal.Image = -DftRcal.Image;

  fImpCar_Type temp;
  temp = AD5940_ComplexDivInt(&DftRtia, &DftRcal);
  temp.Real *= pCalCfg->fRcal;
  temp.Image *= pCalCfg->fRcal;
  if(pCalCfg->bPolarResult == bFALSE)
  {
    *(fImpCar_Type*)pResult = temp;
  }
  else
  {
    ((fImpPol_Type*)pResult)->Magnitude = AD5940_ComplexMag(&temp);
    ((fImpPol_Type*)pResult)->Phase = AD5940_ComplexPhase(&temp);
  }
  
  /* Restore INTC1 DFT configure */
  if(INTCCfg&AFEINTSRC_DFTRDY);
  else
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_DFTRDY, bFALSE); /* Disable DFT Interrupt */

  return AD5940ERR_OK;
}

/**
 * @brief Measure LPTIA internal RTIA impedance with HSTIA. This is the recommended method for LPTIA RTIA calibration.
 * @param pCalCfg: pointer to calibration structure.
 * @param pResult:  Pointer to a variable that used to store result. 
 *                  If bPolarResult in structure is set, then use type fImpPol_Type otherwise use fImpCar_Type. 
 * @return AD5940ERR_OK if succeed.
**/
AD5940Err AD5940_LPRtiaCal(LPRTIACal_Type *pCalCfg, void *pResult)
{
  HSLoopCfg_Type hs_loop;
  LPLoopCfg_Type lp_loop;
  DSPCfg_Type dsp_cfg;
  ADCBaseCfg_Type *pADCBaseCfg; 
  SWMatrixCfg_Type *pSWCfg;  
  uint32_t INTCCfg, reg_afecon;
  BoolFlag bADCClk32MHzMode = bFALSE;
  BoolFlag bDCMode = bFALSE;                /* Indicate if frequency is 0, which means we calibrate at DC. */

  float ExcitVolt; /* Excitation voltage, unit is mV */
  uint32_t RtiaVal;
  /* RTIA value table when RLOAD set to 100Ohm */
  uint32_t const LpRtiaTable[]={0,110,1000,2000,3000,4000,6000,8000,10000,12000,16000,20000,24000,30000,32000,40000,48000,64000,85000,96000,100000,120000,128000,160000,196000,256000,512000};
  float const ADCPGAGainTable[] = {1, 1.5, 2, 4, 9};
  uint32_t WgAmpWord;

  uint32_t ADCPgaGainRtia, ADCPgaGainRcal;
  float GainRatio;

  iImpCar_Type DftRcal, DftRtia;

  if(pCalCfg == NULL)  return AD5940ERR_NULLP;  /* Parameters illegal */
  
  if(pCalCfg->fRcal == 0)
    return AD5940ERR_PARA;
  if(pCalCfg->LpTiaRtia > LPTIARTIA_512K)
    return AD5940ERR_PARA;
  if(pCalCfg->LpTiaRtia == LPTIARTIA_OPEN)
    return AD5940ERR_PARA; /* Not supported now. By setting RTIA to open and set corresponding switches can calibrate external RTIA */
  if(pResult == NULL)
      return AD5940ERR_NULLP;

  if(pCalCfg->AdcClkFreq > (32000000*0.8))
    bADCClk32MHzMode = bTRUE;   /* Clock frequency is high. */
  if(pCalCfg->fFreq == 0.0f)    /* Frequency is zero means we calibrate RTIA at DC. */
    bDCMode = bTRUE;
  /* Init two pointers */
  pSWCfg = &hs_loop.SWMatCfg;
  pADCBaseCfg = &dsp_cfg.ADCBaseCfg;
  /* Calculate the excitation voltage we should use based on RCAL/Rtia */
  RtiaVal = LpRtiaTable[pCalCfg->LpTiaRtia];
  /*
   * DAC output voltage calculation
   * Note: RCAL value should be similar to RTIA so the accuracy is best.
   * LPTIA output voltage should be limited to 0.3V to AVDD-0.4V, with 1.3V bias. We use 80% of this range for safe. 
   * That's 2.0Vpp*80%@2.7V AVDD
   * Formula is:    ExcitVolt(in mVpp) = (2000mVpp*80% / RTIA) * RCAL
   * ADC input range is +-1.5V which is enough for calibration.
   * Limitations:
   * Note: HSTIA output range is AVDD-0.4V to AGND+0.2V
   * HSTIA input common voltage range is 0.3V to AVDD-0.7V;
   * When AVDD is 2.7V, the input range is 0.3V to 2.0V; 
   * If we set Vbias to 1.3V, then maximum AC signal is 0.7Vp*2 = 1.4Vpp.
   * Maximum AC signal is further limited by HSTIA RTIA=200Ohm, when RCAL is 200Ohm(for ADuCM355). The maximum output of HSTIA is limited to 2.3V.
   * Maximum Vzero voltage is 1.9V when Rcal is 200Ohm and Switch On resistance is 50Ohm*2. Vzero_max = 1.3V + (2.3V-1.3V)/(200+200+50*2)*300. 
   * Maximum AC signal is (1.9-1.3)*2 = 1.2Vpp(for ADuCM355, RCAl=200Ohm).
  */
 /** @cond */
  #define MAXVOLT_P2P 1400  /* Maximum peak to peak voltage 1200mV for ADuCM355. */  
                            /* Maximum peak2peak voltage for AD5940 10kOhm RCAL is 1400mV */
  #define __MAXVOLT_AMP_CODE  (MAXVOLT_P2P*2047L/2200)
 /** @endcond */
  ExcitVolt = 2000*0.8*pCalCfg->fRcal/RtiaVal;
  WgAmpWord = ((uint32_t)(ExcitVolt/2200*2047*2)+1)>>1; /* Assign value with rounding (0.5 LSB error) */
  if(WgAmpWord > __MAXVOLT_AMP_CODE)
    WgAmpWord = __MAXVOLT_AMP_CODE;
  /**
   * Determine the best ADC PGA gain for both RCAL and RTIA voltage measurement.
  */
  {
    float RtiaVolt, RcalVolt, temp;
    ExcitVolt = WgAmpWord*2000.0f/2047; /* 2000mVpp -->ExcitVolt in Peak to Peak unit */
    RtiaVolt = ExcitVolt/(pCalCfg->fRcal + 100)*RtiaVal;
    RcalVolt = RtiaVolt/RtiaVal*pCalCfg->fRcal;
    /* The input range of ADC is 1.5Vp, we calculate how much gain we need */
    temp = 3000.0f/RcalVolt;
    if(temp >= 9.0f)  ADCPgaGainRcal = ADCPGA_9;
    else if(temp >= 4.0f) ADCPgaGainRcal = ADCPGA_4;
    else if(temp >= 2.0f) ADCPgaGainRcal = ADCPGA_2;
    else if(temp >= 1.5f) ADCPgaGainRcal = ADCPGA_1P5;
    else ADCPgaGainRcal = ADCPGA_1;
    temp = 3000.0f/RtiaVolt;
    if(temp >= 9.0f)  ADCPgaGainRtia = ADCPGA_9;
    else if(temp >= 4.0f) ADCPgaGainRtia = ADCPGA_4;
    else if(temp >= 2.0f) ADCPgaGainRtia = ADCPGA_2;
    else if(temp >= 1.5f) ADCPgaGainRtia = ADCPGA_1P5;
    else ADCPgaGainRtia = ADCPGA_1;
    GainRatio = ADCPGAGainTable[ADCPgaGainRtia]/ADCPGAGainTable[ADCPgaGainRcal];
  }
  reg_afecon = AD5940_ReadReg(REG_AFE_AFECON);
  /* INTC configuration */
  INTCCfg = AD5940_INTCGetCfg(AFEINTC_1);
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_DFTRDY|AFEINTSRC_SINC2RDY, bTRUE); /* Enable SINC2 Interrupt in INTC1 */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */
  /* Configure reference system */
	__AD5940_ReferenceON();
	/* Configure DSP */
	AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
	dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* Don't care because it's disabled */
	dsp_cfg.ADCFilterCfg.ADCRate = bADCClk32MHzMode?ADCRATE_1P6MHZ:ADCRATE_800KHZ;
	dsp_cfg.ADCFilterCfg.ADCSinc2Osr = pCalCfg->ADCSinc2Osr;
	dsp_cfg.ADCFilterCfg.ADCSinc3Osr = pCalCfg->ADCSinc3Osr;
	dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
	dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
	dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
	memcpy(&dsp_cfg.DftCfg, &pCalCfg->DftCfg, sizeof(pCalCfg->DftCfg));
	AD5940_DSPCfgS(&dsp_cfg);
  /* Configure LP Loop */
  AD5940_StructInit(&lp_loop, sizeof(lp_loop));
  /* Configure LP Amplifies(LPPA and LPTIA). We won't use LP-PA */
  lp_loop.LpDacCfg.LpdacSel = (pCalCfg->LpAmpSel  == LPAMP0)?LPDAC0:LPDAC1;
	lp_loop.LpDacCfg.DacData12Bit = 0x800;                 		/* Controlled by WG */
  lp_loop.LpDacCfg.DacData6Bit = 32;                    /* middle scale value */
  lp_loop.LpDacCfg.DataRst =bFALSE;                    	/* Do not keep DATA registers at reset status */
  lp_loop.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VZERO2HSTIA;
  lp_loop.LpDacCfg.LpDacRef = LPDACREF_2P5;            	/* Select internal 2.5V reference */
  lp_loop.LpDacCfg.LpDacSrc = LPDACSRC_WG;             	/* The LPDAC data comes from WG not MMR in this case */
  lp_loop.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_6BIT;    	/* Connect Vbias signal to 6Bit LPDAC output */
  lp_loop.LpDacCfg.LpDacVzeroMux = LPDACVZERO_12BIT;   	/* Connect Vzero signal to 12bit LPDAC output */
  lp_loop.LpDacCfg.PowerEn = bTRUE;                    	/* Power up LPDAC */

  lp_loop.LpAmpCfg.LpAmpSel = pCalCfg->LpAmpSel;
  lp_loop.LpAmpCfg.LpAmpPwrMod = pCalCfg->LpAmpPwrMod;  /* Set low power amplifiers to normal power mode */
  lp_loop.LpAmpCfg.LpPaPwrEn = bTRUE;                  	/* Enable LP PA(potential-stat amplifier) power */
  lp_loop.LpAmpCfg.LpTiaPwrEn = bTRUE;                	/* Enable LPTIA*/
  lp_loop.LpAmpCfg.LpTiaRload = LPTIARLOAD_100R;
  lp_loop.LpAmpCfg.LpTiaRtia = pCalCfg->LpTiaRtia;
  lp_loop.LpAmpCfg.LpTiaRf = LPTIARF_OPEN;
  lp_loop.LpAmpCfg.LpTiaSW = LPTIASW(6)|LPTIASW(8)|(pCalCfg->bWithCtia==bTRUE?LPTIASW(5)/*|LPTIASW(9)*/:0);
  AD5940_LPLoopCfgS(&lp_loop);
  /* Configure HS Loop */
  AD5940_StructInit(&hs_loop, sizeof(hs_loop));
  /* Take care of HSTIA, we need to disconnect internal RTIA because it connects to Tswitch directly. */
	hs_loop.HsTiaCfg.DiodeClose = bFALSE;
  hs_loop.HsTiaCfg.HstiaBias = (pCalCfg->LpAmpSel  == LPAMP0)?HSTIABIAS_VZERO0:HSTIABIAS_VZERO1;
  hs_loop.HsTiaCfg.HstiaCtia = 31;
  hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop.HsTiaCfg.HstiaDe1Rload = HSTIADERLOAD_OPEN;
  hs_loop.HsTiaCfg.HstiaDe1Rtia = HSTIADERTIA_OPEN;
  hs_loop.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_200;
  /* Configure HSDAC */
	hs_loop.HsDacCfg.ExcitBufGain = 0;
  hs_loop.HsDacCfg.HsDacGain = 0;  					/* Don't care */
  hs_loop.HsDacCfg.HsDacUpdateRate = 255;  	/* Lowest for LPDAC */

  hs_loop.SWMatCfg.Dswitch = SWD_RCAL0|((pCalCfg->LpAmpSel  == LPAMP0)?SWD_SE0:SWD_SE1);
  hs_loop.SWMatCfg.Pswitch = SWP_RCAL0;
  hs_loop.SWMatCfg.Nswitch = SWN_RCAL1;
  hs_loop.SWMatCfg.Tswitch = SWT_TRTIA|SWT_RCAL1;
  if(bDCMode)
  {
    int32_t time_out = -1;    /* Always wait. */
    int32_t offset_rcal, offset_rtia;  
    /* Configure WG */
    hs_loop.WgCfg.WgType = WGTYPE_MMR;
    hs_loop.WgCfg.WgCode = WgAmpWord;       /* Amplitude word is exactly the maximum DC voltage we could use */
    hs_loop.WgCfg.GainCalEn = bFALSE;		    /* We don't have calibration value for LPDAC, so we don't use it. */
    hs_loop.WgCfg.OffsetCalEn = bFALSE;
    AD5940_HSLoopCfgS(&hs_loop);
    AD5940_WGDACCodeS(WgAmpWord + 0x800);
		AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_WG|AFECTRL_ADCPWR, bTRUE); /* Apply voltage to loop and turn on ADC */
    /* Do offset measurement */
    pSWCfg->Dswitch = SWD_RCAL0;//|SWD_SE0;   /* Disconnect SE0 for now to measure the offset voltage. */
    pSWCfg->Pswitch = SWP_RCAL0;
    pSWCfg->Nswitch = SWN_RCAL1;
    pSWCfg->Tswitch = SWT_TRTIA|SWT_RCAL1;
    AD5940_SWMatrixCfgS(pSWCfg);    
    AD5940_Delay10us(1000);   /* Wait some time here. */
    /* Measure RCAL channel voltage offset */
    pADCBaseCfg->ADCMuxN = ADCMUXN_N_NODE;
    pADCBaseCfg->ADCMuxP = ADCMUXP_P_NODE;
    pADCBaseCfg->ADCPga = ADCPgaGainRcal;
    AD5940_ADCBaseCfgS(pADCBaseCfg);
    AD5940_Delay10us(50);   /* Wait some time here. */
    offset_rcal = __AD5940_TakeMeasurement(&time_out);  /* Turn on ADC to get one valid data and then turn off ADC. */
    /* Measure RTIA channel voltage offset */
    if(pCalCfg->LpAmpSel == LPAMP0)
    {
      pADCBaseCfg->ADCMuxN = ADCMUXN_LPTIA0_N;
      pADCBaseCfg->ADCMuxP = ADCMUXP_LPTIA0_P;
    }else
    {
      pADCBaseCfg->ADCMuxN = ADCMUXN_LPTIA1_N;
      pADCBaseCfg->ADCMuxP = ADCMUXP_LPTIA1_P;
    }
    pADCBaseCfg->ADCPga = ADCPgaGainRtia;    
    AD5940_ADCBaseCfgS(pADCBaseCfg);
    AD5940_Delay10us(50);   /* Wait some time here. */
    offset_rtia = __AD5940_TakeMeasurement(&time_out);  /* Turn on ADC to get one valid data and then turn off ADC. */
    /* Connect LPTIA loop, let current flow to RTIA. */
    pSWCfg->Dswitch = SWD_RCAL0|((pCalCfg->LpAmpSel == LPAMP0)?SWD_SE0:SWD_SE1);
    pSWCfg->Pswitch = SWP_RCAL0;
    pSWCfg->Nswitch = SWN_RCAL1;
    pSWCfg->Tswitch = SWT_TRTIA|SWT_RCAL1;
    AD5940_SWMatrixCfgS(pSWCfg);
    AD5940_Delay10us(1000);   /* Wait some time here. */
		/* Measure RCAL */
    pADCBaseCfg = &dsp_cfg.ADCBaseCfg;
    pADCBaseCfg->ADCMuxN = ADCMUXN_N_NODE;
    pADCBaseCfg->ADCMuxP = ADCMUXP_P_NODE;
    pADCBaseCfg->ADCPga = ADCPgaGainRcal;
    AD5940_ADCBaseCfgS(pADCBaseCfg);
    AD5940_Delay10us(50);   /* Wait some time here. */
    DftRcal.Real = (int32_t)__AD5940_TakeMeasurement(&time_out)- offset_rcal;
    DftRcal.Image = 0;
		/* Measure RTIA */    
    if(pCalCfg->LpAmpSel == LPAMP0)
    {
      pADCBaseCfg->ADCMuxN = ADCMUXN_LPTIA0_N;
      pADCBaseCfg->ADCMuxP = ADCMUXP_LPTIA0_P;
    }else
    {
      pADCBaseCfg->ADCMuxN = ADCMUXN_LPTIA1_N;
      pADCBaseCfg->ADCMuxP = ADCMUXP_LPTIA1_P;
    }
    pADCBaseCfg->ADCPga = ADCPgaGainRtia;
    AD5940_ADCBaseCfgS(pADCBaseCfg);
    AD5940_Delay10us(50);   /* Wait some time here. */
    DftRtia.Real = (int32_t)__AD5940_TakeMeasurement(&time_out)- offset_rtia;
    DftRtia.Image = 0;
  }
  else
  {
		hs_loop.WgCfg.SinCfg.SinAmplitudeWord = WgAmpWord;
		hs_loop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(pCalCfg->fFreq, pCalCfg->SysClkFreq);
		hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
		hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
		hs_loop.WgCfg.WgCode = 0;
		hs_loop.WgCfg.WgType = WGTYPE_SIN;
    hs_loop.WgCfg.GainCalEn = bFALSE;      /* disable it */
    hs_loop.WgCfg.OffsetCalEn = bFALSE;
    AD5940_HSLoopCfgS(&hs_loop);
    AD5940_INTCClrFlag(AFEINTSRC_DFTRDY);

    AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR, bTRUE);
    AD5940_Delay10us(100);      /* Wait for loop stable. */
    pADCBaseCfg = &dsp_cfg.ADCBaseCfg;
		/* DFT on RCAL */
    pADCBaseCfg->ADCMuxN = ADCMUXN_N_NODE;
    pADCBaseCfg->ADCMuxP = ADCMUXP_P_NODE;
    pADCBaseCfg->ADCPga = ADCPgaGainRcal;
    AD5940_ADCBaseCfgS(pADCBaseCfg);
    AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_WG, bTRUE);
    AD5940_Delay10us(25);
    AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);
    /* Wait until DFT ready */
    while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DFTRDY) == bFALSE);  
    AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* Stop ADC convert and DFT */
    AD5940_INTCClrFlag(AFEINTSRC_DFTRDY);
    DftRcal.Real = AD5940_ReadAfeResult(AFERESULT_DFTREAL);
    DftRcal.Image = AD5940_ReadAfeResult(AFERESULT_DFTIMAGE);
		/* DFT on RTIA */  
    if(pCalCfg->LpAmpSel == LPAMP0)
    {
      pADCBaseCfg->ADCMuxN = ADCMUXN_LPTIA0_N;
      pADCBaseCfg->ADCMuxP = ADCMUXP_LPTIA0_P;
    }else
    {
      pADCBaseCfg->ADCMuxN = ADCMUXN_LPTIA1_N;
      pADCBaseCfg->ADCMuxP = ADCMUXP_LPTIA1_P;
    }
    pADCBaseCfg->ADCPga = ADCPgaGainRtia;
    AD5940_ADCBaseCfgS(pADCBaseCfg);
    AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_WG, bTRUE);
    AD5940_Delay10us(25);
    AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);
    /* Wait until DFT ready */
    while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_DFTRDY) == bFALSE);  
    AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* Stop ADC convert and DFT */
    AD5940_INTCClrFlag(AFEINTSRC_DFTRDY);
    DftRtia.Real = AD5940_ReadAfeResult(AFERESULT_DFTREAL);
    DftRtia.Image = AD5940_ReadAfeResult(AFERESULT_DFTIMAGE);
    if(DftRcal.Real&(1L<<17))
      DftRcal.Real |= 0xfffc0000;
    if(DftRcal.Image&(1L<<17))
      DftRcal.Image |= 0xfffc0000;
    if(DftRtia.Real&(1L<<17))
      DftRtia.Real |= 0xfffc0000;
    if(DftRtia.Image&(1L<<17))
      DftRtia.Image |= 0xfffc0000;
  }
  /*
      The impedance engine inside of AD594x give us Real part and Imaginary part of DFT. Due to technology used, the Imaginary 
      part in register is the opposite number. So we add a negative sign on the Imaginary part of results. 
  */
  DftRtia.Image = -DftRtia.Image;
  DftRcal.Image = -DftRcal.Image;

  fImpCar_Type res;
  /* RTIA = (DftRtia.Real, DftRtia.Image)/(DftRcal.Real, DftRcal.Image)*fRcal */
  res = AD5940_ComplexDivInt(&DftRtia, &DftRcal);
  res.Real *= pCalCfg->fRcal/GainRatio;
  res.Image *= pCalCfg->fRcal/GainRatio;
  if(pCalCfg->bPolarResult == bFALSE)
  {
    ((fImpCar_Type*)pResult)->Real = res.Real;
    ((fImpCar_Type*)pResult)->Image = res.Image;
  }
  else
  {
    ((fImpPol_Type*)pResult)->Magnitude = AD5940_ComplexMag(&res);
    ((fImpPol_Type*)pResult)->Phase = AD5940_ComplexPhase(&res);
  }
    
  /* Restore INTC1 DFT configure */
  if(INTCCfg&AFEINTSRC_DFTRDY);
  else
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_DFTRDY, bFALSE);    /* Disable DFT Interrupt */
  if(INTCCfg&AFEINTSRC_SINC2RDY);
  else
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_SINC2RDY, bFALSE);  /* Disable SINC2 Interrupt */
  AD5940_WriteReg(REG_AFE_AFECON, reg_afecon);              /* Restore AFECON register */
  /* Open all switches in switch-matrix */
  hs_loop.SWMatCfg.Dswitch = SWD_OPEN;
  hs_loop.SWMatCfg.Pswitch = SWP_OPEN;
  hs_loop.SWMatCfg.Nswitch = SWN_OPEN;
  hs_loop.SWMatCfg.Tswitch = SWT_OPEN;
  AD5940_SWMatrixCfgS(&hs_loop.SWMatCfg);
  
  return AD5940ERR_OK;
}

/**
 * @brief calibrate HSDAC output voltage using ADC.
 * @note It acutally calibrates voltage output of excitation buffer.
 * @param pCalCfg: pointer to configuration structure
 * @return return AD5940ERR_OK if succeeded.
*/
AD5940Err AD5940_HSDACCal(HSDACCal_Type *pCalCfg)
{
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  HSLoopCfg_Type hsloop_cfg;
  LPLoopCfg_Type lploop_cfg;
  
  /* LSB_Numerator and LSB_Denometer are used to calculate 
  the codes to write to calibration registers depending on
  which calibration register is used
  There are LSB_Numerator ADC LSBs in
  LSB_Denominator DAC Calibration LSBs*/
  int32_t LSB_Numerator;
  int32_t LEB_Denominator;
  int32_t time_out;
  int32_t ADCCode;
  uint32_t HSDACCode = 0x800;     /* Mid scale DAC */
  
  uint32_t regaddr_offset;
  uint32_t ADCPGA_Sel;
  BoolFlag bHPMode;

  if(pCalCfg == NULL) return AD5940ERR_NULLP;
  if(pCalCfg->ExcitBufGain > 1) return AD5940ERR_PARA;
  if(pCalCfg->HsDacGain > 1) return AD5940ERR_PARA;

  bHPMode = pCalCfg->AfePwrMode == AFEPWR_HP?bTRUE:bFALSE;

  switch(pCalCfg->ExcitBufGain)
  {
  case EXCITBUFGAIN_2:
    regaddr_offset = bHPMode?REG_AFE_DACOFFSETHP:REG_AFE_DACOFFSET;
    if(pCalCfg->HsDacGain == HSDACGAIN_0P2)
    {
      LSB_Numerator = 40;
      LEB_Denominator = 14;
      ADCPGA_Sel = ADCPGA_4;
    }
    else
    {
      LSB_Numerator = 7;
      LEB_Denominator = 2;
      ADCPGA_Sel = ADCPGA_1;
    }
    break;
  case EXCITBUFGAIN_0P25:
    regaddr_offset = bHPMode?REG_AFE_DACOFFSETATTENHP:REG_AFE_DACOFFSETATTEN;
    if(pCalCfg->HsDacGain == HSDACGAIN_0P2)
    {
      LSB_Numerator = 5;
      LEB_Denominator = 14;
    }
    else
    {
      LSB_Numerator = 25;
      LEB_Denominator = 14;
    }
    ADCPGA_Sel = ADCPGA_4;
    break;
	default:
		return AD5940ERR_PARA;
  }

  /* Turn On References*/
  __AD5940_ReferenceON();
  /* Step0.0 Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH. Use SIN2 data for calibration-->Lower noise */
  adc_filter.ADCSinc3Osr = pCalCfg->ADCSinc3Osr;
  adc_filter.ADCSinc2Osr = pCalCfg->ADCSinc2Osr;  /* 800KSPS/4/1333 = 150SPS */
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = bHPMode?ADCRATE_1P6MHZ:ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
  AD5940_ADCFilterCfgS(&adc_filter);
  /* Step0.1 Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_P_NODE;
  adc_base.ADCMuxN = ADCMUXN_N_NODE;
  adc_base.ADCPga = ADCPGA_Sel;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Step0.2 Configure LPDAC to connect VZERO to HSTIA */
  lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
  lploop_cfg.LpDacCfg.DacData12Bit = 0x7C0;
  lploop_cfg.LpDacCfg.DacData6Bit = 0x1F;  
  lploop_cfg.LpDacCfg.DataRst = bFALSE;
  lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  lploop_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  lploop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  lploop_cfg.LpDacCfg.PowerEn = bTRUE;
  lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2HSTIA;
  AD5940_LPLoopCfgS(&lploop_cfg);
  
  /* Step0.3 Configure HSLOOP */
  hsloop_cfg.HsDacCfg.ExcitBufGain = pCalCfg->ExcitBufGain;
  hsloop_cfg.HsDacCfg.HsDacGain = pCalCfg->HsDacGain;
  hsloop_cfg.HsDacCfg.HsDacUpdateRate = bHPMode?0x7:0x1B;
  hsloop_cfg.HsTiaCfg.DiodeClose = bFALSE;
  hsloop_cfg.HsTiaCfg.HstiaBias = HSTIABIAS_VZERO0;
  hsloop_cfg.HsTiaCfg.HstiaCtia = 8;
  hsloop_cfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hsloop_cfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hsloop_cfg.HsTiaCfg.HstiaDe1Rload = HSTIADERLOAD_OPEN;
  hsloop_cfg.HsTiaCfg.HstiaDe1Rtia = HSTIADERTIA_OPEN;
  hsloop_cfg.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_200;
  hsloop_cfg.SWMatCfg.Dswitch = SWD_RCAL0;
  hsloop_cfg.SWMatCfg.Pswitch = SWP_RCAL0;
  hsloop_cfg.SWMatCfg.Nswitch = SWN_RCAL1;
  hsloop_cfg.SWMatCfg.Tswitch = SWT_TRTIA|SWT_RCAL1;
  hsloop_cfg.WgCfg.GainCalEn = bTRUE;
  hsloop_cfg.WgCfg.OffsetCalEn = bTRUE;
  hsloop_cfg.WgCfg.WgType = WGTYPE_MMR;
  hsloop_cfg.WgCfg.WgCode = HSDACCode;
  AD5940_HSLoopCfgS(&hsloop_cfg);
  /* Step0.4 Turn ON reference and ADC power, and DAC power and DAC reference. We use DAC 1.8V reference to calibrate ADC. */
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); /* Disable all */
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_HPREFPWR|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|AFECTRL_SINC2NOTCH|\
    AFECTRL_EXTBUFPWR|AFECTRL_INAMPPWR|AFECTRL_HSTIAPWR|AFECTRL_WG, bTRUE);
  AD5940_Delay10us(25);   /* Wait 250us for reference power up */
  /* Step0.5 INTC configure and open calibration lock */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_SINC2RDY, bTRUE); /* Enable SINC2 Interrupt in INTC1 */
  AD5940_WriteReg(REG_AFE_CALDATLOCK, KEY_CALDATLOCK);  /* Unlock KEY */
	/* Reset Offset register before calibration */
	AD5940_WriteReg(regaddr_offset, 0);
	/* Update HSDACDAT after resetting calibration register */
	AD5940_WriteReg(REG_AFE_HSDACDAT, 0x800);
  /* Step1: Do offset calibration. */
  {
    int32_t ExpectedCode = 0x8000;        /* Ideal ADC output */
    AD5940_Delay10us(10);
    time_out = 1000;   /* Reset time out counter */
    ADCCode = __AD5940_TakeMeasurement(&time_out);
#ifdef ADI_DEBUG
    ADI_Print("Voltage before cal: %f \n", AD5940_ADCCode2Volt(ADCCode, ADCPGA_Sel, 1.82));
#endif

    if(time_out == 0) goto DACCALERROR_TIMEOUT;  /* Time out error. */
    ADCCode = ADCCode - ExpectedCode;
    ADCCode = (((ADCCode)*LEB_Denominator)/LSB_Numerator); 
    if(ADCCode>0)
      ADCCode = 0xFFF - ADCCode;
    else
      ADCCode = -ADCCode;
    AD5940_WriteReg(regaddr_offset, ADCCode);
    AD5940_Delay10us(10);
    AD5940_WriteReg(REG_AFE_HSDACDAT, 0x800);
    AD5940_Delay10us(10);
#ifdef ADI_DEBUG
		ADCCode = __AD5940_TakeMeasurement(&time_out);
		ADI_Print("Voltage after cal: %f \n", AD5940_ADCCode2Volt(ADCCode, ADCPGA_Sel, 1.82));
#endif
  }
  AD5940_WriteReg(REG_AFE_CALDATLOCK, 0);  /* Lock KEY */
  return AD5940ERR_OK;
DACCALERROR_TIMEOUT:
  AD5940_ADCConvtCtrlS(bFALSE);  /* Stop conversion */
  AD5940_WriteReg(REG_AFE_CALDATLOCK, 0);  /* Lock KEY */
  return AD5940ERR_TIMEOUT;
}

/**
 * @brief Use ADC to measure LPDAC offset and gain factor.
 * @note Assume ADC is accurate enough or accurate than LPDAC at least.
 * @param pCalCfg: pointer to structure.
 * @param pResult: the pointer to save calibration result.
 * @return AD5940ERR_OK if succeed.
**/
AD5940Err AD5940_LPDACCal(LPDACCal_Type *pCalCfg, LPDACPara_Type *pResult)
{
  AD5940Err error = AD5940ERR_OK;
  LPDACCfg_Type LpDacCfg;
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;

  int32_t time_out;
  uint32_t INTCCfg;
  int32_t ADCCode, ADCCodeVref1p1;
  BoolFlag bADCClk32MHzMode;
  
  if(pCalCfg == NULL) return AD5940ERR_NULLP; 
  if(pResult == NULL) return AD5940ERR_NULLP;  
  if(pCalCfg->AdcClkFreq > (32000000*0.8))
    bADCClk32MHzMode = bTRUE;

  /* Step0: Do initialization */
  /* Turn on AD5940 references in case it's disabled. */
  __AD5940_ReferenceON();
  LpDacCfg.LpdacSel = pCalCfg->LpdacSel;
  LpDacCfg.DacData12Bit = 0;
  LpDacCfg.DacData6Bit = 0;  
  LpDacCfg.DataRst = bFALSE;
  LpDacCfg.LpDacRef = LPDACREF_2P5;
  LpDacCfg.LpDacSrc = LPDACSRC_MMR;
  LpDacCfg.LpDacSW = LPDACSW_VBIAS2PIN|LPDACSW_VZERO2PIN;
  LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT;
  LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;
  LpDacCfg.PowerEn = bTRUE;
  AD5940_LPDACCfgS(&LpDacCfg);

  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH. Use SIN2 data for calibration-->Lower noise */
  adc_filter.ADCSinc3Osr = pCalCfg->ADCSinc3Osr;
  adc_filter.ADCSinc2Osr = pCalCfg->ADCSinc2Osr;  /* 800KSPS/4/1333 = 150SPS */
  adc_filter.ADCAvgNum = ADCAVGNUM_2;               /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = bADCClk32MHzMode?ADCRATE_1P6MHZ:ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                       /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                      /* We use SINC3 filter. */
  adc_filter.Sinc2NotchEnable = bTRUE;              /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
  AD5940_ADCFilterCfgS(&adc_filter);
  /* Initialize ADC MUx and PGA */
  adc_base.ADCMuxP = ADCMUXP_AGND;
  adc_base.ADCMuxN = ADCMUXN_VSET1P1;
  adc_base.ADCPga = ADCPGA_1;
  AD5940_ADCBaseCfgS(&adc_base);
  /* Turn ON ADC and its reference. And SINC2. */
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); /* Disable all firstly, we only enable things we use */
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_HPREFPWR|AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_Delay10us(25);                     /* Wait 250us for reference power up */
  /* INTC configure and open calibration lock */
  INTCCfg = AD5940_INTCGetCfg(AFEINTC_1);
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_SINC2RDY, bTRUE); /* Enable SINC2 Interrupt in INTC1 */
  /* Step1: Measure internal 1.1V reference. */
  {
    //AD5940_ADCMuxCfgS(ADCMUXP_AGND, ADCMUXN_VSET1P1);
    time_out = pCalCfg->TimeOut10us;   /* Reset time out counter */
    ADCCodeVref1p1 = __AD5940_TakeMeasurement(&time_out);  /* Turn on ADC to get one valid data and then turn off ADC. */
    if(time_out == 0)
    {
      error = AD5940ERR_TIMEOUT;
      goto LPDACCALERROR;
    }  /* Time out error. */
    /* Equation1: ADCCodeVref1p1 = AGND - Vref1p1 */
  }
  /* Step2: Do offset measurement. */
  {
    /* Equation2': ADCCode = Vbias0/1 - Vref1p1 */
    AD5940_LPDACWriteS(0,0);  /* Set LPDAC output voltage to 0.2V(zero code) */
    if(pCalCfg->SettleTime10us > 0)
      AD5940_Delay10us(pCalCfg->SettleTime10us);  /* Delay nx10us */
    if(pCalCfg->LpdacSel == LPDAC0)
      AD5940_ADCMuxCfgS(ADCMUXP_VBIAS0, ADCMUXN_VREF1P1); /* Vbias0 is routed to 12BIT LPDAC */
    else
      AD5940_ADCMuxCfgS(ADCMUXP_VBIAS1, ADCMUXN_VREF1P1); /* Vbias1 is routed to 12BIT LPDAC */

    AD5940_Delay10us(5);  /* Delay 50us */
    time_out = pCalCfg->TimeOut10us;   /* Reset time out counter */
    ADCCode = __AD5940_TakeMeasurement(&time_out);  /* Turn on ADC to get one valid data and then turn off ADC. */
    if(time_out == 0)
    {
      error = AD5940ERR_TIMEOUT;
      goto LPDACCALERROR;
    }  /* Time out error. */
    /* Calculate the offset voltage using Equation2 - Equation1 */
    ADCCode -= ADCCodeVref1p1;  /* Get the code of Vbias0-AGND. Then calculate the offset voltage in mV. */
    pResult->bC2V_DAC12B = ADCCode*pCalCfg->ADCRefVolt*1e3f/32768*1.835f/1.82f; /*mV unit*/
    /* Measure 6BIT DAC output(Vzero0/1) */
    if(pCalCfg->LpdacSel == LPDAC0)
      AD5940_ADCMuxCfgS(ADCMUXP_VZERO0, ADCMUXN_VREF1P1); /* Vbias0 is routed to 12BIT LPDAC */
    else
      AD5940_ADCMuxCfgS(ADCMUXP_VZERO1, ADCMUXN_VREF1P1); /* Vbias1 is routed to 12BIT LPDAC */
    AD5940_Delay10us(5);  /* Delay 50us */
    time_out = pCalCfg->TimeOut10us;   /* Reset time out counter */
    ADCCode = __AD5940_TakeMeasurement(&time_out);  /* Turn on ADC to get one valid data and then turn off ADC. */
    if(time_out == 0)
    {
      error = AD5940ERR_TIMEOUT;
      goto LPDACCALERROR;
    }  /* Time out error. */
    /* Calculate the offset voltage */
    ADCCode -= ADCCodeVref1p1;  /* Get the code of Vbias0-AGND. Then calculate the offset voltage in mV. */
    pResult->bC2V_DAC6B = ADCCode*pCalCfg->ADCRefVolt*1e3f/32768*1.835f/1.82f; /*mV unit*/
  }
  /* Step3: Do gain measurement */
  {
    /* Equation2: ADCCode = Vbias0 - Vref1p1 */
    AD5940_LPDACWriteS(0xfff,0x3f);  /* Set LPDAC output voltage to 2.4V(zero code) */
    if(pCalCfg->SettleTime10us > 0)
      AD5940_Delay10us(pCalCfg->SettleTime10us);  /* Delay nx10us */
    if(pCalCfg->LpdacSel == LPDAC0)
      AD5940_ADCMuxCfgS(ADCMUXP_VBIAS0, ADCMUXN_VREF1P1); /* Vbias0 is routed to 12BIT LPDAC */
    else
      AD5940_ADCMuxCfgS(ADCMUXP_VBIAS1, ADCMUXN_VREF1P1); /* Vbias1 is routed to 12BIT LPDAC */
    AD5940_Delay10us(5);  /* Delay 50us */
    time_out = pCalCfg->TimeOut10us;   /* Reset time out counter */
    ADCCode = __AD5940_TakeMeasurement(&time_out);  /* Turn on ADC to get one valid data and then turn off ADC. */
    if(time_out == 0)
    {
      error = AD5940ERR_TIMEOUT;
      goto LPDACCALERROR;
    }  /* Time out error. */
    /* Calculate the offset voltage */
    ADCCode -= ADCCodeVref1p1;  /* Get the code of Vbias0-AGND. Then calculate the gain factor 'k'. */
    pResult->kC2V_DAC12B = (ADCCode*pCalCfg->ADCRefVolt*1e3f/32768*1.835f/1.82f - pResult->bC2V_DAC12B)/0xfff;/*mV unit*/
    /* Measure 6BIT DAC output(Vzero0) */
    if(pCalCfg->LpdacSel == LPDAC0)
      AD5940_ADCMuxCfgS(ADCMUXP_VZERO0, ADCMUXN_VREF1P1); /* Vbias0 is routed to 12BIT LPDAC */
    else
      AD5940_ADCMuxCfgS(ADCMUXP_VZERO1, ADCMUXN_VREF1P1); /* Vbias1 is routed to 12BIT LPDAC */
    AD5940_Delay10us(5);  /* Delay 50us */
    time_out = pCalCfg->TimeOut10us;   /* Reset time out counter */
    ADCCode = __AD5940_TakeMeasurement(&time_out);  /* Turn on ADC to get one valid data and then turn off ADC. */
    if(time_out == 0)
    {
      error = AD5940ERR_TIMEOUT;
      goto LPDACCALERROR;
    }  /* Time out error. */
    /* Calculate the offset voltage */
    ADCCode -= ADCCodeVref1p1;  /* Get the code of Vbias0-AGND. Then calculate the offset voltage in mV. */
    pResult->kC2V_DAC6B = (ADCCode*pCalCfg->ADCRefVolt*1e3f/32768*1.835f/1.82f - pResult->bC2V_DAC6B)/0x3f;/*mV unit*/
  }
  /* Step4: calculate the parameters for voltage to code calculation. */
  pResult->kV2C_DAC12B = 1/pResult->kC2V_DAC12B;
  pResult->bV2C_DAC12B = -pResult->bC2V_DAC12B/pResult->kC2V_DAC12B;
  pResult->kV2C_DAC6B = 1/pResult->kC2V_DAC6B;
  pResult->bV2C_DAC6B = -pResult->bC2V_DAC6B/pResult->kC2V_DAC6B;
  /* Restore INTC1 SINC2 configure */
  if(INTCCfg&AFEINTSRC_SINC2RDY);
  else
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_SINC2RDY, bFALSE); /* Disable SINC2 Interrupt */
  /* Done */
  return AD5940ERR_OK;

LPDACCALERROR:
  AD5940_ADCConvtCtrlS(bFALSE);  /* Stop conversion */
  return error;
}

/**
 * @brief Use system clock to measure LFOSC frequency.
 * @note Set system clock to external crystal to get a better measurement accuracy.
 *       This function use 3 sequences and the start address is specified by parameter.
 * @param pCfg: pointer to structure.
 * @param pFreq:  Pointer to a variable that used to store frequency in Hz. 
 * @return AD5940ERR_OK if succeed.
**/
AD5940Err AD5940_LFOSCMeasure(LFOSCMeasure_Type *pCfg, float *pFreq) /* Measure current LFOSC frequency. */
{
  /**
   * @code
   *  Sleep wakeup timer running...
   *  -SLP----WKP----SLP----WKP----SLP----WKP
   *  --|-----|-------------|-------------|------------Trigger sequencer when Wakeup Timer over.
   *  --------|SEQA---------|SEQB----------------------Execute SeqA then SeqB
   *  ---------|InitT--------|StopT--------------------SeqA start timer and SeqB trigger interrupt so MCU read back current count
   *  ------------------------|INT---------------------
   *  -----------------------------------------|Read---We read SEQTIMEOUT register here
   *  ---------|-----TimerCount----------------|-------
   *  ---------|--------------|---TimerCount2--|-------We change SeqB to reset timer so we measure how much time needed for MCU to read back SEQTIMEOUT register(TimerCount2)
   * @endcode
   * **/
  uint32_t TimerCount, TimerCount2;
  SEQCfg_Type seq_cfg, seq_cfg_backup;
  SEQInfo_Type seqinfo;
  WUPTCfg_Type wupt_cfg;
  uint32_t INTCCfg;
  uint32_t WuptPeriod;

  static const uint32_t SeqA[]=
  {
    SEQ_TOUT(0x3fffffff),   /* Set time-out timer. It will always run until disable Sequencer by SPI interface. */
  };
  static const uint32_t SeqB[]=
  {
    /**
     * Interrupt flag AFEINTSRC_ENDSEQ will be set after this command. So We can inform MCU to read back 
     * current timer value. MCU will need some additional time to read back time count.
     * So we use SeqB to measure how much time needed for MCU to read back 
     * */
    SEQ_STOP(),             
  };
  static const uint32_t SeqBB[]=
  {
    SEQ_TOUT(0x3fffffff),   /* Re-Set time-out timer, so we can measure the time needed for MCU to read out Timer Count register. */
    SEQ_STOP(),             /* Interrupt flag AFEINTSRC_ENDSEQ will be set here */
  };

  if(pCfg == NULL) return AD5940ERR_NULLP;
  if(pFreq == NULL) return AD5940ERR_NULLP;
  if(pCfg->CalDuration < 1.0f)
    return AD5940ERR_PARA;
  AD5940_SEQGetCfg(&seq_cfg_backup);
  INTCCfg = AD5940_INTCGetCfg(AFEINTC_1);
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ENDSEQ, bTRUE);
	AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM is used for sequencer */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bFALSE;
  seq_cfg.SeqEnable = bTRUE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);          /* Enable sequencer */
  
  seqinfo.pSeqCmd = SeqA;
  seqinfo.SeqId = SEQID_0;
  seqinfo.SeqLen = SEQ_LEN(SeqA);
  seqinfo.SeqRamAddr = pCfg->CalSeqAddr;
  seqinfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&seqinfo);
  seqinfo.SeqId = SEQID_1;
  seqinfo.SeqRamAddr = pCfg->CalSeqAddr + SEQ_LEN(SeqA) ;
  seqinfo.SeqLen = SEQ_LEN(SeqB);
  seqinfo.pSeqCmd = SeqB;
  AD5940_SEQInfoCfg(&seqinfo);      /* Configure sequence0 and sequence1 with command SeqA and SeqB */
	
  wupt_cfg.WuptEn = bFALSE;
  wupt_cfg.WuptOrder[0] = SEQID_0;
  wupt_cfg.WuptOrder[1] = SEQID_1;
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_B;
  wupt_cfg.SeqxWakeupTime[0] = 4;       /* Don't care. >4 is acceptable */
  wupt_cfg.SeqxSleepTime[0] = (uint32_t)((pCfg->CalDuration)*32 + 0.5f) - 1 - 4;
  wupt_cfg.SeqxWakeupTime[1] = 4-1;
  wupt_cfg.SeqxSleepTime[1] = 0xffffffff; /* Don't care */
  WuptPeriod = (wupt_cfg.SeqxSleepTime[0]+1) + (wupt_cfg.SeqxWakeupTime[1]+1);
  AD5940_WUPTCfg(&wupt_cfg);
  
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
  AD5940_WUPTCtrl(bTRUE);
  
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  TimerCount = AD5940_SEQTimeOutRd();
  
  AD5940_WUPTCtrl(bFALSE);
	AD5940_WUPTTime(SEQID_0, 4, 4);	/* Set it to minimum value because we don't care about sequence0 now. We only want to measure how much time MCU will need to read register */
  seqinfo.SeqId = SEQID_1;
  seqinfo.SeqRamAddr = pCfg->CalSeqAddr + SEQ_LEN(SeqA) ;
  seqinfo.SeqLen = SEQ_LEN(SeqBB);
  seqinfo.pSeqCmd = SeqBB;
  seqinfo.WriteSRAM = bTRUE;
  AD5940_SEQInfoCfg(&seqinfo);
  AD5940_SEQCtrlS(bTRUE); /* Enable Sequencer again */

  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
  AD5940_WUPTCtrl(bTRUE);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  TimerCount2 = AD5940_SEQTimeOutRd();
	AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_ENDSEQ);

  AD5940_WUPTCtrl(bFALSE);
  AD5940_SEQCfg(&seq_cfg_backup);          /* restore sequencer configuration */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ENDSEQ, (INTCCfg&AFEINTSRC_ENDSEQ)?bTRUE:bFALSE); /* Restore interrupt configuration */
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
  //printf("Time duration:%d ", (TimerCount2 - TimerCount));
	*pFreq = pCfg->SystemClkFreq*WuptPeriod/(TimerCount2 - TimerCount);
  return AD5940ERR_OK;
}

/**
 * @} Calibration
 * @} Calibration_Block
*/

/**
 * @} AD5940_Functions
 * @} AD5940_Library
*/
