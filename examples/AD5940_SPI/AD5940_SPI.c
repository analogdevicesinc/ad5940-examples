/*!
 *****************************************************************************
 @file:    AD5940_SPI.c
 @author:  $Author: nxu2 $
 @brief:   Basic register read/write test example.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

/**
 * This example shows how to read/write AD5940 registers through SPI.
 * Use function called AD5940_ReadReg and AD5940_WriteReg.
**/
#include "ad5940.h"
#include "AD5940.h"
#include <stdio.h>
#include "string.h"
#include <stdlib.h>

void AD5940_Main(void)
{
  unsigned long temp, i;
  /**
   * Hardware reset can always put AD5940 to default state. 
   * We recommend to use hardware reset rather than software reset
   * because there are some situations that SPI won't work, for example, AD59840 is in hibernate mode, 
   * or AD5940 system clock is 32kHz that SPI bus clock should also be limited..
   * */
  AD5940_HWReset();
  /**
   * @note MUST call this function whenever there is reset happened. This function will put AD5940 to right state.
   *       The reset can be software reset or hardware reset or power up reset.
  */
  AD5940_Initialize();
  /**
   * Normal application code starts here.
  */
  /**
   * Read register test.
  */
  temp = AD5940_ReadReg(REG_AFECON_ADIID);
  printf("Read ADIID register, got: 0x%04lx\n", temp);
  if(temp != AD5940_ADIID)
    printf("Read register test failed.\n" );
  else
    printf("Read register test pass\n");
  /**
   * Write register test.
   * */
  srand(0x1234);
  i =10000;
  while(i--)
  {
    static unsigned long count;
    static unsigned long data;
    /* Generate a 32bit random data */
    data = rand()&0xffff;
    data <<= 16;
    data |= rand()&0xffff;
    count ++;	/* Read write count */
    /**
     * Register CALDATLOCK is 32-bit width, it's readable and writable.
     * We use it to test SPI register access.
    */
    AD5940_WriteReg(REG_AFE_CALDATLOCK, data);
    temp = AD5940_ReadReg(REG_AFE_CALDATLOCK);
    if(temp != data)
      printf("Write register test failed @0x%08lx\n", data);
    if(!(count%1000))
      printf("Read/Write has been done %ld times, latest data is 0x%08lx\n", count, data);
  }
  printf("SPI read/write test completed");
  while(1);
}

