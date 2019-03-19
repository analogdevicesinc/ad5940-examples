/*!
 *****************************************************************************
 @file:    AD5940_Reset.c
 @author:  $Author: nxu2 $
 @brief:   Demostrate three methods to reset AD5940: External Reset, MMR Reset and Power On Reset.
 @version: $Revision: 766 $
 @date:    $Date: 2017-08-21 14:09:35 +0100 (Mon, 21 Aug 2017) $
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

/**
 * The example shows three kinds of reset source of AD5940:
 *  - Hardware/External Reset, this is done via RESET pin. Pull it low to reset AD5940.
 *  - Software Reset, this is done by write regiter.
 *  - POR Reset, power on reset is done when the power is firstly applied.
 * 
 * After power up, the program firlsty check reset status, there should be only POR reset flag set.
 * Then we perform hardware reset. The reset status should reflect this.
 * Note the flag in RSTSTA register is stiky. You can clear it by write 1 to corresponding bit.
 * Finally, we perform software reset.
 * Program then complete required initialization which should be done whenever there is a reset.
*/

#include "ad5940.h"
#include "AD5940.h"
#include <stdio.h>
#include "string.h"
#include <stdlib.h>

void print_rststa(uint32_t reg)
{
  printf("<<<<<<<Reset Status<<<<<\n");
  if(reg & 0x01)
    printf("POR Reset Happened\n");
  if(reg & 0x02)
    printf("Hardware/External Reset Happened\n");
  if(reg & 0x08)
    printf("Software Reset Happened\n");
  if((reg&0xb) == 0)
    printf("No reset happened\n");
  printf(">>>>>>>Reset Status Done>>>>>\n");
}

void AD5940_Main(void)
{
  uint32_t temp;
  printf("Wait 5 secondes\n");
  AD5940_Delay10us(100*5000); /* Delay 5s */
  printf("\n1. AD5940 Power ON\n");
  temp = AD5940_ReadReg(REG_ALLON_RSTSTA);
  print_rststa(temp);
  AD5940_WriteReg(REG_ALLON_RSTSTA, 0xf);  /* Clear reset status. This register will remain its value until we manually clear it. Reset operation won't reset this register. */

  printf("\n2. Perform Hardware reset now!\n");
  AD5940_HWReset();
  printf("Hardware reset done, status is:\n");
  temp = AD5940_ReadReg(REG_ALLON_RSTSTA);
  print_rststa(temp);
  AD5940_WriteReg(REG_ALLON_RSTSTA, 0xf);

  printf("\n3. Perform Software Reset now \n");
  AD5940_SoftRst();
  printf("Software reset done, status is:\n");
  temp = AD5940_ReadReg(REG_ALLON_RSTSTA);
  print_rststa(temp);
  printf("\nReset Test done \n");
  /**
   * @note MUST call this function whenever there is reset happened. This function will put AD5940 to right state.
  */
  AD5940_Initialize();
  AD5940_WriteReg(REG_ALLON_RSTSTA, 0xf); /* Clear reset status register. */

  while(1);
}

