AD5940 Firmware Library
======================================================
[AD5940](https://www.analog.com/en/products/ad5940.html)/AD5941 and [ADuCM355](https://www.analog.com/en/products/aducm355.html) is the latest on-chip system for electrochemical and biosensors. It has more than 180 registers and makes application development quite complicated. This library targets to offer a register-less method to operate the chip.

There are also example codes based on this library. Check the corresponding repository for details.(The code is not ready in GitHub at the time of writing.)
# Read Before Use

To make the chip easier to use, some registers are initialized with non-default value. This is done in function @ref AD5940_Initialize()

***Thus it's important to call this function whenever AFE is reset(POR, hardware reset or soft-reset)***

## The settings by this library
- Disable AFE Watch Dog Timer. You can enable it in user code if there is need.
- Set register 0xC08(not documented) for FIFO read. **This has to be done, otherwise FIFO read command(SPICMD_READFIFO) won't work.**
- Disable unused clocks(eg. test clock) to save power.
- Enable the function to wake up AFE whenever there is bus activity(CS falling edge on AD594x). So we can wakeup AFE by any register read(discard this read).
- Enable the function to retain SRAM data during hibernate
- Enable the function that allows Sequencer to put AFE to hibernate mode. Note, another condition must be met is to input a correct key. Use API @ref AD5940_SleepKeyCtrlS()
- Configure register PMBW to zero, so PMBW.bit[3:2] and bit[0] can control system bandwidth and chip power mode.

# Differences between AD594x/ADuCM355
AD5940/AD5941 can be regarded as the AFE(Analog Front End) of ADuCM355. But they have differences.

AD5940 and AD5941 only differs in package, thus has slight difference in available pins.
## AD5940 vs AD5941
- AD5940 has 8 AFE GPIOs and AD5941 has only 3.
- All others are exactly same.
## AD594x vs ADuCM355
- **AD594x only has one Low Power channel, while ADuCM355 has two.** You have to select channel0 for AD594x. To choose channel0, configure corresponding struct members with constant LPDAC0 and LPAMP0 which means LPDAC0 and LP amplifiers(PA and TIA) channel0.
- AD594x and ADuCM355 use totally different GPIO control register. Considering AD594x use GPIOAx but ADuCM355 use GPIOBx
- ADuCM355 has only two GPIOs connected to AFE and it has PWM function, while not AD594x.
- AD594x's GPIO can be controlled by sequencer,  while not ADuCM355.
- AD594x has two INTC(interrupt controller). ADuCM355 also has two, **AND** INTC0's output is connected to MCU's P2.1 pin inside of the chip. Check file aducm355examples/common/ADuCM355Port.c to see the code where configure P2.1 as interrupt input.

# How to use it.
There are plenty example codes for both ADuCM355 and AD5940. They use this library, so start from the example code is easier. Check the repositories below for example code(To be added).

[aducm355examples]()

[ad5940examples]()

- For ADuCM355, you can use it directly by define macro CHIPSEL_M355. 
- For AD594x, you should define macro CHIPSEL_594X. Basically, you only need to provide the low level APIs to access AD594X.

## Port to another hardware platform
The library only requires a SPI(4 wire) and an interrupt input pin(or use a normal GPIO in poll mode).

Below functions should be provided at least to make the library work.

- void      AD5940_CsClr(void);
- void      AD5940_CsSet(void);
- void      AD5940_RstClr(void);
- void      AD5940_RstSet(void);
- void      AD5940_Delay10us(uint32_t time);
- void      AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,unsigned char *pRecvBuff,unsigned long length);

# License
Copyright (c) 2019 Analog Devices, Inc.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.  
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors/products manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner that would cause the software to become subject to terms and conditions which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one or more patent holders.  This license does not release you from the requirement that you obtain separate licenses from these patent holders to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

2019-01-10-7CBSD SLA
