AD5940 Examples Change Log
==========================
## 0.3.0(onging)

### Added
* Add notch filter example in AD5940_ADC.

### Changed
* EDA:
  - Fix bug in ISR that count is not cleared when there is error.
  - Added method to check if EDA is stopped.
* Update ADC example to include PGA calibration.
* Ramp example update.
  - Update default gain to 1.5.
  - Update current calculation equation.
  - Add control to restart test.
* ECSNS_EIS example update
  - Remove redundant variable SensorBias
  - Modified initialization sequence to use BiasVolt parameter to set DC bias
  - Fixed bug in setting ac pk-pk voltage
* Update ad5940lib submodule.
* Improve SPI speed of ADuCM3029 by using FIFO.
  - Remove clock enable/disable option in ADCFilterCfg.
  - Added clock calculator for notch filter data.
  - Fix error of LPTIA RTIA ideal value table.
  - Fix bug the HSTIA RTIA calibration result is not steady.
  - Fix bug in PGACal, function rewrited.

### Removed
* Remove ADCFILTERCON bit[18:16] which should be reserved as value zero.
  - Existing examples are updated to remove related code.


## 0.2.1(2019-11-7)

### Added
* Add Keil example projects for board [NUCLEO-F411RE](https://www.st.com/en/evaluation-tools/nucleo-f411re.html).
* Add Keil example projects for board [EVAL-ADICUP3029](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-ADICUP3029.html)
* Add on-chip temperature sensor example.

### Changed
* Clean up comments, fix spell error.
* BAT-Impedance example updates to use HFOSC clock. Add RCAL measurements for all sweep points.
* Update ad5940lib submodule.
  - Fix bug that SeqGenDB should be initialized.
  - Added DE1 node configuration, prepared for ADuCM355
  - Version number to 0.2.1

### Deprecated

### Fixed
* Fix frequency sweep in example BioElec, BIA, BIOZ-2wire.
* ad5940.h: fix FIFO_STREAM bug.
* fix warnings.

## 0.2.0(2019-4-23)

### Added
* **Add support to ADuCM355. The library(ad5940lib) support both Lower power channel 0 and channel1(only available for ADuCM355). Channel0 must be specified for AD594x.**
* Add HSDAC calibration routine
* Add Battery Impedance example which need hardware from CircuitNote.

### Changed
* Update ad5940lib to submodule which is also available on [ADI Github](https://github.com/analogdevicesinc/ad5940lib)
* Use CMSIS pack for MCU related library.
* Keil projects are removed and will be added back when license is ready.

### Deprecated
* Function AD5940_LPDACWriteS is replaced by AD5940_LPDAC0WriteS and AD5940_LPDAC1WriteS respectively.

### Fixed
* Fix warnings.

## 0.1.6(2018-10-31)

### Added
* Add macro to identify channel ID of FIFO data.
* Add new trim word to function AD5940_Initialize()
* Add GPIO function AD5940_AGPIOIn() to read GPIO status.
* Add example to show how to use Statistic Block to get mean result of ADC data. Example locates at folder AD5940_ADC.
* Add function AD5940_HSTIARtiaCfgS() to configure HSTIA RTIA resistor. This is commonly used when there is need to dynamically change RTIA resistor when measuring impedance.
* Add function AD5940_LPDACCal() to 'calibrate' LPDAC. 
* Add function AD5940_SEQCycleTime to calculate number of ACLK cycles that a generated sequence will take.
* Add example Biased-EIS, this example shows how to do impedance and potentiostat together.

### Changed
* Change name SWP_RE1 to SWP_AFE2
* Change Impedance example default parameters to measure EVAL-AD5940ELECZ on-board dummy sensor impedance. 
* **Update data sheet to PrN.**
* Update SensorPal to v1011.

### Deprecated

### Removed
* **SILICON_VER macro in AD5940.H is removed. The firmware will automatically detect the silicon version based on register REG_AFECON_CHIPID in function AD5940_Initialize().**

### Fixed
* Fix bug in function AD5940_ADCCode2Volt() that should use signed variable.
* Fix AD5940_LPloop example that AD5940_Initialize() function should be called right after HW reset.
* Fix bug in function AD5940_LPDACCal that the equation used to calculate voltage from ADC code should include a 'kFactor', 1.835/1.82.

## 0.1.5(2018-7-24)

### Added
* Add more comments.
* Add example to test basic SPI register read/write.
* Add BIOZ-2Wire to do 2-wire isolated impedance measurement. This is modified from BIA example. The difference is we measure the excitation voltage instead of body voltage.
* Add AGPIO related functions.
* Add function to configure GPIO to trigger sequence.
* Add Sequencer example to show basic operations of sequencer.
* Add STM8 project example and its library.

### Changed
* Change GPIO related function return type to void rather than uint32_t.
* Change wait time in function AD5940_HWReset to 200us to ensure AD5940 has exited reset state.
* Modify variable type to make it running on STM8L(AD5940_SPI example and AD5940.C/.H)
* Add Reset example to show three kinds of reset source of AD5940.
* Add function AD5940_GetChipID().
* Add schematics. EVAL-AD5940-RevC, EVAL-AD5940LowCurrent-RevA, AD5940 Z Test.
* Set pin RESET and CS in file xxxPORT.C.
* **Update Application Note.**

### Fixed
* Fix bug in BIA example that DE0 is connected to HSTIA output. It should be left OPEN.
* Fix bug in BIA that RTIA calibration is fixed to 1K. It should be AppBIACfg.HstiaRtiaSel.
* Fix WG example to add Trapezoid waveform generator.
* Improve 'Impedance' example loop stability. Do not change switch matrix when the loop is turned ON.
* Fix bug in function AD5940_CLKCfg that only configure HFOSC mode when it's enabled. Add some delay when clock configured. ADC clock divider is not configured.
* Fix bug in function AD5940_Initialize that use sizeof to calculate byte of structure element.

## 0.1.4(2018-6-13)

### Added
* ADC PGA offset and gain calibration function.
* Add LPTIA offset error calibration function.
* Add function used to translate ADC code to voltage.
* LPTIA RTIA calibration add DC method. Set frequency to zero means do DC calibration.
* Add PrM data sheet.
* Add warning to select correct silicon version number. Will remove this after release.

### Changed
* Change RAMP example LPAMP power mode to BOOST3 to drive capacitive load.
* Simplify SPI operation. The SPI commands are reduced to 4. 
* Change LPTIA RTIA calibration method to LPDAC. Using HSLOOP is not recommended.
* Use sequencer to control WG/DFT for LPTIARTIA calibration routine to guarantee phase accuracy.
* Change Pin name of AFE1/2/3/4.

### Fixed
* Fix bug in function AD5940_Delay10us. The delay won't work if the time is too long.
* Fix function AD5940_ClksCalculate(). Change the DataType back to its original value when calculation done.
* Fix AD5940_ClksCalculate() when use SINC2 as data source, it need 15 extra clocks.
* Fix RAMP bug. The CodePerStep should be always positive.
* Fix LPTIA calibration bug. The table for Rtia resistor is wrong(12k and 10k).
* Fix bug in function AD5940_EnterSleepS(). Need to clear register REF_AFE_SEQTRGSLP before set it to 1.
* Fix BIA ISR function. Hibernate related codes changed.

## 0.1.3 (2018-5-14)

### Fixed
* Fix structure for AD5940_ClksCalculate() for ECG example to calculate correct number of clocks for ADC data capture

## 0.1.2a(2018-5-14)

### Fixed
* Fix Impedance issue that switches settings are fixed to SE0

## 0.1.2(2018-5-14)

### Added
* Ramp example: add support of external RTIA.

### Changed
* EDA example use SINC3 OSR changes from 2 to 5 for lower noise. Change excitation signal amplitude to 0.75*1100mV. Use 250us for HPREF to settle. 
* EDA: Add function to deal with EDA impedance baseline. The output is referred to baseline. Add UART commands: START/STOP/SETBASE/RSTBASE/GETAVR.

### Deprecated

### Removed

### Fixed
* Fix some project setting to include correct source files.
* Fix EDA example, the "repeat convert" function won't work correctly. Re-write function AD5940_LPModeCtrlS. Issue is LPMODECON.BIT3 should be cleared immediately after set it.

## 0.1.1(2018-4-19)

### Added
* *Support Silicon version 2 now. Change this by macro SILICON_VER locates in file AD5940.H*
* Add HSTIA DE node RTIA 50Ohm/100Ohm/200Ohm settings.
* Add SWT_DE0 and SWT_DE1 option.
* Add BiasVolt option to Impedance example. The excitation signal is now composed of AC+DC.

### Changed
* Use AD5940_HWReset() to reset AD5940.

### Deprecated
* Silicon version 1 won't be supported in future.
* The name of channel 1 pins will be replaced by AFE1 to AFE4.

### Removed
* Remove LpDacMode option. This is always over-ride by LpDacSW. For Silicon 1, LPDAC mode is set to normal mode.

### Fixed
* Fix bug in function AD5940_LPDACCfgS(), PowerEn and DataRst bit process is incorrect; AD5940_ClksCalculate(), ADC SINC2 and SINC3 OSR settings are index value not real OSR value. For example, 0 means ADCSINC3_OSR5.
* Fix Impedance example bug. The last sequencer command is STOP not NOP in initialization sequence. 

## 0.1.0(2018-4-13)

* Initial release of project