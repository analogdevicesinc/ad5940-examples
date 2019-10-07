# AD5940 Examples
[AD5940](https://www.analog.com/en/products/ad5940.html) is the latest high precision impedance and electrochemical front end. It communicates with external MCU via SPI bus. 

This repository targets to provide rich examples for you to get start with and provide system level examples like EDA(electrodermal activity), BIA(Body Impedance Analysis) which you can use directly in your project.

# Useful links
* [AD5940 Wiki](https://wiki.analog.com/resources/eval/user-guides/ad5940)
* [AD5940 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/AD5940.pdf)
* [AD5940 FAQ](https://ez.analog.com/data_converters/precision_adcs/w/documents/14012/ad5940-faqs)
* GUI tool used with Evaluation Board: SensorPal v1011 ftp://ftp.analog.com/pub/MicroConverter/ADxxxxV2.0/SensorPal%20Installer_v1011.exe
* Evaluation Boards:
  - [EVAL-AD5940BIOZ](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD5940BIOZ.html) Board information: [Wiki](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/hardware/eval-ad5940bioz)
  - [EVAL-AD5940ELEC](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD5940ELCZ.html) Board information: [Wiki](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/hardware/eval-ad5940elcz)

# How to Use it
Firstly, checkout the repository using below command. Make sure you have cloned the [ad5940lib](https://github.com/analogdevicesinc/ad5940lib) submodule which these examples are based on.

> git clone --recursive https://github.com/analogdevicesinc/ad5940-examples.git

If you are downloading code from web browser, it won't automatically download folder examples/ad5940lib, please download it manually from [ad5940lib](https://github.com/analogdevicesinc/ad5940lib) and extract it to examples/ad5940lib.

The final file structure should be:

```
ad5940-examples\doc...
ad5940-examples\examples\ad5940lib\ad5940.c
ad5940-examples\examples\ad5940lib\ad5940.h
ad5940-examples\LICENSE...
ad5940-examples\README.md...
```

To run the example code, you need the AD5940 evaluation board, and the software development IDE like IAR or [Keil](http://www.keil.com).

**Keil is verified on all examples and it's recommended.**

If you want to use IAR, do not forget to manually install ADuCM3029 Device Support Pack(ADuCM302x_DFP) from IAR CMSIS-Pack-Manager, under tab 'Packs/AnalogDevices.ADuCM302x_DFP'.

## Hardware
Currently, there are two kinds of EVB. 
* [EVAL-AD5940BIOZ](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD5940BIOZ.html) is used for healthcare application like EDA/BIA/ECG. You can find the related introduction in [Wiki](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/hardware/eval-ad5940bioz). 
![pic](https://www.analog.com/-/media/analog/en/evaluation-board-images/images/eval-ad5940biozangle-web.gif?h=270&thn=1&hash=C0C6E2638C3E12641F9D79A0121B56AAB7003391)
* [EVAL-AD5940ELEC](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD5940ELCZ.html) is used for industrial application like Gas Detection with electrochemical sensor, Water Quality etc. It has on board socket for gas sensor and a BNC connector for PH sensor. The USB connector actually carries the analog signal from AD5940 which allows you to connect any other sensors so you can do either impedance measurement or used as potentiostat circuit. Find more introduction on [Wiki](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/hardware/eval-ad5940elcz).
![pic](https://www.analog.com/-/media/analog/en/evaluation-board-images/images/eval-ad5940elczangle-web.gif?h=270&thn=1&hash=C7A2DE91D5A315F0F4A167EBB83F8ECBE02EC79B)

## Software
By the time of writing, only IAR projects are checked into this repository. Keil project is still WIP.

On how to use the IAR project, follow [this link](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/tools/iar_setup_guide) from ADI Wiki.

# License
Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
