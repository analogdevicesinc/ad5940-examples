# AD5940 Examples
[AD5940](https://www.analog.com/en/products/ad5940.html) is the latest high precision impedance and electrochemical front end. It communicates with external MCU via SPI bus. 

This repository targets to provide rich examples for you to get start with and provide system level examples like EDA(electrodermal activity), BIA(Body Impedance Analysis) which you can use directly in your project.

# Useful links
* [AD5940 Wiki](https://wiki.analog.com/resources/eval/user-guides/ad5940)
* [AD5940 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/AD5940.pdf)
* [AD5940 FAQ](https://ez.analog.com/data_converters/precision_adcs/w/documents/14012/ad5940-faqs)
* [SensorPal](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/tools/sensorpal_setup_guide) The GUI tool used with Evaluation Board
* Evaluation Boards:
  - [EVAL-AD5940BIOZ](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD5940BIOZ.html) Board information: [Wiki](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/hardware/eval-ad5940bioz)
  - [EVAL-AD5940ELEC](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD5940ELCZ.html) Board information: [Wiki](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/hardware/eval-ad5940elcz)

# How to Use it
Firstly, checkout the repository using below command. Make sure you have cloned the [ad5940lib](https://github.com/analogdevicesinc/ad5940lib) submodule which these examples are based on.

> git clone --recursive https://github.com/analogdevicesinc/ad5940-examples.git

To run the example code, you need the AD5940 evaluation board. And the software development IDE like IAR or Keil. 
## Hardware
Currently, there are two kinks of EVB. 
* [EVAL-AD5940BIOZ](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD5940BIOZ.html) is used for healthcare application like EDA/BIA/ECG. You can find the related introduction in [Wiki](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/hardware/eval-ad5940bioz). 
![pic](https://www.analog.com/-/media/analog/en/evaluation-board-images/images/eval-ad5940biozangle-web.gif?h=270&thn=1&hash=C0C6E2638C3E12641F9D79A0121B56AAB7003391)
* [EVAL-AD5940ELEC](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD5940ELCZ.html) is used for industrial application like Gas Detection with electrochemical sensor, Water Quality etc. It has on board socket for gas sensor and a BNC connector for PH sensor. The USB connector actually carries the analog signal from AD5940 which allows you to connect any other sensors so you can do either impedance measurement or used as potentiostat circuit. Find more introduction on [Wiki](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/hardware/eval-ad5940elcz).
![pic](https://www.analog.com/-/media/analog/en/evaluation-board-images/images/eval-ad5940elczangle-web.gif?h=270&thn=1&hash=C7A2DE91D5A315F0F4A167EBB83F8ECBE02EC79B)

## Software
Both IAR and Keil example projects are provided.

On how to use the IAR project, follow [this link](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/tools/iar_setup_guide) from ADI Wiki.

For Keil project, follow [this link](https://wiki.analog.com/resources/eval/user-guides/eval-ad5940/tools/keil_setup_guide).

**Note: CMSIS pack is used in all examples, make sure you installed the pack no matter in Keil or IAR.**


# License
Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.
