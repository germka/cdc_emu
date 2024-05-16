# SAAB compact disk changer emulator

An easy way to add new multimedia functionality to your old car, based on the STM32 microcontroller and bluetooth receiver module with Apt-X/AAC support (CSR8645, RRD-305, QCC3005) which communicates via the CAN BUS protocol and completely simulates the presence and functionality of the original device.

![3d render](Pcb/3D_PCB.png)

## Features

Support AVCTP control from steerling wheel buttons and IHU buttons;

Reducing the volume level on revese gear shaft position;

## Installation

Can be installed in the original place in the trunk of the car;

## Documentation

- [PCB project](https://oshwlab.com/germka/cdcemu_stm32)
- [3D modeling](https://www.tinkercad.com/things/ilOajaed17D-saabcdcemuv20?sharecode=B5w6PaUzLr1GzISFAL70bzKVDpNO_U5eLHJtqSOlsjQ)

## Compiling

Project maked in STM32CubeIDE and including only user files, but system code (HAL and RTOS) can be regenerated in STM32Cube and compiled in another IDE or manualy with toolchain arm-none-eabi-gcc;

## Flashing

For programming MCU used ST-LINK and STM32CubeProgrammer;

Bluetooth reciever need to bee reconfigured to use input control levels instead impulse, CCID and another settings, you can make it with FT232 USB module [:howto:](https://bois083.wordpress.com/2016/10/08/playing-audio-files-with-csr8645-bluetooth-chip/) or with CSR programmer;

CSR programmer can use developer sowtware (ADK_CSR867x, ADK_QCC300x) and configure the chip for maximum performance, ft232 can use HeadsetConfigToolS and BlueSuite with low perfomance and the probability of errors;

## Hardware

- Standard CDC harness `827229-1` [aliexpress](https://aliexpress.ru/wholesale?SearchText=827229-1)
- Standard CDC dual harness (2002+) `1-962344-1` [2 pin](https://aliexpress.ru/wholesale?SearchText=1-962344-1) `1-965426-1` [4 pin](https://aliexpress.ru/wholesale?SearchText=1-965426-1)
- PCB order [link](https://jlcpcb.com/) or [link for RU and BY](https://www.pcbwave.com/)
- CSR programmer [link](https://aliexpress.ru/wholesale?SearchText=CSR+Bluetooth+burner)
- bluetooth module [link](https://aliexpress.ru/wholesale?SearchText=RRD305)
- Electronic components (use bom file) [link](https://www.lcsc.com/) or [ChipDip](https://www.chipdip.ru/)
- ST-LINK [link](https://aliexpress.ru/wholesale?SearchText=st-link)

## TODO

- Power save mode (sleep)

## Known issues

- To use Hands-Free Profile with a standard microphone, you need to lay a cable in the trunk of the car from the multimedia device;
