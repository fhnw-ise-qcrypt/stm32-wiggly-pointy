<a href="https://fhnw.ch/ise"><img src="./img/fhnw.jpg" alt="FHNW Logo" width=50%/></a><a href="https://www.unige.ch/gap/qic/"><img src="./img/unige.jpg" alt="UNIGE Logo" width=15%/></a>

# PSD and MEMS mirror based laser tracking and pointing

stm32f373 firmware for tracking laser error detection on a 2D-Positional Sensitive Photodiode (PSD) and correction of the downward pointing beam using a MEMS micro mirror.
Working in the range of +/- 1°.

[![Build Status](https://jenkins.kaon.ch/buildStatus/icon?job=QSIT_MEMS_Firmware)](https://jenkins.kaon.ch/job/QSIT_MEMS_Firmware/)

---

![img/QSIT_MEMS_driver_loop.drawio.png](img/QSIT_MEMS_driver_loop.drawio.png)

---

### Hardware

KiCAD:

[fhnw-ise-qcrypt/qsit_psd_payload_module_kicad](https://github.com/fhnw-ise-qcrypt/qsit_psd_payload_module_kicad)

#### MEMS mirror light path
![img/MEMS_mirror_CAD.png](img/MEMS_mirror_CAD.png)
![img/MEMS_mirror_light_path.png](img/MEMS_mirror_light_path.png)

#### Calibration Setup
![img/desk_setup_calibration.png](img/desk_setup_calibration.png)

#### All Circuit Boards
![img/all_boards.png](img/all_boards.png)

#### Hamamatsu PSD Board
![img/PSD_board.jpg](img/PSD_board.jpg)

#### Control Board (STM32f373)
![img/STM32_board.jpg](img/STM32_board.jpg)


---

### Tasks

- ✅ test PCB board no. 1
	+ ✅ USART debug communication with PC working
	+ ✅ LEDs working
	+ ✅ SPI, ADC working (responding)
- ✅ seperate PSD board
	+ ✅ place PSD sensor 
	+ ✅ place potentiometer
	+ ✅ configure offest voltage --> HOW?? any positive voltage 0...5V seems to work
- ✅ **MCP3564** ADC from PSD functionality
	+ ✅ ADC has IRQ data "streaming"
	+ 🔄 calibrate ADC voltage values
	+ 🔄 calibrate PSD position values (**how?**)
- 🔄 **AD5664R** DAC for MEMS mirror functionality
	+ ✅ include DAC library
	+ ✅ generate FCLK for driver board using TIM
	+ ✅ test ⚡️200V⚡️ DC driver without MEMS mirror (sine patterns)
	+ ✅ test pattern driver with MEMS mirror
	+ 🔄 angle-to-DAC linearization/calibration from mirrorcle datasheet --> is this necessary?
- ✅ add watchdog timer if ADC stops working
- 🗓 calibrate full system with laser, PSD and mirror

---

( ✅ 🔄 ⚠️ 🗓 🔘 )

--- 

### Important Notes

Active UART communication severely impacts the signal measured by the ADC.

Using single polling and printing to UART (measure, print, measure, print...) produces 20mVpp errors!!!

Using multiple polling without active UART (measure, measure, measure, print...) produces 1mVpp errors only.





