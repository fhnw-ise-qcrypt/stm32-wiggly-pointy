/*
Mirrorcle PicoAmp digital driver board Teensy 3.2 driver library

Michael Taylor
March 19th 2017
*/

#include "PicoAmp.h"

// pre-instatiate the class, there is only one driver being accessed
// as soon as you include PicoAmp.h it should make the object 'PicoAmpMEMS'
PicoAmp PicoAmpMEMS;

// reach into PicoAmp class Init function (declared in header)
// Initial setup for driver board, including pin assignment and FCLK timer initialization.
void PicoAmp::Init(){
// transfer 24 bits to DAC register (3 bytes) at a time
// 0x280001 FULL_RESET
DAC_reg_data = 0x280001;
digitalWrite(slaveSelectPin,LOW);
SPI.transfer(DAC_reg_data,3);
digitalWrite(slaveSelectPin,HIGH);
// 0x380001 INT_REF_EN
DAC_reg_data = 0x380001;
digitalWrite(slaveSelectPin,LOW);
SPI.transfer(DAC_reg_data,3);
digitalWrite(slaveSelectPin,HIGH);
// 0x20000F DAC_EN_ALL
DAC_reg_data = 0x20000F;
digitalWrite(slaveSelectPin,LOW);
SPI.transfer(DAC_reg_data,3);
digitalWrite(slaveSelectPin,HIGH);
// 0x300000 LDAC_EN
DAC_reg_data = 0x300000;
digitalWrite(slaveSelectPin,LOW);
SPI.transfer(DAC_reg_data,3);
digitalWrite(slaveSelectPin,HIGH);

FCLK_setup();
}

void PicoAmp::FCLK_setup(){
FCLK_timer.begin(FCLK_ISR,FCLK_period_us);
FCLK.attachInterrupt(FCLK_ISR);
}

void PicoAmp::Set_FCLK_period_us(unsigned long new_FCLK_period_us){
FCLK_period_us = new_FCLK_period_us;
}

void PicoAmp::FCLK_ISR(void) {
  // toggle the FCLK pin every time
  if (FCLK_state == LOW){
    FCLK_state = HIGH;
  } else {
    FCLK_state = LOW;
  }
  digitalWrite(FCLK_pin,FCLK_state);
}

void PicoAmp::HV_EN(int enable){
	if (enable) {
		digitalWrite(DRIVER_HV_EN_pin,LOW);
	} else {
		digitalWrite(DRIVER_HV_EN_pin,LOW);
	}
}

void PicoAmp::WriteDAC_reg(unsigned long reg_input){
DAC_reg_data[1] = (reg_input >> 16) & 0xFF;
DAC_reg_data[2] = (reg_input >> 8) & 0xFF;
DAC_reg_data[3] = reg_input & 0xFF;
digitalWrite(slaveSelectPin,LOW);
SPI.transfer(DAC_reg_data,3);
digitalWrite(slaveSelectPin,HIGH);
}