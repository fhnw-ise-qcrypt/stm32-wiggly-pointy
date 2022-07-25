/*
PicoAmp.h
Michael Taylor
March 19th 2017

Header file for Mirrorcle PicoAmp MEMS mirror driver board Teensy drivers.
Full disclosure: I do not know how to write good drivers.

REQUIREMENTS:
This driver is written specifically for the Teensy 3.2, and requires existing libraries written for that part.
SPI.h SPI Library for Teensy 3.2
IntervalTimer.h Internal interval timers in Teensy 3.2

TODO:
 
*/

#ifndef _PICOAMP_MEMS_H
#define _PICOAMP_MEMS_H

// Includes
#include <Arduino.h>
#include <SPI.h>
#include <IntervalTimer.h>

// Defines
#define DRIVER_HV_EN_pin 8
#define FCLK_pin 9
#define slaveSelectPin 10


// Header things
// declare functions and any variables you want to be accessible 
class PicoAmp {

private:
// IntervalTimer object to run FCLK
IntervalTimer FCLK_timer;
// Period for FCLK, uses IntervalTimer object
// FCLK freq should be 60* desired LPF cutoff. Set for 300Hz now (18KHz ~=56us, /2 for toggle on off)
unsigned long FCLK_period_us = 28;
// state to toggle FCLK pin
int FCLK_state = LOW;
// buffer to hold DAC register bits (24 bits, 3 bytes)
uint8_t DAC_reg_data [3];


// function to set FCLK up (configure and run its interval timer)
static void FCLK_setup();
static void FCLK_ISR();

public:

static void Init();
static void Set_FCLK_period_us(unsigned long new_FCLK_period_us);
static void WriteDAC_reg();
static void HV_EN(int enable);

//static void WriteDAC_ch_raw();
//static void Write_X_raw();
//static void Write_Y_raw();
//static void Write_X_angle();
//static void Write_Y_angle();
}

extern PicoAmp PicoAmpMEMS;

#endif