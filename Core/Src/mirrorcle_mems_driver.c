/***********************************************************************************
  * @file    mirrorcle_mems_driver.c
  * @brief   C Library for interfacing the digital (SPI) MEMS mirror driver board
  *          from Mirrorcle
  * @version 0.1
  * @date    2021-12-03
  * @license Apache 2.0
  * @author  Simon Burkhardt
  *
  *          FHNW University of Applied Sciences and Arts Northwestern Switzerland
  *          https://www.fhnw.ch/ise/
  *          https://github.com/fhnw-ise-qcrypt/mcp3564
  *
  *          GAP Quantum Technologies University of Geneva
  *          https://www.unige.ch/gap/qic/qtech/
  *
************************************************************************************
*/

#include "mirrorcle_mems_driver.h"

void MEMS_DRIVER_Write_Voltage(SPI_HandleTypeDef *hspi, uint8_t chan, uint8_t volt){

}

void MEMS_DRIVER_Write_Channel(SPI_HandleTypeDef *hspi){
	uint8_t cmd[3];
	// manually operate the !CS signal, because the STM32 hardware NSS signal is (sadly) useless

	/* @author: Michael Taylor
	 * @brief Check that max and min voltage levels will not be exceeded. Clamp if they are.
	 * */
	if(MEMS_DAC_ch_A > MEMS_DRIVER_MAX_DAC_WRITE){ // all the way +x
	  MEMS_DAC_ch_A = MEMS_DRIVER_MAX_DAC_WRITE;
	  MEMS_DAC_ch_B = MEMS_DRIVER_MIN_DAC_WRITE;
	} else if(MEMS_DAC_ch_A < MEMS_DRIVER_MIN_DAC_WRITE){
	  MEMS_DAC_ch_A = MEMS_DRIVER_MIN_DAC_WRITE;
	  MEMS_DAC_ch_B = MEMS_DRIVER_MAX_DAC_WRITE;
	} //endif check DAC_ch_A over / under volt
	if(MEMS_DAC_ch_B > MEMS_DRIVER_MAX_DAC_WRITE){ // all the way -x
	  MEMS_DAC_ch_B = MEMS_DRIVER_MAX_DAC_WRITE;
	  MEMS_DAC_ch_A = MEMS_DRIVER_MIN_DAC_WRITE;
	} else if(MEMS_DAC_ch_B < MEMS_DRIVER_MIN_DAC_WRITE){
	  MEMS_DAC_ch_B = MEMS_DRIVER_MIN_DAC_WRITE;
	  MEMS_DAC_ch_A = MEMS_DRIVER_MAX_DAC_WRITE;
	} //endif check DAC_ch_B over / under volt
	if(MEMS_DAC_ch_C > MEMS_DRIVER_MAX_DAC_WRITE){ // all the way +y
	  MEMS_DAC_ch_C = MEMS_DRIVER_MAX_DAC_WRITE;
	  MEMS_DAC_ch_D = MEMS_DRIVER_MIN_DAC_WRITE;
	} else if(MEMS_DAC_ch_C < MEMS_DRIVER_MIN_DAC_WRITE){
	  MEMS_DAC_ch_C = MEMS_DRIVER_MIN_DAC_WRITE;
	  MEMS_DAC_ch_D = MEMS_DRIVER_MAX_DAC_WRITE;
	} //endif check DAC_ch_C over / under volt
	if(MEMS_DAC_ch_D > MEMS_DRIVER_MAX_DAC_WRITE){ // all the way -y
	  MEMS_DAC_ch_D = MEMS_DRIVER_MAX_DAC_WRITE;
	  MEMS_DAC_ch_C = MEMS_DRIVER_MIN_DAC_WRITE;
	} else if(MEMS_DAC_ch_D < MEMS_DRIVER_MIN_DAC_WRITE){
	  MEMS_DAC_ch_D = MEMS_DRIVER_MIN_DAC_WRITE;
	  MEMS_DAC_ch_C = MEMS_DRIVER_MAX_DAC_WRITE;
	} //endif check DAC_ch_D over / under volt

	// 0b 0001 1000; // write to and update (C = 011) channel A DAC (A = 000), first 2 bits dont care.
	cmd[0] = 0x18;
	cmd[1] = MEMS_DAC_ch_A >> 8;   // uint16_t high byte
	cmd[2] = MEMS_DAC_ch_A & 0xff; // uint16_t low  byte
	HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd, 3, MEMS_DRIVER_SPI_TIMEOUT);
	HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_SET);

	// 0b 0b0001 1001; // write to and update (C = 011) channel B DAC (B = 001), first 2 bits dont care.
	cmd[0] = 0x19;
	cmd[1] = MEMS_DAC_ch_B >> 8;   // uint16_t high byte
	cmd[2] = MEMS_DAC_ch_B & 0xff; // uint16_t low  byte
	HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd, 3, MEMS_DRIVER_SPI_TIMEOUT);
	HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_SET);

	// 0b 0001 1010; // write to and update (C = 011) channel C DAC (B = 010), first 2 bits dont care.
	cmd[0] = 0x1A;
	cmd[1] = MEMS_DAC_ch_C >> 8;   // uint16_t high byte
	cmd[2] = MEMS_DAC_ch_C & 0xff; // uint16_t low  byte
	HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd, 3, MEMS_DRIVER_SPI_TIMEOUT);
	HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_SET);

	// 0b 0001 1011; // write to and update (C = 011) channel D DAC (B = 011), first 2 bits dont care.
	cmd[0] = 0x1B;
	cmd[1] = MEMS_DAC_ch_D >> 8;   // uint16_t high byte
	cmd[2] = MEMS_DAC_ch_D & 0xff; // uint16_t low  byte
	HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, cmd, 3, MEMS_DRIVER_SPI_TIMEOUT);
	HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_SET);

}

