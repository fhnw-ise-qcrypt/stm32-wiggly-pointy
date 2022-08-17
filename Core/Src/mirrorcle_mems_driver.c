/***********************************************************************************
  * @file    mirrorcle_mems_driver.c
  * @brief   C Library for interfacing the digital (SPI) MEMS mirror driver board
  *          from Mirrorcle
  * @version 1.0
  * @date    2022-08-17
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

void MEMS_DRIVER_Init(SPI_HandleTypeDef *hspi){
  uint8_t dac_data[8];

  /* @brief MEMS mirror DAC setup
   * Set up DAC. Following the AD5664 DAC datasheet, we recommend the following
   * initialization sequence which must be run by the master controller which
   * communicates commands to the PicoAmp on every power up of the PicoAmp.
   * The sequence is to reset the DAC, turn on its internal reference,
   * enable all 4 channels, and set up for software loading.
   *
   * 2621441 Decimal or 0x280001 to command FULL RESET
   * 3670017 Decimal or 0x380001 to command ENABLE INTERNAL REFERENCE
   * 2097167 Decimal or 0x20000F to command ENABLE ALL DAC CHANNELS
   * 3145728 Decimal or 0x300000 to command ENABLE SOFTWARE LDAC
   */

  // FULL RESET
  dac_data[0] = 0x28;
  dac_data[1] = 0x00;
  dac_data[2] = 0x01;
  HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, dac_data, 3, MEMS_DRIVER_SPI_TIMEOUT);
  HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_SET);

  // ENABLE INTERNAL REFERENCE
  dac_data[0] = 0x38;
  dac_data[1] = 0x00;
  dac_data[2] = 0x01;
  HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, dac_data, 3, MEMS_DRIVER_SPI_TIMEOUT);
  HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_SET);

  // ENABLE ALL DAC CHANNELS
  dac_data[0] = 0x20;
  dac_data[1] = 0x00;
  dac_data[2] = 0x0F;
  HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, dac_data, 3, MEMS_DRIVER_SPI_TIMEOUT);
  HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_SET);

  // ENABLE SOFTWARE LDAC
  dac_data[0] = 0x30;
  dac_data[1] = 0x00;
  dac_data[2] = 0x00;
  HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(hspi, dac_data, 3, MEMS_DRIVER_SPI_TIMEOUT);
  HAL_GPIO_WritePin(MEMS_DRIVER_SPI_CS_GPIO_Port, MEMS_DRIVER_SPI_CS_GPIO_Pin, GPIO_PIN_SET);
}

void MEMS_DRIVER_SetAngle(float phi_x, float phi_y){
	/* @todo calibrate angle to voltage */
	if(phi_x > ( MEMS_MIRROR_ANGLE_MAX_X)) phi_x = ( MEMS_MIRROR_ANGLE_MAX_X);
	if(phi_x < (-MEMS_MIRROR_ANGLE_MAX_X)) phi_x = (-MEMS_MIRROR_ANGLE_MAX_X);
	if(phi_y > ( MEMS_MIRROR_ANGLE_MAX_Y)) phi_y = ( MEMS_MIRROR_ANGLE_MAX_Y);
	if(phi_y < (-MEMS_MIRROR_ANGLE_MAX_Y)) phi_y = (-MEMS_MIRROR_ANGLE_MAX_Y);

	// Equations: see Datasheet p. 31

	MEMS_DAC_ch_A = (uint32_t) (MEMS_VBIAS_CODE + ( phi_x*MEMS_DRIVER_VGAIN*65535/MEMS_DRIVER_MAX_V) );
	MEMS_DAC_ch_B = (uint32_t) (MEMS_VBIAS_CODE - ( phi_x*MEMS_DRIVER_VGAIN*65535/MEMS_DRIVER_MAX_V) );
	MEMS_DAC_ch_C = (uint32_t) (MEMS_VBIAS_CODE + ( phi_y*MEMS_DRIVER_VGAIN*65535/MEMS_DRIVER_MAX_V) );
	MEMS_DAC_ch_D = (uint32_t) (MEMS_VBIAS_CODE - ( phi_y*MEMS_DRIVER_VGAIN*65535/MEMS_DRIVER_MAX_V) );

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

