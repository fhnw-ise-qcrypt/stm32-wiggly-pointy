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

#ifndef INC_MIRRORCLE_MEMS_DRIVER_H_
#define INC_MIRRORCLE_MEMS_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mirrorcle_mirror_parameters.h"

/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define MEMS_DRIVER_FILTER_FREQ_X (240)  // see calibration sheet of specific mirror
#define MEMS_DRIVER_FILTER_FREQ_Y (240)  // see calibration sheet of specific mirror
#define MEMS_DRIVER_ENABLE_GPIO_Port SPI1_CS_GPIO_Port // todo
#define MEMS_DRIVER_ENABLE_GPIO_Pin  SPI1_CS_Pin // todo
#define MEMS_DRIVER_SPI_CS_GPIO_Port SPI1_CS_GPIO_Port // todo
#define MEMS_DRIVER_SPI_CS_GPIO_Pin  SPI1_CS_Pin // todo
#define MEMS_DRIVER_SPI_TIMEOUT (3)
// PicoAmp and MEMS Voltages from datasheet
// This is a last defense against damaging the mirror by over-driving it
#define MEMS_DRIVER_MAX_V     (200) //
#define MEMS_DRIVER_MAX_Vdiff (150) // max diff drive voltage from device datasheet
#define MEMS_DRIVER_BIAS_V    (80)  // bias voltage from datasheet (need to ask if biasing at 100V is ok)

// Then calculate the range of allowable integers between 0-65535 that you can send to not break the mirror
const uint16_t MEMS_DRIVER_MAX_DAC_WRITE  = 65536 * (MEMS_DRIVER_BIAS_V + MEMS_DRIVER_MAX_Vdiff/2)/MEMS_DRIVER_MAX_V;
const uint16_t MEMS_DRIVER_MIN_DAC_WRITE  = 65536 * (MEMS_DRIVER_BIAS_V - MEMS_DRIVER_MAX_Vdiff/2)/MEMS_DRIVER_MAX_V;
const uint16_t MEMS_DRIVER_BIAS_DAC_WRITE = 65536 * MEMS_DRIVER_BIAS_V/MEMS_DRIVER_MAX_V;
const uint16_t MEMS_DRIVER_Vp_DAC_WRITE   = 65536 * MEMS_DRIVER_MAX_Vdiff/(2*MEMS_DRIVER_MAX_V);

uint16_t MEMS_DAC_ch_A;
uint16_t MEMS_DAC_ch_B;
uint16_t MEMS_DAC_ch_C;
uint16_t MEMS_DAC_ch_D;

/* @todo write a function that converts floating point angles to differential voltages for DAC channels */
void MEMS_DRIVER_SetAngle(SPI_HandleTypeDef *hspi, float phi_x, float phi_y);
void MEMS_DRIVER_Write_Channel(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif


#endif /* INC_MIRRORCLE_MEMS_DRIVER_H_ */
