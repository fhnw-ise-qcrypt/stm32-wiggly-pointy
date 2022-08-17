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

#ifndef INC_MIRRORCLE_MEMS_DRIVER_H_
#define INC_MIRRORCLE_MEMS_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mirrorcle_mirror_parameters.h"

/* Private defines -----------------------------------------------------------*/
#define MEMS_DRIVER_FILTER_FREQ_X (240)  // see calibration sheet of specific mirror
#define MEMS_DRIVER_FILTER_FREQ_Y (240)  // see calibration sheet of specific mirror
#define MEMS_DRIVER_ENABLE_GPIO_Port MEMS_HV_EN_GPIO_Port
#define MEMS_DRIVER_ENABLE_GPIO_Pin  MEMS_HV_EN_Pin
#define MEMS_DRIVER_SPI_CS_GPIO_Port SPI2_CS_GPIO_Port
#define MEMS_DRIVER_SPI_CS_GPIO_Pin  SPI2_CS_Pin
#define MEMS_DRIVER_SPI_TIMEOUT (3)
// PicoAmp and MEMS Voltages from datasheet
// This is a last defense against damaging the mirror by over-driving it
#define MEMS_DRIVER_MAX_V     (200)

/* @brief MEMS mirror bias voltage (taken from datasheet of specific mirror)
 * Example, if a device datasheet states Vbias of 80V:
 * The Vbias digital value would be (80/200)*65535 = 26214
 */
#define MEMS_VBIAS_CODE (uint32_t)((65535/MEMS_DRIVER_MAX_V)*MEMS_DRIVER_BIAS_V)

/* Private macros ------------------------------------------------------------*/
/* @important DISABLE HIGH VOLTAGE MEMS DRIVER !
 * @see   p. 24 of MEMS_Drivers_5.x_User_Guide.pdf
 * @note  ACTIVE HIGH --> LOW = disabled */
#define MEMS_DRIVER_HV_Disable() HAL_GPIO_WritePin(MEMS_DRIVER_ENABLE_GPIO_Port, MEMS_DRIVER_ENABLE_GPIO_Pin, GPIO_PIN_RESET);
#define MEMS_DRIVER_HV_Enable()  HAL_GPIO_WritePin(MEMS_DRIVER_ENABLE_GPIO_Port, MEMS_DRIVER_ENABLE_GPIO_Pin, GPIO_PIN_SET);

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
// Then calculate the range of allowable integers between 0-65535 that you can send to not break the mirror
static const uint16_t MEMS_DRIVER_MAX_DAC_WRITE  = 65536 * (MEMS_DRIVER_BIAS_V + MEMS_MIRROR_MAX_Vdiff/2)/MEMS_DRIVER_MAX_V;
static const uint16_t MEMS_DRIVER_MIN_DAC_WRITE  = 65536 * (MEMS_DRIVER_BIAS_V - MEMS_MIRROR_MAX_Vdiff/2)/MEMS_DRIVER_MAX_V;
static const uint16_t MEMS_DRIVER_BIAS_DAC_WRITE = 65536 * MEMS_DRIVER_BIAS_V/MEMS_DRIVER_MAX_V;
static const uint16_t MEMS_DRIVER_Vp_DAC_WRITE   = 65536 * MEMS_MIRROR_MAX_Vdiff/(2*MEMS_DRIVER_MAX_V);

static uint16_t MEMS_DAC_ch_A;
static uint16_t MEMS_DAC_ch_B;
static uint16_t MEMS_DAC_ch_C;
static uint16_t MEMS_DAC_ch_D;

/* Exported functions prototypes ---------------------------------------------*/
/* @todo write a function that converts floating point angles to differential voltages for DAC channels */
void MEMS_DRIVER_SetAngle(float phi_x, float phi_y);
void MEMS_DRIVER_Write_Channel(SPI_HandleTypeDef *hspi);
void MEMS_DRIVER_Init(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* INC_MIRRORCLE_MEMS_DRIVER_H_ */
