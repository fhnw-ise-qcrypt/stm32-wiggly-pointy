/***********************************************************************************
  * @file    mirrorcle_MEMS_MIRROR_parameters.h
  * @brief   device specific parameters of the MEMS mirror
  *          (one Device Parameters Document per Serial Number)
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

#ifndef INC_MIRRORCLE_MEMS_MIRROR_PARAMETERS_H_
#define INC_MIRRORCLE_MEMS_MIRROR_PARAMETERS_H_

#define MEMS_MIRROR_SERIAL_NUMBER "S40214"

#define MEMS_MIRROR_ANGLE_MAX_X (1.0263f)
#define MEMS_MIRROR_ANGLE_MAX_Y (1.0528f)
#define MEMS_MIRROR_MAX_Vdiff (158)
#define MEMS_MIRROR_CUTTOFF_F (240)

#endif /* INC_MIRRORCLE_MEMS_MIRROR_PARAMETERS_H_ */
