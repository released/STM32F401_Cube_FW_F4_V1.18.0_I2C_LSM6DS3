/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MEMS_CALCULATE_H
#define __MEMS_CALCULATE_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "Macro.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx_LSM6DS3.h"

//Add to calculate angle
#include "kalman_filter.h"

/* Define config -------------------------------------------------------------*/

#define	ENABLE_AVERAGE_FILTER
#define	ENABLE_KALMAN_FILTER

/* Macro ---------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

void MEMS_Calibration(void);

void Accelerator_filter(void);
void Gyroscope_filter(void);

void Tilt_Angle_Calculate(void);
/* Exported constants --------------------------------------------------------*/

#endif  /* __MEMS_CALCULATE_H */

