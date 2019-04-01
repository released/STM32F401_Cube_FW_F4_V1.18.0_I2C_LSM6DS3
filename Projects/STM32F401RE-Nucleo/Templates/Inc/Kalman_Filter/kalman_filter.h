#ifndef __filter_H
#define __filter_H

#include "stm32f4xx_hal.h"
#include <stdio.h>

void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f);

#endif
