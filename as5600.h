#ifndef __AS5600_H
#define __AS5600_H

#include "stm32g4xx_hal.h"
#include <math.h>

uint16_t AS5600_ReadData(void);
float AS5600_GetAngle(void);
float AS5600_GetVelocity(void);
float AS5600_GetAcceleration(void);
float AS5600_GetAngleRad(void);
float AS5600_GetVelocityRad(void);
float AS5600_GetRPM(void);


#endif
