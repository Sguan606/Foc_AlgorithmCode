#ifndef __FOC_H
#define __FOC_H

#include "stm32g4xx_hal.h"
#include "fast_sin.h"
#include "svpwm.h"
#include "as5600.h"
#include "ina240A2.h"

#define POLE_PAIRS 7                        // 电机极对数（根据电机参数12M14P）

float RadToDegrees(float angle_rad);
void FOC_Init(void);
void FOC_GetPhaseVoltages(uint16_t Angle, uint32_t* Ua, uint32_t* Ub, uint32_t* Uc);
void FOC_PwmSet(uint16_t desired_angle);
void FOC_SpeedSet(float Speed);


#endif  //F0C_H

