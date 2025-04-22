#ifndef __INA240A2_H
#define __INA240A2_H

#include "stm32g4xx_hal.h"

#define ADC_BUFFER_SIZE 32
extern float INA240_voltage1;
extern float INA240_voltage2;
extern uint32_t ina240Buffer1[ADC_BUFFER_SIZE];
extern uint32_t ina240Buffer2[ADC_BUFFER_SIZE];

typedef struct 
{
    float CH1;
    float CH2;
    float CH3;
}INA240_ValueTypeDef;


void INA240_Init(void);
float INA240_GetValue(void);
float INA240_GetValue_DMA(ADC_HandleTypeDef *hadc);
void INA240_FocValue(INA240_ValueTypeDef *Channel);

#endif

