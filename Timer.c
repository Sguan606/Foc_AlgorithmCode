/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-08 19:26:48
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-05-09 19:48:21
 * @FilePath: \test_2804FocMotor\Hardware\Timer.c
 * @Description: TIM定时中断统一管理函数;
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Timer.h"

/**
 * @description: TIM2中断回调函数，1ms的定时器定时中断;
 * @param {TIM_HandleTypeDef*} htim
 * @return {*}
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {  // 1kHz中断
        FOC_Control_Loop();
        // FOC_SpeedSet(5.0f);
        Key_Tick();
        Stepper_Tick();
    }
}



