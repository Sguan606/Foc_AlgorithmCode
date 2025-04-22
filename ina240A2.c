/*
 * @Author: 星必尘Sguan
 * @Date: 2025-04-19 22:08:33
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-04-22 16:01:38
 * @FilePath: \test_2804FocMotor\Hardware\ina240A2.c
 * @Description: 使用INA240A2测量3相无刷电机的相电流;
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "ina240A2.h"

//如有多个采样，在此添加ADC通道
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

//如有多个采样，在此添加ADC缓存
uint32_t ina240Buffer1[ADC_BUFFER_SIZE];
uint32_t ina240Buffer2[ADC_BUFFER_SIZE];
// float INA240_voltage1;
// float INA240_voltage2;


/**
 * @description: DMA开启函数，要在cubeMX中开启DMA循环模式;
 * @return {*}
 */
void INA240_Init(void) 
{   //如有多个采样，在此添加ADC使能
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ina240Buffer1, ADC_BUFFER_SIZE);  // 开启ADC1的循环模式
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ina240Buffer2, ADC_BUFFER_SIZE);  // 开启ADC2的循环模式
}


/**
 * @description: 阻塞式ADC1的采样，读取模拟电压值变化大小（不推荐使用）;
 * @return {*}
 */
float INA240_GetValue(void)
{
    float temp;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
    uint32_t raw_val = HAL_ADC_GetValue(&hadc1);
    temp = raw_val*(2.0f - 0.0f)/4095.0f -1.0f;
    return temp;
}


/**
 * @description: (ADC模拟电压)直接读取DMA缓冲区的最新值
 * @return {*}
 */
float INA240_GetValue_DMA(ADC_HandleTypeDef *hadc) 
{
    if (hadc == &hadc1){
        uint32_t raw_val = ina240Buffer1[0] & 0xFFF;  // 确保提取12位有效值
        return raw_val * 2.0f / 4095.0f - 1.0f;
    }
    else if (hadc == &hadc2){
        uint32_t raw_val = ina240Buffer2[0] & 0xFFF;  // 确保提取12位有效值
        return raw_val * 2.0f / 4095.0f - 1.0f;
    }
    return 0.0f;
}


/**
 * @description: 获取三相电流值并存储到结构体
 * @param {INA240_ValueTypeDef*} Channel: 存储三相电流值的结构体指针
 * @return {void}
 */
void INA240_FocValue(INA240_ValueTypeDef *Channel)
{
    // 假设三相电流通过以下方式测量：
    // - CH1: ADC1通道1（对应A相电流）
    // - CH2: ADC2通道1（对应B相电流）
    
    // 获取三相电流值（根据实际硬件连接调整）
    Channel->CH1 = INA240_GetValue_DMA(&hadc1);  // A相电流
    Channel->CH2 = INA240_GetValue_DMA(&hadc2);  // B相电流
    
    // 假设C相电流通过A、B相计算（根据克拉克变换假设 ia + ib + ic = 0）
    Channel->CH3 = - (Channel->CH1 + Channel->CH2);  // C相电流
}


/**
 * @description: DMA方式的ADC采样回调函数;
 * @param {ADC_HandleTypeDef*} hadc
 * @return {*}
 */
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) 
// {
//     if (hadc == &hadc1) {
//         uint32_t raw_val = ina240Buffer1[0] & 0xFFF;  // 获取最新值
//         INA240_voltage1 = raw_val * 2.0f / 4095.0f - 1.0f;
//         // 在这里处理电压值（例如发送到串口或触发逻辑）
//     }
//     if (hadc == &hadc2) {
//         uint32_t raw_val = ina240Buffer2[0] & 0xFFF;  // 获取最新值
//         INA240_voltage2 = raw_val * 2.0f / 4095.0f - 1.0f;
//         // 在这里处理电压值（例如发送到串口或触发逻辑）
//     }
// }





