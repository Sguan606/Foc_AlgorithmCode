/*
 * @Author: 星必尘Sguan
 * @Date: 2025-04-19 22:08:33
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-05-09 19:35:53
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

#define CH1_Calibration 0.090f
#define CH2_Calibration 0.240f
#define MOVING_AVG_WINDOW  5  // 窗口大小（根据电机最高电频率调整）


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
    temp = (raw_val*(2.0f - 0.0f)/4095.0f -1.0f)*3.3f;
    return temp;
}



/**
 * @brief 获取 INA240 测量的电流值（单位：A）
 * @param hadc ADC 句柄指针
 * @param shunt_resistor 分流电阻值（单位：Ω，如 0.01f）
 * @param gain INA240 增益（如 50.0f）
 * @param vref ADC 参考电压（单位：V，如 3.3f）
 * @return 电流值（单位：A），正值表示流入，负值表示流出
 */
float INA240_GetValue_DMA(ADC_HandleTypeDef *hadc, float shunt_resistor, float gain, float vref) 
{
    uint32_t raw_val = 0;
    if (hadc == &hadc1) {
        raw_val = ina240Buffer1[0] & 0xFFF;
    }
    else if (hadc == &hadc2) {
        raw_val = ina240Buffer2[0] & 0xFFF;
    }
    else {
        return 0.0f;
    }
    // 将ADC读数转换为电压（假设ADC配置为测量0-Vref范围）
    float vadc = (raw_val / 4095.0f) * vref;
    
    // 计算分流电压（考虑INA240的输出偏移为Vref/2）
    float vshunt = (vadc - vref/2) / gain;
    
    // 计算电流
    float current = vshunt / shunt_resistor;
    
    if (hadc == &hadc1) {
        current += CH1_Calibration;
    }
    else if (hadc == &hadc2){
        current += CH2_Calibration;
    }
    return current;
}

/**
 * @description: 获取三相电流值并存储到结构体
 * @param {INA240_ValueTypeDef*} Channel: 存储三相电流值的结构体指针
 * @return {void}
 */
void INA240_FocValue(INA240_ValueTypeDef *Channel) {
    static float ch1_buf[MOVING_AVG_WINDOW] = {0};
    static float ch2_buf[MOVING_AVG_WINDOW] = {0};
    static uint8_t idx = 0;
    
    // 更新缓冲区
    ch1_buf[idx] = INA240_GetValue_DMA(&hadc1, 0.01f, 50.0f, 3.3f) * 0.8f + ch1_buf[idx] * 0.2f; // 带惯性
    ch2_buf[idx] = INA240_GetValue_DMA(&hadc2, 0.01f, 50.0f, 3.3f) * 0.8f + ch2_buf[idx] * 0.2f;
    idx = (idx + 1) % MOVING_AVG_WINDOW;
    
    // 计算平均值
    float sum1 = 0, sum2 = 0;
    for (uint8_t i = 0; i < MOVING_AVG_WINDOW; i++) {
        sum1 += ch1_buf[i];
        sum2 += ch2_buf[i];
    }
    Channel->CH1 = sum1 / MOVING_AVG_WINDOW;
    Channel->CH2 = sum2 / MOVING_AVG_WINDOW;
    Channel->CH3 = -(Channel->CH1 + Channel->CH2);
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
//         INA240_voltage1 = (raw_val * 2.0f / 4095.0f - 1.0f)*3.3f;
//         // 在这里处理电压值（例如发送到串口或触发逻辑）
//     }
//     if (hadc == &hadc2) {
//         uint32_t raw_val = ina240Buffer2[0] & 0xFFF;  // 获取最新值
//         INA240_voltage2 = (raw_val * 2.0f / 4095.0f - 1.0f)*3.3f;
//         // 在这里处理电压值（例如发送到串口或触发逻辑）
//     }
// }




