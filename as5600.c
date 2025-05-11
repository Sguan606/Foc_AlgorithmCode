/*
 * @Author: 星必尘Sguan
 * @Date: 2025-04-12 19:49:17
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-05-07 00:31:47
 * @FilePath: \test_2804FocMotor\Hardware\as5600.c
 * @Description: AS5600磁编码器
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "as5600.h"
#include "Kerman.h"

#define M_PI 3.14159265358979323846f
extern I2C_HandleTypeDef hi2c3;

#define AS5600_I2C_Address 0x36
#define AS5600_Reg_H 0x0C
#define AS5600_Reg_L 0x0D

static float prev_velocity = 0.0f;   // 上一次角速度（度/秒）
static uint32_t prev_time_velocity = 0; // 上一次角速度时间戳（ms）

// 角度相关全局变量
static uint16_t prev_raw_angle = 0;
static uint32_t prev_timestamp = 0;
static float angle_rad_prev = 0;


/**
 * @description: 读取AS5600磁编码内部寄存器;
 * @return {*}
 */
uint16_t AS5600_ReadData(void) {
    uint8_t Reg_Data[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c3,
        AS5600_I2C_Address << 1,
        AS5600_Reg_H,
        I2C_MEMADD_SIZE_8BIT,
        Reg_Data,
        2,
        100
    );
    if (status != HAL_OK) return 0xFFFF;
    return ((Reg_Data[0] << 8) | Reg_Data[1]) & 0x0FFF;
}


/**
 * @description: 计算得到磁编码器的角度值;
 * @return {*}
 */
float AS5600_GetAngle(void) {
    uint16_t raw_angle = AS5600_ReadData();
    if (raw_angle == 0xFFFF) return NAN;
    return (raw_angle / 4095.0f) * 360.0f;
}



/**
 * @description: 计算获取角加速度值(***误差微分后被放大，数值滤波后，已经不怎么可见了***);
 * @return {*}
 */
float AS5600_GetAcceleration(void) {
    uint32_t current_time = HAL_GetTick();
    float current_velocity = AS5600_GetVelocityRad();
    if (isnan(current_velocity)) return NAN;
 
    if (prev_time_velocity == 0) {
        prev_velocity = current_velocity;
        prev_time_velocity = current_time;
        return 0.0f;
    }
 
    float delta_velocity = current_velocity - prev_velocity;
    float delta_time = (current_time - prev_time_velocity) / 1000.0f;
 
    if (delta_time <= 0.1f) return 0.0f; // 增加时间阈值
 
    prev_velocity = current_velocity;
    prev_time_velocity = current_time;
 
    static float filtered_acceleration = 0.0f;
    float alpha = 0.5f; // 调整滤波器参数
    float raw_acceleration = delta_velocity / delta_time;
    filtered_acceleration = alpha * raw_acceleration + (1 - alpha) * filtered_acceleration;
    return filtered_acceleration;
}


/**
 * @description: 角度换算to弧度制角度（rad）
 * @return {*}
 */
float AS5600_GetAngleRad(void) {
    return AS5600_GetAngle() * (M_PI / 180.0f);
}

/**
 * @description: 角度换算toPI制角度（-Pi到Pi）
 * @return {*}
 */
float AS5600_GetAngleRadNormalized(void) {
    // 1. 读取原始值（0-4095对应0-360度）
    uint16_t raw_angle = AS5600_ReadData();
    if (raw_angle == 0xFFFF) return NAN;  // 处理读取失败

    // 2. 转换为弧度（0~2π）
    float rad = (raw_angle / 4095.0f) * (2.0f * M_PI);

    // 3. 规范化到[-π, +π]
    if (rad > M_PI) {
        rad -= 2.0f * M_PI;  // 大于π时减去2π
    } else if (rad < -M_PI) {
        rad += 2.0f * M_PI;  // 小于-π时加上2π
    }
    return rad;
}



/**
 * @brief 计算角速度（rad/s）
 * @return 角速度（rad/s），若读取失败返回0
 */
float AS5600_GetVelocityRad(void) {
    
    // 1. 读取当前角度（0-4095对应0-2π）
    uint16_t current_raw = AS5600_ReadData();
    if (current_raw == 0xFFFF) return 0.0f; // 读取失败
    
    // 2. 获取当前时间戳（毫秒）
    uint32_t current_timestamp = HAL_GetTick();
    
    // 3. 计算时间差（秒）
    float delta_t = (current_timestamp - prev_timestamp) / 1000.0f;
    if (delta_t <= 0) return 0.0f; // 避免除零
    
    // 4. 转换为弧度（0-4095 → 0-2π）
    const float rad_per_count = 2.0f * 3.1415926f / 4096.0f;
    float angle_rad_current = current_raw * rad_per_count;
    
    // 5. 处理角度跨越0点的边界条件
    float delta_angle = angle_rad_current - angle_rad_prev;
    if (delta_angle > 3.1415926f) delta_angle -= 2.0f * 3.1415926f;  // 正向跨越0点
    if (delta_angle < -3.1415926f) delta_angle += 2.0f * 3.1415926f; // 反向跨越0点
    
    // 6. 计算角速度（rad/s）
    float angular_velocity = delta_angle / delta_t;
    
    // 7. 更新历史数据
    prev_raw_angle = current_raw;
    prev_timestamp = current_timestamp;
    angle_rad_prev = angle_rad_current;
    
    // 使用卡尔曼滤波平滑数据
    float filtered_velocity = Kalman_Update(&kf, angular_velocity);
    return filtered_velocity;
}


