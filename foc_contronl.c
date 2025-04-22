/*
 * @Author: 星必尘Sguan
 * @Date: 2025-04-19 19:56:11
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-04-22 21:22:33
 * @FilePath: \test_2804FocMotor\Hardware\foc_contronl.c
 * @Description: 实现三相无刷电机的FOC高级电机控制;
 * 具体实现->FOC需要的位置环，电流环，速度环（串级闭环调控）;
 * 位置环：实际测量AS5600的寄存器得到（使用函数uint16_t AS5600_ReadData(void)）
 * 速度环：采用AS5600测得的角度变化（使用函数float AS5600_GetVelocity(void)）
 * 电流环：使用INA240A2测量，并通过stm32的ADC采样得到3相电流
 * 位置环->速度环->电流环
 * 
 * @function:在这里编写我们的：位置环PID|速度环PI|电流环PI
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "foc_contronl.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
INA240_ValueTypeDef phase_currents;


// 默认控制参数
FOC_ControlParams foc_control_params = {
    .mode = FOC_MODE_OPEN_LOOP,
    .target_position = 0.0f,
    .target_speed = 0.0f,
    .target_current_q = 0.20f,
    .target_current_d = 0.1f,
    .max_voltage = 0.25f
};

// PID控制器实例
PID_Controller current_pid_q = {.Kp = 0.5f, .Ki = 0.1f, .Kd = 0.0f, .output_limit = 0.25f};
PID_Controller current_pid_d = {.Kp = 0.5f, .Ki = 0.1f, .Kd = 0.0f, .output_limit = 0.25f};
PID_Controller speed_pid = {.Kp = 0.1f, .Ki = 0.01f, .Kd = 0.001f, .output_limit = 1.0f};
PID_Controller position_pid = {.Kp = 1.0f, .Ki = 0.0f, .Kd = 0.01f, .output_limit = 10.0f};

/**
 * @description: 初始化PID控制器
 * @return {*}
 */
void FOC_InitControllers(void) {
    current_pid_q.integral = 0;
    current_pid_q.prev_error = 0;
    
    current_pid_d.integral = 0;
    current_pid_d.prev_error = 0;
    
    speed_pid.integral = 0;
    speed_pid.prev_error = 0;
    
    position_pid.integral = 0;
    position_pid.prev_error = 0;
}

/**
 * @description: PID计算函数
 * @param {PID_Controller*} pid PID控制器实例
 * @param {float} setpoint 设定值
 * @param {float} measurement 测量值
 * @param {float} dt 时间步长(s)
 * @return {float} PID输出
 */
float PID_Compute(PID_Controller* pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // 比例项
    float P = pid->Kp * error;
    
    // 积分项(带抗饱和)
    pid->integral += error * dt;
    if (pid->integral > pid->output_limit) pid->integral = pid->output_limit;
    if (pid->integral < -pid->output_limit) pid->integral = -pid->output_limit;
    float I = pid->Ki * pid->integral;
    
    // 微分项(带滤波器)
    float D = 0;
    if (dt > 0) {
        float derivative = (error - pid->prev_error) / dt;
        D = pid->Kd * derivative;
    }
    pid->prev_error = error;
    
    // 总和并限幅
    float output = P + I + D;
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    
    return output;
}

/**
 * @description: 电流环控制
 * @param {float} i_d d轴电流测量值
 * @param {float} i_q q轴电流测量值
 * @param {float*} u_d d轴输出电压
 * @param {float*} u_q q轴输出电压
 * @return {*}
 */
void FOC_CurrentLoop(float i_d, float i_q, float *u_d, float *u_q) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f;
    if (dt <= 0) dt = 0.001f; // 默认1ms
    
    // 计算PID输出
    *u_d = PID_Compute(&current_pid_d, foc_control_params.target_current_d, i_d, dt);
    *u_q = PID_Compute(&current_pid_q, foc_control_params.target_current_q, i_q, dt);
    
    last_time = current_time;
}

/**
 * @description: 速度环控制
 * @param {float} target_speed 目标速度(rad/s)
 * @param {float} current_speed 当前速度(rad/s)
 * @param {float*} target_i_q q轴目标电流输出
 * @return {*}
 */
void FOC_SpeedLoop(float target_speed, float current_speed, float *target_i_q) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f;
    if (dt <= 0) dt = 0.001f; // 默认1ms
    
    // 计算PID输出作为q轴电流给定
    *target_i_q = PID_Compute(&speed_pid, target_speed, current_speed, dt);
    
    // d轴电流给定设为0(磁场定向控制)
    foc_control_params.target_current_d = 0.0f;
    
    last_time = current_time;
}

/**
 * @description: 位置环控制
 * @param {float} target_position 目标位置(rad)
 * @param {float} current_position 当前位置(rad)
 * @param {float*} target_speed 目标速度输出(rad/s)
 * @return {*}
 */
void FOC_PositionLoop(float target_position, float current_position, float *target_speed) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f;
    if (dt <= 0) dt = 0.001f; // 默认1ms
    
    // 计算PID输出作为速度给定
    *target_speed = PID_Compute(&position_pid, target_position, current_position, dt);
    
    last_time = current_time;
}

/**
 * @description: 运行FOC控制循环
 * @return {*}
 */
void FOC_RunControlLoop(void) {
    // 1. 获取当前状态
    float current_angle = AS5600_GetAngleRad(); // 获取当前机械角度(rad)
    float current_speed = AS5600_GetVelocityRad(); // 获取当前速度(rad/s)
    
    // 2. 获取三相电流并转换到dq坐标系
    INA240_FocValue(&phase_currents); // 更新三相电流测量值
    
    FOC_HandleTypeDef foc;
    foc.i_a = phase_currents.CH1;
    foc.i_b = phase_currents.CH2;
    foc.i_c = phase_currents.CH3;
    foc.theta = current_angle * POLE_PAIRS; // 电角度 = 机械角度 * 极对数
    
    // 克拉克变换和帕克变换
    clarke(&foc);
    park(&foc);
    
    // 3. 根据控制模式运行相应的控制环
    switch (foc_control_params.mode) {
        case FOC_MODE_POSITION_LOOP: {
            float target_speed;
            FOC_PositionLoop(foc_control_params.target_position, current_angle, &target_speed);
            foc_control_params.target_speed = target_speed;
            // 继续执行速度环
        }
        case FOC_MODE_SPEED_LOOP: {
            float target_i_q;
            FOC_SpeedLoop(foc_control_params.target_speed, current_speed, &target_i_q);
            foc_control_params.target_current_q = target_i_q;
            // 继续执行电流环
        }
        case FOC_MODE_CURRENT_LOOP: {
            float u_d, u_q;
            FOC_CurrentLoop(foc.i_d, foc.i_q, &u_d, &u_q);
            
            // 设置FOC输出
            foc.u_d = u_d;
            foc.u_q = u_q;
            break;
        }
        case FOC_MODE_OPEN_LOOP:
        default: {
            // 开环模式使用固定值
            foc.u_d = foc_control_params.max_voltage;
            foc.u_q = 0.1f;
            break;
        }
    }
    
    // 4. 执行逆变换和SVPWM
    ipark(&foc);
    svpwm(&foc);
    
    // 5. 计算三相电压并设置PWM
    float ua = foc.t_a - 0.5f * (foc.t_b + foc.t_c);
    float ub = foc.t_b - 0.5f * (foc.t_a + foc.t_c);
    float uc = -(ua + ub);
    
    // 缩放和偏移到0-8549范围
    ua = (ua + 0.5f) * 8449.0f;
    ub = (ub + 0.5f) * 8449.0f;
    uc = (uc + 0.5f) * 8449.0f;
    
    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)ua);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)ub);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)uc);
}


/**
 * @description: TIM2中断回调函数，1kHz控制循环
 * @param {TIM_HandleTypeDef*} htim
 * @return {*}
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {  // 1kHz中断
        FOC_RunControlLoop();
    }
}
