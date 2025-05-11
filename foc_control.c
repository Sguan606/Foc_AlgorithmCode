/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-06 17:11:11
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-05-09 19:51:28
 * @FilePath: \test_2804FocMotor\Hardware\foc_control.c
 * @Description: 实现三相无刷电机的FOC高级电机控制;
 * 具体实现->FOC需要的位置环，电流环，速度环（串级闭环调控）;
 * 位置环：实际测量AS5600的寄存器得到（使用函数AS5600_GetAngleRad(void)）
 * 速度环：采用AS5600测得的角度变化（使用函数AS5600_GetVelocityRad(void)）
 * 电流环：使用INA240A2测量，并通过stm32的ADC采样得到3相电流
 * 位置环->速度环->电流环
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "foc_control.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// 全局控制状态变量
static FOC_ControlTypeDef foc_ctrl;

// PID控制器初始化
static void PID_Init(PID_Controller* pid, float kp, float ki, float kd, float output_limit, float integral_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
}

// FOC控制初始化
void FOC_Control_Init(void) {
    // 初始化控制模式
    foc_ctrl.mode = FOC_MODE_IDLE;
    
    // 初始化PID参数 (需要根据实际电机调整)
    PID_Init(&foc_ctrl.current_q_pid, 0.0f, 0.0f, 0.0f, 12.0f, 1.0f); // q轴电流环
    PID_Init(&foc_ctrl.current_d_pid, 0.0f, 0.0f, 0.0f, 12.0f, 1.0f); // d轴电流环
    PID_Init(&foc_ctrl.velocity_pid, 0.1f, 0.0f, 0.0f, 200.0f, 2.0f);  // 速度环
    PID_Init(&foc_ctrl.position_pid, 0.0f, 0.0f, 0.0f, 10.0f, 5.0f);  // 位置环
    
    // 初始化目标值
    foc_ctrl.target_current_q = 0.0f;
    foc_ctrl.target_current_d = 0.0f;
    foc_ctrl.target_velocity = 0.0f;
    foc_ctrl.target_position = 0.0f;
    
    // 初始化保护参数 (根据电机规格设置)
    foc_ctrl.max_current = 1.0f;  // 最大1A电流
    foc_ctrl.max_voltage = 12.0f; // 最大12V电压
    foc_ctrl.overcurrent_count = 0;
    foc_ctrl.fault_flag = 0;
    
    // 初始化FOC硬件
    FOC_Init();
    INA240_Init();
    
    // 初始化SVPWM结构体
    memset(&foc_ctrl.foc, 0, sizeof(FOC_HandleTypeDef));
}

// FOC主控制循环 (应在定时中断中调用，如1kHz)
void FOC_Control_Loop(void) {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f; // 计算时间差(秒)
    if (dt <= 0) dt = 0.001f; // 最小时间1ms
    last_time = current_time;
    
    // 1. 读取传感器数据
    INA240_ValueTypeDef currents;
    INA240_FocValue(&currents); // 获取三相电流
    
    foc_ctrl.current_angle = AS5600_GetAngleRadNormalized(); // 获取转子角度(rad)
    foc_ctrl.current_velocity = AS5600_GetVelocityRad();     // 获取转子速度(rad/s)
    foc_ctrl.current_position += foc_ctrl.current_velocity * dt; // 积分计算位置
    
    // 2. 电流检测与保护
    if (fabs(currents.CH1) > foc_ctrl.max_current || 
        fabs(currents.CH2) > foc_ctrl.max_current || 
        fabs(currents.CH3) > foc_ctrl.max_current) {
        foc_ctrl.overcurrent_count++;
        if (foc_ctrl.overcurrent_count > 10) { // 连续10次过流触发保护
            foc_ctrl.fault_flag = 1;
            FOC_SpeedSet(0); // 停止电机
            return;
        }
    } else {
        foc_ctrl.overcurrent_count = 0;
    }
    
    // 3. 根据控制模式执行不同的控制策略
    switch (foc_ctrl.mode) {
        case FOC_MODE_IDLE:
            // 空闲模式，不输出
            foc_ctrl.foc.u_d = 0.0f;
            foc_ctrl.foc.u_q = 0.0f;
            break;
            
        case FOC_MODE_OPEN_LOOP:
            // 开环模式，直接使用设定值
            // FOC_SpeedSet(foc_ctrl.target_velocity);
            break;
            
        case FOC_MODE_CURRENT: {
            // 电流控制模式 (内环)
            // 设置FOC结构体中的电流值
            foc_ctrl.foc.i_a = currents.CH1;
            foc_ctrl.foc.i_b = currents.CH2;
            foc_ctrl.foc.i_c = currents.CH3;
            
            // Clarke变换 (3相→2相)
            clarke(&foc_ctrl.foc);
            
            // Park变换 (静止→旋转)
            foc_ctrl.foc.theta = foc_ctrl.current_angle;
            park(&foc_ctrl.foc);
            
            // 电流环PID控制
            float d_error = foc_ctrl.target_current_d - foc_ctrl.foc.i_d;
            float q_error = foc_ctrl.target_current_q - foc_ctrl.foc.i_q;
            
            foc_ctrl.foc.u_d = PID_Update(&foc_ctrl.current_d_pid, d_error, dt);
            foc_ctrl.foc.u_q = PID_Update(&foc_ctrl.current_q_pid, q_error, dt);
            
            // 电压限幅
            foc_ctrl.foc.u_d = fmaxf(fminf(foc_ctrl.foc.u_d, foc_ctrl.max_voltage), -foc_ctrl.max_voltage);
            foc_ctrl.foc.u_q = fmaxf(fminf(foc_ctrl.foc.u_q, foc_ctrl.max_voltage), -foc_ctrl.max_voltage);
            break;
        }
            
        case FOC_MODE_VELOCITY: {
            // 速度控制模式 (中环)
            // 速度环PID控制
            float velocity_error = foc_ctrl.target_velocity - foc_ctrl.current_velocity;
            foc_ctrl.target_current_q = PID_Update(&foc_ctrl.velocity_pid, velocity_error, dt);
            
            // 进入电流控制
            foc_ctrl.foc.i_a = currents.CH1;
            foc_ctrl.foc.i_b = currents.CH2;
            foc_ctrl.foc.i_c = currents.CH3;
            
            clarke(&foc_ctrl.foc);
            foc_ctrl.foc.theta = foc_ctrl.current_angle;
            park(&foc_ctrl.foc);
            
            float d_error = foc_ctrl.target_current_d - foc_ctrl.foc.i_d;
            float q_error = foc_ctrl.target_current_q - foc_ctrl.foc.i_q;
            
            foc_ctrl.foc.u_d = PID_Update(&foc_ctrl.current_d_pid, d_error, dt);
            foc_ctrl.foc.u_q = PID_Update(&foc_ctrl.current_q_pid, q_error, dt);
            
            // 电压限幅
            foc_ctrl.foc.u_d = fmaxf(fminf(foc_ctrl.foc.u_d, foc_ctrl.max_voltage), -foc_ctrl.max_voltage);
            foc_ctrl.foc.u_q = fmaxf(fminf(foc_ctrl.foc.u_q, foc_ctrl.max_voltage), -foc_ctrl.max_voltage);
            break;
        }
            
        case FOC_MODE_POSITION: {
            // 位置控制模式 (外环)
            // 位置环PID控制
            float position_error = foc_ctrl.target_position - foc_ctrl.current_position;
            foc_ctrl.target_velocity = PID_Update(&foc_ctrl.position_pid, position_error, dt);
            
            // 速度环PID控制
            float velocity_error = foc_ctrl.target_velocity - foc_ctrl.current_velocity;
            foc_ctrl.target_current_q = PID_Update(&foc_ctrl.velocity_pid, velocity_error, dt);
            
            // 进入电流控制
            foc_ctrl.foc.i_a = currents.CH1;
            foc_ctrl.foc.i_b = currents.CH2;
            foc_ctrl.foc.i_c = currents.CH3;
            
            clarke(&foc_ctrl.foc);
            foc_ctrl.foc.theta = foc_ctrl.current_angle;
            park(&foc_ctrl.foc);
            
            float d_error = foc_ctrl.target_current_d - foc_ctrl.foc.i_d;
            float q_error = foc_ctrl.target_current_q - foc_ctrl.foc.i_q;
            
            foc_ctrl.foc.u_d = PID_Update(&foc_ctrl.current_d_pid, d_error, dt);
            foc_ctrl.foc.u_q = PID_Update(&foc_ctrl.current_q_pid, q_error, dt);
            
            // 电压限幅
            foc_ctrl.foc.u_d = fmaxf(fminf(foc_ctrl.foc.u_d, foc_ctrl.max_voltage), -foc_ctrl.max_voltage);
            foc_ctrl.foc.u_q = fmaxf(fminf(foc_ctrl.foc.u_q, foc_ctrl.max_voltage), -foc_ctrl.max_voltage);
            break;
        }
    }
    
    // 4. 更新FOC输出 (在所有模式下)
    if (foc_ctrl.mode != FOC_MODE_IDLE) {
        // 设置FOC角度
        foc_ctrl.foc.theta = foc_ctrl.current_angle;
        
        // 执行逆Park变换和SVPWM生成
        ipark(&foc_ctrl.foc);
        svpwm(&foc_ctrl.foc);
        
        // 设置PWM输出 (将SVPWM输出映射到0-8549范围)
        uint32_t Ua = (uint32_t)((foc_ctrl.foc.t_a) * 8449.0f);
        uint32_t Ub = (uint32_t)((foc_ctrl.foc.t_b) * 8449.0f);
        uint32_t Uc = (uint32_t)((foc_ctrl.foc.t_c) * 8449.0f);
        
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Ua);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Ub);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Uc);
    }
}

// PID控制器更新
static float PID_Update(PID_Controller* pid, float error, float dt) {
    // 比例项
    float p_term = pid->kp * error;
    
    // 积分项 (带抗饱和)
    pid->integral += error * dt;
    pid->integral = fmaxf(fminf(pid->integral, pid->integral_limit), -pid->integral_limit);
    float i_term = pid->ki * pid->integral;
    
    // 微分项
    float d_term = 0.0f;
    if (dt > 0) {
        d_term = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;
    
    // 计算输出并限幅
    float output = p_term + i_term + d_term;
    output = fmaxf(fminf(output, pid->output_limit), -pid->output_limit);
    
    return output;
}

// 设置控制模式
void FOC_SetMode(FOC_ControlMode mode) {
    foc_ctrl.mode = mode;
    if (mode == FOC_MODE_IDLE) {
        FOC_SpeedSet(0); // 停止电机
    }
}

// 设置目标电流
void FOC_SetTargetCurrent(float iq, float id) {
    foc_ctrl.target_current_q = iq;
    foc_ctrl.target_current_d = id;
}

// 设置目标速度
void FOC_SetTargetVelocity(float velocity) {
    foc_ctrl.target_velocity = velocity;
}

// 设置目标位置
void FOC_SetTargetPosition(float position) {
    foc_ctrl.target_position = position;
}

// 重置PID控制器
void FOC_ResetPID(void) {
    foc_ctrl.current_q_pid.integral = 0.0f;
    foc_ctrl.current_q_pid.prev_error = 0.0f;
    foc_ctrl.current_d_pid.integral = 0.0f;
    foc_ctrl.current_d_pid.prev_error = 0.0f;
    foc_ctrl.velocity_pid.integral = 0.0f;
    foc_ctrl.velocity_pid.prev_error = 0.0f;
    foc_ctrl.position_pid.integral = 0.0f;
    foc_ctrl.position_pid.prev_error = 0.0f;
}

// 更新电流环PID参数
void FOC_UpdateCurrentPID(float kp, float ki, float kd) {
    foc_ctrl.current_q_pid.kp = kp;
    foc_ctrl.current_q_pid.ki = ki;
    foc_ctrl.current_q_pid.kd = kd;
    foc_ctrl.current_d_pid.kp = kp;
    foc_ctrl.current_d_pid.ki = ki;
    foc_ctrl.current_d_pid.kd = kd;
}

// 更新速度环PID参数
void FOC_UpdateVelocityPID(float kp, float ki, float kd) {
    foc_ctrl.velocity_pid.kp = kp;
    foc_ctrl.velocity_pid.ki = ki;
    foc_ctrl.velocity_pid.kd = kd;
}

// 更新位置环PID参数
void FOC_UpdatePositionPID(float kp, float ki, float kd) {
    foc_ctrl.position_pid.kp = kp;
    foc_ctrl.position_pid.ki = ki;
    foc_ctrl.position_pid.kd = kd;
}

// 获取故障标志
uint8_t FOC_GetFaultFlag(void) {
    return foc_ctrl.fault_flag;
}

// 清除故障
void FOC_ClearFault(void) {
    foc_ctrl.fault_flag = 0;
    foc_ctrl.overcurrent_count = 0;
    FOC_ResetPID();
}


/**
 * @description: TIM2中断回调函数，1kHz控制循环
 * @param {TIM_HandleTypeDef*} htim
 * @return {*}
 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim == &htim2) {  // 1kHz中断
//         FOC_Control_Loop();
//     }
// }

