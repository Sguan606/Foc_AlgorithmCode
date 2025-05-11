/*
 * @Author: 星必尘Sguan
 * @Date: 2025-04-19 19:51:28
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-05-09 19:13:12
 * @FilePath: \test_2804FocMotor\Hardware\foc.c
 * @Description: 实现三相无刷电机的基本速度控制(仅开环);
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "foc.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

float uaa,ubb,ucc;


float target_speed = 0.0f;
#define ANGLE_OFFSET 0                      // 固定偏移量（机械角度偏移）
float g_mechanical_angle = 0.0f;            // 当前机械角度（弧度）
#define FocPWM_Period 8500-1                //自己设置的TIM1的频率，控制PWM占空比，这里是0-624
#define FocPWM_SetCycle 0.001f              //自己设置的TIM2的周期，我这里的调用周期为1ms



/**
 * @description: SVPWM和SPWM开启的宏定义(仅能使用其中一个);
 * @return {*}
 */
// 定义一个宏来选择波形类型，只能选择一个
#define USE_SVPWM  // 注释掉此行并取消注释下一行以启用 SPWM
// #define USE_SPWM
 

/**
 * @description: [单位转换]实现rad → 0-360°的单位转换;
 * @param {float} angle_rad
 * @return {*}
 */
float RadToDegrees(float angle_rad) {
    float angle_deg = angle_rad * 180.0f / PI;  // 弧度 → 度
    angle_deg = fmodf(angle_deg, 360.0f);       // 归一化到[0, 360)
    if (angle_deg < 0) angle_deg += 360.0f;     // 处理负值
    return angle_deg;
}


/**
 * @description: 初始化必要定时器TIM1的PWM和TIM2定时中断；
 * @return {*}
 */
void FOC_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   // 单独启动每个通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}


/**
 * @description: 角度控制PWM输出对应三相不同大小的电源（0-4095）
 * @param {uint32_t} Angle
 * @return {*}
 */
void FOC_GetPhaseVoltages(uint16_t desired_angle, uint32_t* Ua, uint32_t* Ub, uint32_t* Uc) {
    // 1. 计算校正后的角度（处理溢出）
    uint16_t corrected_angle = (desired_angle + ANGLE_OFFSET) % 4096;
 
    // 2. 计算三相电压
    float theta = (corrected_angle / 4095.0f) * 2 * PI;
 
    #ifdef USE_SPWM
    // 3. SPWM波形的三相电压
    uaa = fast_sin(theta);
    ubb = fast_sin(theta + 4.0f * PI / 3.0f);
    ucc = fast_sin(theta + 2.0f * PI / 3.0f);
    *Ua = (uint32_t)((4249.5f + 4249.5f * uaa)*0.2f);  // 使用sinf提高计算速度
    *Ub = (uint32_t)((4249.5f + 4249.5f * ubb)*0.2f);
    *Uc = (uint32_t)((4249.5f + 4249.5f * ucc)*0.2f);
    #endif // 使用到USE_SPWM
 
    
    #ifdef USE_SVPWM
    FOC_HandleTypeDef foc;
    foc.u_d = 0.22f;      // 设置合适的d轴电压
    foc.u_q = 0.15f;            // q轴电压设为0.1f
    foc.theta = theta;
    
    // 执行逆Park变换和SVPWM计算
    ipark(&foc);
    svpwm(&foc);
    
    // SVPWM波形还原为纯正弦波
    // float ua = foc.t_a - 0.5f * (foc.t_b + foc.t_c);
    // float ub = foc.t_b - 0.5f * (foc.t_a + foc.t_c);
    // float uc = -(ua + ub);
    float ua = foc.t_a;
    float ub = foc.t_b;
    float uc = foc.t_c;
    
    uaa = ua - 0.5f;
    ubb = ub - 0.5f;
    ucc = uc - 0.5f;
    
    // 缩放到0-8549范围
    *Ua = (uint32_t)(ua * 8449.0f);
    *Ub = (uint32_t)(ub * 8449.0f);
    *Uc = (uint32_t)(uc * 8449.0f);
    #endif // 使用到USE_SVPWM
}


/**
 * @description: 角度控制PWM产生实际的可控输出电压;
 * @param {uint16_t} desired_angle
 * @return {*}
 */
void FOC_PwmSet(uint16_t desired_angle)
{
    uint32_t Ua,Ub,Uc;
    // HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1|TIM_CHANNEL_2|TIM_CHANNEL_3);
    FOC_GetPhaseVoltages(desired_angle,&Ua,&Ub,&Uc);
    
    //PwmSetCompare设置PWM占空比;
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Ua);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Ub);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,Uc);
}


/**
 * @description: 设置旋转角速度（rad/s），使用梯形积分法提高低速精度
 * @param {int32_t} Speed 角速度（rad/s），正值为正转，负值为反转
 * @return {*}
 */
void FOC_SpeedSet(float Speed) 
{
    // 静态变量保存积分状态
    static float g_angle_integral = 0.0f;    // 角度积分累加器
    static float last_delta = 0.0f;          // 上一次的角度增量
    
    // 1. 计算当前角度增量
    float delta_angle = Speed * FocPWM_SetCycle;
    
    // 2. 使用梯形积分法计算角度 (更精确的积分方法)
    // 公式: 当前积分 = 上次积分 + (本次增量 + 上次增量)/2 * 时间
    g_angle_integral += (delta_angle + last_delta) / 2.0f;
    last_delta = delta_angle;
    
    // 3. 机械角度归一化到 [0, 2π]
    g_mechanical_angle = fmodf(g_angle_integral, 2 * PI);
    if(g_mechanical_angle < 0) {
        g_mechanical_angle += 2 * PI;
    }
    
    // 4. 转换为电角度（电角度 = 机械角度 * 极对数）
    float electrical_angle = g_mechanical_angle * POLE_PAIRS;
    
    // 5. 将电角度映射到编码器范围（0-4095）
    // uint16_t desired_angle = (uint16_t)((electrical_angle / (2 * PI)) * 4095.0f);
    uint16_t desired_angle = (uint16_t)(electrical_angle / (2 * PI) * 4095.0f) % 4096;
    
    // 6. 调用PWM生成函数
    FOC_PwmSet(desired_angle);
}




