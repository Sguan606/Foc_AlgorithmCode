#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "foc.h"
#include <string.h>  // 包含memset函数声明

// PID控制器结构体
typedef struct {
    float kp;           // 比例增益
    float ki;           // 积分增益
    float kd;           // 微分增益
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output_limit; // 输出限幅
    float integral_limit; // 积分限幅
} PID_Controller;

// FOC控制模式枚举
typedef enum {
    FOC_MODE_IDLE = 0,      // 空闲模式
    FOC_MODE_OPEN_LOOP,     // 开环模式
    FOC_MODE_CURRENT,       // 电流控制模式
    FOC_MODE_VELOCITY,      // 速度控制模式
    FOC_MODE_POSITION       // 位置控制模式
} FOC_ControlMode;

// FOC控制状态结构体
typedef struct {
    FOC_ControlMode mode;   // 当前控制模式
    
    // 目标值
    float target_current_q; // q轴目标电流(A)
    float target_current_d; // d轴目标电流(A)
    float target_velocity;  // 目标速度(rad/s)
    float target_position;  // 目标位置(rad)
    
    // PID控制器
    PID_Controller current_q_pid;   // q轴电流环PID
    PID_Controller current_d_pid;   // d轴电流环PID
    PID_Controller velocity_pid;    // 速度环PID
    PID_Controller position_pid;    // 位置环PID
    
    // 状态反馈
    float current_angle;    // 当前角度(rad)
    float current_velocity; // 当前速度(rad/s)
    float current_position; // 当前位置(rad)
    
    // 保护参数
    float max_current;      // 最大允许电流(A)
    float max_voltage;      // 最大允许电压(V)
    float overcurrent_count;// 过流计数
    uint8_t fault_flag;     // 故障标志
    
    // SVPWM相关
    FOC_HandleTypeDef foc;  // SVPWM控制结构体
} FOC_ControlTypeDef;

// 函数声明
void FOC_Control_Init(void);
void FOC_Control_Loop(void);
void FOC_SetMode(FOC_ControlMode mode);
void FOC_SetTargetCurrent(float iq, float id);
void FOC_SetTargetVelocity(float velocity);
void FOC_SetTargetPosition(float position);
void FOC_ResetPID(void);
void FOC_UpdateCurrentPID(float kp, float ki, float kd);
void FOC_UpdateVelocityPID(float kp, float ki, float kd);
void FOC_UpdatePositionPID(float kp, float ki, float kd);
uint8_t FOC_GetFaultFlag(void);
void FOC_ClearFault(void);

// 辅助函数
static float PID_Update(PID_Controller* pid, float error, float dt);

#endif // __FOC_CONTROL_H
