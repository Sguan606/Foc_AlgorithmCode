#ifndef __FOC_CONTRONL_H
#define __FOC_CONTRONL_H

#include "foc.h"
#include "as5600.h"
#include "ina240A2.h"

// PID控制器结构体
typedef struct {
    float Kp;           // 比例增益
    float Ki;           // 积分增益
    float Kd;           // 微分增益
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output_limit; // 输出限幅
} PID_Controller;

// FOC控制模式
typedef enum {
    FOC_MODE_OPEN_LOOP,     // 开环模式
    FOC_MODE_CURRENT_LOOP,  // 电流环模式
    FOC_MODE_SPEED_LOOP,    // 速度环模式
    FOC_MODE_POSITION_LOOP  // 位置环模式
} FOC_ControlMode;

// FOC控制参数
typedef struct {
    FOC_ControlMode mode;       // 当前控制模式
    float target_position;      // 目标位置(rad)
    float target_speed;         // 目标速度(rad/s)
    float target_current_q;     // q轴目标电流(A)
    float target_current_d;     // d轴目标电流(A)
    float max_voltage;          // 最大输出电压
} FOC_ControlParams;

// 函数声明
void FOC_InitControllers(void);
void FOC_CurrentLoop(float i_d, float i_q, float *u_d, float *u_q);
void FOC_SpeedLoop(float target_speed, float current_speed, float *target_i_q);
void FOC_PositionLoop(float target_position, float current_position, float *target_speed);
void FOC_RunControlLoop(void);

// 外部变量声明
extern FOC_ControlParams foc_control_params;
extern PID_Controller current_pid_q;
extern PID_Controller current_pid_d;
extern PID_Controller speed_pid;
extern PID_Controller position_pid;

#endif
