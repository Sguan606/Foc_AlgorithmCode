/*
 * @Author: 星必尘Sguan
 * @Date: 2025-04-21 16:58:11
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-04-22 16:12:10
 * @FilePath: \test_2804FocMotor\Hardware\svpwm.c
 * @Description: 实现SVPWM波的生成;
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "svpwm.h"


/**
 * @description: [逆Park变换1]计算并更新u_alpha、u_beta，同时更新sinθ和cosθ;
 * @param {FOC*} foc
 * @return {*}
 */
void ipark(FOC_HandleTypeDef* foc) {
    foc->sine = fast_sin(foc->theta);
    foc->cosine = fast_cos(foc->theta);
    foc->u_alpha = foc->u_d * foc->cosine - foc->u_q * foc->sine;
    foc->u_beta = foc->u_q * foc->cosine + foc->u_d * foc->sine;
}


/**
 * @description: [逆Park变换2]用于生成SVPWM的输入电压(α-β轴);
 * @function:仅计算u_alpha和u_beta，复用已有的sinθ和cosθ;
 * @param {FOC*} foc
 * @return {*}
 */
void ipark2(FOC_HandleTypeDef* foc) {
    foc->u_alpha = foc->u_d * foc->cosine - foc->u_q * foc->sine;
    foc->u_beta = foc->u_q * foc->cosine + foc->u_d * foc->sine;
}


/**
 * @description: [克拉克变换]将三相电流（a-b-c轴）转换为两相静止坐标系(α-β轴);
 * @param {FOC*} foc
 * @return {*}
 */
void clarke(FOC_HandleTypeDef* foc) {
    foc->i_alpha = foc->i_a;
    foc->i_beta = (foc->i_a + 2 * foc->i_b) * 0.5773502691896257f;
}


/**
 * @description: [Park变换]将静止坐标系（α-β轴）的电流转换到旋转坐标系(d-q轴);
 * @param {FOC*} foc
 * @return {*}
 */
void park(FOC_HandleTypeDef* foc) {
    foc->sine = fast_sin(foc->theta);
    foc->cosine = fast_cos(foc->theta);
    foc->i_d = foc->i_alpha * foc->cosine + foc->i_beta * foc->sine;
    foc->i_q = foc->i_beta * foc->cosine - foc->i_alpha * foc->sine;
}


/**
 * @description: [函数调用]将α-β轴电压转换为三相PWM占空比（t_a, t_b, t_c），驱动逆变器;
 * @function:1.电流采样 → clarke() → park() → 电流环PID计算u_d、u_q。
 * @function:2.电压输出 → ipark() → svpwm() → 更新PWM占空比。
 * @param {FOC*} foc
 * @return {*}
 */
void svpwm(FOC_HandleTypeDef* foc) {
    const float ts = 1;

    float u1 = foc->u_beta;
    float u2 = -0.8660254037844386f * foc->u_alpha - 0.5f * foc->u_beta;
    float u3 = 0.8660254037844386f * foc->u_alpha - 0.5f * foc->u_beta;

    uint8_t sector = (u1 > 0.0f) + ((u2 > 0.0f) << 1) + ((u3 > 0.0f) << 2);

    if (sector == 5) {
        float t4 = u3;
        float t6 = u1;
        float sum = t4 + t6;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t4 = foc->k_svpwm * t4;
            t6 = foc->k_svpwm * t6;
        }
        float t7 = (ts - t4 - t6) / 2;
        foc->t_a = t4 + t6 + t7;
        foc->t_b = t6 + t7;
        foc->t_c = t7;
    } else if (sector == 1) {
        float t2 = -u3;
        float t6 = -u2;
        float sum = t2 + t6;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t2 = foc->k_svpwm * t2;
            t6 = foc->k_svpwm * t6;
        }
        float t7 = (ts - t2 - t6) / 2;
        foc->t_a = t6 + t7;
        foc->t_b = t2 + t6 + t7;
        foc->t_c = t7;
    } else if (sector == 3) {
        float t2 = u1;
        float t3 = u2;
        float sum = t2 + t3;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t2 = foc->k_svpwm * t2;
            t3 = foc->k_svpwm * t3;
        }
        float t7 = (ts - t2 - t3) / 2;
        foc->t_a = t7;
        foc->t_b = t2 + t3 + t7;
        foc->t_c = t3 + t7;
    } else if (sector == 2) {
        float t1 = -u1;
        float t3 = -u3;
        float sum = t1 + t3;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t1 = foc->k_svpwm * t1;
            t3 = foc->k_svpwm * t3;
        }
        float t7 = (ts - t1 - t3) / 2;
        foc->t_a = t7;
        foc->t_b = t3 + t7;
        foc->t_c = t1 + t3 + t7;
    } else if (sector == 6) {
        float t1 = u2;
        float t5 = u3;
        float sum = t1 + t5;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t1 = foc->k_svpwm * t1;
            t5 = foc->k_svpwm * t5;
        }
        float t7 = (ts - t1 - t5) / 2;
        foc->t_a = t5 + t7;
        foc->t_b = t7;
        foc->t_c = t1 + t5 + t7;
    } else if (sector == 4) {
        float t4 = -u2;
        float t5 = -u1;
        float sum = t4 + t5;
        if (sum > ts) {
            foc->k_svpwm = ts / sum;
            t4 = foc->k_svpwm * t4;
            t5 = foc->k_svpwm * t5;
        }
        float t7 = (ts - t4 - t5) / 2;
        foc->t_a = t4 + t5 + t7;
        foc->t_b = t7;
        foc->t_c = t5 + t7;
    }
}


