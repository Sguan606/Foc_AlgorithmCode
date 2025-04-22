# Foc_AlgorithmCode
Brushless motors are commonly driven by magnetic directional vector control FOC, which generates a three-phase 120-degree phase angle current by controllable PWM waves, and the three phases can be synthesized into a two-dimensional plane magnetic vector at any angle for controlling the motor. Therefore, brushless motors can be used for smooth rotation, or rotation in a specified position, or to simulate physical knobs, physical springs, and force feedback knobs.

无刷电机常用磁定向矢量控制foc驱动，由可控PWM波产生三相120度相位角的电流，三个相可合成二维平面任意角度的磁矢量，用于控制电机。因此无刷电机可以用于平滑转动，或者指定位置转动，或者模拟物理旋钮、物理弹簧、力反馈旋钮。

本项目用到的硬件资源有：2804三相无刷电机，AS5600磁编码器，INA240A2的50倍运放电流检测模块x2，Drv8313的三相电机驱动板，12v电源供电，一块Stm32G431cbt6。我使用到了STM32微控制器的外设如下：
![image](https://github.com/user-attachments/assets/f80d7eff-ce76-41b7-acc5-4adc655b2f15)

(ADC)将INA240电流检测模块输出的模拟电压ADC采样，得到相电流。采样两路，并通过基尔霍夫电流定律计算出第3路...然后就可以把数据简单滤波，用于电机的电流闭环控制。

(I2C)使用stm32的I2C外设与AS5600通信，时刻获取电机旋转位置的角度和角加速度...可用于实现3相无刷电机的位置闭环和速度闭环。

(SVPWM)通过定时器TIM1产生3相可调占空比的PWM波，再使用stm32的TIM2产生定时中断，更迭PWM波至SVPWM驱动无刷电机开环转动。

(ADC) samples the analog voltage ADC output by the INA240 current detection module to obtain the phase current. Two channels were sampled, and the third channel was calculated by Kirchhoff's current law... Then the data can be simply filtered and used for closed-loop current control of the motor.

(I2C) uses the I2C peripherals of the stm32 to communicate with the AS5600 to obtain the angle and angular acceleration of the motor's rotation position at all times... It can be used to realize the position and speed closed loop of 3-phase brushless motors.

(SVPWM) generates a 3-phase PWM wave with adjustable duty cycle through timer TIM1, and then uses TIM2 of stm32 to generate a timing interrupt, and the PWM wave is changed to SVPWM to drive the brushless motor to rotate in the open loop.
![IMG_3824](https://github.com/user-attachments/assets/8143ccb0-e906-4154-9e3a-427c67112574)


初次使用，需要在main函数中调用FOC_InitControllers(void)初始化PID控制器，并设置foc_control_params.mode选择控制模式。

控制模式：

FOC_MODE_OPEN_LOOP: 开环速度控制

FOC_MODE_CURRENT_LOOP: 仅电流环控制

FOC_MODE_SPEED_LOOP: 速度环+电流环控制

FOC_MODE_POSITION_LOOP: 位置环+速度环+电流环控制

后期我们为了适应不同规格的无刷电机，可以通过修改PID控制器的Kp、Ki、Kd参数来调整控制性能，设置output_limit参数限制PID输出范围。

位置环: 设置foc_control_params.target_position(rad)

速度环: 设置foc_control_params.target_speed(rad/s)

电流环: 设置foc_control_params.target_current_q和foc_control_params.target_current_d

![image](https://github.com/user-attachments/assets/b075af8b-7cf5-4d9a-93d2-0afcdda24654)     ![image](https://github.com/user-attachments/assets/46287e48-3f0d-4f3b-8463-68bfc616c729)     ![image](https://github.com/user-attachments/assets/9bb1257e-4586-4326-a23b-3bb79dfedfc8)

位置环->速度环->电流环这样的串级控制结构可以使得系统响应更快、更稳定。每个环的PID参数需要根据我们自己实际电机特性进行调整...我自己的也暂时没有细调。目前就是刚好能满足实验需求而已。然后值得说的是，我们需要开启TIM1的任意3通道的PWM输出比较模式，TIM2设置成1ms的定时中断（即中断频率设置为1kHZ）。Then，生成SVPWM的代码和计算PID控制的函数运行对微控制器的主频和FPU要求还挺高的。为了方便运行，我们在实际项目中，主函数就需要考虑少在“while(1)死循环”中放一些长期占用运行时间的东西。
