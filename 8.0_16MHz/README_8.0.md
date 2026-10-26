# 8.0：8 MHz与接口重映射

本目录由当前7.0完整复制，7.0保持不变。步态、图形、窄道、越障、一次发送和所有现场参数均继承7.0。

## 8.0硬件变化

- 外部晶振：8 MHz；`HSE_VALUE=8000000U`，`PLLM=4`，系统时钟仍为168 MHz。
- HC-SR04：PA0=TRIG，PA1=ECHO。PA1的5 V容限未按本板原理图确认，ECHO建议经分压或电平转换。
- 视觉串口：USART6，PC6=TX、PC7=RX，115200 8N1，中断收发。
- 调试控制台：USART1，PA9=TX、PA10=RX，115200 8N1。
- 舵机串口：USART2，PD5/PD6，1 Mbps，保持不变。

主控工程：`MDK-ARM/HumanoidRobot_8_0.uvprojx`。

CanMV程序在`Vision_8.0/main.py`，算法未因STM32引脚重映射而改变。
