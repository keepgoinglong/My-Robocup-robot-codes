# 6.0：基于5.3的HC-SR04独立调试版

本目录由5.3完整复制，新建6.0；5.3原目录未修改。原步态、视觉、SG90和VL53L0X代码均保留，新增HC-SR04独立串口调试，不把超声波结果接入行走控制。

## 接线

- HC-SR04 VCC → 板上5V
- HC-SR04 GND → 板上GND（必须共地）
- HC-SR04 TRIG → PA2
- HC-SR04 ECHO → PA3（直接连接）

照片右侧排针明确引出了PA2、PA3、5V和GND。5.3当前代码未占用PA2/PA3；PA11留给SG90，PC10/PC11留给视觉UART4，PD5/PD6留给飞特舵机调试板，PB10/PB11及PC0/PC1留给VL53L0X。

控制板原理图确认主控为STM32F407VET6，PA3从MCU直接引到扩展排针，中间没有板载分压或缓冲；STM32F407数据手册中PA3属于FT（5V tolerant）数字I/O，因此本板本接法允许HC-SR04的ECHO直接接PA3。代码将PA3配置为普通数字输入，不能改成模拟输入。这个结论只适用于本板PA3，不能把HC-SR04的ECHO直接接到任意GPIO。

上电前务必核对排针丝印，传感器与控制板必须共地；VCC只能接5V，不能接3.3V。若更换主控板或传感器型号，必须重新核对电平。

## 调试方法

1. 编译并烧录 `MDK-ARM/HumanoidRobot_6_0.uvprojx` 生成的6.0 HEX。
2. USART1调试串口保持115200 8N1。
3. 上电后不启动步态，确认输出 `HumanoidRobot 6.0 HC-SR04 debug boot`。
4. 输入 `u` 开始，每250ms输出一次距离；再次输入 `u` 停止。
5. 只有 `ROBOT_IDLE`、舵机扭矩关闭时允许启动。进入其他机器人状态会自动停止；输入 `x` 也会停止测距并请求舵机卸力。

正常输出示例：

```text
HC-SR04 distance=356 mm echo=2065 us
```

错误提示：`ECHO_STUCK_HIGH`、`NO_ECHO_RISE`、`ECHO_TOO_LONG` 或 `OUT_OF_RANGE`。无回波最长等待约30ms，因此本调试功能禁止在行走时运行。距离使用常见换算 `mm=(echo_us*10)/58`，实际精度受温度、安装角度、障碍物材质和模块个体影响，必须用尺子实测校准。


## 2026-10-15：替换行走动作

动作源换为用户提供的 `D:/Program Datas/QQ Datas/my_actionsfinalorigin (3).json`，原样复制到 `Actions/my_actionsfinalorigin (3).json`。Validation/action_model.py与generate_actions.py已指向新文件，重新生成Inc/action_frames.h及配套图形腿部表Inc/shape_frames.h。旧源文件保留。

仍由第6帧起始，再按第1～6帧循环；原第2/5帧为落地锚点。完整保留新JSON的关节位置、限位、运动时间、停留、单舵机时序、速度和ACC，六帧周期仍为2700ms。`checked`是编辑器勾选状态，不用于跳过帧；本次仍执行完整六帧。图形动作保持原三秒以及转向髋保留逻辑。

修改前生成器、动作表、图形表、说明与已有固件备份至 `Backup_before_walk_origin3_20261015_155726`。6.0的HC-SR04调试及视觉代码未修改。

验证：Keil全量编译0错误、180警告；读取新6.0 ARM固件，六帧位置/时长/速度/ACC/单舵机时序与源JSON逐项一致，起始帧等于新第6帧；1ms/7ms轮询轨迹共2190包通过，2700ms周期通过；右纠偏及图形1～5左右转向共10组组合回归通过。证据在Validation/origin3_results.json和keil_origin3_build.log。仅软件与模拟验证，尚未烧录验证机械稳定性。

工程：`MDK-ARM/HumanoidRobot_6_0.uvprojx`；固件：`MDK-ARM/HumanoidRobot_6_0/HumanoidRobot_6_0.hex`。
