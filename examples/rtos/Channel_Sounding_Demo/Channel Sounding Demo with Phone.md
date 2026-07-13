# Channel Sounding Demo with Phone

# 硬件

1. 1x LP-EM-CC2745R10-Q1 Launchpad
2. 荣耀 Magic 8系列 或其他支持Channel Sounding的手机

# 软件环境

1. Code Composer Studio 集成开发环境
2. SimpleLink Low Power F3 SDK (9.20.00.81) or Above

# 步骤

1. 进入car_node_CC2745_920工程，找到app_car_node.c，把`xPHONE`改成`PHONE`
2. 编译并烧录程序。
3. 使用手机蓝牙调试助手连接“Car Node”
4. 使用串口工具观察结果

# 结果

使用串口工具，打开CC2745对应的串口，波特率为115200.

会看到下方结果

![img_v3_0213b_76254c2f-96b7-487f-a17f-a1640c0527ag.jpg](70d6b7bc7503258de2ecb0be00f31273.jpg)

## 注意：
1. 当前demo的模式是： 手机为`Central + Reflector`，CC2745 Launchpad为`Peripheral + Initiator`
2. Demo默认使用的为单天线进行测距。可以在app_car_node.c的`csProcedureParams`结构体变量进行修改。具体修改的定义请参考[https://software-dl.ti.com/simplelink/esd/simplelink_lowpower_f3_sdk/9.20.00.81/exports/docs/ble5stack/ble_user_guide/html/channel-sounding/channel-sounding.html](https://)