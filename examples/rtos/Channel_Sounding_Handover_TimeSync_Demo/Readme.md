# CS_Handover_TimeSync_Demo Readme

# 硬件

1. 3x LP-EM-CC2745R10-Q1 Launchpad

# 软件环境

1. Code Composer Studio 集成开发环境
2. SimpleLink Low Power F3 SDK (9.20.01.21) or Above
3. Python(可选)

# 步骤

1. 编译`car_node_tsa`及`car_node_tso`工程，并烧录到其中的两个CC2745 Launchpad作为CS_Handover的节点
2. 将这两个CC2745的板子按下方进行GPIO连接。
   |CC2745-Launchpad 1|CC2745 Launchpad 2|
   |--|--|
   |DIO21|DIO22|
   |DIO22|DIO21|
   |GND|GND|
3. 将所附key_node out文件，烧录到第三块CC2745 Launchpad.
4. Key_node会搜索CS_Handover的TSO节点发出的光波广播并进行连接，连接后TSO节点会发起CS。
5. 使用串口工具观察结果两个节点的串口，波特率为`3000000`，可以观察到CS结果在两个节点间交替打印。

# 注意

1. Channel Sounding的参数默认使用2x2的天线，可以在car node里面进行调整，参考channel sounding demo目录下的readme。

