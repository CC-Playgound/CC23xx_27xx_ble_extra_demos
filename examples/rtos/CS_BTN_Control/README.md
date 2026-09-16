# Car Node 按钮控制 CS 测距

本例程基于 SimpleLink LowPower F3 SDK 9.20.01.21 的 `car_node` 例程修改，通过 LaunchPad 上的两个物理按钮控制 BLE Channel Sounding 测距流程的开始和停止，可用于 demo 演示和调试。

## 软件环境

- SDK：SimpleLink LowPower F3 SDK 9.20.01.21
- IDE：Code Composer Studio

## 硬件环境

- CAR Node：LP-EM-CC2745R10 LaunchPad（烧录本例程）
- Key Node：LP-EM-CC2755R10 LaunchPad（烧录 SDK 自带 key_node 例程）

## 启动步骤

1. 先给 Key Node 烧录 SDK 自带的 `key_node` 例程并上电
2. 再给 Car Node 烧录本例程并上电
3. 打开串口终端，连接 Car Node 的 XDS110 虚拟串口，波特率 115200
4. Car Node 上电后自动扫描并连接 Key Node，连接成功后串口会打印连接信息
5. 按 Button 0 启动 CS 测距，串口持续输出距离结果
6. 按 Button 1 停止 CS 测距
