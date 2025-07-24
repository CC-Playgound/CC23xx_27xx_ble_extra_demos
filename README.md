# CC23xx_27xx_ble_extra_demos

基于TI CC23xx和CC27xx系列芯片的BLE demo合集. 所有demo均基于SimpleLink F3 SDK开发，目的是演示一些常用的BLE功能

# 环境搭建

Demo需要运行在CC23xx或CC27xx开发板上，导入工程前请先下载SimpleLink F3 SDK

#### 硬件环境

Demo基于CC23xx或CC27xx开发板，根据工程所在文件夹名称选择对应开发板：

- [LP-EM-CC2340R5 Launchpad](https://www.ti.com/tool/LP-EM-CC2340R5)
- [LP-EM-CC2745R10-Q1 Launchpad](https://www.ti.com/tool/LP-EM-CC2745R10-Q1)
- [LP-XDS110 Debugger](https://www.ti.com/tool/LP-XDS110ET)

#### 软件环境

- [Code Composer Studio集成开发环境](https://www.ti.com/tool/CCSTUDIO)
- [SimpleLink Low Power F3 SDK (9.11.00.18)](https://www.ti.com/tool/download/SIMPLELINK-LOWPOWER-F3-SDK)

#### 手机App（可选）

- [SimpleLink Connect Application on iOS](https://apps.apple.com/app/simplelink-connect/id6445892658)
- [SimpleLink Connect Application on Android](https://play.google.com/store/apps/details?id=com.ti.connectivity.simplelinkconnect)
- [SimpleLink Connect Application source code](https://www.ti.com/tool/SIMPLELINK-CONNECT-SW-MOBILE-APP)

# Demo列表

<table>
  <tbody>
    <tr>
      <th>Demo名称</th>
      <th>硬件平台</th>
      <th>简介</th>
    </tr>
    <tr>
      <td>basic_ble_practical_features_peripheral</td>
      <td>LP-EM-CC2340R5</td>
      <td>
        基于basic_ble例程从机角色添加以下功能：
        <ul>
          <li>定时发送参数更新请求</li>
          <li>关闭CSA#2(或其它LE feature set)</li>
          <li>通过连接事件回调获取RSSI</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>basic_ble_practical_features_central</td>
      <td>LP-EM-CC2340R5</td>
      <td>
        基于basic_ble例程主机角色添加以下功能：
        <ul>
          <li>利用ADV report过滤扫描结果</li>
          <li>GATT服务发现</li>
        </ul>
      </td>
    </tr>
    <tr>
      <td>Two_diffAddr_ADV</td>
      <td>LP-EM-CC2745R10-Q1</td>
      <td>实现两路基于不同地址类型的广播</td>
    </tr>
  </tbody>
</table>