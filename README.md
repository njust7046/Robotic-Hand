# DH-5-6 灵巧手驱动 (Linux)

基于 Modbus-RTU 协议的 DH-5-6 灵巧手 C++ 驱动，支持位置控制、力控、电流反馈及抓取动作。

## 硬件连接

| 灵巧手引脚 | 连接 |
|-----------|------|
| 485_A (T/R+) | RS485转USB模块 T/R+ |
| 485_B (T/R-) | RS485转USB模块 T/R- |
| 24V | 24V 直流稳压电源正极 |
| GND | 24V 直流稳压电源负极 |

- 串口：`/dev/ttyUSB0`（默认）
- 波特率：115200，8N1，无校验
- 从机 ID：0x01（右手默认）

## 轴映射（右手）

| 轴 | 自由度 |
|----|--------|
| 轴1 | 拇指左右摆动 |
| 轴2 | 食指屈伸 |
| 轴3 | 中指屈伸 |
| 轴4 | 无名指屈伸 |
| 轴5 | 小拇指屈伸 |
| 轴6 | 拇指上下摆动 |

## 编译

```bash
g++ -std=c++14 -O2 -o dh5_hand_driver dh5_hand_driver.cpp
```

## 用法

```bash
# 演示（握拳 -> 剪刀 -> 布）
./dh5_hand_driver /dev/ttyUSB0 demo

# 单个动作
./dh5_hand_driver /dev/ttyUSB0 fist      # 握拳
./dh5_hand_driver /dev/ttyUSB0 scissors  # 剪刀
./dh5_hand_driver /dev/ttyUSB0 paper     # 布（张开）
./dh5_hand_driver /dev/ttyUSB0 bottle    # 抓瓶子
```

每次运行会自动执行：清除错误 → 张开回零 → 执行指定动作。

## 代码结构

```
LinuxSerialPort   串口读写封装（termios）
DhHand            Modbus-RTU 驱动层
  ├── writeSingleReg()    0x06 写单寄存器
  ├── writeMultiRegs()    0x10 写多寄存器
  ├── readHoldingRegs()   0x03 读寄存器
  ├── homeCloseAllAxes()  闭合回零
  ├── homeOpenAllAxes()   张开回零
  ├── setAxesPositions6() 设置6轴位置
  ├── setAxesSpeeds6()    设置6轴速度
  ├── setAxesForces6()    设置6轴力
  ├── readAxisStates6()   读6轴运行状态
  ├── readAxesPositions6()读6轴当前位置
  └── readCurrents5()     读5轴电流
HandActions       动作层
  ├── openHand()          张开手
  ├── fist()              握拳
  ├── paper()             布
  ├── scissors()          剪刀
  ├── pinch()             对捏（拇指+食指）
  ├── graspBottle()       抓瓶子（电流/堵转保护）
  └── calibrateAxis()     单轴标定
```

## 寄存器速查

| 地址 | 功能 | 说明 |
|------|------|------|
| 0x0100 | 回零控制 | 0x0555=闭合，0x0AAA=张开 |
| 0x0101~0x0106 | 轴1~6目标位置 | 单位 0.01mm |
| 0x0107~0x010C | 轴1~6力 | 20~100，百分比 |
| 0x010D~0x0112 | 轴1~6速度 | 1~100，百分比 |
| 0x0200 | 回零状态 | 每轴2bit：00未初始化/01成功/10进行中 |
| 0x0201~0x0206 | 轴1~6运行状态 | 0运动中/1到位/2堵转 |
| 0x0207~0x020C | 轴1~6当前位置 | 单位 0.01mm |
| 0x0213~0x0217 | 轴1~5电流 | 单位 mA，有符号 |
| 0x0501 | 清除错误 | 写1清除 |
| 0x0503 | 系统重启 | 写1重启 |

## 抓瓶子策略

`graspBottle()` 采用开环 + 接触检测：

1. 张开到初始位置
2. 拇指预定位（轴1/轴6）
3. 四指（轴2~5）同时向闭合方向运动
4. 高频轮询（默认60ms）检测堵转或电流突增
5. 触发的轴立即锁定当前位置，其余轴继续
6. 四指全部触发后完成

## 标定

首次使用或更换硬件后，建议对各轴进行标定：

```cpp
HandActions actions(hand);
// 标定食指（轴2），从0开始，步进50，最大1000
actions.calibrateAxis(1, 0, 50, 1000);
```

标定完成后，用 `setOpenPositions()` / `setClosePositions()` 更新配置。

## 指示灯

- 蓝灯：正常上电工作
- 红灯：IAP 升级状态，需通过上位机升级固件

## 注意事项

- 每次断电重新上电后必须先执行回零（`homeOpenAllAxes` 或 `homeCloseAllAxes`）
- 上电后若自动张开闭合往复运动，说明处于老化测试模式，写 `0x0504=0` 关闭
- 串口权限不足时：`sudo chmod 666 /dev/ttyUSB0` 或将用户加入 `dialout` 组
