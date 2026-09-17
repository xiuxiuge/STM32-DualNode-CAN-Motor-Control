# 项目开发演进

这个项目没有直接从完整系统开始，而是逐阶段演进。

## P0 - PWM Motor Control

目标：

```
MCU → PWM → DRV8871 → Motor
```

完成：

- TIM PWM
- 电机驱动
- PWM 占空比调速

------

## P1 - Encoder RPM

增加：

- AB Encoder
- TIM Encoder Mode
- RPM 计算

解决：

> 如何知道电机实际转速？

------

## P2 - PI Closed Loop

增加：

```
target_rpm
↓
PI
↓
PWM
↓
Motor
↓
Encoder
↓
rpm
```

并实现：

- PI 控制
- Anti-Windup
- 扰动恢复测试

------

## P3 - Dual Node CAN

Controller Node：

```
Potentiometer
↓
ADC
↓
target_rpm
↓
CAN
```

Motor Node：

```
CAN
↓
target_rpm
↓
PI
↓
Motor
```

系统从单 MCU 升级为分布式双节点结构。

------

## P4 - PC Control

加入：

```
PC
↓
USB-CAN
↓
Motor Node
```

实现：

- PC 实时监控
- REMOTE 控制
- LOCAL / REMOTE 控制权切换
- CAN 应用层协议

------

## P5 - FreeRTOS Architecture

随着系统业务增多，引入 FreeRTOS。

从：

```
while(1) polling
```

演进到：

```
ISR
Task
Queue
Thread Flag
Scheduler
```

实现：

- CAN RX Task
- CAN TX Task
- System Task
- Queue 解耦
- Event-driven CAN receive
- Critical Section

------

## P6 - Reliability & Protection

最后加入工程可靠性设计：

- CAN Timeout
- Stall Detection
- Encoder Fault
- Fault State Machine
- Fault Priority
- Manual Fault Reset
- Watchdog
- Task Health Monitoring
- Flash Parameter Persistence
- Online PI Parameter Configuration

至此完成 V1。