# FreeRTOS 软件架构

Motor Node 使用 FreeRTOS 管理通信和系统业务。

主要 Task：

```
CAN_Rx_Task
CAN_Tx_Task
System_Task
Watchdog_Task
```

实时 PI 控制仍然放在 TIM6 ISR 中执行。

这样可以保证控制周期具有稳定的时间基准。

------

## CAN 接收路径

```
CAN Bus
   ↓
FDCAN Hardware
   ↓
Rx FIFO
   ↓
FDCAN RX Interrupt
   ↓
osThreadFlagsSet()
   ↓
CAN_Rx_Task
   ↓
解析 CAN Frame
   ↓
ControlCommandQueue
   ↓
System_Task
   ↓
业务逻辑
```

CAN 接收采用：

> Interrupt + Task Notification + Message Queue

而不是 Task 周期轮询。

------

## 为什么 ISR 不直接处理业务

ISR 只负责：

```
检测事件
↓
通知 Task
```

复杂的协议解析、控制模式切换等业务放入 Task 中处理。

原因：

- 减少中断执行时间
- 降低实时系统抖动
- 降低通信层与业务层耦合
- 更容易扩展和调试

------

### ControlCommandQueue

CAN_Rx_Task 不直接修改系统控制状态。

收到 CAN 帧后先转换为：

```
ControlCommand
```

然后发送到：

```
ControlCommandQueue
```

System_Task 再统一处理。

结构：

```
CAN_Rx_Task
     ↓
parse CAN frame
     ↓
ControlCommand
     ↓
Message Queue
     ↓
System_Task
```

这样实现了：

> 通信接收层与系统业务层解耦。