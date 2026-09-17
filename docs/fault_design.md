# Fault 状态机

系统状态：

```
SYSTEM_NORMAL
SYSTEM_FAULT
```

故障类型：

```
FAULT_NONE
FAULT_COMM_TIMEOUT
FAULT_STALL
FAULT_ENCODER
```

------

## CAN Timeout

LOCAL 模式监控：

```
0x123
```

REMOTE 模式监控：

```
0x301
```

超过 500 ms 未收到当前控制源的数据：

```
SYSTEM_FAULT
FAULT_COMM_TIMEOUT
target_rpm = 0
```

通信恢复后：

```
FAULT_COMM_TIMEOUT
↓
SYSTEM_NORMAL
```

该故障允许自动恢复。

------

## Stall Fault

典型判断条件：

```
target_rpm 较高
+
PWM 输出较高
+
actual_rpm 接近 0
+
持续一定时间
```

确认堵转后：

```
SYSTEM_FAULT
FAULT_STALL
```

该故障锁存。

即使通信恢复，也不会自动恢复。

必须由 PC 发送：

```
0x302
```

手动 Clear Fault。

------

## Encoder Fault

当编码器得到明显超出合理范围的转速时：

```
SYSTEM_FAULT
FAULT_ENCODER
```

同样属于锁存型故障，需要手动清除。

------

## Fault 优先级

通信超时不能覆盖已经存在的锁存型 Fault。

优先级逻辑：

```
FAULT_STALL
FAULT_ENCODER
        ↑
        │ 不允许被覆盖
        │
FAULT_COMM_TIMEOUT
        ↑
FAULT_NONE
```

例如：

```
FAULT_STALL
↓
此时 CAN 又发生 Timeout
↓
仍然保持 FAULT_STALL
```

避免通信恢复后误将严重故障自动清除。