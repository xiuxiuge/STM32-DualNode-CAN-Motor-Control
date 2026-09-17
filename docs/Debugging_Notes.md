# Debugging Notes

开发过程中遇到过一些典型问题：

### StdFiltersNbr 配置错误

配置了：

```
Filter0
Filter1
Filter2
```

但：

```
StdFiltersNbr = 2
```

导致第三个 Filter 实际没有正确分配。

同时由于 Global Filter 设置为 Reject，0x302 无法进入 Rx FIFO。

------

### CubeMX 重新生成覆盖配置

曾直接修改生成代码中的 FDCAN 配置。

重新 Generate Code 后：

```
StdFiltersNbr
```

恢复为 CubeMX 中保存的旧值。

因此：

> 外设初始化参数必须优先修改 CubeMX / .ioc，而不是只修改自动生成代码。

------

### FreeRTOS Heap 不足

新增 Queue / Task 后：

```
HEAP STILL AVAILABLE = 0
```

导致最后创建的 Task 返回 NULL。

扩大：

```
configTOTAL_HEAP_SIZE
```

后解决。

------

### TIM6 Interrupt 未开启

迁移 RTOS 后：

```
target_rpm 正常变化
rpm / pwm_value 不变化
```

最终发现 TIM6 NVIC 未启用。

------

### CAN Physical Layer Issue

曾出现：

```
PC ↔ Motor 正常
Controller ↔ Motor 异常
```

最终原因是 Controller CAN 接线松动。

说明：

> 总线整体可工作，并不代表每个节点分支都正常。

------

### Logic Analyzer / USB Hub Interference

Logic Analyzer 和开发板连接同一个 USB Hub 时，曾出现 CAN ACK 异常和重复发送。

更换 USB 连接路径后恢复。

------

### Float Encoding Error

错误写法：

```
(uint16_t)kp * 100
```

会先丢掉小数部分。

正确方式：

```
(uint16_t)lroundf(kp * 100.0f)
```