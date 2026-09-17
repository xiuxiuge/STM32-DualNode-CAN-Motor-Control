# Watchdog

系统使用 STM32 IWDG。

不是简单让某个 Task 周期喂狗，而是使用关键任务健康检测：

```
System_Task ─────┐
                 │ alive flag
CAN_Tx_Task ─────┤
                 ↓
           Watchdog_Task
                 ↓
       所有关键任务健康？
           ↓          ↓
          Yes         No
           ↓          ↓
     Refresh IWDG    不喂狗
                      ↓
                  MCU Reset
```

这样可以检测：

> 某个关键 Task 已失效，但其他 Task 仍然正常执行

这种普通“单 Task 喂狗”无法发现的问题。

系统同时能够通过 RCC Reset Flag 区分：

```
IWDG Reset
Normal / Manual Reset
```