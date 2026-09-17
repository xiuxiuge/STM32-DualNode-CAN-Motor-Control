# Flash 参数持久化

PI 参数保存于 STM32G431 内部 Flash。

参数结构：

```
typedef struct
{
    uint32_t magic;
    float kp;
    float ki;
    uint32_t reserved;
} PI_Params;
```

Flash 参数页：

```
Page 63
Address: 0x0801F800
```

启动流程：

```
Power On
↓
LoadPIParams()
↓
检查 magic
↓
有效？
├── Yes → 加载 Flash kp / ki
└── No  → 使用默认 PI 参数
```

保存流程：

```
PC 修改 PI
↓
RAM 参数变化
↓
测试控制效果
↓
PC Save Command
↓
SavePIParams()
↓
Flash
```

保存前会比较当前 Flash 参数与准备写入的参数。

如果参数相同：

```
直接返回
不擦除
不重新写入
```

用于减少无意义 Flash 擦写。