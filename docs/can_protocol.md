# CAN 应用层协议

## 0x123 - Local Target RPM

方向：

```
Controller → Motor
```

| Byte | 内容             |
| ---- | ---------------- |
| 0~1  | local_target_rpm |
| 2~3  | Reserved         |

16 位数据采用 Big Endian。

------

## 0x201 - Motor Runtime Status

方向：

```
Motor → PC
```

| Byte | 内容       |
| ---- | ---------- |
| 0~1  | actual_rpm |
| 2~3  | pwm_value  |

------

## 0x202 - System / Fault Status

方向：

```
Motor → PC
```

| Byte | 内容         |
| ---- | ------------ |
| 0    | system_state |
| 1    | fault_code   |
| 2    | control_mode |
| 3    | Reserved     |

### system_state

```
0 = SYSTEM_NORMAL
1 = SYSTEM_FAULT
```

### fault_code

```
0 = FAULT_NONE
1 = FAULT_COMM_TIMEOUT
2 = FAULT_STALL
3 = FAULT_ENCODER
```

### control_mode

```
0 = LOCAL
1 = REMOTE
```

------

## 0x203 - PI Parameters

方向：

```
Motor → PC
```

| Byte | 内容      |
| ---- | --------- |
| 0~1  | kp × 100  |
| 2~3  | ki × 1000 |

例如：

```
kp = 3.50
ki = 0.150
```

编码为：

```
01 5E 00 96
```

------

## 0x301 - Remote Control

方向：

```
PC → Motor
```

| Byte | 内容              |
| ---- | ----------------- |
| 0    | control_mode      |
| 1~2  | remote_target_rpm |

```
Byte0 = 0 → LOCAL
Byte0 = 1 → REMOTE
```

------

## 0x302 - Fault Reset

方向：

```
PC → Motor
Byte0 = 0x01
```

用于清除锁存型 Fault。

------

## 0x303 - PI Parameter Configuration

方向：

```
PC → Motor
```

### 修改 PI 参数

```
Byte0 = 0x01
Byte1~2 = kp × 100
Byte3~4 = ki × 1000
```

### 保存 PI 参数

```
Byte0 = 0x02
```

当前 RAM 中的 kp / ki 将保存至内部 Flash。