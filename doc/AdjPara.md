# 调参指南

所有参数位置：[src/ctrl.cpp](src/ctrl.cpp#L174) | [src/motor.cpp](src/motor.cpp#L6)

---

## LQR 输出增益

```cpp
float lqrTRatio = 0.24f;  // 车轮扭矩系数
float lqrTpRatio = 1.0f;  // 髋关节扭矩系数
```

| 参数 | 调大 | 调小 |
|------|------|------|
| lqrTRatio | 加速响应，可能震荡 | 减弱响应，可能站不住 |
| lqrTpRatio | 腿更硬 | 腿更软 |

---

## kRatio 反馈矩阵权重

```cpp
float kRatio[2][6] = {{1,1,1,1,1,1},  // kRatio[0]: 车轮方向
                      {1,1,1,1,1,1}}; // kRatio[1]: 髋关节方向
```

| 列 | 状态量 | 物理含义 |
|:---:|------|------|
| [0] | theta | 腿偏离竖直的角度 |
| [1] | dTheta | 腿摆动角速度 |
| [2] | x | 车体位置偏差 |
| [3] | dx | 车体前进速度 |
| [4] | phi | 车身 pitch 倾角 |
| [5] | dPhi | 车身 pitch 角速度 |

---

## 物理参数

```cpp
const float wheelRadius = 0.0325; // 车轮半径(m)
const float legMass = 0.062f;     // 单腿质量(kg)
```

---

## PID 参数

### roll 轴（左右平衡）
```cpp
PID_Init(&rollPID.inner,   Kp=0.5,  Ki=0, Kd=2.5, deadband=0, max=5);
PID_Init(&rollPID.outer,   Kp=10,   Ki=0, Kd=0,   deadband=0, max=3);
```

### yaw 轴（转向）
```cpp
PID_Init(&yawPID.inner,    Kp=0.01, Ki=0, Kd=0,   deadband=0, max=0.1);
PID_Init(&yawPID.outer,    Kp=10,   Ki=0, Kd=0,   deadband=0, max=2);
```

### 腿长控制
```cpp
PID_Init(&legLengthPID.inner, Kp=10, Ki=1, Kd=30, deadband=2, max=10);
PID_Init(&legLengthPID.outer, Kp=5,  Ki=0, Kd=0,  deadband=0, max=0.5);
```

### 左右腿角度差
```cpp
PID_Init(&legAnglePID.inner,  Kp=0.04, Ki=0, Kd=0, deadband=0, max=1);
PID_Init(&legAnglePID.outer,  Kp=12,   Ki=0, Kd=0, deadband=0, max=20);
```

---

## 电机全局倍率

```cpp
float motorOutRatio = 1.0f; // motor.cpp:6
```
初次测试可降到 0.5 降低所有电机出力。

---

## 场景速查

| 现象 | 先调 |
|------|------|
| 前后快速震荡 | 降 `lqrTRatio`、降 `kRatio[0][4]` `[0][5]` |
| 前后慢晃站不住 | 升 `lqrTRatio`、升 `kRatio[0][4]` |
| 腿伸缩太软 | 升 `legLengthPID.inner` Kp |
| 左右晃 | 升 `rollPID.inner` Kp / Kd |
| 转向无力/过猛 | 调 `yawPID.inner` Kp |
| 左右腿不同步 | 调 `legAnglePID.outer` Kp |
