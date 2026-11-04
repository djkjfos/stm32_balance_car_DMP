# 平衡车调试速查表

## 串口命令
- `DBG` → 打印全部状态（调 `Balance_PrintDiagnostics()`）
- `CAL LAST` → 最近一次校准失败原因
- `CAL WINDOW` → 校准窗口统计量
- `ARM 1` → 串口启动
- `ARM 0` → 串口停止
- `CAL` → 重新校准

---

## 症状1：校准一直不成功 / 一直停在CALIBRATING

| 查什么 | 在哪看 | 正常应该是 |
|---|---|---|
| 当前校准到第几个 | `CAL=xxx/200` | 从0涨到200 |
| 失败原因 | `CAL LAST` → `cal_last_block` | NONE |
| 重收次数 | `RESTARTS=xxx` | 0或很小 |

**常见失败原因对照：**
- `ANGLE_RANGE` → 倾角超过15°，车没放正
- `GYRO_RANGE` → 陀螺轴超20°/s，车在转
- `GRAVITY_RANGE` → 加速度模长不在0.8~1.2g，车被拿起来了
- `WHEEL_MOVED` → 轮子在转，推车了
- `ANGLE_NOISE` → 倾角SD>1°，手在抖/桌子震
- `GYRO_NOISE` → 陀螺SD>2°/s
- `ANGLE_DRIFT` → 前后半段差>0.5°，你在慢慢挪车

---

## 症状2：校准完了但不启动 / 启动不了

| 查什么 | 在哪看 | 正常应该是 |
|---|---|---|
| 当前状态 | `STATE=` | READY |
| 故障码 | `FAULT=` | 0(NONE) |
| 数据新鲜度 | `AGE=xxx` ms | <20 |
| 倾角偏差 | `REL=xxx` | <10° |
| 按键状态 | `KEY_PRESSED=` | 0（没按） |
| 上电自启请求 | `BOOT_AUTO=` | 1或0 |
| 零点角 | `ZERO=xxx` | 接近你放车的角度 |

**常见原因：**
- 数据AGE>20 → IMU读失败，查I2C
- REL>10° → 车没扶正
- KEY_PRESSED=1 → PB1按着不放，松开
- 校准完串口提示 `auto-start blocked` → 按键按着，松开后按PB1

---

## 症状3：运行中突然停机

| 查什么 | 在哪看 | 正常应该是 |
|---|---|---|
| 状态 | `STATE=` | FAULT |
| 故障码 | `FAULT=` | 看下面 |

**故障码对照：**
- `FAULT=2 (TILT)` → 倒了45°，扶正后自动恢复
- `FAULT=1 (IMU)` → I2C读MPU失败，看 `IMU DETAIL`
- `FAULT=3 (OVERRUN)` → 主循环卡死，pending_ticks积压
- `FAULT=4 (HARDWARE)` → 电机/编码器初始化失败

---

## 症状4：倒了之后不恢复 / 恢复不了

| 查什么 | 在哪看 | 正常应该是 |
|---|---|---|
| 恢复请求 | `AUTO=` | 1（运行中倒的才有） |
| 恢复计数 | `RECOVER=xx/40` | 数到40就恢复 |
| 当前倾角 | `UPRIGHT=10` | |

**常见原因：**
- `AUTO=0` → 不是运行中倒的（校准阶段倒的不自动恢复）
- 计数一直0 → 角度还没回到±10°内 / 轮子在转 / 按键按着

---

## 症状5：IMU通信失败

| 查什么 | 在哪看 |
|---|---|
| HAL返回值 | `IMU=xxx`（0=OK） |
| I2C错误码 | `I2CERR=0x...` |
| 失败阶段 | `stage=` |
| DMP错误 | `dmp=` |

---

## 症状6：电机不转 / 转向反了

| 查什么 | 在哪看 |
|---|---|
| STBY电平 | `STBY=`（1=使能） |
| PWM占空比 | `PWM_A=xxx PWM_B=xxx` |
| 编码器读数 | `ENC_A=xxx ENC_B=xxx`（推车应该变化） |

---

## 核心变量速查

| 变量 | 含义 |
|---|---|
| `balance_state` | 0=校准 1=就绪 2=运行 3=故障 |
| `balance_fault` | 0=无 1=IMU 2=倾倒 3=超时 4=硬件 |
| `angle_zero` | 直立零点（校准记下的角度） |
| `angle` | 当前实测倾角 |
| `pending_ticks` | 中断积压节拍，≥2=主循环卡死 |
| `tilt_resume_pending` | 1=允许扶正后自动恢复 |
| `boot_auto_start_pending` | 1=上电校准完要自动跑 |
| `cal_last_block` | 最近一次校准失败原因字符串 |
