# 蓝牙指令协议文档

> 适用固件版本：20260604
> 通信参数：9600 baud，8N1，每条指令以 `\r\n` 结尾

---

## 1. 通用说明

- 所有指令**不区分大小写**，固件内部统一转为小写处理
- 响应均以 `\r\n` 结尾
- 指令发送方向：App → 设备
- 响应方向：设备 → App（部分指令无响应或响应多行）

---

## 2. 模式切换

| 指令 | 说明 | 响应示例 |
|------|------|----------|
| `rd mode` | 切换到 RD 模式（北斗短报文通信模式） | `Switched to RD mode\r\n` `RD TXR output enabled\r\n` |
| `rn mode` | 切换到 RN 模式（GNSS 定位上报模式） | `Switched to RN mode\r\n` `RN RD standby ready\r\n` |

> **注意**：切换模式不会清除卡号信息。RN 模式切换后约 2 秒才会输出 `RN RD standby ready`，App 等待该字符串后再发送后续指令。

---

## 3. 卡号管理

| 指令 | 说明 | 响应示例 |
|------|------|----------|
| `target <卡号>` | 设置目标卡号 | `Target card set: 0362746\r\n` |
| `self <卡号>` | 设置自身卡号 | `Self card set: 0365966\r\n` |
| `get target` | 查询目标卡号 | `Target card: 0362746\r\n` |
| `get self` | 查询自身卡号 | `Self card: 0365966\r\n` |
| `cardword` | 查询自身卡号（仅 RD 模式，向模块发送查询指令） | 模块原始回复透传到蓝牙 |

> 卡号格式：4~12 位纯数字字符串。设置失败响应 `Invalid target card ID\r\n` 或 `Invalid self card ID\r\n`。

---

## 4. 短报文发送（RD 模式）

| 指令 | 说明 | 响应示例 |
|------|------|----------|
| `send <内容>` | 发送自定义文本到目标卡号 | `BD2 sent [hello] to 0362746\r\n` |
| `sos` | 发送 SOS 到目标卡号 | `BD2 SOS sent to 0362746\r\n` |
| `hello` / `ok` / `test` / `help` | 预设快捷消息 | `BD2 sent [hello] to 0362746\r\n` |

> **发送时**：设备同时向蓝牙回显发送帧内容（`$CCTXA,...*XX\r\n`），以及来自北斗模块的 `$BDFKI` 应答（代表模块已执行发送指令）。

### $BDFKI 应答解析

```
$BDFKI,082920,TCQ,Y,0,0*70
                    ↑ ↑
                    Y=发送成功  N=失败
                       ↑ 失败原因（0=无错误，1=频率未到，5=未检测到IC，11=校验错误）
```

---

## 5. 定位上报（RN 模式）

### 5.1 定时/定距上报设置

| 指令 | 说明 | 响应示例 |
|------|------|----------|
| `report time <分钟>` | 设置定时上报间隔，最小 1 分钟 | `Report time set: 2 min\r\n` |
| `report dist <米>` | 设置定距上报阈值，最小 500 米 | `Report distance set: 500 m\r\n` |
| `report off` | 恢复默认模式（2 分钟定时） | `Report default: 2 min\r\n` |
| `report get` | 查询当前上报配置及状态 | 见下方 |

`report get` 返回两行：
```
Report mode: time, interval: 2 min\r\n
Points: 15, elapsed: 65s, interval: 120s\r\n
```

字段说明：
- `Points`：当前窗口内已积累的定位点数（=0 说明 GNSS 尚未定位）
- `elapsed`：距上次上报已过去的秒数
- `interval`：当前设定的上报间隔秒数

### 5.2 上报数据包格式

上报触发后，设备向蓝牙输出：

```
RN report frame: $CCTXA,0362746,1,1,<HEX>*XX\r\n
\r\n
RN report sent: D|260392438N1190977779E;260392279N1190977699E;260392260N1190977691E\r\n
```

**`RN report sent:` 后的内容为定位数据包**，格式如下：

```
D|<纬度9位><N/S><经度10位><E/W>[;<纬度><N/S><经度><E/W>[;<纬度><N/S><经度><E/W>]]
```

| 字段 | 说明 |
|------|------|
| `D\|` | 固定包头 |
| 纬度 9 位 | NMEA 原始值去掉小数点，前补零（如 `2603.92438` → `260392438`） |
| `N` / `S` | 北纬 / 南纬 |
| 经度 10 位 | NMEA 原始值去掉小数点，前补零（如 `11909.77779` → `1190977779`） |
| `E` / `W` | 东经 / 西经 |
| `;` | 点分隔符 |

**代表点数量规则**：

| 当前窗口点数 | 包含点 |
|---|---|
| 1 | 仅 1 个点 |
| 2 | 2 个点（第 1、第 2） |
| ≥3 | 3 个点（第 1、中间、最后） |

**坐标还原（NMEA → 十进制度）**：

```
degrees    = raw / 10000000
minutes    = (raw % 10000000) / 100000.0
decimal_deg = degrees + minutes / 60.0
```

示例：`260392438` → degrees=26, minutes=03.92438 → 26.0654063°N

### 5.3 GNSS 诊断日志（每 30 秒）

```
GNSS: pts=15 el=65s iv=120s\r\n
```

该日志仅在**未触发上报时**输出，可用于判断定位和计时状态：
- `pts=0`：GNSS 模块尚未获得有效定位
- `pts>0, elapsed < interval`：定位正常，等待时间到

---

## 6. 循环发送（两种模式均可用）

| 指令 | 说明 | 响应示例 |
|------|------|----------|
| `loop on` | 开启循环发送（使用当前消息，默认 `SAFE`） | `Loop send enabled: SAFE\r\n` |
| `loop on <内容>` | 开启循环发送并设置消息内容（最长 32 字符） | `Loop send enabled: I am OK\r\n` |
| `loop off` | 关闭循环发送 | `Loop send disabled\r\n` |
| `loop time <分钟>` | 设置循环间隔，最小 1 分钟 | `Loop interval: 2 min\r\n` |
| `loop get` | 查询循环发送状态 | `Loop: on, msg: SAFE, interval: 2 min\r\n` 或 `Loop: off\r\n` |

每次循环发送触发后，蓝牙输出：
```
$CCTXA,0362746,1,1,<HEX>*XX\r\n
\r\n
Loop sent [SAFE] to 0362746\r\n
```

---

## 7. 电池电量查询

| 指令 | 说明 | 响应格式 |
|------|------|----------|
| `battery` | 查询电池电压和电量百分比 | `Battery: 3850mV 36%\r\n` |

**电量计算规则**：
- 3650 mV = 0%
- 4200 mV = 100%
- 线性插值，超出范围钳位

**硬件说明**：VBAT 经 R32/R33 各 100K 电阻分压后接 ADC（PA0），固件读取 ADC 值后乘以 2 还原真实电压。

---

## 8. 设备主动推送的非指令输出

以下内容由设备主动向蓝牙推送，App 需能识别并过滤或展示：

| 内容特征 | 来源 | 建议处理 |
|---|---|---|
| `$CCRMO,` / `$CCTXA,` | 设备发往北斗模块的指令回显 | 可忽略或显示在调试区 |
| `$BDFKI,...` | 北斗模块发送应答 | 解析第4字段（Y/N）判断是否发送成功 |
| `$GNRMC,...` | GNSS 原始数据（RN 模式） | 可解析用于显示实时位置 |
| `GNSS: pts=X ...` | 定位诊断日志（30秒一次） | 可展示在调试区或用于判断定位状态 |
| `System ready. Default mode: RD\r\n` | 开机完成 | 触发 App 初始化流程 |
| `RN RD standby ready\r\n` | RN 模式就绪 | 解锁 App 上报相关功能 |
| `Loop sent [...] to ...\r\n` | 循环发送成功 | 可更新 App 上次发送时间显示 |
| `RN report sent: D\|...\r\n` | 定位上报成功 | 解析坐标更新地图 |

---

## 9. 指令速查表

```
battery          → 查询电池电量
rd mode          → 切换 RD 模式
rn mode          → 切换 RN 模式
target <id>      → 设置目标卡号
self <id>        → 设置自身卡号
get target       → 查询目标卡号
get self         → 查询自身卡号
cardword         → 模块卡号查询（RD 模式）
send <text>      → 发送短报文（RD 模式）
sos              → 发送 SOS（RD 模式）
report time <n>  → 定时上报，n≥1（分钟）
report dist <n>  → 定距上报，n≥500（米）
report off       → 恢复默认上报（2分钟）
report get       → 查询上报状态
loop on          → 开启循环发送
loop on <msg>    → 开启并设置循环消息
loop off         → 关闭循环发送
loop time <n>    → 设置循环间隔（分钟）
loop get         → 查询循环发送状态
```
