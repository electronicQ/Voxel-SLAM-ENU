# 北云 C2 ROS1 GNSS 驱动

`beiyun_gnss` 从北云 C2 的串口字节流中读取 ASCII 语句，校验并解析 `#BESTPOSA` 和 `GPRMC/GNRMC`，然后合成为 `sensor_msgs/NavSatFix`。

驱动支持两种 `NavSatFix.header.stamp` 来源：

- `gprmc_utc`：使用 GPRMC 的 UTC 日期和时间，保持当前 BESTPOSA+GPRMC 配对逻辑；
- `system_now`：使用 `ros::Time::now()`，只要收到有效 BESTPOSA 即可发布。

默认值是 `gprmc_utc`，因此默认行为与现有程序保持一致。

当前正式消息采用：

```text
BESTPOSA：经纬度、高程、解状态、位置标准差
GPRMC：gprmc_utc 模式下提供完整 UTC 日期和时间
GPGGA：仅发布原始诊断话题，不参与正式 NavSatFix 组装
```

## 运行

```bash
roslaunch beiyun_gnss beiyun_gnss.launch
```

当前 launch 使用：

```text
port = /dev/ttyACM1
baudrate = 115200
frame_id = gps
navsatfix_topic = /rtk/navsatfix
pair_tolerance_seconds = 0.2
timestamp_source = gprmc_utc
signal_timeout_seconds = 2.0
```

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `~port` | `/dev/ttyUSB0` | 串口设备 |
| `~baudrate` | `115200` | 串口波特率 |
| `~serial_timeout` | `0.1` | 串口读取超时，秒 |
| `~frame_id` | `gps` | NavSatFix 坐标系名称 |
| `~navsatfix_topic` | `/rtk/navsatfix` | 正式 NavSatFix 话题 |
| `~pair_tolerance_seconds` | `0.2` | BESTPOSA 与 RMC 时间配对容差，秒 |
| `~timestamp_source` | `gprmc_utc` | 时间戳来源：`gprmc_utc` 或 `system_now` |
| `~signal_timeout_seconds` | `2.0` | 超过该时间没有有效输出时提示信号异常，秒 |

## 发布话题

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/rtk/navsatfix` | `sensor_msgs/NavSatFix` | BESTPOSA 与时间戳模式对应时间源合成后的正式消息 |
| `/rtk/raw_rmc` | `std_msgs/String` | 原始 RMC 字符串 |
| `/rtk/raw_gga` | `std_msgs/String` | 原始 GGA 字符串 |
| `/rtk/raw_bestposa` | `std_msgs/String` | 原始 `#BESTPOSA` 字符串 |

## 消息组合逻辑

### RMC 解析

驱动要求 RMC 通过 NMEA 校验，并且：

- 时间字段有效；
- 日期字段有效；
- 状态字段为 `A`，表示有效数据。

RMC 的 `hhmmss.ss` 和 `ddmmyy` 被组合为完整 UTC 时间，并转换成 ROS `header.stamp`。

RMC 中的经纬度只作为解析结构中的可选信息保留，不用于正式 NavSatFix 的位置输出。

### BESTPOSA 解析

驱动要求 BESTPOSA 通过北云协议规定的 CRC32 校验，并且：

- 时间状态不是 `UNKNOWN`；
- 时间状态不是 `FINESTEERING_INVALID`；
- GPS 周和周内秒有效；
- `sol_status` 为 `SOL_COMPUTED`；
- `pos_type` 有效且不是 `NONE`；
- 经纬度、高程和三个位置标准差有效。

在 `gprmc_utc` 模式下，BESTPOSA 的 GPS 周和周内秒会转换为 UTC，仅用于和 RMC 的 UTC 时间进行配对，正式 ROS 时间戳来自 RMC。`system_now` 模式不要求 RMC。

### 时间戳模式

#### `gprmc_utc`

这是默认模式，发布条件保持不变：

```text
有效 BESTPOSA + 有效 GPRMC + BESTPOSA/RMC 时间匹配
    -> 发布 NavSatFix
    -> header.stamp 使用 GPRMC UTC 时间
```

`BESTPOSA` 的 GPS 周和周内秒会转换为 UTC，仅用于和 RMC 的 UTC 时间进行配对。正式 ROS 时间戳来自 RMC。

#### `system_now`

系统时间模式不要求等待 GPRMC：

```text
有效 BESTPOSA
    -> 发布 NavSatFix
    -> header.stamp 使用 ros::Time::now()
```

此时 GPRMC 仍然发布到 `/rtk/raw_rmc`，但不再是正式 NavSatFix 发布的必要条件。驱动使用 BESTPOSA 的 GPS 周和周内秒判断重复历元。

`ros::Time::now()` 表示驱动接收或发布消息时的 ROS 时间，不等于 GNSS 观测发生的真实 UTC 时间。若用于 LIGO 建图，系统时钟应当已经通过 PPS、NTP 或其他方式与 GNSS 时间同步；否则优先使用 `gprmc_utc`。

发布成功后，驱动会清空当前 BESTPOSA/RMC 缓存，避免重复使用同一时间数据。

### 信号缺失提示

驱动会持续检查有效定位数据和正式 NavSatFix 发布状态。超过 `signal_timeout_seconds` 没有有效 BESTPOSA 时，终端会节流提示：

```text
GNSS signal missing or no valid BESTPOSA; cannot publish NavSatFix
```

在 `gprmc_utc` 模式下，如果 BESTPOSA 持续有效但始终没有匹配的 GPRMC，还会提示：

```text
Valid BESTPOSA received, but no matching GPRMC; cannot publish NavSatFix in gprmc_utc mode
```

提示使用节流日志，不会在每次循环中重复刷屏。串口设备无法打开时，驱动仍会输出原有的串口打开失败提示。

## NavSatFix 字段映射

下面列出当前驱动对 `sensor_msgs/NavSatFix` 各字段的实际来源。

### Header

| NavSatFix 字段 | 来源 |
| --- | --- |
| `header.seq` | 未由驱动显式设置，由 ROS 消息机制处理 |
| `header.stamp` | `gprmc_utc` 模式使用 RMC 的 UTC 日期和时间；`system_now` 模式使用 `ros::Time::now()` |
| `header.frame_id` | ROS 私有参数 `~frame_id`，当前 launch 为 `gps` |

在 `gprmc_utc` 模式下，`BESTPOSA` 的 GPS 周和周内秒只用于时间匹配，不直接作为最终 `header.stamp`。在 `system_now` 模式下，BESTPOSA 的 GPS 历元用于重复消息判断。

### NavSatStatus

| NavSatFix 字段 | 来源和规则 |
| --- | --- |
| `status.status` | 由 BESTPOSA 的 `sol_status` 和 `pos_type` 映射 |
| `status.service` | 固定为 `sensor_msgs/NavSatStatus::SERVICE_GPS` |

具体状态映射：

```text
sol_status != SOL_COMPUTED  -> STATUS_NO_FIX
pos_type == SINGLE          -> STATUS_FIX
pos_type == WAAS           -> STATUS_SBAS_FIX
其他 SOL_COMPUTED 解        -> STATUS_GBAS_FIX
```

ROS1 的 `NavSatStatus` 没有专门的 RTK Fixed/Float 枚举，因此 RTK 解类型会归入 `STATUS_GBAS_FIX`。BESTPOSA 的原始 `pos_type` 仍可通过 `/rtk/raw_bestposa` 查看。

### 经纬高

| NavSatFix 字段 | BESTPOSA 来源 |
| --- | --- |
| `latitude` | `lat`，单位度 |
| `longitude` | `lon`，单位度 |
| `altitude` | `hgt`，单位米 |

当前使用的是 BESTPOSA 的椭球高字段。GGA 的海拔字段不会覆盖该值。

### 位置协方差

BESTPOSA 的 `Lat sigma`、`Lon sigma`、`Hgt sigma` 是位置标准差，单位米；NavSatFix 的 `position_covariance` 要求填写方差，单位 `m^2`。

当前映射如下：

| NavSatFix 字段 | 来源 | 含义 |
| --- | --- | --- |
| `position_covariance[0]` | `Lon sigma^2` | East/East 方差 |
| `position_covariance[1]` | 未设置，默认 `0.0` | East/North 协方差 |
| `position_covariance[2]` | 未设置，默认 `0.0` | East/Up 协方差 |
| `position_covariance[3]` | 未设置，默认 `0.0` | North/East 协方差 |
| `position_covariance[4]` | `Lat sigma^2` | North/North 方差 |
| `position_covariance[5]` | 未设置，默认 `0.0` | North/Up 协方差 |
| `position_covariance[6]` | 未设置，默认 `0.0` | Up/East 协方差 |
| `position_covariance[7]` | 未设置，默认 `0.0` | Up/North 协方差 |
| `position_covariance[8]` | `Hgt sigma^2` | Up/Up 方差 |

| NavSatFix 字段 | 来源 |
| --- | --- |
| `position_covariance_type` | 固定设置为 `COVARIANCE_TYPE_DIAGONAL_KNOWN` |

驱动设置：

```cpp
position_covariance_type =
    sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
```

因此当前消息表达的是已知的对角协方差，不包含经纬高之间的相关性。

## BESTPOSA 字段索引

以北云协议中 `;` 后的位置字段为准：

| 字段索引 | 协议字段 | 当前用途 |
| --- | --- | --- |
| `0` | `sol stat` | 解算状态 |
| `1` | `pos type` | 定位类型 |
| `2` | `lat` | 纬度 |
| `3` | `lon` | 经度 |
| `4` | `hgt` | 椭球高 |
| `5` | `Undulation` | 当前未用于 NavSatFix |
| `6` | `Datum ID` | 当前未用于 NavSatFix |
| `7` | `Lat sigma` | 纬度标准差 |
| `8` | `Lon sigma` | 经度标准差 |
| `9` | `Hgt sigma` | 高度标准差 |
| `10` | `Stn ID` | 当前未用于 NavSatFix |
| `11` | `Diff age` | 当前未用于 NavSatFix |
| `12` | `Sol age` | 当前未用于 NavSatFix |
| `13` | `#SVs` | 当前仅解析保存，不写入 NavSatFix |
| `14` | `#solnSVs` | 当前仅解析保存，不写入 NavSatFix |

协议头中的 GPS Week 和 Seconds 用于 BESTPOSA UTC 转换和 RMC 配对。

## 北云 C2 输出配置

驱动使用北云 C2 的 ASCII 输出。一个串口可以配置为：

```text
INTERFACEMODE COM3 BYNAV BYNAV
LOG COM3 GPRMC ONTIME 1
LOG COM3 GPGGA ONTIME 1
LOG COM3 BESTPOSA ONTIME 1
```

其中：

- `GPRMC` 是正式 NavSatFix 的时间来源；
- `BESTPOSA` 是正式 NavSatFix 的位置、状态和精度来源；
- `GPGGA` 可以用于诊断，但不会参与正式消息组装。

如果 Livox 驱动也需要读取 GPRMC 进行时间同步，建议使用北云板卡的另一个串口输出。不要让 `beiyun_gnss` 和 `livox_ros_driver` 同时打开同一个 `/dev/tty*` 设备。

## 验证

```bash
rostopic echo -n 1 /rtk/navsatfix
rostopic hz /rtk/navsatfix
rostopic echo -n 1 /rtk/raw_rmc
rostopic echo -n 1 /rtk/raw_bestposa
```

正常情况下：

1. 默认 `gprmc_utc` 模式下，`/rtk/navsatfix` 的时间戳来自 RMC UTC；`system_now` 模式下来自 `ros::Time::now()`；
2. 经纬度、高程来自 BESTPOSA；
3. 解状态来自 BESTPOSA；
4. 协方差来自 BESTPOSA 三个标准差的平方；
5. `gprmc_utc` 模式要求 BESTPOSA 与 RMC 时间匹配，`system_now` 模式不要求 RMC；
6. GGA 不会改变正式 NavSatFix 的位置和协方差。

## 协议文档

北云 BESTPOSA 字段定义和 CRC32 规则参见：

[UG016 数据通信接口协议](<../../../docs/UG016_数据通信接口协议_北云科技 .pdf>)

## 测试

解析器包含 RMC、GGA、BESTPOSA 校验、时间配对和协方差策略测试。启用 catkin 测试后可执行：

```bash
catkin_make run_tests_beiyun_gnss
```
