# Voxel-SLAM 离线 ENU 地图导出（ROS1）

方案二：Voxel-SLAM 独立建图，结束时可选择只保存或先做最终全局 BA 再保存；GNSS 不进入其前端或后端。
用最终轨迹和同时间 RTK 天线位置求固定 SE(3)，随后按最终位姿重建 ENU 地图。
刚体对齐不修复地图内部漂移。默认只处理单次、新建 session，不加载 previous_map。

## 1. 安装与准备

目标为 ROS1 Noetic / Python 3.8+。离线工具依赖 Python3、NumPy 和 Matplotlib，无需 ROS 或图形桌面。
在部署机器工作空间安装依赖并编译（原 Voxel-SLAM 的依赖仍按原工程安装）：

```bash
sudo apt install python3-numpy python3-matplotlib
catkin_make
source devel/setup.bash
```

确认 LiDAR、IMU、RTK `header.stamp` 为同一时间基准。节点只透传时间戳，不自动同步。
确认雷达配置里的外参/话题/逐点时间等参数与实际传感器一致。
先创建一个已存在的地图父目录；`save_path` 必须以 `/` 结尾，`bagname` 每次使用新名字。
使用不含空格或 shell 特殊字符的路径（原 Voxel-SLAM 用 shell mkdir 创建目录）。

默认 `use_auto_datum=true`：第一条通过质量检查的 GNSS 测量经纬高作为 ENU 原点，
该条 ENU 天线位置为 (0,0,0)，之后原点保持固定。实际经纬高自动写入 `datum.json`。
也可设置 `use_auto_datum=false` 并指定固定原点；原点高度和 NavSatFix 高度必须使用相同基准。
转换公式采用 WGS84 椭球高；若板卡输出海拔高，应先转换。不会自动做大地水准面改正。

## 2. 建图并记录 RTK

下面是自动原点模式命令格式，替换所有大写路径占位值：

```bash
roslaunch gnss_to_enu avia_mapping_record_rtk.launch \
  config:=/ABS/PATH/TO/lidar_config.yaml \
  save_path:=/ABS/PATH/TO/maps/ bagname:=flight01 \
  record_dir:=/ABS/PATH/TO/rtk_flight01 \
  fix_topic:=/rtk/navsatfix use_sim_time:=false
```

如需固定原点，在上述命令末尾增加：

```text
use_auto_datum:=false datum_lat:=LAT_DEG datum_lon:=LON_DEG datum_alt:=ELLIPSOID_HEIGHT_M
```

`datum_lat/lon/alt` 默认 0.0 仅用于让自动模式免填参数；自动模式会忽略它们。
手动模式请显式填写实际经纬高，否则会使用 (0°,0°,0 m)。

回放 rosbag 时改为 `use_sim_time:=true`，然后 `rosbag play --clock ...`。
不要同时启动原 Voxel-SLAM launch；上述 launch 已启动建图节点、记录节点及可选 RViz。
如果已有自定义建图 launch，可仅启动 `record_voxel_enu.launch`（同样传 datum、record_dir 等），
并自行确保 `General/is_save_map=1`、保存路径正确。

记录节点输出：

- `/rtk/gps_odom`：ENU 天线位置，frame_id 默认 `enu`；姿态为单位四元数占位，不是姿态观测。
- `rtk_enu.csv`：原始测量时间戳、ENU 位置、原始 ENU 对角方差、NavSatStatus、协方差类型。
- `datum.json`：实际原点、坐标转换说明及高度基准。

CSV 记录发生在发布节流之前，保留全部通过质量门限的观测，逐行 flush。
默认记录 launch 方差上限为 XY=0.01 m²，Z=0.04 m²，可通过 `xy_variance_max`、`z_variance_max` 调整。
拒绝无定位、NaN/Inf、未知或非正对角协方差、重复/倒退时间戳。
标准 NavSatStatus 无法区分 RTK fixed/float；方差门限不等同于固定解鉴别。
若板卡提供独立固定解标志，请在上游驱动中筛选或用其信息确认数据。
节点拒绝覆盖已有记录；重新回放、重启或换原点必须使用新的 record_dir。
两个新增 launch 默认均开启自动原点，并将 `use_auto_datum` 传递给转换节点。
CSV 保存原始 Z 方差；旧 Odometry 发布的 Z×10 逻辑保留，不影响本离线工具。

## 3. 结束并保存（两种模式）

等待 bag 播放完成、积压数据处理完成后，选择一种请求：

```bash
# 新增：只保存当前轨迹及已有逐帧地图，不执行最终全局 BA
rosparam set /save_only true

# 原有：执行最终全局 BA 后保存
rosparam set /finish true
```

这两个命令是替代选项，不要先后一起执行。同次检测到时 `/finish` 优先，结束模式选定后不再切换。
`/save_only` 会结束前端并等待后端队列处理完成，停止后台子图线程后保存当前位姿，
不进入最终 `build_graph/topDownProcess`。它不禁用此前在线局部 BA、回环优化或已经启动的后台子图任务。
`General/is_save_map=1` 必须在建图启动时启用，否则只保存请求被拒绝并提示重启配置；本方案 launch 已启用。

不要直接 Ctrl+C 结束建图。等待打印 `Final trajectory written: ...`，检查轨迹文件后再停止 launch。
地图仍以原有逐帧 PCD 和对应轨迹保存；后续离线导出才合并成 ENU 地图。
本工具不会在轨迹仍写入时自动启动。未移出滑窗的尾部扫描仍遵循原保存范围，不额外补写。

session 目录格式为：

```text
maps/flight01/
  alidarState.txt   # t px py pz qx qy qz qw，后面还有 v/bg/ba/g 等字段
  0.pcd
  1.pcd
  ...
```

当前 Voxel-SLAM 每次滑窗输出一个 ScanPose，轨迹第 i 行对应 i.pcd（从 0 开始）。
PCD 中是去畸变后的 IMU 系点，最终位姿为 IMU→地图；导出时不再乘 LiDAR 外参。
轨迹时间为对应扫描参考时刻（帧尾）。滑窗末尾尚未移出的扫描可能不在保存轨迹中，
本工具只重建有最终轨迹的完整帧，忽略超过轨迹行数的 PCD；缺少任一所需帧则报错，不静默跳过。
本次修改取消了原工程少于 100 帧不保存轨迹的限制，但极短或无转弯轨迹仍可能无法配准。

## 4. 离线对齐和导出

### ROS1 launch 入口（推荐）

编辑 `launch/align_voxel_enu.launch` 顶部的 `arg default`，设置 session、RTK CSV、输出路径、
实际杆臂以及匹配门限，然后运行：

```bash
roslaunch gnss_to_enu align_voxel_enu.launch
```

默认路径为 `$HOME/voxel_enu_data` 下的 flight01；默认 IMU→天线杆臂为 `[-0.30835, 0.07326, 0.07160]` m。
计算依据为天线在雷达系下坐标 `[-0.35, 0.05, 0.1]` m、Avia 的单位旋转及
LiDAR→IMU 平移 `[0.04165, 0.02326, -0.0284]` m。更换安装或外参后需重新计算。
也可在启动时覆盖任一参数，例如：

```bash
roslaunch gnss_to_enu align_voxel_enu.launch \
  session:=/ABS/PATH/TO/maps/flight01 \
  rtk:=/ABS/PATH/TO/rtk_flight01/rtk_enu.csv \
  output:=/ABS/PATH/TO/enu_flight01 \
  lever_x:=LX lever_y:=LY lever_z:=LZ \
  sample_distance:=1.0 alignment_only:=false
```

`alignment_only:=true` 为仅检查配准，`datum` 留空使用 RTK CSV 旁的 `datum.json`。
其他参数与下面 CLI 表格一致，但使用下划线名称，例如 `time_offset`、`max_rmse`。
节点从 ROS 私有参数读取配置，执行一次后退出。它不会等待或触发 Voxel-SLAM 的结束保存，
需确认通过 `/finish` 或 `/save_only` 完成轨迹保存后再启动。输出目录必须是新目录。路径通过 ROS 参数传入，不受命令行空格拆分影响。

### 命令行入口（仍支持，无需 ROS 也可直接用 Python）

先只做对齐并查看误差（不需要读取 PCD）：

```bash
rosrun gnss_to_enu align_voxel_enu.py \
  --session /ABS/PATH/TO/maps/flight01 \
  --rtk /ABS/PATH/TO/rtk_flight01/rtk_enu.csv \
  --output /ABS/PATH/TO/check_flight01 \
  --lever-arm LX LY LZ --alignment-only
```

确认结果后用另一个新目录导出地图，去掉 `--alignment-only`：

```bash
rosrun gnss_to_enu align_voxel_enu.py \
  --session /ABS/PATH/TO/maps/flight01 \
  --rtk /ABS/PATH/TO/rtk_flight01/rtk_enu.csv \
  --output /ABS/PATH/TO/enu_flight01 \
  --lever-arm LX LY LZ
```

`LX LY LZ` 单位米：从 IMU 原点指向天线参考点、表达在 IMU 坐标系中的平移向量，不需要 GNSS 旋转外参。
真实杆臂为零时需显式传 `--lever-arm 0 0 0`。
也可以直接运行 `python3 /ABS/PATH/TO/align_voxel_enu.py ...`。

算法：

1. 用结束时保存的轨迹（可选最终全局 BA）做位置线性插值和姿态 SLERP；不外推，也不使用另行记录的在线旧位姿。
2. `a_map = p_map_imu + R_map_imu * lever_arm`，与同时间 ENU 天线点配对。
3. 默认相邻拟合样本相距至少 1 m（3D 距离）；完整匹配数据保留用于验收。
4. 自动选择多段候选窗口并比较时间偏差、运动覆盖和相邻独立窗口的变换一致性，再用 RANSAC + 加权 SVD 求固定尺度的 `p_enu = R * p_map + t`。
   权重为 ENU 三轴方差均值的倒数，是标量权重，并非完整各向异性协方差优化。
5. 检查拟合内点 RMSE、数量、比例及全程误差。质量超限发出警告，但只要存在有效变换仍导出。
6. 各帧 IMU 系 PCD 先应用最终 IMU→地图位姿，再应用地图→ENU 变换，逐帧写盘，避免整张地图同时驻留内存。

下表为 Python CLI 默认参数；ROS launch 默认自动时间搜索开启，部分质量门限以 launch 中的值为准：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--time-offset` | 0 s | 人工偏移，`t_map = t_RTK + offset`；开启自动时间搜索时可被估计值替代 |
| `--max-gap` | 0.25 s | 插值两端最大间隔；精确匹配不需要插值 |
| `--sample-distance` | 1 m | 拟合点间三维位移；0 表示全部匹配点 |
| `--inlier-threshold` | 0.5 m | 三维残差内点门限 |
| `--min-pairs` | 10 | 最少拟合内点数量 |
| `--min-inlier-ratio` | 0.7 | 拟合点及全部匹配点最少内点比例 |
| `--max-rmse` | 0.3 m | 拟合内点三维 RMSE 上限 |
| `--ransac-trials` | 500 | RANSAC 尝试次数，固定随机种子 |

门限只是初始值，应按任务精度要求设置。报错时先检查时间同步、杆臂和轨迹内部漂移，
不要只放宽门限。直线或近共线轨迹会拒绝；有转弯的平面航迹可以配准，无需一定有明显高度变化。
所有参数见 `--help`。

### 自动选段，无需手工指定飞行时间窗

默认运行原命令即可：

```bash
roslaunch gnss_to_enu align_voxel_enu.launch
```

流程自动比较不同起点、半长/基准/两倍长度的重叠窗口，不需要手工设置 fit_start/fit_end：

1. 优先使用协方差合格点；有原始解状态时优先选择固定解。数据不足会报警，允许备用数据求解。
2. 检查片段的运动范围、非共线程度，排除只有静止/近直线支撑的可信候选。
3. 各窗口使用相同时间支持的点搜索偏差，进行截尾加权 SVD，再统计内点比例和误差。
4. 比较相邻不同窗口的旋转、平移效果和时间偏差；近乎重复/完全包含的窗口不重复计独立支持。
5. 优先选择多窗口一致的结果，兼顾空间覆盖、GNSS 方差、固定解比例和残差；不是只挑最低 RMSE。
6. 合并一致窗口，再执行 RANSAC + 加权 SVD；全程残差仍统计、绘图，不删除窗外点。
7. 没有可靠共识时，尝试最佳候选或全程备用 SVD，标为不可信并报警；只要实际求得有效 RT，仍按用户要求导出。

质量检查未通过不会伪装成达标。完全无法求出变换时，程序不会用单位矩阵或随意的旋转代替。
固定刚体变换不会对地图做缩放或局部变形，也无法让任意 GNSS 偏差段都吻合。

自动选择相关参数均在 launch 中可设置，默认无需按每次飞行修改：

| ROS 参数 | 默认值 | 作用 |
|---|---:|---|
| `auto_time_offset` | true | 每个候选窗口自动搜索固定偏差；false 使用 `time_offset` |
| `offset_min/max/step` | -2 / 2 / 0.1 s | 时间搜索范围/最大步长，不与 time_offset 相加 |
| `window_seconds` | 120 s | 基准窗口长度，自动也比较 60、240 s；短数据自动缩小 |
| `max_xy_variance` | 0.04 m² | 优先点 East/North 方差上限，0 不限制 |
| `max_z_variance` | 0.25 m² | 优先点 Up 方差上限，0 不限制 |
| `min_motion_span` | 10 m | 可信窗口的轨迹范围下限 |
| `min_motion_ratio` | 0.02 | 轨迹第二/第一奇异值之比下限，约束非共线程度 |
| `consistency_distance` | 2 m | 相邻候选变换在轨迹参考点的允许差异 |
| `consistency_angle` | 2 deg | 候选旋转差上限 |
| `consistency_time` | 0.3 s | 候选时间偏差差异上限 |
| `min_consistent_windows` | 2 | 至少多少个不同窗口支持同一解（含自身） |

静止、匀速直线和特定周期运动可能无法辨识时间差；此时保留人工 `time_offset` 并报警。
最佳偏差位于搜索边界也会报警，不把它当作可靠时间标定。候选窗口最多 120 个、每窗搜索最多 160 个空间样本，
最终合并拟合使用完整符合条件的空间抽样点。自动时间搜索和自动选段是启发式估计，不保证识别所有系统性偏差。

### 北云固定解原始状态的记录

更新后的北云驱动在 `/rtk/solution_quality` 发布 `std_msgs/String`，内容为：

```text
timestamp,sol_status,pos_type
```

每条消息使用对应 NavSatFix 的测量时间戳，例如 `1792129778.000000000,SOL_COMPUTED,NARROW_INT`。
记录节点将其写入 `rtk_quality.csv`，离线按时间戳精确关联；不会因两个话题回调先后不同而把状态配错。
目前识别 `SOL_COMPUTED` 下的 NARROW_INT、L1_INT、WIDE_INT、RTK_FIXED、FIXED 为固定解。
标准 NavSatStatus 不用于判断 fixed/float。旧 bag 或其他驱动没有该话题时状态为 unknown，
程序仍依据方差、运动和跨窗口一致性选段，并在终端及报告说明状态缺失。
新采集可录制该话题，或建图时让记录节点直接监听；旧 bag 不会凭空获得过去未记录的解状态。
**本次涉及北云 C++ 驱动更新，ROS1 部署后需重新 `catkin_make`。**

### 完整地图与降采样地图双输出

默认一次运行保存两份地图，不覆盖原始逐帧 PCD：

```bash
roslaunch gnss_to_enu align_voxel_enu.launch map_voxel_size:=0.15
```

`map_voxel_size` 是 ENU 体素边长，单位 m；默认 0.15，越大通常点越少、文件越小、细节越少，
越小保留更多点和细节。设为 0 时关闭额外降采样文件，仍保存完整 `map_enu.pcd`。
Python CLI 对应 `--map-voxel-size 0.15`。这与拟合轨迹用的 `sample_distance` 无关。
`alignment_only=true` 时不读取点云，两份地图均不生成。

每帧点先用最终保存的 IMU 位姿和地图→ENU 变换得到 ENU 坐标，再累计到统一全局网格。
所有帧落入同一体素的点只产生一个质心，按点数加权，不是先逐帧降采样后直接拼接，
也不是对每帧质心简单平均。负坐标同样按 floor 划分网格。
降采样使用写入完整 float32 地图的坐标；完整地图内容和点数不因降采样改变。

实现只依赖 Python 标准库 SQLite 和已有 NumPy，按帧累计到磁盘临时数据库，避免把完整地图一次性加载进内存；
大地图需预留临时磁盘空间，导出耗时会增加。完成或失败后清理临时数据。
`alignment.json` 新增 `map_voxel_size_m`、`map_points`、`downsampled_map_points` 和 `map_files`，分别记录体素大小、
完整/降采样点数和文件名；关闭降采样时降采样点数及路径为 null。

## 5. 输出与验收

- `alignment.json`：原点、T_enu_map、杆臂、时间偏移、门限、误差统计和导出状态。
- `T_enu_map.txt`：4×4 地图→ENU 刚体变换；可用于后续定位结果转换。
- `trajectory_enu.tum`：所有最终帧的 ENU 下 IMU 位姿（时间戳不变）。
- `residuals.csv`：全部时间匹配点的 XYZ/三维残差、是否拟合样本、是否在门限内；存在候选变换时生成。
- `trajectory_comparison.png`：轨迹对比图，在正常输入可读取的情况下，配准成功或失败都会生成。
- `diagnostic.json`：绘图所用候选变换、来源、验收状态及失败原因。
- `matched_pairs.csv`：全部匹配天线点、三轴方差、采用窗口/质量标记及 `rtk_fixed`（1=固定、0=已知非固定、-1=未知）。
- `auto_windows.json`：所有候选窗口的运动范围、误差、时间偏差及一致性支持数量。
- `automatic_selection.png`：候选窗口的误差/时间偏差图，星号表示采用窗口。
- `map_enu.pcd`：完整 ENU 地图，binary XYZ float32；原保存格式没有可靠强度，本工具不输出 intensity。
- `map_enu_downsampled.pcd`：完整地图在 ENU 全局体素网格中的质心降采样结果，默认体素 0.15 m。

完整成功的 `alignment.json.status` 为 `complete`，有警告但成功导出为 `complete_with_warnings`；
只对齐为 `alignment_validated` 或 `alignment_with_warnings`；
PCD 导出失败为 `map_export_failed`，不会留下不完整的 map_enu.pcd 或 map_enu_downsampled.pcd。
仅完全无法计算非退化变换时为 `alignment_failed`，`T_enu_map` 为 null，保留能生成的诊断。
`quality_passed` 与导出状态独立，所有未达标项目写入 `warnings` 并在终端输出。

### 轨迹诊断图

同一个 `roslaunch gnss_to_enu align_voxel_enu.launch` 命令自动生成 PNG，无需新增开关。
使用 Matplotlib 的 Agg 后端，不弹窗口，也不依赖桌面环境。新依赖：`sudo apt install python3-matplotlib`。
图中轨迹使用蓝色空心圆（SLAM 天线）和橙色叉号（RTK 天线）：

1. 左上：匹配点轨迹分别平移到自身起点为零，只对比形状；没有消除坐标轴方向差异。
2. 右上：变换后的 ENU 水平轨迹，用灰线连接同时间戳点对。
3. 左下：变换后的高度随时间变化。
4. 右下：三轴残差和三维距离随时间变化，红色虚线为内点门限。

所有匹配点都画出，包括外点；灰色对应连线最多显示 120 条，避免轨迹被遮住。
质量不足时展示实际采用的备用变换并标为 QUALITY WARNINGS，地图仍使用这个变换导出。
候选窗口筛选结果在 `automatic_selection.png` 中展示，主诊断图保留全部有效匹配点。
完全没有变换时仍输出原始轨迹及相对高度图，对齐及残差子图注明 unavailable。
时间无重叠时会显示完整有效输入轨迹，不生成虚假的时间对应关系。
输入文件无法读取、格式损坏或输出目录不可写时，无法保证绘图；输出目录已存在仍拒绝覆盖。

质量不足不再导致退出码 1，也不阻止导出；无可计算变换、文件损坏或地图导出失败仍以非零退出码结束。
图片或地图存在不代表质量通过，必须查看 `quality_passed`、`warnings` 和全程误差。
输出目录必须不存在，避免覆盖上一轮结果。支持 Voxel-SLAM 默认 binary PCD，也支持 ASCII；不支持 binary_compressed。

至少检查全程/分航段残差，以及起终点附近地图的一致性。低配准残差不能证明每个地图点都准确；
高残差也可能来自时间同步或杆臂错误。此方案既不缩放地图，也不通过 GNSS 拉伸历史轨迹。

## 开发验证

```bash
python3 -m unittest discover -s src/gnss_to_enu/tests -v
```

覆盖非零杆臂、时间插值、已知 SE(3) 恢复、异常点、直线退化、PCD 读取/重建和文件覆盖保护；
新增测试覆盖 RMSE 超限、内点不足、抽样不足、退化、无时间重叠及无有效 RTK 时的诊断图和无效变换保护。
ROS1 构建与真实板卡/rosbag 回放需在部署环境验证。
