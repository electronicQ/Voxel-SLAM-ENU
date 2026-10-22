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
4. 固定尺度为 1，用确定性 RANSAC + 加权 SVD 求 `p_enu = R * p_map + t`。
   权重为 ENU 三轴方差均值的倒数，是标量权重，并非完整各向异性协方差优化。
5. 检查拟合内点 RMSE 和指定验收范围的内点比例；默认范围为全部匹配点。验证通过才导出。
6. 各帧 IMU 系 PCD 先应用最终 IMU→地图位姿，再应用地图→ENU 变换，逐帧写盘，避免整张地图同时驻留内存。

常用参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--time-offset` | 0 s | `t_map = t_RTK + offset`，明确为加号，不自动估计 |
| `--max-gap` | 0.25 s | 插值两端最大间隔；精确匹配不需要插值 |
| `--sample-distance` | 1 m | 拟合点间三维位移；0 表示全部匹配点 |
| `--inlier-threshold` | 0.5 m | 三维残差内点门限 |
| `--min-pairs` | 10 | 最少拟合内点数量 |
| `--min-inlier-ratio` | 0.7 | 拟合点及 validation_scope 指定范围的最少内点比例 |
| `--max-rmse` | 0.3 m | 拟合内点三维 RMSE 上限 |
| `--ransac-trials` | 500 | RANSAC 尝试次数，固定随机种子 |

门限只是初始值，应按任务精度要求设置。报错时先检查时间同步、杆臂和轨迹内部漂移，
不要只放宽门限。直线或近共线轨迹会拒绝；有转弯的平面航迹可以配准，无需一定有明显高度变化。
所有参数见 `--help`。

### 中段可信、前后段有疑问时的拟合方式

新增参数默认关闭额外处理，旧的全程拟合和验收方式不变：

| ROS 参数 | 默认值 | 作用 |
|---|---|---|
| `fit_start` / `fit_end` | 0 / -1 s | 仅使用该时间窗拟合；-1 表示到末尾 |
| `max_xy_variance` | 0 m² | East/North 任一方差超限则不参与拟合；0 不额外限制 |
| `max_z_variance` | 0 m² | Up 方差超限则不参与拟合；0 不额外限制 |
| `auto_time_offset` | false | true 时用原始 SLAM 轨迹搜索固定时间偏移 |
| `offset_min` / `offset_max` | -2 / 2 s | 搜索的绝对偏移范围，符号仍为 `t_map=t_RTK+offset` |
| `offset_step` | 0.1 s | 最大网格步长，越小越细但越慢 |
| `validation_scope` | all | all 检查全程；fit_window 只验收窗内且协方差合格的点 |

**时间窗从 CSV 第一条数据有效、时间戳合法且方差为正的 RTK 记录开始计时，位于离线协方差上限过滤和时间匹配之前。**
因此调整方差门限、手动时间偏移不会悄悄改变窗口起点；绝对参考时间保存在 `fit_reference_rtk_timestamp`。
窗外和协方差超限的匹配点仍出现在图片、CSV 和 `all_*` 误差中，但不会影响 RT 或自动时间搜索。
这不等于恢复原始 GNSS 数据，记录阶段已过滤的观测无法从 CSV 补回。

针对本次约 15 分钟、疑似约 +1 s 时间差的 flight01，可先用 100～450 s 中段做诊断：

```bash
roslaunch gnss_to_enu align_voxel_enu.launch \
  fit_start:=100 fit_end:=450 \
  max_xy_variance:=0.01 max_z_variance:=0.04 \
  auto_time_offset:=true offset_min:=0.0 offset_max:=2.0 offset_step:=0.1 \
  validation_scope:=fit_window alignment_only:=true \
  output:="$HOME/voxel_enu_data/check_mid_sync01"
```

这些窗口和方差值是本次数据的排查起点，不适合未经检查直接用于所有飞行。
如果候选点不足，先查看协方差和时间覆盖，不要直接扩大误差门限。

自动时间搜索只在拟合窗内、方差合格、经空间抽样的点上进行（最多均匀取 300 对用于搜索）。
所有时间候选使用相同的 RTK 点集合，避免某个偏移只匹配到较容易的航段而占优。
每个偏移都从原始轨迹插值、补偿杆臂，用保留约 70% 较小残差的加权 SVD 代价比较。
选出时间偏移后，重新匹配全部有效 RTK，再执行原来的 RANSAC 和加权 SVD。
搜索仅返回网格估计值，不是传感器硬同步标定；地图漂移可能影响它。
静止、匀速直线或特定周期运动下，时间差可能与平移/旋转混淆。
若代价几乎不变化、没有足够非共线支持或最佳值在搜索边界，程序保留曲线和诊断后报错，需换窗口/范围或使用已知人工偏移。
`auto_time_offset=true` 时成功估计值替代 `time_offset`，两者不会相加；报告同时保存请求值与实际采用值。

**局部验收与全程准确性是两件事。** 默认 `validation_scope=all` 保留全程内点比例检查。
显式设置 `fit_window` 后，在拟合内点 RMSE 达标的基础上，仅检查窗内所有方差合格点的内点比例，
允许输出“由可信中段估计的固定变换”；窗口外的误差不会因此消失。
仍会报告全程误差，图标题注明 LOCAL 验收范围，若全程误差较大则给出 `global_warning`。
确认中段诊断后，才可用同样参数、`alignment_only:=false` 和一个新输出目录导出地图。
导出使用全部保存帧，但只应用同一个刚体变换，不修复后段地图漂移。

## 5. 输出与验收

- `alignment.json`：原点、T_enu_map、杆臂、时间偏移、门限、误差统计和导出状态。
- `T_enu_map.txt`：4×4 地图→ENU 刚体变换；可用于后续定位结果转换。
- `trajectory_enu.tum`：所有最终帧的 ENU 下 IMU 位姿（时间戳不变）。
- `residuals.csv`：全部时间匹配点的 XYZ/三维残差、是否拟合样本、是否在门限内；存在候选变换时生成。
- `trajectory_comparison.png`：轨迹对比图，在正常输入可读取的情况下，配准成功或失败都会生成。
- `diagnostic.json`：绘图所用候选变换、来源、验收状态及失败原因。
- `matched_pairs.csv`：地图系天线点、RTK ENU 点、原始三轴方差、是否在拟合时间窗以及 `quality_pass`，供进一步排查。
- `time_offset_search.csv/png`：启用自动时间搜索时，各偏移对应的代价及曲线。
- `map_enu.pcd`：ENU 地图，binary XYZ float32；原保存格式没有可靠强度，本工具不输出 intensity。

完整成功的 `alignment.json.status` 为 `complete`；只对齐为 `alignment_validated`；
PCD 导出失败为 `map_export_failed`，不会留下半张 map_enu.pcd。
拟合或质量验收失败为 `alignment_failed`，有效 `T_enu_map` 为 null，保留诊断文件但不导出地图。

### 轨迹诊断图

同一个 `roslaunch gnss_to_enu align_voxel_enu.launch` 命令自动生成 PNG，无需新增开关。
使用 Matplotlib 的 Agg 后端，不弹窗口，也不依赖桌面环境。新依赖：`sudo apt install python3-matplotlib`。
图中轨迹使用蓝色空心圆（SLAM 天线）和橙色叉号（RTK 天线）：

1. 左上：匹配点轨迹分别平移到自身起点为零，只对比形状；没有消除坐标轴方向差异。
2. 右上：变换后的 ENU 水平轨迹，用灰线连接同时间戳点对。
3. 左下：变换后的高度随时间变化。
4. 右下：三轴残差和三维距离随时间变化，红色虚线为内点门限。

所有匹配点都画出，包括外点；灰色对应连线最多显示 120 条，避免轨迹被遮住。
RANSAC 内点不足时优先画被拒绝的候选；没有候选但符合拟合时间窗和协方差门限的点可以做 SVD 时，
仅为诊断生成全点 SVD 变换；图片和 `diagnostic.json` 会明确标注未经验收。
完全没有变换时仍输出原始轨迹及相对高度图，对齐及残差子图注明 unavailable。
时间无重叠时会显示完整有效输入轨迹，不生成虚假的时间对应关系。
输入文件无法读取、格式损坏或输出目录不可写时，无法保证绘图；输出目录已存在仍拒绝覆盖。

质量失败继续以退出码 1 结束；图片的存在不代表配准通过，也不会跳过地图导出前的质量门限。
输出目录必须不存在，避免覆盖上一轮结果。支持 Voxel-SLAM 默认 binary PCD，也支持 ASCII；不支持 binary_compressed。

至少检查全程/分航段残差，以及起终点附近地图的一致性。低配准残差不能证明每个地图点都准确；
高残差也可能来自时间同步或杆臂错误。此方案既不缩放地图，也不通过 GNSS 拉伸历史轨迹。

## 开发验证

```bash
python3 -m unittest discover -s src/gnss_to_enu/tests -v
```

覆盖非零杆臂、时间插值、已知 SE(3) 恢复、异常点、直线退化、PCD 读取/重建和文件覆盖保护；
新增测试覆盖 RMSE 超限、内点不足、抽样不足、退化、无时间重叠及无有效 RTK 时的诊断图和无效变换保护；
覆盖时间窗、协方差过滤、非零杆臂下自动恢复时间差、局部通过/全程失败的区别，以及时间搜索边界和不可辨识运动。
ROS1 构建与真实板卡/rosbag 回放需在部署环境验证。
