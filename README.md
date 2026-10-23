# Voxel-SLAM-ENU

基于 Voxel-SLAM 的 ENU 坐标系地图导出工具。保留原有前后端估计与优化流程，新增 RTK 数据记录、天线杆臂补偿和轨迹 SVD 刚体配准，将最终点云地图转换至具有已知经纬高原点的 ENU 坐标系。

本项目采用**离线对齐方案**：GNSS 不参与 Voxel-SLAM 的在线滤波或后端优化。建图结束并完成轨迹保存后，利用最终保存的轨迹和 RTK 的时间对应关系求一个固定旋转、平移，再重建 ENU 地图。

```text
LiDAR + IMU → Voxel-SLAM → 结束保存的轨迹 + 逐帧 IMU 系点云
                                     ↓
RTK NavSatFix → ENU 位置记录 → 时间匹配、杆臂补偿、SVD 配准
                                     ↓
                          ENU 地图、轨迹、变换及误差报告
```

## 功能与范围

- 默认使用第一条通过质量检查的 GNSS 数据作为 ENU 原点，也支持手动指定原点。
- 记录 RTK 测量时间戳、ENU 位置、协方差和实际原点。
- 基于结束时保存的轨迹（可选最终全局 BA）进行位置线性插值、姿态 SLERP 和天线杆臂补偿。
- 自动筛选多个候选片段，结合 RTK 协方差/原始解状态、运动覆盖、邻窗变换一致性，再用 RANSAC 和加权 SVD 估计固定尺度变换。
- 自动比较候选片段的固定时间偏差，无需手工指定每次飞行的时间段。
- 用最终逐帧位姿重建 ENU 点云地图，输出全程配准残差。

刚体对齐仅改变地图整体位置和朝向，**不消除地图内部漂移或变形**。当前导出流程针对单次建图 session。
原 Voxel-SLAM 的保存部分另有少量调整：支持少于 100 帧的最终轨迹保存、初始化保存点的强度字段、增加轨迹写入结果提示。

## 仓库结构

仓库根目录对应 ROS 工作空间的 `src/`：

```text
src/
├── Voxel-SLAM/
│   ├── VoxelSLAM/              # 建图、回环和优化
│   └── VoxelSLAMPointCloud2/   # RViz 插件
├── gnss_to_enu/                # GNSS 转 ENU、记录和离线对齐
└── sensors/
    ├── beiyun_gnss/           # 北云 GNSS ROS1 驱动
    └── livox_ros_driver/      # Livox ROS1 驱动
```

## 环境与构建

目标部署环境为 Ubuntu 20.04、ROS1 Noetic、Python 3.8+。
Voxel-SLAM 还需要 Eigen、PCL、GTSAM、Livox SDK 等依赖，具体参考 [Voxel-SLAM 原说明](Voxel-SLAM/README.md)和 [Livox 驱动说明](sensors/livox_ros_driver/README_CN.md)。离线工具依赖 NumPy 和 Matplotlib（用于轨迹诊断图）。

把仓库克隆为一个新工作空间的 `src` 目录：

```bash
mkdir -p ~/voxel_enu_ws
cd ~/voxel_enu_ws
git clone git@github.com:electronicQ/Voxel-SLAM-ENU.git src
sudo apt install python3-numpy python3-matplotlib
# 完成原工程其他依赖安装后：
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

已经有工作空间时，不要克隆成 `src/仓库名/src/`；按上面的包结构放置，并避免重复的软件包。

## 建图与导出

### 1. 检查传感器和时间基准

根据实际雷达修改配置文件里的话题、外参、逐点时间等参数。
LiDAR、IMU、RTK 的测量时间戳必须处于同一时间基准；本项目不会自动完成硬同步。
RTK 输入默认为 `/rtk/navsatfix`（`sensor_msgs/NavSatFix`）。
新版本北云驱动另发 `/rtk/solution_quality`，记录节点保存为 `rtk_quality.csv`，按相同测量时间戳离线关联。
旧 bag/其他驱动没有该话题时仍可运行，但固定解状态标记为 unknown，不会根据 NavSatStatus 猜测。
本次修改了北云 C++ 驱动，部署更新后请执行 `catkin_make`。如需使用本仓库北云驱动，先按 [驱动说明](sensors/beiyun_gnss/README.md)设置串口及时间戳来源。

### 2. 建图并记录 RTK

以下命令使用 Livox Avia 示例配置；换雷达时通过 `config:=/绝对路径/配置.yaml` 指定正确配置。
每次运行使用新的 session 名称和 RTK 目录：

```bash
mkdir -p "$HOME/voxel_enu_data/maps"
roslaunch gnss_to_enu avia_mapping_record_rtk.launch \
  save_path:="$HOME/voxel_enu_data/maps/" \
  bagname:=flight01 \
  record_dir:="$HOME/voxel_enu_data/rtk_flight01" \
  fix_topic:=/rtk/navsatfix \
  use_sim_time:=false
```

此 launch 启动 Voxel-SLAM、RTK 转换记录节点及 RViz，启用地图保存；不会启动传感器驱动。
`save_path` 需为已存在的父目录并以 `/` 结尾。由于原工程的目录创建实现，请使用不含空格或 shell 特殊字符的路径。

默认 `use_auto_datum=true`，第一条有效 GNSS 的 ENU 位置为 `(0,0,0)`，其经纬高自动保存到 `datum.json`。
若需要固定原点，额外传入：

```text
use_auto_datum:=false datum_lat:=LAT_DEG datum_lon:=LON_DEG datum_alt:=HEIGHT_M
```

手动模式必须填写实际经纬高；自动模式忽略这三个参数。转换采用 WGS84 椭球高，输入高程基准需一致。
回放时设置 `use_sim_time:=true`，并在另一终端运行：

```bash
rosbag play --clock /ABS/PATH/TO/flight01.bag
```

### 3. 结束建图并保存

数据播放及积压处理完成后，在另一终端选择以下一种方式。

**只保存当前结果，跳过结束阶段的全局 BA：**

```bash
rosparam set /save_only true
```

**保留原有行为，先执行最终全局 BA，再保存：**

```bash
rosparam set /finish true
```

`/save_only` 不调用最终 `build_graph/topDownProcess`，保存已完成在线局部优化、回环处理的当前轨迹；
在线局部 BA、回环及后台子图处理机制仍保留，已经开始的后台任务会结束后再释放数据。
两参数同时被检测到时 `/finish` 优先；结束模式一旦选定不可中途切换。
只保存模式要求启动时已启用 `General/is_save_map=1`（本 launch 已设置），否则无法补回此前没有保存的逐帧 PCD。

等待后端队列处理及轨迹保存完成，看到 `Final trajectory written: ...` 后再停止 launch。
不要用直接 Ctrl+C 代替上述请求。保存的扫描范围仍遵循原滑窗逻辑，尚未移出滑窗的尾部扫描不会额外补写。
这里的地图保存是逐帧 PCD 加对应轨迹；合并后的 ENU 地图仍由下一步离线工具导出。

最终轨迹位于 `maps/flight01/alidarState.txt`；同目录 `0.pcd`、`1.pcd` 等保存对应的 IMU 系点云。
RTK 目录保存 `rtk_enu.csv` 与 `datum.json`。

### 4. 对齐并导出

推荐使用 [align_voxel_enu.launch](gnss_to_enu/launch/align_voxel_enu.launch)，将路径、杆臂和门限保存在 launch 顶部的 `arg default` 中。
默认路径使用 `$HOME/voxel_enu_data`，默认 IMU→天线杆臂为 `[-0.30835, 0.07326, 0.07160]` m，
由当前 Avia 外参与天线在雷达系下的坐标 `[-0.35, 0.05, 0.1]` m 计算得到。更换安装或外参后需重新计算。
替换下面 `LX LY LZ`：它们是从 IMU 原点指向 GNSS 天线参考点、表达在 IMU 坐标系中的三维杆臂，单位米。
不需要 GNSS 旋转外参；不要将 LiDAR→IMU 平移直接当作 GNSS 杆臂。

```bash
roslaunch gnss_to_enu align_voxel_enu.launch \
  lever_x:=LX lever_y:=LY lever_z:=LZ
```

修改好 launch 默认值后，也可直接运行 `roslaunch gnss_to_enu align_voxel_enu.launch`。
这是一次性离线任务，结束后进程退出，不持续订阅数据；不会自动触发或等待 SLAM 结束保存。
输出目录必须不存在。可传 `alignment_only:=true output:=/新的检查目录` 先检查配准；完整导出时另选新目录。
原有 `rosrun gnss_to_enu align_voxel_enu.py --session ... --rtk ... --output ... --lever-arm LX LY LZ` 用法仍保留。
默认自动选段，并在候选片段内按 1 m 距离抽样；全部有效时间匹配点仍参与误差统计和绘图。
**有可计算的刚体变换时，质量门限只报警，不阻止 ENU 地图导出**。没有可信片段时会尝试备用解，并明确标为不可信。
若点数/几何退化导致完全无法计算变换，或输入文件缺失、输出不可写，则报错，不生成假地图。
工具不进行尺度缩放、不使用 ICP 最近邻配对，也不会重复施加 LiDAR 外参。
导出默认同时保存完整地图和体素降采样地图。可设置：

```bash
roslaunch gnss_to_enu align_voxel_enu.launch map_voxel_size:=0.15
```

`map_voxel_size` 单位米，越大地图越稀疏、细节越少，越小保留细节越多；0 表示仅保存完整地图。
该参数只作用于合并转换后的 ENU 点云，不影响轨迹配准或原始逐帧 PCD。
`alignment_only:=true` 时两张地图均不导出。
参数门限、时间偏移符号及完整用法见 [ENU 工具说明](gnss_to_enu/README.md)。

## 输出文件

| 文件 | 内容 |
|---|---|
| `map_enu.pcd` | 完整 ENU 点云地图，binary XYZ |
| `map_enu_downsampled.pcd` | 全局体素质心降采样后的 ENU 地图，默认体素 0.15 m |
| `trajectory_enu.tum` | ENU 下的最终 IMU 位姿轨迹 |
| `T_enu_map.txt` | 4×4 变换，`p_enu = R_enu_map * p_map + t_enu_map` |
| `alignment.json` | 原点、变换、自动选中窗口、实际时间偏差、质量是否达标、告警及导出状态 |
| `residuals.csv` | 全部匹配点的三轴及三维残差（存在候选变换时） |
| `trajectory_comparison.png` | 无论配准通过或失败均输出的轨迹对比图 |
| `diagnostic.json` | 图片所用变换的来源、是否通过验收及失败原因 |
| `matched_pairs.csv` | 全部匹配点、三轴方差、自动选段标记及 fixed/float/unknown 标记 |
| `auto_windows.json` | 各候选窗口的运动范围、误差、估计时间偏差及一致性支持数量 |
| `automatic_selection.png` | 候选窗口质量/时间偏差图，星号标出采用窗口 |

完整导出无警告时 `alignment.json.status=complete`；有警告但导出成功为 `complete_with_warnings`。
`quality_passed` 独立表示质量检查是否通过，不能仅凭地图文件存在来判断精度。

运行对齐后，直接查看输出目录中的 `trajectory_comparison.png`：

- 蓝色空心圆：SLAM 天线位置；橙色叉号：RTK 天线位置。
- 左上：两条轨迹分别平移到各自起点为零后的水平形状，未校正坐标系方向差异。
- 右上：候选或已通过验收的变换下，ENU 水平轨迹叠加；灰线连接同时间点对。
- 左下：两条轨迹的高度随时间变化。
- 右下：所有匹配点的三轴偏差、三维距离及内点门限。

保留全部匹配点（含外点），只对灰色连线均匀稀疏显示，最多 120 条，避免遮挡。
求解失败时仍保存图片；完全没有可用变换时显示原始轨迹形状/相对高度，无法计算的对齐图和残差会注明原因。
若时间没有重叠，原始形状图改用完整可用输入轨迹，并不伪造对应点。
质量超限时图中注明 QUALITY WARNINGS，仍输出变换、ENU 轨迹和地图。只有完全无法计算变换时才使用
`alignment_failed` 并返回非零退出码；此时保留可生成的轨迹诊断，不写单位矩阵冒充结果。
输出目录必须是新的；文件缺失或损坏导致输入轨迹无法读取时，无法保证生成轨迹图。
部署前检查全程及各航段残差、起终点地图一致性；配准结果不能替代对原地图质量的验收。

## 验证状态

```bash
# 在仓库根目录运行
python3 -m unittest discover -s gnss_to_enu/tests -v
```

已通过 12 项自动测试，覆盖时间插值、杆臂补偿、已知刚体变换恢复、异常点、退化轨迹、PCD 重建，自动/手动原点记录及 ROS 参数入口映射。
已检查 Python 语法和 launch XML，并已在 ROS1 环境测试通过。新增自动选段、时间搜索、解状态记录、报警导出及全局体素降采样已通过合成测试；本次驱动 C++ 改动及新流程尚待 ROS1 编译、回放验证。

## 上游与许可证

本项目基于 [hku-mars/Voxel-SLAM](https://github.com/hku-mars/Voxel-SLAM)，并包含 Livox 驱动等第三方组件。原作者、论文及使用说明见各组件 README。

各组件保留原有许可证，参见 [Voxel-SLAM/LICENSE](Voxel-SLAM/LICENSE)及 [Livox LICENSE](sensors/livox_ros_driver/LICENSE.txt)。
本仓库中部分软件包的 `package.xml` 许可证字段仍为 `TODO`；根 README 不代表重新授予第三方代码许可，发布前需核实并完善新增代码的许可证信息。
