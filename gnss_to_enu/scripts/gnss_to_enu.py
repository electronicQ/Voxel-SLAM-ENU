#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import csv
import json
from pathlib import Path
import numpy as np
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class GNSSToENU:
    def __init__(self):
        rospy.init_node('gnss_to_enu_node', anonymous=True)


        # 初始化时打开文件
        # self.gt_file = open('/home/cclg/traj_rtk_origin.tum', 'w')

        # 🌟 【新增：读取原点配置参数】
        # 默认 true: 自动获取第一帧作为原点； false: 使用下面设定的固定原点
        self.use_auto_datum = rospy.get_param('~use_auto_datum', True) 

        # Optional complete quality-filtered RTK recording (before publication throttling).
        self.record_dir = rospy.get_param('~record_dir', '')
        self.enu_frame = rospy.get_param('~enu_frame', 'enu')
        self.altitude_reference = rospy.get_param('~altitude_reference', 'WGS84_ellipsoid')
        self.quality_file = None
        self.quality_writer = None
        self.record_file = None
        self.last_record_stamp = None
        self.csv_writer = None
        if self.record_dir:
            directory = Path(self.record_dir).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            if (directory / 'datum.json').exists():
                raise RuntimeError('Recording directory already contains datum.json; use a new run directory')
            self.record_file = (directory / 'rtk_enu.csv').open('x', newline='')
            self.csv_writer = csv.writer(self.record_file)
            self.csv_writer.writerow(['timestamp', 'east', 'north', 'up', 'var_e', 'var_n', 'var_u',
                                      'status', 'covariance_type'])
            self.record_file.flush()
            self.quality_file = (directory / 'rtk_quality.csv').open('x', newline='')
            self.quality_writer = csv.writer(self.quality_file)
            self.quality_writer.writerow(['timestamp','sol_status','pos_type'])
            self.quality_file.flush()
            rospy.on_shutdown(self.close_recording)

        # 核心状态标志位
        self.has_origin = False
        
        # 初始锚点 (Datum) 记录
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_alt = 0.0
        self.origin_ecef = np.zeros(3)

        #新增两个参数，一个是x和y方向的协方差截至值，一个是Z截至值。
        self.gnss_xy_variance_threshold = rospy.get_param('~gnss_xy_var_threshold', 2.0)
        self.gnss_z_variance_threshold = rospy.get_param('~gnss_z_var_threshold', 5.0)


        # 如果选择手动设置原点，在节点启动时直接初始化
        if not self.use_auto_datum:
            self.origin_lat = rospy.get_param('~datum_lat', 0.0)
            self.origin_lon = rospy.get_param('~datum_lon', 0.0)
            self.origin_alt = rospy.get_param('~datum_alt', 0.0)
            if (not np.isfinite([self.origin_lat, self.origin_lon, self.origin_alt]).all()
                    or not -90 <= self.origin_lat <= 90 or not -180 <= self.origin_lon <= 180):
                raise ValueError('Invalid manual ENU datum')
            self.origin_ecef = self.lla_to_ecef(self.origin_lat, self.origin_lon, self.origin_alt)
            self.has_origin = True # 标记已获取原点，阻断回调函数中的自动获取
            
            rospy.loginfo("\n" + "="*50)
            rospy.loginfo(" 【DATUM LOADED】 成功加载[自定义]建图原点！")
            rospy.loginfo(f"Latitude  : {self.origin_lat:.8f}")
            rospy.loginfo(f"Longitude : {self.origin_lon:.8f}")
            rospy.loginfo(f"Altitude  : {self.origin_alt:.8f}")
            rospy.loginfo("="*50 + "\n")

        if self.has_origin:
            self.save_datum()

        # 🌟 【核心护城河：双重降帧与空间滤波】
        self.last_pub_time = 0.0
        self.last_pub_enu = None
        # 1. 时间锁：最高允许 2Hz 输出 (防止 100Hz 塞爆图优化器)1
        self.min_time_interval = rospy.get_param('~min_T_intervel', 0.0) 
        # 2. 空间锁：两次发布之间，车子必须至少移动 0.2 米 (防原地静止时的厘米级漂移导致航向角计算出 NaN)
        self.min_dist_interval = rospy.get_param('~min_D_interval', 0.0)

        # 话题配置 (根据你的实际话题名进行修改)
        self.sub_topic = rospy.get_param('~sub_topic', '/chcnav_fix_demo/fix') # 注意确认这里的话题名
        self.pub_topic = rospy.get_param('~pub_topic', '/rtk/gps_odom')

        # 订阅与发布
        self.pub = rospy.Publisher(self.pub_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(self.sub_topic, NavSatFix, self.gnss_cb, queue_size=1000)
        if self.record_dir:
            topic = rospy.get_param('~quality_topic', '/rtk/solution_quality')
            self.quality_sub = rospy.Subscriber(topic, String, self.quality_cb, queue_size=1000)

        if self.use_auto_datum:
            rospy.loginfo(f"🚀 [GNSS2ENU] Node started. Waiting for valid Fix on {self.sub_topic} to AUTO-SET origin...")
        else:
            rospy.loginfo(f"🚀 [GNSS2ENU] Node started. Using MANUAL origin. Listening to {self.sub_topic}...")

    def quality_cb(self, msg):
        # Record a separate stamped sidecar so callback order cannot mislabel a fix.
        try:
            fields = next(csv.reader([msg.data]))
            stamp = float(fields[0])
            if len(fields) != 3 or not np.isfinite(stamp) or stamp <= 0:
                raise ValueError('invalid quality record')
            if self.quality_file is not None:
                self.quality_writer.writerow([format(stamp,'.9f'),fields[1],fields[2]])
                self.quality_file.flush()
        except (ValueError, IndexError):
            rospy.logwarn_throttle(5.0, 'Ignoring malformed GNSS solution quality message')

    def close_recording(self):
        if self.quality_file is not None:
            self.quality_file.close()
            self.quality_file = None
        if self.record_file is not None:
            self.record_file.close()
            self.record_file = None

    def save_datum(self):
        if self.record_dir:
            metadata = {'latitude': self.origin_lat, 'longitude': self.origin_lon,
                        'altitude': self.origin_alt, 'altitude_reference': self.altitude_reference,
                        'frame_id': self.enu_frame, 'use_auto_datum': self.use_auto_datum,
                        'coordinate_conversion': 'WGS84 LLA -> ECEF -> datum ENU',
                        'covariance': 'NavSatFix ENU diagonal, recorded without Z inflation'}
            path = Path(self.record_dir).expanduser() / 'datum.json'
            with path.open('x') as stream:
                json.dump(metadata, stream, indent=2, allow_nan=False)

    # WGS84 经纬高转 ECEF 地心坐标系
    def lla_to_ecef(self, lat, lon, alt):
        a = 6378137.0 # WGS84 椭球长半轴
        e2 = 0.00669437999014 # 偏心率平方
        
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        N = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
        
        X = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        Y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        Z = (N * (1 - e2) + alt) * math.sin(lat_rad)
        
        return np.array([X, Y, Z])

    # ECEF 转 局部 ENU (东北天)
    def ecef_to_enu(self, current_ecef, ref_lat, ref_lon, ref_ecef):
        diff = current_ecef - ref_ecef
        lat_rad = math.radians(ref_lat)
        lon_rad = math.radians(ref_lon)
        
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)
        
        R = np.array([
            [-sin_lon,           cos_lon,          0],
            [-sin_lat*cos_lon,  -sin_lat*sin_lon,  cos_lat],
            [ cos_lat*cos_lon,   cos_lat*sin_lon,  sin_lat]
        ])
        
        return R.dot(diff)

    
    def gnss_cb(self, msg):
        values = [msg.latitude, msg.longitude, msg.altitude, msg.header.stamp.to_sec(),
                  msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8]]
        if (not np.isfinite(values).all() or msg.status.status < 0 or values[3] <= 0
                or not -90 <= msg.latitude <= 90 or not -180 <= msg.longitude <= 180
                or msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN
                or min(values[4:]) <= 0):
            rospy.logwarn_throttle(5.0, 'Rejecting GNSS fix with invalid position/time or unknown/nonpositive variance')
            return
        # NavSatStatus is not an RTK fixed/float indicator; no status==2 assumption.
        # ==========================================
        # 1. 物理状态关卡：必须是地基增强 ，对于rsensor_msgs/NavSatFix，包括有消息头（包含时间戳和坐标系）、status定位状态、经纬高、位置协方差；
        # 消息类型对应的状态有-1，0，1，2(Fix 或 Float是NMEA消息的类型,对应ros中的2状态)，这个不是很需要，因为和精度熔断重叠了，除非协方差有问题，那只能通过状态熔断解决了
        # ==========================================
        # if msg.status.status != 2: 
        #   return

        # ==========================================
        # 2. 精度熔断关卡：提取并过滤动态方差
        # ==========================================
        rtk_var_x = msg.position_covariance[0]
        rtk_var_y = msg.position_covariance[4]
        rtk_var_z = msg.position_covariance[8]

        # # 驱动中 Fix 为 0.0004，Float 为 0.04
        # # 这里阈值设为 0.005，确保安全的 Float 不会被熔断，同时挡住真正的劣质数据0.005
        if rtk_var_x > self.gnss_xy_variance_threshold or rtk_var_y > self.gnss_xy_variance_threshold or rtk_var_z > self.gnss_z_variance_threshold:
            rospy.logwarn_throttle(
                2.0,
                f" [GNSS2ENU] Low accuracy "
                f"(Var X:{rtk_var_x:.4f}, Y:{rtk_var_y:.4f}, Z:{rtk_var_z:.4f}). Ignoring."
            )
            return
        
        #============================================

        # GNSS 与 LiDAR/IMU 已通过硬同步使用同一 UTC 时间基准。
        # 节流判断也必须使用输入消息时间，不能使用节点接收时刻。
        current_time = msg.header.stamp.to_sec()

        # ==========================================
        # 3. 锚点初始化 (Datum - 终生只执行一次)
        # ==========================================
        if not self.has_origin:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            self.origin_ecef = self.lla_to_ecef(self.origin_lat, self.origin_lon, self.origin_alt)
            self.has_origin = True
            self.save_datum()
            
            rospy.loginfo("\n" + "="*50)
            rospy.loginfo(" 【DATUM ACQUIRED】 成功获取[自动]建图原点！")
            rospy.loginfo(f"Latitude  : {self.origin_lat:.8f}")
            rospy.loginfo(f"Longitude : {self.origin_lon:.8f}")
            rospy.loginfo(f"Altitude  : {self.origin_alt:.8f}")
            rospy.loginfo("="*50 + "\n")

        # ==========================================
        # 4. 实时坐标转换 (LLA -> ECEF -> ENU)
        # ==========================================
        current_ecef = self.lla_to_ecef(msg.latitude, msg.longitude, msg.altitude)
        enu = self.ecef_to_enu(current_ecef, self.origin_lat, self.origin_lon, self.origin_ecef)

        # A bag time reset must start a new recording, rather than mix two runs.
        if self.last_record_stamp is not None and current_time <= self.last_record_stamp:
            rospy.logwarn_throttle(5.0, 'Ignoring duplicate/regressing GNSS timestamp; restart node for a new bag run')
            return
        self.last_record_stamp = current_time
        if self.csv_writer:
            self.csv_writer.writerow([format(current_time, '.9f'), *enu,
                                      rtk_var_x, rtk_var_y, rtk_var_z,
                                      msg.status.status, msg.position_covariance_type])
            self.record_file.flush()

        # ==========================================
        # 5. 时空双重锁：防塞爆与防原地瞎算航向
        # ==========================================
        if self.last_pub_enu is not None:
            # 时间锁 (例如 0.5 秒发一次)
            if current_time - self.last_pub_time < self.min_time_interval:
                return
            
            # 空间锁 (水平移动不够不发数据)
            dist = np.linalg.norm(enu[:2] - self.last_pub_enu[:2]) 
            if dist < self.min_dist_interval:
                return 

        # 刷新记录
        self.last_pub_time = current_time
        self.last_pub_enu = enu

        # ==========================================
        # 6. 组装并发布最终的 Odometry 里程计
        # ==========================================
        odom_msg = Odometry()

        # 透传北云 GNSS NavSatFix 的 UTC 时间戳，使 /rtk/gps_odom 与
        # 已硬同步到 GNSS UTC 的 LiDAR/IMU 位于同一时间轴。
        odom_msg.header.stamp = msg.header.stamp

        odom_msg.header.frame_id = self.enu_frame
        odom_msg.child_frame_id = "gps_link" 
        
        # 位置填入
        odom_msg.pose.pose.position.x = enu[0]
        odom_msg.pose.pose.position.y = enu[1]
        odom_msg.pose.pose.position.z = enu[2]

        # 姿态填入 (全为0，因为我们只提供位置)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0


        #====================================================
        # # 🌟 核心：填入动态协方差
        cov = [0.0] * 36
        
        # 将从驱动拿到的真实精度完美透传给 EKF/GTSAM
        cov[0]  = rtk_var_x  # 动态 X
        cov[7]  = rtk_var_y  # 动态 Y
        
        # Z 轴不信任处理：高度误差天然较大，人为放大 10 倍
        cov[14] = rtk_var_z * 10.0 
        
        # 姿态没有观测，必须给极大值方差
        cov[21] = 100.0 # Roll
        cov[28] = 100.0 # Pitch
        cov[35] = 100.0 # Yaw
        
        odom_msg.pose.covariance = tuple(cov)
        #====================================================

        # 仅在此处发布唯一的一次
        self.pub.publish(odom_msg)

        # 写入 TUM 格式轨迹文件时也使用 GNSS UTC 时间戳。
        # timestamp x y z qx qy qz qw，这个是把局部enu里程计的轨迹写道轨迹文件中
        # self.gt_file.write(f"{current_time:.6f} {enu[0]:.6f} {enu[1]:.6f} {enu[2]:.6f} 0.0 0.0 0.0 1.0\n")
        
if __name__ == '__main__':
    try:
        node = GNSSToENU()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
