//使用标准的 POSIX TTY API，
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <limits>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/String.h>

#include "beiyun_gnss/nmea_parser.h"

namespace {

// ROS 参数使用整数波特率，termios 使用系统常量，二者在这里做映射。
speed_t BaudToTermios(int baudrate) {
  switch (baudrate) {
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    default: return 0;
  }
}

// 将 RMC 的完整 UTC 日期时间转换为 ROS 时间戳。
// timegm 明确按 UTC 解释 tm，避免受电脑本地时区影响。
ros::Time ToRosTime(const beiyun_gnss::UtcTime &utc) {
  std::tm calendar = {};
  calendar.tm_year = utc.year - 1900;
  calendar.tm_mon = utc.month - 1;
  calendar.tm_mday = utc.day;
  calendar.tm_hour = utc.hour;
  calendar.tm_min = utc.minute;
  calendar.tm_sec = static_cast<int>(std::floor(utc.second));
  const std::time_t epoch_seconds = timegm(&calendar);
  if (epoch_seconds < 0) return ros::Time(0);
  const double fractional = utc.second - std::floor(utc.second);
  uint32_t nanoseconds = static_cast<uint32_t>(std::llround(fractional * 1e9));
  std::time_t adjusted_epoch = epoch_seconds;
  if (nanoseconds >= 1000000000U) {
    ++adjusted_epoch;
    nanoseconds -= 1000000000U;
  }
  return ros::Time(static_cast<uint32_t>(adjusted_epoch), nanoseconds);
}

// 只用于快速路由语句类型，真正的数据合法性由 nmea_parser 校验。
bool IsPrefixed(const std::string &line, const char *prefix) {
  return line.compare(0, std::strlen(prefix), prefix) == 0;
}

bool IsNmeaType(const std::string &line, const char *type) {
  return line.size() >= 6 && line[0] == '$' &&
         line.compare(3, 3, type) == 0;
}

// POSIX 串口封装：负责打开设备、配置 8N1/波特率/超时、读取字节和关闭设备。
// 设备文件由 ROS 参数提供，因此 /dev/ttyUSB0 可以替换为后续的 udev 映射名。
class SerialPort {
 public:
  SerialPort() : fd_(-1) {}
  ~SerialPort() { Close(); }

  // 以原始模式打开串口。C2 的 NMEA 输出是普通字节流，不需要行编辑或软件流控。
  bool Open(const std::string &device, int baudrate, double timeout_seconds,
            std::string *error) {
    Close();
    const speed_t speed = BaudToTermios(baudrate);
    if (speed == 0) {
      if (error) *error = "unsupported baudrate";
      return false;
    }

    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) {
      if (error) *error = std::strerror(errno);
      return false;
    }

    struct termios settings;
    if (tcgetattr(fd_, &settings) != 0) {
      if (error) *error = std::strerror(errno);
      Close();
      return false;
    }
    cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CRTSCTS;
    cfsetispeed(&settings, speed);
    cfsetospeed(&settings, speed);
    const int tenths = static_cast<int>(std::ceil(timeout_seconds * 10.0));
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = static_cast<cc_t>(std::max(1, std::min(255, tenths)));
    if (tcsetattr(fd_, TCSANOW, &settings) != 0) {
      if (error) *error = std::strerror(errno);
      Close();
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  // 关闭文件描述符；下次 Run 循环会重新打开设备。
  void Close() {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

  // 一次读取可能得到半条、整条或多条语句，调用方必须自行进行分帧。
  ssize_t Read(char *buffer, size_t size, int *error_number) {
    const ssize_t count = read(fd_, buffer, size);
    if (count < 0 && error_number) *error_number = errno;
    return count;
  }

  bool IsOpen() const { return fd_ >= 0; }

 private:
  int fd_;
};

// 北云 GNSS ROS 节点：串口读取、BESTPOSA/RMC 配对和 ROS 发布均在此完成。
class BeiyunGnssNode {
 public:
  BeiyunGnssNode()
      : nh_(),
        private_nh_("~"),
        port_("/dev/ttyUSB0"),
        baudrate_(115200),
        serial_timeout_(0.1),
        frame_id_("gps"),
        navsatfix_topic_("/rtk/navsatfix"),
        pair_tolerance_seconds_(0.2),
        timestamp_source_("gprmc_utc"),
        signal_timeout_seconds_(2.0),
        has_rmc_(false),
        has_bestpos_(false) {
    // 私有参数允许在端口映射前后使用同一个程序，不需要重新编译。
    private_nh_.param("port", port_, port_);
    private_nh_.param("baudrate", baudrate_, baudrate_);
    private_nh_.param("serial_timeout", serial_timeout_, serial_timeout_);
    private_nh_.param("frame_id", frame_id_, frame_id_);
    private_nh_.param("navsatfix_topic", navsatfix_topic_, navsatfix_topic_);
    private_nh_.param("pair_tolerance_seconds", pair_tolerance_seconds_,
              pair_tolerance_seconds_);
    private_nh_.param("timestamp_source", timestamp_source_, timestamp_source_);
    private_nh_.param("signal_timeout_seconds", signal_timeout_seconds_,
                      signal_timeout_seconds_);
    if (timestamp_source_ != "gprmc_utc" && timestamp_source_ != "system_now") {
      ROS_WARN("Unknown timestamp_source '%s'; falling back to gprmc_utc",
               timestamp_source_.c_str());
      timestamp_source_ = "gprmc_utc";
    }
    if (signal_timeout_seconds_ <= 0.0) {
      ROS_WARN("signal_timeout_seconds must be positive; using 2.0 seconds");
      signal_timeout_seconds_ = 2.0;
    }
    last_valid_bestpos_wall_time_ = ros::WallTime::now();
    last_published_wall_time_ = last_valid_bestpos_wall_time_;
    // NavSatFix 是融合后的正式输出；三个 raw 话题用于现场排查串口内容。
    navsatfix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(navsatfix_topic_, 10);
    raw_rmc_pub_ = nh_.advertise<std_msgs::String>("/rtk/raw_rmc", 10);
    raw_gga_pub_ = nh_.advertise<std_msgs::String>("/rtk/raw_gga", 10);
    raw_bestposa_pub_ = nh_.advertise<std_msgs::String>("/rtk/raw_bestposa", 10);
  }

  void Run() {
    // 主循环负责断线重连和读取。串口本身使用 VTIME，ROS 节点仍保持 100 Hz 响应。
    ros::Rate loop_rate(100.0);
    while (ros::ok()) {
      if (!serial_.IsOpen()) {
        std::string error;
        if (serial_.Open(port_, baudrate_, serial_timeout_, &error)) {
          ROS_INFO("Opened Beiyun GNSS serial %s at %d baud", port_.c_str(),
                   baudrate_);
        } else {
          ROS_WARN_THROTTLE(2.0, "Cannot open GNSS serial %s: %s", port_.c_str(),
                            error.c_str());
          ros::spinOnce();
          loop_rate.sleep();
          continue;
        }
      }

      // read() 返回的是任意长度字节块，不能假定一次对应一条 NMEA 语句。
      char bytes[512];
      int error_number = 0;
      const ssize_t count = serial_.Read(bytes, sizeof(bytes), &error_number);
      if (count > 0) {
        receive_buffer_.append(bytes, static_cast<size_t>(count));
        ProcessBuffer();
      } else if (count < 0 && error_number != EINTR && error_number != EAGAIN &&
                 error_number != EWOULDBLOCK) {
        ROS_WARN("GNSS serial read failed: %s; reconnecting", std::strerror(error_number));
        serial_.Close();
        receive_buffer_.clear();
        has_rmc_ = false;
        has_bestpos_ = false;
        last_published_stamp_ = ros::Time(0);
      }
      ros::spinOnce();
      CheckSignalStatus();
      loop_rate.sleep();
    }
  }

 private:
  // 从累计字节流中取出以 '\n' 结尾的完整行，并去掉 C2 使用的 '\r'。
  void ProcessBuffer() {
    while (true) {
      const std::string::size_type newline = receive_buffer_.find('\n');
      if (newline == std::string::npos) {
        if (receive_buffer_.size() > 4096) {
          ROS_WARN_THROTTLE(2.0, "Discarding overlong GNSS serial line");
          receive_buffer_.clear();
        }
        return;
      }
      std::string line = receive_buffer_.substr(0, newline);
      receive_buffer_.erase(0, newline + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (!line.empty()) HandleLine(line);
    }
  }

  // RMC 提供完整 UTC 时间；BESTPOSA 提供位置、状态和真实精度。
  // GGA 只保留原始话题，不参与正式 NavSatFix 组装。
  void HandleLine(const std::string &line) {
    std_msgs::String raw;
    raw.data = line;
    if (IsNmeaType(line, "RMC")) {
      raw_rmc_pub_.publish(raw);
      beiyun_gnss::RmcData parsed;
      if (beiyun_gnss::ParseRmc(line, &parsed)) {
        rmc_ = parsed;
        has_rmc_ = true;
        TryPublishFix();
      }
    } else if (IsNmeaType(line, "GGA")) {
      raw_gga_pub_.publish(raw);
    } else if (IsPrefixed(line, "#BESTPOSA")) {
      raw_bestposa_pub_.publish(raw);
      beiyun_gnss::BestPosData parsed;
      if (beiyun_gnss::ParseBestPos(line, &parsed)) {
        bestpos_ = parsed;
        has_bestpos_ = true;
        has_valid_bestpos_ever_ = true;
        last_valid_bestpos_wall_time_ = ros::WallTime::now();
        TryPublishFix();
      }
    }
  }

  // gprmc_utc 模式保持 BESTPOSA/RMC 配对逻辑；system_now 模式只依赖有效
  // BESTPOSA。BESTPOSA 的标准差是 sigma，NavSatFix 需要填 sigma^2。
  void TryPublishFix() {
    if (!has_bestpos_) return;

    ros::Time stamp;
    if (timestamp_source_ == "gprmc_utc") {
      if (!has_rmc_ ||
          !beiyun_gnss::BestPosTimeClose(
              bestpos_, rmc_.utc, pair_tolerance_seconds_)) {
        return;
      }
      stamp = ToRosTime(rmc_.utc);
      if (!last_published_stamp_.isZero() && stamp == last_published_stamp_) {
        return;
      }
    } else {
      // 系统时间模式下用 BESTPOSA 的 GNSS 历元去重，不能用 ros::Time::now()
      // 去重，因为每次发布时系统时间通常都不同。
      if (has_last_published_bestpos_time_ &&
          bestpos_.gps_time.week == last_published_bestpos_week_ &&
          bestpos_.gps_time.seconds == last_published_bestpos_seconds_) {
        return;
      }
      stamp = ros::Time::now();
    }

    if (stamp.isZero()) {
      ROS_WARN_THROTTLE(2.0,
                        "ROS time is zero; NavSatFix timestamp is not valid yet");
      return;
    }

    sensor_msgs::NavSatFix message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.status.status = StatusForBestPos(bestpos_);
    message.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    message.latitude = bestpos_.latitude;
    message.longitude = bestpos_.longitude;
    message.altitude = bestpos_.altitude;
    // BESTPOSA 的 lon/lat/hgt sigma 分别对应 ENU 的 east/north/up。
    message.position_covariance[0] =
        bestpos_.longitude_stddev * bestpos_.longitude_stddev;
    message.position_covariance[4] =
        bestpos_.latitude_stddev * bestpos_.latitude_stddev;
    message.position_covariance[8] =
        bestpos_.altitude_stddev * bestpos_.altitude_stddev;
    message.position_covariance_type =
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    navsatfix_pub_.publish(message);
    last_published_stamp_ = stamp;
    last_published_wall_time_ = ros::WallTime::now();
    if (timestamp_source_ == "system_now") {
      last_published_bestpos_week_ = bestpos_.gps_time.week;
      last_published_bestpos_seconds_ = bestpos_.gps_time.seconds;
      has_last_published_bestpos_time_ = true;
    }
    has_rmc_ = false;
    has_bestpos_ = false;
  }

  void CheckSignalStatus() {
    const ros::WallTime now = ros::WallTime::now();
    const double bestpos_age =
        (now - last_valid_bestpos_wall_time_).toSec();
    if (!has_valid_bestpos_ever_ || bestpos_age > signal_timeout_seconds_) {
      ROS_WARN_THROTTLE(2.0,
                        "GNSS signal missing or no valid BESTPOSA; "
                        "cannot publish NavSatFix");
      return;
    }

    if (timestamp_source_ == "gprmc_utc" &&
        (now - last_published_wall_time_).toSec() >
            signal_timeout_seconds_) {
      ROS_WARN_THROTTLE(
          2.0,
          "Valid BESTPOSA received, but no matching GPRMC; "
          "cannot publish NavSatFix in gprmc_utc mode");
    }
  }

  // NavSatStatus 没有 RTK 专用枚举：普通单点映射为 FIX，差分/RTK
  // 映射为 GBAS_FIX，只有明确无效解才映射为 NO_FIX。
  static int8_t StatusForBestPos(
      const beiyun_gnss::BestPosData &bestpos) {
    if (bestpos.sol_status != "SOL_COMPUTED") {
      return sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }
    if (bestpos.pos_type == "SINGLE") {
      return sensor_msgs::NavSatStatus::STATUS_FIX;
    }
    if (bestpos.pos_type == "WAAS") {
      return sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    }
    return sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
  }

  // ROS、串口和参数
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  SerialPort serial_;
  std::string port_;
  int baudrate_;
  double serial_timeout_;
  std::string frame_id_;
  std::string navsatfix_topic_;
  double pair_tolerance_seconds_;
  std::string timestamp_source_;
  double signal_timeout_seconds_;
  // 串口字节流和最近一条有效 BESTPOSA/RMC
  std::string receive_buffer_;
  beiyun_gnss::RmcData rmc_;
  beiyun_gnss::BestPosData bestpos_;
  bool has_rmc_;
  bool has_bestpos_;
  bool has_valid_bestpos_ever_{false};
  ros::WallTime last_valid_bestpos_wall_time_;
  ros::WallTime last_published_wall_time_;
  ros::Time last_published_stamp_;
  bool has_last_published_bestpos_time_{false};
  int last_published_bestpos_week_{0};
  double last_published_bestpos_seconds_{0.0};
  // ROS 发布器
  ros::Publisher navsatfix_pub_;
  ros::Publisher raw_rmc_pub_;
  ros::Publisher raw_gga_pub_;
  ros::Publisher raw_bestposa_pub_;
};

}  // namespace

int main(int argc, char **argv) {
  try {
    ros::init(argc, argv, "beiyun_gnss_node");
    BeiyunGnssNode node;
    node.Run();
    return 0;
  } catch (const std::exception &e) {
    fprintf(stderr, "beiyun_gnss_node terminated by exception: %s\n", e.what());
  } catch (...) {
    fprintf(stderr, "beiyun_gnss_node terminated by unknown exception\n");
  }
  return 1;
}
