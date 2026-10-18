#ifndef BEIYUN_GNSS_NMEA_PARSER_H_
#define BEIYUN_GNSS_NMEA_PARSER_H_

#include <string>

namespace beiyun_gnss {

// UTC 时间拆成日期和时分秒，RMC 提供完整字段。
struct UtcTime {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  double second;

  UtcTime();
};

// RMC 提供完整 UTC 时间；其中的坐标只作为可选信息保留。
struct RmcData {
  UtcTime utc;
  bool valid;
  bool has_position;
  double latitude;
  double longitude;

  RmcData();
};

// GGA 解析结构保留用于兼容和诊断。
struct GgaData {
  UtcTime utc;
  bool valid;
  double latitude;
  double longitude;
  double altitude;
  int fix_quality;
  int satellites;
  double hdop;

  GgaData();
};

// BESTPOSA 头部使用 GPS 周和周内秒，不能直接当作 UTC。
struct GpsTime {
  int week;
  double seconds;

  GpsTime();
};

// BESTPOSA 的位置、解状态和真实位置标准差。
struct BestPosData {
  GpsTime gps_time;
  std::string time_status;
  std::string sol_status;
  std::string pos_type;
  double latitude;
  double longitude;
  double altitude;
  double latitude_stddev;
  double longitude_stddev;
  double altitude_stddev;
  int tracked_satellites;
  int solved_satellites;
  bool valid;

  BestPosData();
};

// 解析带有效 NMEA 校验和的 RMC/GGA，以及带 CRC32 的 BESTPOSA。
// 返回 false 表示语句格式、校验和或定位状态不满足发布条件。
bool ParseRmc(const std::string &sentence, RmcData *data);
bool ParseGga(const std::string &sentence, GgaData *data);
bool ParseBestPos(const std::string &sentence, BestPosData *data);

// 将 BESTPOSA 的 GPS 周时间转换为 UTC。
bool GpsTimeToUtc(const GpsTime &gps_time, UtcTime *utc);

// 比较 BESTPOSA 的 GPS 时间和 RMC 的 UTC 时间，处理跨午夜的边界情况。
bool BestPosTimeClose(const BestPosData &bestpos, const UtcTime &rmc_time,
                      double tolerance_seconds);

// GGA 没有日期，因此配对时比较时分秒；保留给兼容代码使用。
bool UtcClose(const UtcTime &rmc_time, const UtcTime &gga_time,
              double tolerance_seconds);

}  // namespace beiyun_gnss

#endif  // BEIYUN_GNSS_NMEA_PARSER_H_
