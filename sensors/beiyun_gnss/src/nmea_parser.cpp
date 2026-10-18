#include "beiyun_gnss/nmea_parser.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <ctime>
#include <vector>

namespace beiyun_gnss {
namespace {

// NMEA 校验和是 '$' 与 '*' 之间所有字符的逐字节 XOR，结果以两位十六进制表示。
bool IsHex(char value) {
  return (value >= '0' && value <= '9') ||
         (value >= 'a' && value <= 'f') ||
         (value >= 'A' && value <= 'F');
}

int HexValue(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return value - 'A' + 10;
}

bool ChecksumIsValid(const std::string &sentence) {
  if (sentence.size() < 5 || sentence[0] != '$') return false;
  const std::string::size_type star = sentence.find('*');
  if (star == std::string::npos || star + 2 >= sentence.size()) return false;
  if (!IsHex(sentence[star + 1]) || !IsHex(sentence[star + 2])) return false;

  unsigned char checksum = 0;
  for (std::string::size_type i = 1; i < star; ++i) {
    checksum ^= static_cast<unsigned char>(sentence[i]);
  }
  const int expected = HexValue(sentence[star + 1]) * 16 +
                       HexValue(sentence[star + 2]);
  return checksum == expected;
}

uint32_t Hex32Value(const std::string &value) {
  uint32_t result = 0;
  for (char digit : value) {
    result <<= 4;
    if (digit >= '0' && digit <= '9') {
      result |= static_cast<uint32_t>(digit - '0');
    } else if (digit >= 'a' && digit <= 'f') {
      result |= static_cast<uint32_t>(digit - 'a' + 10);
    } else {
      result |= static_cast<uint32_t>(digit - 'A' + 10);
    }
  }
  return result;
}

uint32_t CalcCrc32Value(int value) {
  const uint32_t polynomial = 0xEDB88320U;
  uint32_t crc = static_cast<uint32_t>(value);
  for (int i = 8; i > 0; --i) {
    crc = (crc & 1U) ? ((crc >> 1) ^ polynomial) : (crc >> 1);
  }
  return crc;
}

uint32_t CalcBlockCrc32(const std::string &value) {
  uint32_t crc = 0;
  for (unsigned char byte : value) {
    const uint32_t tmp1 = (crc >> 8) & 0x00FFFFFFU;
    const uint32_t tmp2 = CalcCrc32Value(static_cast<int>((crc ^ byte) & 0xFFU));
    crc = tmp1 ^ tmp2;
  }
  return crc;
}

bool BestPosCrcIsValid(const std::string &sentence) {
  if (sentence.size() < 12 || sentence[0] != '#') return false;
  const std::string::size_type star = sentence.find('*');
  if (star == std::string::npos || star + 9 != sentence.size()) return false;
  const std::string crc_text = sentence.substr(star + 1, 8);
  for (char digit : crc_text) {
    if (!IsHex(digit)) return false;
  }
  return CalcBlockCrc32(sentence.substr(1, star - 1)) == Hex32Value(crc_text);
}

// 去掉校验和部分并按逗号切分字段。空字段必须保留，否则字段下标会错位。
// 双引号内的逗号不作为字段分隔符。
std::vector<std::string> SplitFields(const std::string &sentence) {
  const std::string::size_type star = sentence.find('*');
  const std::string body = sentence.substr(0, star == std::string::npos
                                                   ? sentence.size()
                                                   : star);
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (char value : body) {
    if (value == '"') {
      quoted = !quoted;
    } else if (value == ',' && !quoted) {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(value);
    }
  }
  fields.push_back(field);
  return fields;
}

// 下面两个函数负责把 NMEA 字符串字段转换成数值，并拒绝空值和非法字符。
bool ParseUnsigned(const std::string &value, int *result) {
  if (value.empty() || !result) return false;
  char *end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0') return false;
  *result = static_cast<int>(parsed);
  return true;
}

bool ParseDouble(const std::string &value, double *result) {
  if (value.empty() || !result) return false;
  char *end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  *result = parsed;
  return true;
}

// 解析 hhmmss.sss 格式的 UTC 时刻。
bool ParseTime(const std::string &value, UtcTime *utc) {
  if (!utc || value.size() < 6) return false;
  int hour = 0;
  int minute = 0;
  double second = 0.0;
  if (!ParseUnsigned(value.substr(0, 2), &hour) ||
      !ParseUnsigned(value.substr(2, 2), &minute) ||
      !ParseDouble(value.substr(4), &second)) {
    return false;
  }
  if (hour > 23 || minute > 59 || second < 0.0 || second >= 60.0) {
    return false;
  }
  utc->hour = hour;
  utc->minute = minute;
  utc->second = second;
  return true;
}

// 解析 ddmmyy 格式的日期。NMEA 两位年份按 80 年分界转换为四位年份。
bool ParseDate(const std::string &value, UtcTime *utc) {
  if (!utc || value.size() != 6) return false;
  int day = 0;
  int month = 0;
  int year = 0;
  if (!ParseUnsigned(value.substr(0, 2), &day) ||
      !ParseUnsigned(value.substr(2, 2), &month) ||
      !ParseUnsigned(value.substr(4, 2), &year)) {
    return false;
  }
  if (day < 1 || day > 31 || month < 1 || month > 12) return false;
  utc->day = day;
  utc->month = month;
  utc->year = year >= 80 ? 1900 + year : 2000 + year;
  return true;
}

// NMEA 坐标格式是度分，例如纬度 3112.3456、经度 12128.1234。
// direction 决定南纬和西经是否取负号。
bool ParseCoordinate(const std::string &value, const std::string &direction,
                     bool latitude, double *coordinate) {
  if (!coordinate || value.empty() || direction.size() != 1) return false;
  double raw = 0.0;
  if (!ParseDouble(value, &raw)) return false;
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  const double max_degrees = latitude ? 90.0 : 180.0;
  if (degrees < 0 || degrees > max_degrees || minutes < 0.0 || minutes >= 60.0 ||
      (degrees == static_cast<int>(max_degrees) && minutes > 0.0)) {
    return false;
  }
  if ((latitude && direction != "N" && direction != "S") ||
      (!latitude && direction != "E" && direction != "W")) {
    return false;
  }
  *coordinate = degrees + minutes / 60.0;
  if (direction == "S" || direction == "W") *coordinate = -*coordinate;
  return true;
}

// 通过字段首项识别 GGA/RMC，同时兼容 GP 和 GN 两类 talker ID。
bool IsSentenceType(const std::vector<std::string> &fields,
                    const char *type) {
  return !fields.empty() && fields[0].size() == 6 && fields[0][0] == '$' &&
         fields[0].substr(3) == type;
}

// 将时分秒转换为当天经过的秒数，用于 GGA/RMC 配对。
double SecondsOfDay(const UtcTime &utc) {
  return utc.hour * 3600.0 + utc.minute * 60.0 + utc.second;
}

bool IsFiniteCoordinate(double latitude, double longitude, double altitude) {
  return std::isfinite(latitude) && std::isfinite(longitude) &&
         std::isfinite(altitude) && latitude >= -90.0 && latitude <= 90.0 &&
         longitude >= -180.0 && longitude <= 180.0;
}

struct LeapSecond {
  int year;
  int month;
  int day;
  int gps_utc_offset;
};

// GPS-UTC offset history through the current leap-second value.
const LeapSecond kLeapSeconds[] = {
    {1981, 7, 1, 1},  {1982, 7, 1, 2},  {1983, 7, 1, 3},
    {1985, 7, 1, 4},  {1988, 1, 1, 5},  {1990, 1, 1, 6},
    {1991, 1, 1, 7},  {1992, 7, 1, 8}, {1993, 7, 1, 9},
    {1994, 7, 1, 10}, {1996, 1, 1, 11}, {1997, 1, 1, 12},
    {1999, 1, 1, 13}, {2006, 1, 1, 14}, {2009, 1, 1, 15},
    {2012, 7, 1, 16}, {2015, 7, 1, 17}, {2017, 1, 1, 18}};

const time_t kGpsEpochUnixSeconds = 315964800;

time_t UtcCalendarToUnix(const LeapSecond &leap) {
  std::tm calendar = {};
  calendar.tm_year = leap.year - 1900;
  calendar.tm_mon = leap.month - 1;
  calendar.tm_mday = leap.day;
  return timegm(&calendar);
}

int GpsUtcOffset(double absolute_gps_seconds) {
  int offset = 0;
  for (const LeapSecond &leap : kLeapSeconds) {
    const double leap_utc_gps_seconds =
        static_cast<double>(UtcCalendarToUnix(leap) -
                            kGpsEpochUnixSeconds) +
        leap.gps_utc_offset;
    if (absolute_gps_seconds >= leap_utc_gps_seconds) {
      offset = leap.gps_utc_offset;
    }
  }
  return offset;
}

}  // namespace

UtcTime::UtcTime()
    : year(0), month(0), day(0), hour(0), minute(0), second(0.0) {}

RmcData::RmcData()
    : valid(false), has_position(false), latitude(0.0), longitude(0.0) {}

GgaData::GgaData()
    : valid(false),
      latitude(0.0),
      longitude(0.0),
      altitude(std::numeric_limits<double>::quiet_NaN()),
      fix_quality(0),
      satellites(0),
      hdop(std::numeric_limits<double>::quiet_NaN()) {}

GpsTime::GpsTime() : week(0), seconds(0.0) {}

BestPosData::BestPosData()
    : time_status(),
      sol_status(),
      pos_type(),
      latitude(0.0),
      longitude(0.0),
      altitude(std::numeric_limits<double>::quiet_NaN()),
      latitude_stddev(std::numeric_limits<double>::quiet_NaN()),
      longitude_stddev(std::numeric_limits<double>::quiet_NaN()),
      altitude_stddev(std::numeric_limits<double>::quiet_NaN()),
      tracked_satellites(0),
      solved_satellites(0),
      valid(false) {}

bool ParseRmc(const std::string &sentence, RmcData *data) {
  // RMC 的日期和时间是 NavSatFix.header.stamp 的唯一来源。
  if (!data || !ChecksumIsValid(sentence)) return false;
  const std::vector<std::string> fields = SplitFields(sentence);
  if (!IsSentenceType(fields, "RMC") || fields.size() < 10) return false;
  if (!ParseTime(fields[1], &data->utc) || !ParseDate(fields[9], &data->utc)) {
    return false;
  }
  if (fields[2] != "A") return false;

  data->valid = true;
  if (fields.size() >= 7 &&
      ParseCoordinate(fields[3], fields[4], true, &data->latitude) &&
      ParseCoordinate(fields[5], fields[6], false, &data->longitude)) {
    data->has_position = true;
  }
  return true;
}

bool ParseGga(const std::string &sentence, GgaData *data) {
  // GGA 必须有有效时间、经纬度和大于 0 的定位质量，才允许进入发布缓存。
  if (!data || !ChecksumIsValid(sentence)) return false;
  const std::vector<std::string> fields = SplitFields(sentence);
  if (!IsSentenceType(fields, "GGA") || fields.size() < 10) return false;
  if (!ParseTime(fields[1], &data->utc) ||
      !ParseCoordinate(fields[2], fields[3], true, &data->latitude) ||
      !ParseCoordinate(fields[4], fields[5], false, &data->longitude) ||
      !ParseUnsigned(fields[6], &data->fix_quality) || data->fix_quality <= 0) {
    return false;
  }

  ParseUnsigned(fields[7], &data->satellites);
  ParseDouble(fields[8], &data->hdop);
  ParseDouble(fields[9], &data->altitude);
  data->valid = true;
  return true;
}

bool ParseBestPos(const std::string &sentence, BestPosData *data) {
  if (!data || !BestPosCrcIsValid(sentence)) return false;

  const std::string::size_type semicolon = sentence.find(';');
  const std::string::size_type star = sentence.find('*');
  if (semicolon == std::string::npos || semicolon > star) return false;

  const std::vector<std::string> header =
      SplitFields(sentence.substr(0, semicolon));
  const std::vector<std::string> fields =
      SplitFields(sentence.substr(semicolon + 1, star - semicolon - 1));
  if (header.size() < 7 || fields.size() < 21 ||
      header[0] != "#BESTPOSA") {
    return false;
  }

  int week = 0;
  double seconds = 0.0;
  if (!ParseUnsigned(header[5], &week) || !ParseDouble(header[6], &seconds) ||
      week < 0 || seconds < 0.0 || seconds >= 604800.0) {
    return false;
  }
  if (header[4] == "UNKNOWN" || header[4] == "FINESTEERING_INVALID") {
    return false;
  }

  BestPosData parsed;
  parsed.gps_time.week = week;
  parsed.gps_time.seconds = seconds;
  parsed.time_status = header[4];
  parsed.sol_status = fields[0];
  parsed.pos_type = fields[1];
  if (!ParseDouble(fields[2], &parsed.latitude) ||
      !ParseDouble(fields[3], &parsed.longitude) ||
      !ParseDouble(fields[4], &parsed.altitude) ||
      !ParseDouble(fields[7], &parsed.latitude_stddev) ||
      !ParseDouble(fields[8], &parsed.longitude_stddev) ||
      !ParseDouble(fields[9], &parsed.altitude_stddev) ||
      !ParseUnsigned(fields[13], &parsed.tracked_satellites) ||
      !ParseUnsigned(fields[14], &parsed.solved_satellites)) {
    return false;
  }
  if (parsed.sol_status != "SOL_COMPUTED" ||
      parsed.pos_type.empty() || parsed.pos_type == "NONE" ||
      !IsFiniteCoordinate(parsed.latitude, parsed.longitude, parsed.altitude) ||
      parsed.latitude_stddev < 0.0 || parsed.longitude_stddev < 0.0 ||
      parsed.altitude_stddev < 0.0) {
    return false;
  }

  parsed.valid = true;
  *data = parsed;
  return true;
}

bool GpsTimeToUtc(const GpsTime &gps_time, UtcTime *utc) {
  if (!utc || gps_time.week < 0 || gps_time.seconds < 0.0 ||
      gps_time.seconds >= 604800.0) {
    return false;
  }

  const double absolute_gps_seconds =
      static_cast<double>(gps_time.week) * 604800.0 + gps_time.seconds;
  const int gps_utc_offset = GpsUtcOffset(absolute_gps_seconds);
  const double unix_seconds =
      static_cast<double>(kGpsEpochUnixSeconds) + absolute_gps_seconds -
      gps_utc_offset;
  const double whole = std::floor(unix_seconds);
  const time_t whole_seconds = static_cast<time_t>(whole);
  std::tm calendar = {};
  if (!gmtime_r(&whole_seconds, &calendar)) return false;

  utc->year = calendar.tm_year + 1900;
  utc->month = calendar.tm_mon + 1;
  utc->day = calendar.tm_mday;
  utc->hour = calendar.tm_hour;
  utc->minute = calendar.tm_min;
  utc->second = calendar.tm_sec + (unix_seconds - whole);
  return true;
}

bool BestPosTimeClose(const BestPosData &bestpos, const UtcTime &rmc_time,
                      double tolerance_seconds) {
  if (!bestpos.valid || tolerance_seconds < 0.0) return false;
  UtcTime bestpos_utc;
  if (!GpsTimeToUtc(bestpos.gps_time, &bestpos_utc)) return false;
  return UtcClose(rmc_time, bestpos_utc, tolerance_seconds);
}

bool UtcClose(const UtcTime &rmc_time, const UtcTime &gga_time,
              double tolerance_seconds) {
  // 串口读取顺序不能作为配对依据；使用语句内 UTC 时刻进行匹配。
  if (tolerance_seconds < 0.0) return false;
  if (rmc_time.year != 0 && gga_time.year != 0 &&
      (rmc_time.year != gga_time.year || rmc_time.month != gga_time.month ||
       rmc_time.day != gga_time.day)) {
    return false;
  }
  double difference = std::fabs(SecondsOfDay(rmc_time) - SecondsOfDay(gga_time));
  if (difference > 43200.0) difference = 86400.0 - difference;
  return difference <= tolerance_seconds;
}

}  // namespace beiyun_gnss
