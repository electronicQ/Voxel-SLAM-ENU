#ifndef BEIYUN_GNSS_COVARIANCE_POLICY_H_
#define BEIYUN_GNSS_COVARIANCE_POLICY_H_

namespace beiyun_gnss {

// 各解类型使用的单轴位置方差，单位为 m^2。
// 当前策略将同一个方差填入 east/north/up 三个对角元素。
struct CovarianceConfig {
  double single_point_covariance_m2;
  double dgps_covariance_m2;
  double rtk_fixed_covariance_m2;
  double rtk_float_covariance_m2;

  CovarianceConfig()
      : single_point_covariance_m2(25.0),
        dgps_covariance_m2(1.0),
        rtk_fixed_covariance_m2(0.0004),
        rtk_float_covariance_m2(0.04) {}
};

// 根据 C2 GGA quality 选择要写入 NavSatFix 的位置方差。
// 返回 0 表示没有可用的已知协方差。
inline double CovarianceForFixQuality(int quality,
                                      const CovarianceConfig &config) {
  double covariance = 0.0;
  switch (quality) {
    case 1:
      covariance = config.single_point_covariance_m2;
      break;
    case 2:
      covariance = config.dgps_covariance_m2;
      break;
    case 4:
      covariance = config.rtk_fixed_covariance_m2;
      break;
    case 5:
      covariance = config.rtk_float_covariance_m2;
      break;
    default:
      return 0.0;
  }
  return covariance >= 0.0 ? covariance : 0.0;
}

}  // namespace beiyun_gnss

#endif  // BEIYUN_GNSS_COVARIANCE_POLICY_H_
