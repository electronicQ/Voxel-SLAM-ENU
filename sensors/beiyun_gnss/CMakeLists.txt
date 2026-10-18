cmake_minimum_required(VERSION 3.0.2)
project(beiyun_gnss)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

# 解析器头文件作为公共接口，节点链接解析器库。
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(beiyun_gnss_parser
  src/nmea_parser.cpp
)
# 解析器不依赖 ROS，既可被节点使用，也可单独做单元测试。
target_compile_features(beiyun_gnss_parser PUBLIC cxx_std_11)
target_compile_options(beiyun_gnss_parser PRIVATE -Wall -Wextra -Wpedantic)

add_executable(beiyun_gnss_node
  src/beiyun_gnss_node.cpp
)
# ROS 节点负责串口和消息发布，核心语句处理复用 beiyun_gnss_parser。
target_compile_features(beiyun_gnss_node PUBLIC cxx_std_11)
target_compile_options(beiyun_gnss_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(beiyun_gnss_node
  beiyun_gnss_parser
  ${catkin_LIBRARIES}
)

install(TARGETS beiyun_gnss_parser beiyun_gnss_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # 测试 GGA/RMC 字段解析、校验和以及 UTC 配对规则。
  catkin_add_gtest(beiyun_gnss_nmea_test
    test/nmea_parser_test.cpp
  )
  if(TARGET beiyun_gnss_nmea_test)
    target_link_libraries(beiyun_gnss_nmea_test beiyun_gnss_parser)
  endif()
endif()
