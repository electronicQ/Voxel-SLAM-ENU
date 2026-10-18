# Livox costom pointcloud format.

uint32 offset_time      # offset time relative to the base time  激光点相对于该帧首个激光点的相对时间偏移
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255   这个表示的是反射率，值越大表示目标表面反射强度越强，可用于区分材质，过滤噪声
uint8 tag               # livox tag             标识点的质量
uint8 line              # laser number in lidar

