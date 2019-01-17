# pozyx_ros
A ros package for interfacing with pozyx

## Instalation Requirements

### pip
pip is required to inorder to install the pozyx library. Pip can be install from [here](https://pip.pypa.io/en/stable/installing/)

## Topics
### Range
The range topic is published at /pozyx/range, is of type [DeviceRange](msg/DeviceRange.msg) and is published at a rate of about _hz.

### Position
The position topic is published at /pozyx/pos, is of type [geometery_msgs/PoseStamped](https://docs.ros.org/api/geometry_msgs/html/msg/PosStamped.html) and is published at a rate of about _hz.

### Heading
The position topic is published at /pozyx/heading, is of type [geometery_msgs/Vector3Stamped](https://docs.ros.org/api/geometry_msgs/html/msg/Vector3Stamped.html) and is published at a rate of about _hz.

### Transform
The transform topic is published at /pozyx/trans, is of type [geometery_msgs/TransformStamped](https://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html) and is published at a rate of about _hz.

### IMU
The imu topic is published at /pozyx/imu, is of type [sensor_msgs/Imu](https://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) and is published at a rate of about _hz.
