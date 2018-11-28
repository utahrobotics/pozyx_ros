# pozyx_ros
A ros package for interfacing with pozyx

## Instalation Requirements

### pip
pip is required to inorder to install the pozyx library. Pip can be install from [here](https://pip.pypa.io/en/stable/installing/)

## Topics
### Position
The position topic is published at /pozyx/pos, is of type geometery_msgs/PoseStamped and is published at a rate of about _hz.

### Orientation
The position topic is published at /pozyx/orient, is of type geometery_msgs/QuaternionStamped and is published at a rate of about _hz.

### Transform
The transform topic is published at /pozyx/trans, is of type geometery_msgs/TransformStamped and is published at a rate of about _hz.

