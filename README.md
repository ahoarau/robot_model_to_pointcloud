[![Build Status](https://travis-ci.org/ahoarau/robot_model_to_pointcloud.svg?branch=master)](https://travis-ci.org/ahoarau/robot_model_to_pointcloud)

RobotModel to PointCloud2
====================

![kuka cloud](https://drive.google.com/uc?id=0B6zWJ1Gzg1UTQ3RZM2cxRV9EblE)

![pr2 cloud](https://drive.google.com/uc?id=0B6zWJ1Gzg1UTSjNnYVVMa1Ewa1E)

![meka cloud](https://drive.google.com/uc?id=0B6zWJ1Gzg1UTWXVYWWxfN09MaDg)

This packages allows you to converts any robotModel (for now only the collision meshes) to a PointCloud2.

This can be used directly using pcl for processing (detection, collision checking etc).

### Instructions

First load your robot model and make sure it receives joint states. Ex with kuka : 
```bash
roslaunch kuka_description lbr4.launch
rosrun joint_state_publisher joint_state_publisher _use_gui:=true
```
Then run the converter node : 

```bash
rosrun robot_model_to_pointcloud robot_model_to_pointcloud
```

> Author : Antoine Hoarau <hoarau.robotics@gmail.com> 


