RobotModel to PointCloud2
====================
![pr2 cloud](http://googledrive.com/host/0B6zWJ1Gzg1UTUW1rYVpiMUZvUkk/pr2_cloud_small.png)

![meka cloud](http://googledrive.com/host/0B6zWJ1Gzg1UTUW1rYVpiMUZvUkk/meka_cloud_small.png)

![kuka cloud](http://googledrive.com/host/0B6zWJ1Gzg1UTUW1rYVpiMUZvUkk/kuka_cloud_small.png)

This packages allows you to converts any robotModel (for now only meshes) to a PointCloud2.

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


