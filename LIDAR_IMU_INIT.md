
```
ls   /dev | grep sdb
fsdisk
n
w

mkfs
mount disk
```
```
https://github.com/atinfinity/lab/wiki/change_docker_image_directory
/etc/docker/daemon.json
{
	  "data-root": "/mnt/extra/docker"
} 
```
```
https://github.com/hku-mars/LiDAR_IMU_Init

cd ~/catkin_ws/src
git clone https://github.com/hku-mars/LiDAR_IMU_Init.git
cd ..
catkin_make -j
source devel/setup.bash

cd catkin_ws
source devel/setup.bash
#roslaunch lidar_imu_init xxx.launch
roslaunch lidar_imu_init livox_avia.launch
```
