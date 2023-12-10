
# ref

```
https://github.com/koide3/hdl_graph_slam

# for noetic
sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o

cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/koide3/hdl_graph_slam

cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release


```

```
rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_501.launch

roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz

rosbag play --clock hdl_501_filtered.bag
```

```
rosrun hdl_graph_slam bag_player.py hdl_501_filtered.bag
```

```
# save map
rosservice call /hdl_graph_slam/save_map "resolution: 0.05 destination: '/root/map.pcd'"
```
