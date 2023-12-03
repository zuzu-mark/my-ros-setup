# install cloudcompare

```
https://www.cloudcompare.org/release/notes/
```


# install velodyne

```
https://github.com/ros-drivers/velodyne/tree/master
https://ishiguro440.wordpress.com/2016/04/05/%E5%82%99%E5%BF%98%E9%8C%B2%E3%80%80ros-loam_velodyne/
https://ishiguro440.wordpress.com/2017/02/26/%e5%82%99%e5%bf%98%e9%8c%b2%ef%bc%9avelodyne-raspberry-pi3/
```

```
cd catkin_ws/src
git clone https://github.com/ros-drivers/velodyne.git
cd -
cd catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
cd -
```

```
cd catkin_ws/src
git clone https://github.com/laboshinl/loam_velodyne.git
cd -
cd catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
cd -
```

# execute loam

```
# send pcap
roslaunch velodyne_pointcloud VLP16_points.launch pcap:="$HOME/data/test.pcap"

# calibration
#roslaunch velodyne_pointcloud VLP16_points.launch calibration:=’（キャリブファイルのパス）/VLP16_db.yaml’
roslaunch velodyne_pointcloud VLP16_points.launch calibration:="/root/data/catkin_ws/src/velodyne/velodyne_pointcloud/params/VLP16db.yaml"

# save pcd
rosrun pcl_ros pointcloud_to_pcd input:=/velodyne_cloud_registered
```


----

# clang compiler
```
#https://qiita.com/evakichi/items/e6c3bd0c7f510fda7cbd
set(CMAKE_C_COMPILER "/usr/bin/clang" CACHE string "clang compiler" FORCE)
set(CMAKE_CXX_COMPILER "/usr/bin/clang++" CACHE string "clang++ compiler" FORCE)
```

# install clang,opt

```
#The binary (symbolic link) /usr/bin/opt is in the package llvm. Therefore install with
sudo apt-get install llvm
```


# exec opt

```
$ clang++ -S -emit-llvm main1.cpp -o - | opt -enable-new-pm=0  -analyze -dot-callgraph --callgraph-dot-filename-prefix=test
Writing 'test.callgraph.dot'...
Printing analysis 'Print call graph to 'dot' file':
Pass::print not implemented for pass: 'Print call graph to 'dot' file'!
$
```


# memo

```
#clang++ -S -emit-llvm main.cpp -o test.S
#clang++ -S -emit-llvm main1.cpp -o - | 
#opt -enable-new-pm=0  -analyze -dot-callgraph --callgraph-dot-filename-prefix=test
#dot -Tpng test.callgraph.dot > test.png
#
#cat callgraph.dot | \
#c++filt | \
#sed 's,>,\\>,g; s,-\\>,->,g; s,<,\\<,g' | \
#gawk '/external node/{id=$1} $1 != id' | \
#dot -Tpng -ocallgraph.png
```

# cmake

```
add_custom_command(TARGET multiScanRegistration PRE_BUILD 
COMMAND clang++ -S -emit-llvm -I${EIGEN3_INCLUDE_DIR}
-I${PCL_INCLUDE_DIRS} -I${CMAKE_CURRENT_SOURCE_DIR}/include -I${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src/multi_scan_registration_node.cpp -o - |
opt -analyze -dot-callgraph )
```

# ref

```
https://github.com/peng225/class_dep
```

