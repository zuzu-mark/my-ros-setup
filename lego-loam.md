```
https://github.com/RobustFieldAutonomyLab/LeGO-LOAM
```

```
wget -O $(pwd)/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
cd $(pwd)/ && unzip gtsam.zip -d ~/Downloads/
cd $(pwd)/gtsam-4.0.0-alpha2/
mkdir build && cd build
cmake ..
make 
sudo make install
```

```
catkin_make -DCATKIN_WHITELIST_PACKAGES="cloud_msgs"
```
