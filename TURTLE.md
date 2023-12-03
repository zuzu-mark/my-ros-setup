# DEMO

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch turtlebot3_gazebo turtlebot3_simulation.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```


# SLAM

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_slam turtlebot3_slam.launch

export TURTLEBOT3_MODEL=waffle
rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_gmapping.rviz

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
rosrun map_server map_saver -f ~/map
```


# EXEC SLAM

```
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

export TURTLEBOT3_MODEL=waffle
rosrun rviz rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_navigation.rviz
```
