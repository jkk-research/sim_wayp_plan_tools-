# robotverseny

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

## Build

``` bash
colcon build --symlink-install --packages-select robotverseny_application robotverseny_description robotverseny_bringup robotverseny_gazebo 
```

## Run


``` bash
export IGN_GAZEBO_RESOURCE_PATH=~/ros2_ws/install/robotverseny_gazebo/share/robotverseny_gazebo/worlds:~/ros2_ws/install/robotverseny_description/share:${IGN_GAZEBO_RESOURCE_PATH}
```

``` bash
export IGN_GAZEBO_MODEL_PATH=~/ros2_ws/src/robotverseny/robotverseny_description/models:${IGN_GAZEBO_MODEL_PATH}
```

``` bash 
echo $IGN_GAZEBO_RESOURCE_PATH
```

``` bash 
echo $IGN_GAZEBO_MODEL_PATH
```

``` bash
source ~/ros2_ws/install/setup.bash
```

``` bash
ros2 launch robotverseny_bringup roboworks.launch.py
```