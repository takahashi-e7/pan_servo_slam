Slam with Pan Servo
====


## 使い方

`$ source devel/setup.bash`  

`# world launch`  
`$ roslaunch turtlebot_gazebo turtlebot_world.launch`

`# slam with navigation`  
`$ roslaunch turtlebot_gazebo amcl_demo.launch`

`# rviz`  
`$ roslaunch turtlebot_rviz_launchers view_navigation.launch`


## インストール

`$ source /opt/ros/indigo/setup.bash`  
`$ mkdir -p ~/pan_servo_slam_ws/src`  
`$ `  
`$ cd pan_servo_slam_ws/src`  
`$ git clone https://github.com/takahashi-e6/0614test`  
`$ catkin_init_workspace`  
`$ cd ..`  
`$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r`  
`$ catkin_make`  

## ライセンス

???????

## Author

[sho](https://github.com/takahashi-e6)
