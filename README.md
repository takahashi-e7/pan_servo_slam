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
`$ git clone https://github.com/takahashi-e6/0614test`  
`$ cd pan_servo_slam/src`  
`$ catkin_init_workspace`  
`$ cd ..`  
`$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y`  
`$ catkin_make`  

## ライセンス

???????

## Author

[sho](https://github.com/takahashi-e6)
