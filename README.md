Slam with Pan Servo
====

## 概要
ROSのSLAMパッケージと１つ距離センサを用いたシステム。SLAMへのセンサ入力はLRFが一般的だが
今回は距離センサをサーボモータで回転させてLRFの代替とした。


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
`$ git clone https://github.com/takahashi-e6/pan_servo_slam`  
`$ cd pan_servo_slam/src`  
`$ catkin_init_workspace`  
`$ cd ..`  
`$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y`  
`$ catkin_make`  

## ライセンス

??????

## Author

[sho](https://github.com/takahashi-e6)
