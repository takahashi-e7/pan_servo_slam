Slam with Pan Servo
====

## 概要
ROSのSLAMパッケージと１つの距離センサを用いたSLAMシステム。SLAMへのセンサ入力はLRFが一般的だが
今回は距離センサをサーボモータで回転させてLRFの代替とした。
実際、距離センサ（GP2Y0A02YK）を使い、前方60[deg]、10点のデータを1[Hz]で
取得してSLAMを行った。

### なぜ自作のnavigation?
move_baseのnavigationから送られてくるのが、速度指令だから。
今回使用した台車はジャイロセンサもエンコーダもなく、速度指令への追従が難しかった。

## インストール

`$ source /opt/ros/indigo/setup.bash`  
`$ cd ~/`   
`$ git clone https://github.com/takahashi-e6/pan_servo_slam`  
`$ cd pan_servo_slam/src`  
`$ catkin_init_workspace`  
`$ cd ..`  
`$ rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r`  
`$ catkin_make`  

## 使い方

### 起動
`# ターミナル起動時に設定を反映`  
`$ echo "source ~/pan_servo_slam/devel/setup.bash" >> ~/.bashrc`  
`$ source ~/.bashrc`  

`# ターミナルを複数立ち上げて各launchを起動`
`# gazebo起動`  
`$ roslaunch turtlebot_gazebo turtlebot_world.launch`

`# オドメトリ取得用`  
`$ roslaunch turtlebot_gazebo amcl_demo.launch`

`# Rviz立ち上げ`  
`$ roslaunch turtlebot_rviz_launchers view_navigation.launch`

### 動作確認
Rviz上で「2D nav goal」を押下して、向かわせたい場所を押下をするとその場所へ台車が移動する。その際の経路が赤い線で表示される。


## 参考
[Pi Robot Meets ROS](http://www.pirobot.org/blog/0014/)

## ライセンス

??????

## Author

[sho](https://github.com/takahashi-e6)
