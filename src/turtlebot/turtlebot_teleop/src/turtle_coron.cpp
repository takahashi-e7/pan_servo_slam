/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

#include "serial.h"
#include <string>
#include <fstream>

#include <limits>
const double DEF_Inf = std::numeric_limits<double>::infinity();

#define SCAN_TIME (30)  // SCAN_TIME*100ms
//#define SCAN_TIME (5)  // SCAN_TIME*100ms

class CoronController
{
public:
  CoronController();
  ~CoronController();

private:
  void keyCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void publish();
  double getCoronSonars();
  double convertVolt2Distance1_150( double dVolt );
  void setServo();
  void processServo();

  void sendCommand( Serial s, char *command );
  bool myPortOpen();
  void stopCoron();

  ros::Publisher vel_pub_;
  ros::Publisher laser_pub_;
  ros::Publisher laser_tmp_pub_;
  ros::Subscriber key_sub_;
  ros::Subscriber laser_sub_; // レーザのパラメータを受け取る為に一回だけ稼働
  ros::Timer timer_;

  geometry_msgs::Twist last_published_;

  geometry_msgs::Twist cmd_vel_;
  sensor_msgs::LaserScan laser_;
  sensor_msgs::LaserScan tirger_laser_;

  int cmd_cnt_, wait_cnt_;
  int scan_time_, incre_scan_time_;
  double servo_angle_, pre_servo_angle_, direction_;

  double incre_deg_;
  double max_angle_;

  bool is_ready_, sim_flg_;
  bool scan_flg_;     // スキャン中か否か

  Serial ser_;

};


// constructor
CoronController::CoronController():
    cmd_cnt_(0),
    wait_cnt_(0),
    servo_angle_(0.0),
    pre_servo_angle_(0.0),
    direction_(1.0),
    is_ready_(false),
    sim_flg_(false),
    max_angle_(30.0),
    incre_deg_(6.0),
    scan_flg_(false),
    scan_time_(0),
    incre_scan_time_(0)
{
  ros::NodeHandle nh_;

  // tutlebot_teleopから受け取る
  key_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10, &CoronController::keyCallback, this);

  // gazebo上から受け取った距離センサ値
  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("scan2", 10, &CoronController::laserCallback, this);

  // gazebo上の台車への指令
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);

  // 距離センサ値を束ねたLRF
  laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10, true);

  // gazeboのサーボを回すための、トリガー
  laser_tmp_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan3", 10, true);

  // timer at 10hz
  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&CoronController::publish, this));


  // gazeboか実機かの切り替えはここで行う
  sim_flg_ = true;
//  sim_flg_ = false;



  // 実機の時はシリアルポートを開く
  if(!sim_flg_)
    myPortOpen();

  char command[1];
  if( ser_.IsOpen() )
  {
    command[0] = 'z';     // サーボを0度にする
    ser_.Send(command,1); // stop command
  }

  // Fake laser initialize. Mimic the kinect.
  int num_laser = 10;
  laser_.angle_max = 0.521568000317;
  laser_.angle_min = -0.521568000317;
  laser_.range_max = 1.5;
//  laser_.range_max = 10.0;  // kinect range
  laser_.range_min = 0.2;
  laser_.angle_increment = (laser_.angle_max - laser_.angle_min) / num_laser;
  laser_.ranges.resize(num_laser);
  laser_.intensities.resize(num_laser);

  // sim?
  if( sim_flg_ )
    is_ready_ = true;

}


void CoronController::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // データの更新は行う
  laser_.header = msg->header;
  int index_adjuster_ = max_angle_ / incre_deg_;

  // 角度のマイナスをindex_adjuster_で配列のインデックスへ変える
  int range_index = (servo_angle_ / incre_deg_) + index_adjuster_;
  laser_.ranges[range_index] = msg->ranges[0];

  // gazeboサーボトリガー
  tirger_laser_.angle_max = servo_angle_;
  laser_tmp_pub_.publish(tirger_laser_);

  // サーボを次の角度にする
  processServo();

  // シミュレーションの時と実機の時で処理を分ける
  if( sim_flg_ )
  {
    // ここから距離センサを束ねる処理
    // 台車が移動してないことを確認
    if( cmd_cnt_ == 0 )
    {
      // サーボが端にいたらカウント開始
      if( range_index == 0 || range_index == 9 )
        incre_scan_time_ = 1;

      scan_time_ += incre_scan_time_;

      // スキャンタイムが予定に達していたら
      if( scan_time_ > 9 )
      {
        laser_pub_.publish(laser_); // レーザーデータを流す
        scan_time_ = 0;
      }
    }
    // 移動中なのでレーザに関してはなにもしない
    else
    {
//      std::cout << "Moving Now!!" << std::endl;
      incre_scan_time_ = 0;
      scan_time_ = 0;
    }
  }
  // 実機のケース
  else
  {
    ros::Duration(0.1).sleep(); // サーボモータの準備が出来るまで待つ
    is_ready_ = true;
    laser_sub_.shutdown();  // 以降、gazeboからの距離センサデータは受け取らない
  }
}


CoronController::~CoronController()
{
  char command[1];
  if( ser_.IsOpen() )
  {
    command[0] = 'f';
    ser_.Send(command,1);  // stop command
    ser_.Close();
  }
}


// turtlebot_teleopからの入力を処理
void CoronController::keyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // 動作終了まで待つ
  if( wait_cnt_ == 0 )
  {
    double trans_vel = 0.068;
    double angle_vel = 0.53;
    char command[1];


    if( msg->linear.x > 0 )
    {
      cmd_vel_.linear.x = trans_vel;  // この速度はあくまでもgazeboと現実の辻褄合わせ
      command[0] = 'c'; // forward
    }
    else if(msg->linear.x < 0)
    {
      cmd_vel_.linear.x = -trans_vel;  // この速度はあくまでもgazeboと現実の辻褄合わせ
      command[0] = 'b'; // back
    }
    else if(msg->angular.z > 0)
    {
      cmd_vel_.angular.z = angle_vel;  // この速度はあくまでもgazeboと現実の辻褄合わせ
      command[0] = 'e'; // left turn
    }
    else if(msg->angular.z < 0)
    {
      cmd_vel_.angular.z = -angle_vel;  // この速度はあくまでもgazeboと現実の辻褄合わせ
      command[0] = 'd'; // right turn
    }
    sendCommand( ser_, command ); // 方向転換をマイコンへ指示

    // 直進
    if(msg->linear.x != 0.0)
    {
      command[0] = 't';
      cmd_cnt_ = 18;          // 1800msで10cm
      wait_cnt_ = SCAN_TIME;  // 動作終了後はしばらく次のコマンドを受けない
    }
    // 旋回
    else if(msg->angular.z != 0.0)
    {
      command[0] = 'l';
      cmd_cnt_ = 5;             // 500msで30deg
      wait_cnt_ = SCAN_TIME;    // 動作終了後はしばらく次のコマンドを受けない
    }
    sendCommand( ser_, command ); // 進行をマイコンへ指示
  }
}


// センサ取得とモータ駆動時間の管理
// タイマーで呼び出される 0.1s period
void CoronController::publish()
{
//  std::cout << "publish call" << std::endl;

  if( is_ready_ )
  {
    // 実機の場合
    if( !sim_flg_ )
    {
      // センサ取得
      double dist = getCoronSonars(); //
      laser_.header.stamp = ros::Time::now(); // レーザ取得時間

      // サーボトリガー
      tirger_laser_.angle_max = servo_angle_;
      laser_tmp_pub_.publish(tirger_laser_);  // gazeboの見た目合わせ


      int index_adjuster_ = max_angle_ / incre_deg_;
      int range_index = (servo_angle_ / incre_deg_) + index_adjuster_;
      laser_.ranges[range_index] = dist;

      setServo(); // 首振りサーボを次の角度へ

      // 台車が移動してないことを確認
      if( cmd_cnt_ == 0 )
      {
        // サーボが端にいたらカウント開始
        if( range_index == 0 || range_index == 9 )
          incre_scan_time_ = 1;

        scan_time_ += incre_scan_time_;


        if( scan_time_ > 9 )
        {
          laser_pub_.publish(laser_); // レーザーデータを流すだけ
          scan_time_ = 0;
        }
      }
      else
      {
        //std::cout << "Moving Now!!" << std::endl;
        incre_scan_time_ = 0;
        scan_time_ = 0;
      }
    }


    // モータ駆動時間管理
    if( cmd_cnt_ > 0 )
    {
      cmd_cnt_--;
      vel_pub_.publish(cmd_vel_);
    }
    else
    {
      stopCoron();
      wait_cnt_--;
      if( wait_cnt_ < 0 )
        wait_cnt_ = 0;
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;
    }
  }
}


// 距離センサの数は１個なので、そのままdistをリターンしてしまう
double CoronController::getCoronSonars()
{
  char command[] = "a"; // get addata 12bits
  char recv[256];

  //ros::Time tmp = ros::Time::now();
  //double start_time = tmp.sec + (tmp.nsec/1e9);

  sendCommand( ser_, command ); // 読み取りをマイコンへ指示


  if( ser_.IsOpen() )
    ser_.Recieve( recv, 5 );  //

  double dist;  // 距離
  int volt;     // ad値
  double d_volt;// [v]に変換後
  for( int i = 0; i < 1; i++ )  // ここはモータの個数
  {
    volt = 0;
    volt = (recv[5*i+1] - 0x30) * 1000;
    volt += (recv[5*i+2] - 0x30) * 100;
    volt += (recv[5*i+3] - 0x30) * 10;
    volt += recv[5*i+4] - 0x30;
    d_volt = (double)volt / 4096.0 * 3.3; // convert num to volt
    dist = convertVolt2Distance1_150(d_volt);
  }
//  tmp = ros::Time::now();
//  double end_time = tmp.sec + (tmp.nsec/1e9);
//printf("ad convert time is: %.6f [s]\n", end_time - start_time );

  return dist;
}


// サーボモータの角度を計算して次の角度を代入する
void CoronController::processServo()
{
  // calc angle
  servo_angle_ += incre_deg_ * direction_;
  if ( servo_angle_ > (max_angle_ - incre_deg_) || servo_angle_ < -max_angle_ )
  {
    direction_ *= -1.0;
    servo_angle_ = servo_angle_ + (2*incre_deg_ * direction_);
  }
}


// サーボを進める
void CoronController::setServo()
{
  processServo(); // 角度の計算

  // control servo
  char command[1];
  command[0]= 'v';
  if( direction_ == 1.0 )
  {
    command[0] = 'v';
  }
  else
  {
    command[0] = 'w';
  }
  sendCommand( ser_, command );


  // debug
//  int deg = pre_servo_angle_;
//  double rad = deg * M_PI / 180.0;
//  double x = laser_.ranges[0] * cos(rad);
//  double y = laser_.ranges[0] * sin(rad);
//  ofs_ << deg <<","<< laser_.ranges[0] <<","<< rad <<","<< x <<","<< y <<  std::endl;


  pre_servo_angle_ = servo_angle_;  // AD取得した時点ではservoは前周期の角度になっている
}


void CoronController::stopCoron()
{
  char command[1];
  command[0] = 'f';
  sendCommand( ser_, command ); // stop
}


void CoronController::sendCommand( Serial s, char *command )
{
  if( s.IsOpen() )
  {
    s.Send(command, 1);
    ros::Duration(0.01).sleep();
  }
}


bool CoronController::myPortOpen()
{
  std::cout << " myPortOpen call" << std::endl;

  // open serial
  char ser_name0[] = "/dev/ttyACM0\0";
  char ser_name1[] = "/dev/ttyACM1\0";
  char ser_name2[] = "/dev/ttyUSB0\0";
  char ser_name3[] = "/dev/ttyUSB1\0";

  bool res = true;

  int bau1 = 115200;
  int bau2 = 9600;

  if( ser_.Open(ser_name2, bau1 ) == true ){
    std::cout << ser_name2 << " open!" << std::endl;
  }else if( ser_.Open(ser_name3, bau1 ) == true ){
      std::cout << ser_name3 << " open!" << std::endl;
  }else if( ser_.Open(ser_name0, bau2 ) == true ){
      std::cout << ser_name0 << " open!" << std::endl;
  }else if( ser_.Open(ser_name1, bau2 ) == true ){
      std::cout << ser_name1 << " open!" << std::endl;
  }else{
    res = false;
    std::cout << "all port open failed!" << std::endl;
  }
  return res;
}


double CoronController::convertVolt2Distance1_150( double dVolt )
{
  double dDist = -0.1;  // [m]

//     std::cout << dVolt << " [V]" << std::endl;

   // 0.2-1.5 sonar sensor
   if ( dVolt < 0.36)        // over 1.5
    dDist = DEF_Inf;

  else if( dVolt < 0.38 )  // 1.5 - 1.4
    dDist = dVolt * -4.2857142857 + 3.0;

  else if( dVolt < 0.41)  // 1.4 - 1.3
    dDist = dVolt * -3.6842105263 +2.8;

  else if( dVolt < 0.451)  // 1.3 - 1.2
    dDist = dVolt * -3.1707317073 + 2.6;

  else if( dVolt < 0.5)   // 1.2 - 1.1
    dDist = dVolt * -2.6607538803 + 2.4;

  else if( dVolt < 0.552)  // 1.1 - 1.0
    dDist = dVolt * -2.2 + 2.2;

  else if( dVolt < 0.61)   // 1.0 - 0.9
    dDist = dVolt * -1.8115942029 + 2;

  else if( dVolt < 0.705)   // 0.9 - 0.8
    dDist = dVolt * -1.4754098361 + 1.8;

  else if( dVolt < 0.81)  // 0.8 - 0.7
    dDist = dVolt * -1.134751773 + 1.6;

  else if( dVolt < 0.94)  // 0.7 - 0.6
    dDist = dVolt * -0.8641975309 + 1.4;

  else if( dVolt < 1.13)  // 0.6 - 0.5
    dDist = dVolt * -0.6382978723 + 1.2;

  else if( dVolt < 1.42)  // 0.5 - 0.4
    dDist = dVolt * -0.4424778761 + 1.0;

  else if( dVolt < 1.83)  // 0.4 - 0.3
    dDist = dVolt * -0.2816901408 + 0.8;

  else if( dVolt < 2.28)  // 0.3 - 0.2
    dDist = dVolt * -0.1639344262 + 0.6;

  else                    // lower 0.2
    dDist = -DEF_Inf;  // distance too shortry
  return dDist;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "coron_teleop");
  CoronController cc;

  ros::spin();
}
