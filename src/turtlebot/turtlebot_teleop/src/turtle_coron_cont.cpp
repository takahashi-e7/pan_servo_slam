#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

#include "tf/transform_datatypes.h"

#include "serial.h"


#include <string>
#include <fstream>


class MyController
{
public:
  MyController();
  ~MyController();

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);

  void calcDirecAndDist(); // calc distance and direction between odom to goal
  bool isGoal();

  double deg2rad( double deg );
  double rad2deg( double rad );

  double convertPose2Yaw( geometry_msgs::Pose pose );

  void nav();
  void move();
  geometry_msgs::Twist turn();
  void fitPose(); // ゴールについたら次は姿勢を合わせる


  ros::Publisher vel_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Timer timer_;

  nav_msgs::Odometry odom_;
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::PoseStamped goal_ori_; // ゴール時の姿勢を記録しておく

  nav_msgs::Path path_;
  double angleCorrecton( double ref, double yaw );
  void setNextPath();
  // for save angle and range
  std::ofstream ofs_;

  double goal_distance_, diff_angle_;
  double tolerance_angle_, tolerance_distance_;

  int path_pointer_, turn_cnt_;
  int state_;

  enum ROBOT_STATE
  {
    WAIT,MOVE,MOVING,FIT_POSE
  };
};


// constructor
MyController::MyController():
turn_cnt_(0)
,goal_distance_(0.0)
,diff_angle_(0.0)
,tolerance_angle_(15.0)     // [deg]
,tolerance_distance_(0.1)   // [m]
,path_pointer_(0)
,state_(WAIT)
{
  std::cout << "MyController() called!!" << std::endl;
  ros::NodeHandle nh;

  goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &MyController::goalCallback, this);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &MyController::odomCallback, this);
  path_sub_ = nh.subscribe<nav_msgs::Path>("/coron/path", 10, &MyController::pathCallback, this);


  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10, true);


  // timer at ??hz
  timer_ = nh.createTimer(ros::Duration(3.0), boost::bind(&MyController::nav, this));


  // テスト
  double ref_angle, robot_yaw, diff_angle;

  ref_angle = deg2rad(-150);
  robot_yaw = deg2rad(150);
  diff_angle = angleCorrecton( ref_angle, robot_yaw );
  printf("ref[deg]: %f\n", rad2deg(ref_angle));
  printf("robot_yaw[deg]: %f\n", rad2deg(robot_yaw));
  printf("diff[deg]: %f\n\n", rad2deg(diff_angle));

  ref_angle = deg2rad(150);
  robot_yaw = deg2rad(-150);
  diff_angle = angleCorrecton( ref_angle, robot_yaw );
  printf("ref[deg]: %f\n", rad2deg(ref_angle));
  printf("robot_yaw[deg]: %f\n", rad2deg(robot_yaw));
  printf("diff[deg]: %f\n\n", rad2deg(diff_angle));

  ref_angle = deg2rad(50);
  robot_yaw = deg2rad(-50);
  diff_angle = angleCorrecton( ref_angle, robot_yaw );
  printf("ref[deg]: %f\n", rad2deg(ref_angle));
  printf("robot_yaw[deg]: %f\n", rad2deg(robot_yaw));
  printf("diff[deg]: %f\n\n", rad2deg(diff_angle));

  ref_angle = deg2rad(-50);
  robot_yaw = deg2rad(50);
  diff_angle = angleCorrecton( ref_angle, robot_yaw );
  printf("ref[deg]: %f\n", rad2deg(ref_angle));
  printf("robot_yaw[deg]: %f\n", rad2deg(robot_yaw));
  printf("diff[deg]: %f\n\n", rad2deg(diff_angle));

  ref_angle = deg2rad(40);
  robot_yaw = deg2rad(50);
  diff_angle = angleCorrecton( ref_angle, robot_yaw );
  printf("ref[deg]: %f\n", rad2deg(ref_angle));
  printf("robot_yaw[deg]: %f\n", rad2deg(robot_yaw));
  printf("diff[deg]: %f\n\n", rad2deg(diff_angle));

  ref_angle = deg2rad(-50);
  robot_yaw = deg2rad(-40);
  diff_angle = angleCorrecton( ref_angle, robot_yaw );
  printf("ref[deg]: %f\n", rad2deg(ref_angle));
  printf("robot_yaw[deg]: %f\n", rad2deg(robot_yaw));
  printf("diff[deg]: %f\n\n", rad2deg(diff_angle));

 // getchar();
}


void MyController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
 // std::cout << *msg << std::endl;
  odom_ = *msg;
}


void MyController::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  //std::cout << *msg << std::endl;
  path_ = *msg;


  // 経路はいってる？  最低1回は動いてる？
  if( path_.poses.size() > 1 && (state_ == MOVING || state_ == WAIT ))
  {
    printf("start navigation!\n");
    path_pointer_ = 1;
    goal_.pose.position.x = path_.poses[path_pointer_].pose.position.x;
    goal_.pose.position.y = path_.poses[path_pointer_].pose.position.y;

    if(state_ == WAIT)
      turn_cnt_ = 0;

    state_ = MOVE;
  }
}


void MyController::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//  std::cout << *msg << std::endl;
  goal_ori_ = *msg;
}


MyController::~MyController()
{
  std::cout << "~MyController() called!" << std::endl;
}


void MyController::calcDirecAndDist()
{
  geometry_msgs::Vector3 pos;
  double ref_angle;

  pos.x = goal_.pose.position.x - odom_.pose.pose.position.x;
  pos.y = goal_.pose.position.y - odom_.pose.pose.position.y;

  goal_distance_ = sqrt( pos.x*pos.x + pos.y*pos.y );

  double robot_yaw = convertPose2Yaw( odom_.pose.pose );

  if( state_ == FIT_POSE )
  {
    printf("now pose fitting!\n");
    ref_angle = convertPose2Yaw( goal_ori_.pose );
  }
  else
  {
    ref_angle = atan2( pos.y, pos.x );
  }


  // 目標角度が反対の象限にいる場合の補正
  diff_angle_ = angleCorrecton( ref_angle, robot_yaw );

  printf("goal dist[cm]: %f\n", goal_distance_*100);
  printf("ref[deg]: %f\n", rad2deg(ref_angle));
  printf("robot_yaw[deg]: %f\n", rad2deg(robot_yaw));
  printf("diff[deg]: %f\n\n", rad2deg(diff_angle_));

//  std::cout << "distance[m]: " << goal_distance_ << std::endl;
//  std::cout << "diff_angle[deg]: " << diff_angle_ / 3.141592 * 180.0 << std::endl;
//  std::cout << "robot yaw[deg]: " << robot_yaw / 3.141592 * 180.0 << std::endl;
//  std::cout <<  std::endl;
}


double MyController::angleCorrecton( double ref, double yaw )
{
  double angle;
  angle = ref -yaw;
  if( angle < -M_PI )
    angle = angle + 2 * M_PI;
  else if( angle > M_PI)
    angle = angle - 2 * M_PI;
  return angle;
}


double MyController::convertPose2Yaw( geometry_msgs::Pose pose )
{
  double roll, pitch, yaw;
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}


bool MyController::isGoal()
{
  if( goal_distance_ < tolerance_distance_
      && goal_distance_ > -tolerance_distance_ )
  {
    return true;
  }
  return false;
}


void MyController::move()
{
  calcDirecAndDist(); // pathとodomから進行方向と距離を算出する

  if( isGoal() )  // 位置があっているかだけを見る
  {
    // パスの終点についた
    if( path_pointer_ == path_.poses.size()-1)
    {
      //std::cout << "goal arrival. next fit goal pose!" << std::endl;
      state_ = FIT_POSE;  // 旋回状態へ移行
      fitPose();
    }
    else
    {
      setNextPath();

      // 一度も動いてないなら次のmoveにすぐ移行する
      if(state_ == MOVE)
        move();
    }
  }
  // ゴールにまだついてない場合
  else
  {
    geometry_msgs::Twist command;
    command = turn(); // calcDirecAndDist()で設定された角度に達しているか見て、動く

    if( command.angular.z == 0 )  // 旋回になにも設定されてなければ直進を指定
    {
      command.linear.x = 1;
      std::cout << "forward!!" << std::endl;
      turn_cnt_ = 0;
    }
    else
    {
      turn_cnt_++;  // 連続でターンし続けている
      if(turn_cnt_ > 7)  // とりあえず連続でターンを検知したら
      {
        printf("force set next path!\n");
        turn_cnt_ = 0;
        setNextPath();
      }
    }

    //std::cout <<  std::endl;
    vel_pub_.publish(command);

    // 1回動いたよ
    state_ = MOVING;
  }
}


void MyController::setNextPath()
{
  if(path_pointer_ < path_.poses.size()-1)
  {
    path_pointer_++;
    goal_.pose.position.x = path_.poses[path_pointer_].pose.position.x;
    goal_.pose.position.y = path_.poses[path_pointer_].pose.position.y;
  }
  else
  {
    printf("over last path!\n");
  }
}


// ゴール位置での姿勢合わせ
void MyController::fitPose()
{
  calcDirecAndDist(); // pathとodomから進行方向と距離を算出する
  geometry_msgs::Twist command = turn();

  //もう旋回する必要がなければ終わり
  if( command.angular.z == 0 )
  {
   // std::cout << "goal arrival. migration wait state." << std::endl;
    state_ = WAIT;
  }
  else
  {
    vel_pub_.publish(command);
  }
}


geometry_msgs::Twist MyController::turn()
{
  geometry_msgs::Twist command;
  if( diff_angle_ > deg2rad(tolerance_angle_) )
  {
    command.angular.z = 1;
    std::cout << "left!!" << std::endl;
  }
  else if( diff_angle_ < deg2rad(-tolerance_angle_) )
  {
    command.angular.z = -1;
    std::cout << "right!!" << std::endl;
  }
  return command;
}


void MyController::nav()
{
  switch(state_)
  {
  case MOVE:
  case MOVING:
    move();
    break;
  case FIT_POSE:
    fitPose();
    break;
  default:
    //  printf("waiting!\n");
    ;
  }
}


double MyController::deg2rad( double deg ){return deg*3.141592/180.0;}
double MyController::rad2deg( double rad ){return rad/3.141592*180.0;}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "coron_teleop");
  MyController cc;

  ros::spin();
}
