/*
 * Copyright (c) 2013, Yujin Robot.
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
 *     * Neither the name of Yujin Robot nor the names of its
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

/**
 * @author Marcus Liebhardt
 *
 * This work has been inspired by Nate Koenig's Gazebo plugin for the iRobot Create.
 */

#include "kobuki_gazebo_plugins/gazebo_ros_kobuki.h"


namespace gazebo {


void GazeboRosKobuki::updateJointState()
{
  /*
   * Joint states
   */
  std::string baselink_frame = gazebo_ros_->resolveTF("base_link");
  joint_state_.header.stamp = ros::Time::now();
  joint_state_.header.frame_id = baselink_frame;
  joint_state_.position[LEFT] = joints_[LEFT]->GetAngle(0).Radian();
  joint_state_.velocity[LEFT] = joints_[LEFT]->GetVelocity(0);
  joint_state_.position[RIGHT] = joints_[RIGHT]->GetAngle(0).Radian();
  joint_state_.velocity[RIGHT] = joints_[RIGHT]->GetVelocity(0);

  joint_state_pub_.publish(joint_state_);
}


/*
 * Odometry (encoders & IMU)
 */
void GazeboRosKobuki::updateOdometry(common::Time& step_time)
{
  std::string odom_frame = gazebo_ros_->resolveTF("odom");
  std::string base_frame = gazebo_ros_->resolveTF("base_footprint");
  odom_.header.stamp = joint_state_.header.stamp;
  odom_.header.frame_id = odom_frame;
  odom_.child_frame_id = base_frame;

  // Distance travelled by main wheels
  double d1, d2;
  double dr, da;
  d1 = d2 = 0;
  dr = da = 0;

  // 多分ここがencoder 直接速度を取得してる
  // (wheel_diam_ / 2)= 直径/2= 半径
  // 弧度法なら (theta/2*pi)*radius
  d1 = step_time.Double() * (wheel_diam_ / 2) * joints_[LEFT]->GetVelocity(0);
  d2 = step_time.Double() * (wheel_diam_ / 2) * joints_[RIGHT]->GetVelocity(0);

  // Can see NaN values here, just zero them out if needed
  if (isnan(d1))
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo ROS Kobuki plugin: NaN in d1. Step time: " << step_time.Double()
                             << ", WD: " << wheel_diam_ << ", velocity: " << joints_[LEFT]->GetVelocity(0));
    d1 = 0;
  }
  if (isnan(d2))
  {
    ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo ROS Kobuki plugin: NaN in d2. Step time: " << step_time.Double()
                             << ", WD: " << wheel_diam_ << ", velocity: " << joints_[RIGHT]->GetVelocity(0));
    d2 = 0;
  }

  // 参照
  // http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/wheelrobot.html
  // もし旋回ならd1+d2が0になって、並進距離は0になる？
  dr = (d1 + d2) / 2; // 台車中心の速度

  // wheel_sep_は2車輪の距離
  da = (d2 - d1) / wheel_sep_; // 台車の旋回速度

//  std::cout << "omega[rad/s]: " << da << std::endl;

  // imuから直接旋回速度を取得してる
  // Just as in the Kobuki driver, the angular velocity is taken directly from the IMU
//  vel_angular_ = imu_->GetAngularVelocity();

  // Compute odometric pose
  odom_pose_[0] += dr * cos( odom_pose_[2] );
  odom_pose_[1] += dr * sin( odom_pose_[2] );

  // original
//  odom_pose_[2] += vel_angular_.z * step_time.Double();

  // 車輪速度から算出した、旋回速度。の積分
  odom_pose_[2] += da;

  // Compute odometric instantaneous velocity
  odom_vel_[0] = dr / step_time.Double();
  odom_vel_[1] = 0.0;
//  odom_vel_[2] = vel_angular_.z;

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  tf::Quaternion qt;
  qt.setEuler(0,0,odom_pose_[2]);
  odom_.pose.pose.orientation.x = qt.getX();
  odom_.pose.pose.orientation.y = qt.getY();
  odom_.pose.pose.orientation.z = qt.getZ();
  odom_.pose.pose.orientation.w = qt.getW();

//  odom_.pose.covariance[0]  = 0;  // x
//  odom_.pose.covariance[7]  = 0;  // y
//  odom_.pose.covariance[35] = 0 ; // yaw
  odom_.pose.covariance[0]  = 0.1;  // x
  odom_.pose.covariance[7]  = 0.1;  // y
  odom_.pose.covariance[35] = 0.05 ; // yaw
//  odom_.pose.covariance[35] = 0.05; // yaw
  odom_.pose.covariance[14] = 1e6;
  odom_.pose.covariance[21] = 1e6;
  odom_.pose.covariance[28] = 1e6;

  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.linear.y = 0;
  odom_.twist.twist.linear.z = 0;
  odom_.twist.twist.angular.x = 0;
  odom_.twist.twist.angular.y = 0;
  odom_.twist.twist.angular.z = odom_vel_[2];
  odom_pub_.publish(odom_); // publish odom message

  if (publish_tf_)
  {
    odom_tf_.header = odom_.header;
    odom_tf_.child_frame_id = odom_.child_frame_id;
    odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
    odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
    odom_tf_.transform.translation.z = odom_.pose.pose.position.z;
    odom_tf_.transform.rotation = odom_.pose.pose.orientation;
    tf_broadcaster_.sendTransform(odom_tf_);
  }
}



/*
 * Propagate velocity commands
 * TODO: Check how to simulate disabled motors, e.g. set MaxForce to zero, but then damping is important!
 */
void GazeboRosKobuki::propagateVelocityCommands()
{
  if (((prev_update_time_- last_cmd_vel_time_).Double() > cmd_vel_timeout_) || !motors_enabled_)
  {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }
  joints_[LEFT]->SetVelocity(0, wheel_speed_cmd_[LEFT] / (wheel_diam_ / 2.0));
  joints_[RIGHT]->SetVelocity(0, wheel_speed_cmd_[RIGHT] / (wheel_diam_ / 2.0));
  joints_[LEFT]->SetMaxForce(0, torque_);
  joints_[RIGHT]->SetMaxForce(0, torque_);
}




}
