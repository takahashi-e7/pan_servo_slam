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

namespace gazebo
{

/*
 * Prepare receiving motor power commands
 */
void GazeboRosKobuki::prepareMotorPower() {
  motors_enabled_ = true;
}

/*
 * Prepare joint state publishing
 */
bool GazeboRosKobuki::prepareJointState()
{
  if (sdf_->HasElement("left_wheel_joint_name"))
  {
    left_wheel_joint_name_ = sdf_->GetElement("left_wheel_joint_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find left wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("right_wheel_joint_name"))
  {
    right_wheel_joint_name_ = sdf_->GetElement("right_wheel_joint_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find right wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return false;
  }
  joints_[LEFT] = model_->GetJoint(left_wheel_joint_name_);
  joints_[RIGHT] = model_->GetJoint(right_wheel_joint_name_);
  if (!joints_[LEFT] || !joints_[RIGHT])
  {
    ROS_ERROR_STREAM("Couldn't find specified wheel joints in the model! [" << node_name_ <<"]");
    return false;
  }
  joint_state_.header.frame_id = "Joint States";
  joint_state_.name.push_back(left_wheel_joint_name_);
  joint_state_.position.push_back(0);
  joint_state_.velocity.push_back(0);
  joint_state_.effort.push_back(0);
  joint_state_.name.push_back(right_wheel_joint_name_);
  joint_state_.position.push_back(0);
  joint_state_.velocity.push_back(0);
  joint_state_.effort.push_back(0);
  
  return true;
}

/*
 * Prepare publishing odometry data
 */
void GazeboRosKobuki::preparePublishTf()
{
  if (sdf_->HasElement("publish_tf"))
  {
    publish_tf_ = sdf_->GetElement("publish_tf")->Get<bool>();
    if (publish_tf_)
    {
      ROS_INFO_STREAM("Will publish tf." << " [" << node_name_ <<"]");
    }
    else
    {
      ROS_INFO_STREAM("Won't publish tf." << " [" << node_name_ <<"]");
    }
  }
  else
  {
    publish_tf_ = false;
    ROS_INFO_STREAM("Couldn't find the 'publish tf' parameter in the model description."
                     << " Won't publish tf." << " [" << node_name_ <<"]");
  }
}


bool GazeboRosKobuki::prepareWheelAndTorque()
{
  if (sdf_->HasElement("wheel_separation"))
  {
    wheel_sep_ = sdf_->GetElement("wheel_separation")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the wheel separation parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("wheel_diameter"))
  {
    wheel_diam_ = sdf_->GetElement("wheel_diameter")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the wheel diameter parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("torque"))
  {
    torque_ = sdf_->GetElement("torque")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the torque parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  return true;
}


void GazeboRosKobuki::prepareOdom()
{
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
}


/*
 * Prepare receiving velocity commands
 */
bool GazeboRosKobuki::prepareVelocityCommand()
{
  if (sdf_->HasElement("velocity_command_timeout"))
  {
    cmd_vel_timeout_ = sdf_->GetElement("velocity_command_timeout")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the wheel separation parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  last_cmd_vel_time_ = world_->GetSimTime();
  return true;
}



void GazeboRosKobuki::setupRosApi(std::string& model_name)
{
  std::string base_prefix;
  gazebo_ros_->node()->param("base_prefix", base_prefix, std::string("mobile_base"));

  // Public topics

  // joint_states
  std::string joint_states_topic = "joint_states";
  joint_state_pub_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>(joint_states_topic, 1);
  ROS_INFO("%s: Advertise joint_states[%s]!", gazebo_ros_->info(), joint_states_topic.c_str());

  // odom
  std::string odom_topic = "odom";
  odom_pub_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odom_topic, 1);
  ROS_INFO("%s: Advertise Odometry[%s]!", gazebo_ros_->info(), odom_topic.c_str());


  // Private Topics
  // motor power
  std::string motor_power_topic = base_prefix + "/commands/motor_power";
  motor_power_sub_ = gazebo_ros_->node()->subscribe(motor_power_topic, 10, &GazeboRosKobuki::motorPowerCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), motor_power_topic.c_str());

  
  std::string odom_reset_topic = base_prefix + "/commands/reset_odometry";
  odom_reset_sub_ = gazebo_ros_->node()->subscribe(odom_reset_topic, 10, &GazeboRosKobuki::resetOdomCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), odom_reset_topic.c_str());

  // cmd_vel
  std::string cmd_vel = base_prefix + "/commands/velocity";
  cmd_vel_sub_ = gazebo_ros_->node()->subscribe(cmd_vel, 100, &GazeboRosKobuki::cmdVelCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), motor_power_topic.c_str());

}
}
