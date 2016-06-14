/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Dave Coleman
 *  \desc   Example ROS plugin for Gazebo
 */

#include <gazebo_plugins/gazebo_ros_template.h>
#include <ros/ros.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosTemplate);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTemplate::GazeboRosTemplate()
{
  gzdbg << "template" << std::endl;

//  servo_angle_ = -6.0;
  servo_angle_ = 0.0;
  direction_ = 1.0;
  cnt_ = 50;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTemplate::~GazeboRosTemplate()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosTemplate::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosTemplate::UpdateChild, this, _1));


  // パンサーボの乗っているjoint
  cj_ = _parent->GetJoint("camera_rgb_joint");

  if(cj_ != 0)
  {
    cj_->SetProvideFeedback(true);

    ros::NodeHandle n;
    js_pub_ = n.advertise<sensor_msgs::JointState>("joint_states", 10);

    // 第3引数はクラスへの参照　自クラスならthisでよい
    scan_sub_ = n.subscribe("scan3", 10, &GazeboRosTemplate::laserCallback, this);

    js_.header.frame_id =cj_->GetParent()->GetName();
    js_.name.push_back(cj_->GetName());
    js_.effort.push_back(cj_->GetForce(0));
    js_.position.push_back(cj_->GetAngle(0).Radian());
    js_.velocity.push_back(cj_->GetVelocity(0));
  }
  else
  {
    gzdbg << "cj is null" << std::endl;
  }


}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTemplate::UpdateChild(const common::UpdateInfo & /*_info*/)
{

}


// メインノードより、首振りのトリガとして送られてくるレーザ
void GazeboRosTemplate::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
//  gzdbg << "GazeboRosTemplate::laserCallback" << std::endl;

  js_.header.stamp = ros::Time::now();

  js_.position[0] = 0;    // kinect(LRF)偽装のため、サーボは振ってないことにする
  js_.effort[0] = cj_->GetForce(0);
  js_.velocity[0] = cj_->GetVelocity(0);
  js_pub_.publish(js_);

  servo_angle_ = msg->angle_max;  // ここに今回のサーボの角度が入っている

  cj_->SetAngle(0, servo_angle_ * 3.1415 / 180.0 );

}

}
