/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 */

#ifndef GAZEBO_ROS_TEMPLATE_HH
#define GAZEBO_ROS_TEMPLATE_HH

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <boost/bind.hpp>

namespace gazebo
{

class GazeboRosTemplate : public ModelPlugin
{
/// \brief Constructor
public:
  GazeboRosTemplate();
  virtual ~GazeboRosTemplate();
  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

protected:
  virtual void UpdateChild(const common::UpdateInfo & /*_info*/);

private:
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  event::ConnectionPtr updateConnection_;
  physics::JointPtr cj_;
  ros::Publisher js_pub_;
  ros::Subscriber scan_sub_;
  sensor_msgs::JointState js_;

  double servo_angle_;
  double direction_;
  double cnt_;
};

}

#endif

