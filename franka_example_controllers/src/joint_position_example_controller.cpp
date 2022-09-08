// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0.0, 0.0, 0, -M_PI_2, 0.0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.3) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  return true;
}

void JointPositionExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  std::array<double, 7> end_pose_ = {0.0, 0.0, 0, -M_PI_2, 0.0, M_PI_2, M_PI_4};
  elapsed_time_ = ros::Duration(0.0);
}

void JointPositionExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  ros::Duration duration = ros::Duration(50+60);
  // ROS_ERROR("Elapsed Time: %f", elapsed_time_.toSec());
  double T = 10.0;

  double delta_angle = M_PI / 48 * (std::sin(6*M_PI  * elapsed_time_.toSec())) * 0.3;
  // double omega = 0.6*M_PI + (6*M_PI-0.6*M_PI)  * (elapsed_time_.toSec()/10);
  // double delta_angle_var = M_PI / 48 * (std::sin(omega * elapsed_time_.toSec())) * 0.3;

  // ROS_ERROR("Time: %f, Delta Angle: %f, Omega: %f", elapsed_time_.toSec(), delta_angle_var, omega);

  double omega_1 = 0.6*M_PI + (6*M_PI-0.6*M_PI)  * (elapsed_time_.toSec()/10);
  double delta_angle_var_1 = M_PI / 48 * (std::sin(omega_1 * elapsed_time_.toSec())) * 0.2;

  double omega_2 = 0.6*M_PI + (6*M_PI-0.6*M_PI)  * ((elapsed_time_.toSec()-1.5*T)/10);
  double delta_angle_var_2 = M_PI / 48 * (std::sin(omega_2 * (elapsed_time_.toSec()-1.5*T))) * 0.1;

  double omega_3 = 0.6*M_PI + (6*M_PI-0.6*M_PI)  * ((elapsed_time_.toSec()-3*T)/10);
  double delta_angle_var_3 = M_PI / 48 * (std::sin(omega_3 * (elapsed_time_.toSec()-3*T))) * 0.3;

  if (elapsed_time_.toSec() <= T){
    ROS_ERROR("Time: %f, Delta Angle: %f, Omega: %f", elapsed_time_.toSec(), delta_angle_var_1, omega_1);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle_var_1);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  }
  else if (elapsed_time_.toSec() <= 1.5*T && elapsed_time_.toSec() > T){
    ROS_ERROR("Time: %f", elapsed_time_.toSec());
    for (size_t i = 0; i < 7; ++i){
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
  }
  else if (elapsed_time_.toSec() <= 2.5*T)
  {
    
    ROS_ERROR("Time: %f, Delta Angle: %f, Omega: %f", elapsed_time_.toSec(), delta_angle_var_2, omega_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 3) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle_var_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  }
  else if (elapsed_time_.toSec() <= 3*T)
  {
    ROS_ERROR("Time: %f", elapsed_time_.toSec());
    for (size_t i = 0; i < 7; ++i) {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
  }
  else if (elapsed_time_.toSec() <= 4*T)
  {
    ROS_ERROR("Time: %f, Delta Angle: %f, Omega: %f", elapsed_time_.toSec(), delta_angle_var_3, omega_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle_var_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  }
  else if (elapsed_time_.toSec() < (5*T+6*T))
  {
    ROS_ERROR("Time: %f", elapsed_time_.toSec());
    for (size_t i = 0; i < 7; ++i) {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
    }
  }
  else if (elapsed_time_.toSec() == (5*T+6*T))
  {
    ROS_ERROR("Time: %f", elapsed_time_.toSec());
    for (size_t i = 0; i < 7; ++i) {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
    }
    elapsed_time_ = elapsed_time_ - duration;
  }
  
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
