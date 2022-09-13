// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

/*
Joint Position Controller used for testing IMU Sensors. Joints 4, 5 and 6 are individually controlled 
with a frequency varying sine wave to create a vibratory motion in 3 axes.
*/

namespace franka_example_controllers {

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware, // Initialisation of controller
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

  std::array<double, 7> q_start{{0.0, 0.0, 0, -M_PI_2, 0.0, M_PI_2, M_PI_4}}; // Testing for starting pose of the robot to ensure it will not move excessively
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.3) { // 0.3 was originally 0.1, changed as the franka_ros move to start function does not work (need to program one myself)
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  return true;
}

void JointPositionExampleController::starting(const ros::Time& /* time */) { // Controller start, receives and stores initial position of the robot
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  std::array<double, 7> end_pose_ = {0.0, 0.0, 0, -M_PI_2, 0.0, M_PI_2, M_PI_4}; // not needed or used, can be used as joint positions to achieve a 90 bend in robot
  elapsed_time_ = ros::Duration(0.0); // Sets timer to be used to 0
}

void JointPositionExampleController::update(const ros::Time& /*time*/, // Update function to loop continuously to loop through
                                            const ros::Duration& period) {
  elapsed_time_ += period; // Add the time to update with the elapsed time, acts as a timer
  ros::Duration duration = ros::Duration(42.6*3+5*60); // Value set for the duration of the test to reset the timer after the 3 axis movements
  // ros::Duration duration = ros::Duration(42.6*3+10); // Value set for the duration of the test to reset the timer after the 3 axis movements

  double T = 10.0; // unused

  double delta_angle = M_PI / 48 * (std::sin(4*M_PI  * elapsed_time_.toSec())) * 0.2; // unused

  /*
  
  */
  double amplitude = 3 * M_PI / 180;
  
  double w_1 = amplitude * (std::sin(0.20 * 2 * M_PI * (elapsed_time_.toSec())));
  double w_2 = amplitude * (std::sin(0.50 * 2 * M_PI * (-15.0 + elapsed_time_.toSec())));
  double w_3 = amplitude * (std::sin(1.00 * 2 * M_PI * (-23.0 + elapsed_time_.toSec())));
  double w_4 = amplitude * (std::sin(1.25 * 2 * M_PI * (-28.0 + elapsed_time_.toSec())));
  double w_5 = amplitude * (std::sin(2.00 * 2 * M_PI * (-32.8 + elapsed_time_.toSec())));
  double w_6 = amplitude * (std::sin(2.50 * 2 * M_PI * (-37.8 + elapsed_time_.toSec())));
  double w_7 = amplitude * (std::sin(3.20 * 2 * M_PI * (-42.6 + elapsed_time_.toSec())));

  double w_1_2 = amplitude * (std::sin(0.20 * 2 * M_PI * (elapsed_time_.toSec())));
  double w_2_2 = amplitude * (std::sin(0.50 * 2 * M_PI * (-15.0 - 42.6 + elapsed_time_.toSec())));
  double w_3_2 = amplitude * (std::sin(1.00 * 2 * M_PI * (-23.0 - 42.6 + elapsed_time_.toSec())));
  double w_4_2 = amplitude * (std::sin(1.25 * 2 * M_PI * (-28.0 - 42.6 + elapsed_time_.toSec())));
  double w_5_2 = amplitude * (std::sin(2.00 * 2 * M_PI * (-32.8 - 42.6 + elapsed_time_.toSec())));
  double w_6_2 = amplitude * (std::sin(2.50 * 2 * M_PI * (-37.8 - 42.6 + elapsed_time_.toSec())));
  double w_7_2 = amplitude * (std::sin(3.20 * 2 * M_PI * (-42.6 - 42.6 + elapsed_time_.toSec())));

  double w_1_3 = amplitude * (std::sin(0.20 * 2 * M_PI * (elapsed_time_.toSec())));
  double w_2_3 = amplitude * (std::sin(0.50 * 2 * M_PI * (-15.0 - 42.6 - 42.6 + elapsed_time_.toSec())));
  double w_3_3 = amplitude * (std::sin(1.00 * 2 * M_PI * (-23.0 - 42.6 - 42.6 + elapsed_time_.toSec())));
  double w_4_3 = amplitude * (std::sin(1.25 * 2 * M_PI * (-28.0 - 42.6 - 42.6 + elapsed_time_.toSec())));
  double w_5_3 = amplitude * (std::sin(2.00 * 2 * M_PI * (-32.8 - 42.6 - 42.6 + elapsed_time_.toSec())));
  double w_6_3 = amplitude * (std::sin(2.50 * 2 * M_PI * (-37.8 - 42.6 - 42.6 + elapsed_time_.toSec())));
  double w_7_3 = amplitude * (std::sin(3.20 * 2 * M_PI * (-42.6 - 42.6 - 42.6 + elapsed_time_.toSec())));

  if (elapsed_time_.toSec() <= 15){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_1);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_1);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 23){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 28){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 32.8){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_4);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_4);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 37.8){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_5);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_5);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_6);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_6);if (elapsed_time_.toSec() <= 15){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_1);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_1);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 23){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 28){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 32.8){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_4);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_4);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 37.8){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_5);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_5);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_6);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_6);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 15+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_1_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_1_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 23+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_2_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_2_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 28+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_3_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_3_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 32.8+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_4_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_4_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 37.8+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_5_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_5_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 42.6+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_6_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_6_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 15+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_1_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_1_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 23+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_2_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_2_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 28+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_3_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_3_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 32.8+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_4_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_4_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 37.8+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_5_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_5_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 42.6+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_6_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_6_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() < (42.6*3+5*60))
  {
    ROS_ERROR("Time: %f", elapsed_time_.toSec());
    for (size_t i = 0; i < 7; ++i) {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
    }
  }
  else if (elapsed_time_.toSec() == (42.6*3+5*60))
  {
    ROS_ERROR("Time: %f", elapsed_time_.toSec());
    for (size_t i = 0; i < 7; ++i) {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
    }
    elapsed_time_ = elapsed_time_ - duration;
  }
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 15+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_1_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_1_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 23+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_2_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_2_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 28+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_3_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_3_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 32.8+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_4_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_4_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 37.8+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_5_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_5_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 42.6+42.6){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_6_2);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_6_2);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 15+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_1_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_1_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 23+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_2_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_2_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 28+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_3_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_3_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 32.8+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_4_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_4_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 37.8+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_5_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_5_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() <= 42.6+42.6*2){
    ROS_ERROR("Time: %f, Delta Angle: %f", elapsed_time_.toSec(), w_6_3);
    for (size_t i = 0; i < 7; ++i) {
      if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + w_6_3);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
  } else if (elapsed_time_.toSec() < (42.6*3+5*60))
  {
    ROS_ERROR("Time: %f", elapsed_time_.toSec());
    for (size_t i = 0; i < 7; ++i) {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
    }
  }
  else if (elapsed_time_.toSec() == (42.6*3+5*60))
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
