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
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.2) { // 0.3 was originally 0.1, changed as the franka_ros move to start function does not work (need to program one myself)
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
  // centre_arm();
}

void JointPositionExampleController::update(const ros::Time& /*time*/, // Update function to loop continuously to loop through
                                            const ros::Duration& period) {
  elapsed_time_ += period; // Add the time to update with the elapsed time, acts as a timer
  double t = elapsed_time_.toSec();
  double g_out;
  ros::Duration duration = ros::Duration(63.6);
  double amplitude = 2 * M_PI / 180; // Amplitude of sine wave applied to joints
  double f_1 = 1;
  double f_2 = 3.2;
  // double t0 = 0;
  // double t1 = 0.5/f_1;
  // double t2 = 2.5/f_1;
  // double t3 = 2.5/f_1 + 0.5/f_1 + 0.5/f_2;
  // double t4 = 100;
  double joint = 4 + counter;

  if (t > 0.1 && t < 1.5){
    g_out = 6.564226870562092*pow(t,4) - 10.260633832949697*pow(t,5) + 5.564142412461603*pow(t,6) - 1.0384395655145722*pow(t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  } 
  else if (t >= 1.5 && t < 13.5){
    g_out = std::sin(0.2*2*M_PI*t);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 13.5 && t < 15.6){
    g_out = -0.9510565162951535 - 0.38832220774509346*(-13.5 + t) + 0.7509241263129091*pow(-13.5 + t,2) + 0.10220230854937896*pow(-13.5 + t,3) - 2.144867526865293*pow(-13.5 + t,4) + 3.184480343195979*pow(-13.5 + t,5) - 1.5748818732466023*pow(-13.5 + t,6) + 0.25088845758123446*pow(-13.5 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 15.6 && t < 22.4){
    g_out = std::sin(0.5*2*M_PI*(-15. + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 22.4 && t < 23.3){
    g_out = -0.9510565162951522 - 0.9708055193627463*(-22.4 + t) + 4.693275789455673*pow(-22.4 + t,2) + 1.5969110710840666*pow(-22.4 + t,3) - 27.06321478681908*pow(-22.4 + t,4) + 95.8472404146695*pow(-22.4 + t,5) - 119.08173622858041*pow(-22.4 + t,6) + 46.5284045369398*pow(-22.4 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 23.3 && t < 27.7){
    g_out = std::sin(2*M_PI*(-23. + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 27.7 && t < 28.24){
    g_out = -0.9510565162951522 - 1.9416110387254926*(-27.7 + t) + 18.773103157822693*pow(-27.7 + t,2) + 12.775288568672533*pow(-27.7 + t,3) - 26.85234046280939*pow(-27.7 + t,4) - 108.4794238173035*pow(-27.7 + t,5) + 93.17322998471447*pow(-27.7 + t,6) + 31.274440773051055*pow(-27.7 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 28.24 && t < 32.56){
    g_out = std::sin(1.25*2*M_PI*(-28. + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 32.56 && t < 32.95){
    g_out = -0.9510565162951488 - 2.4270137984069486*(-32.559999999999995 + t) + 29.332973684097855*pow(-32.559999999999995 + t,2) + 24.951735485689397*pow(-32.559999999999995 + t,3) - 237.08002876368704*pow(-32.559999999999995 + t,4) + 1636.5435300366235*pow(-32.559999999999995 + t,5) - 5910.279867574944*pow(-32.559999999999995 + t,6) + 6137.73476460005*pow(-32.559999999999995 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 32.95 && t < 37.65){
    g_out = std::sin(2*2*M_PI*(-32.8 + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 37.65 && t < 37.92){
    g_out = -0.9510565162951591 - 3.88322207745072*(-37.65 + t) + 75.09241263129132*pow(-37.65 + t,2) + 102.2023085493733*pow(-37.65 + t,3) - 429.6374474054191*pow(-37.65 + t,4) - 3471.341562149265*pow(-37.65 + t,5) + 5963.086719007836*pow(-37.65 + t,6) + 4003.1284189651624*pow(-37.65 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 37.92 && t < 42.48){
    g_out = std::sin(2.5*2*M_PI*(-37.8 + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 42.48 && t < 42.6937){
    g_out = -0.951056516295166 - 4.854027596813065*(-42.48 + t) + 117.33189473639354*pow(-42.48 + t,2) + 199.6138838854809*pow(-42.48 + t,3) - 1036.4336600911242*pow(-42.48 + t,4) - 9235.665318581134*pow(-42.48 + t,5) + 10474.804815529935*pow(-42.48 + t,6) + 44878.21835804757*pow(-42.48 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 42.6937 && t < 47.5062){
    g_out = std::sin(20.106192982974676*(-42.599999999999994 + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 47.5062 && t < 47.75){
    g_out = -0.9510565162951535 - 6.213155323921495*(-47.506249999999994 + t) + 192.23657633610472*pow(-47.506249999999994 + t,2) + 418.62065581825624*pow(-47.506249999999994 + t,3) - 14501.979321017228*pow(-47.506249999999994 + t,4) + 77710.25311290205*pow(-47.506249999999994 + t,5) - 181961.2562184296*pow(-47.506249999999994 + t,6) + 164758.56303424964*pow(-47.506249999999994 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 42.48 && t < 42.6937){
    g_out = -0.951056516295166 - 4.854027596813065*(-42.48 + t) + 117.33189473639354*pow(-42.48 + t,2) + 199.6138838854809*pow(-42.48 + t,3) - 1036.4336600911242*pow(-42.48 + t,4) - 9235.665318581134*pow(-42.48 + t,5) + 10474.804815529935*pow(-42.48 + t,6) + 44878.21835804757*pow(-42.48 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 42.6937 && t < 47.5062){
    g_out = std::sin(20.106192982974676*(-42.599999999999994 + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 47.5062 && t < 47.75){
    g_out = -0.9510565162951535 - 6.213155323921495*(-47.506249999999994 + t) + 192.23657633610472*pow(-47.506249999999994 + t,2) + 418.62065581825624*pow(-47.506249999999994 + t,3) - 14501.979321017228*pow(-47.506249999999994 + t,4) + 77710.25311290205*pow(-47.506249999999994 + t,5) - 181961.2562184296*pow(-47.506249999999994 + t,6) + 164758.56303424964*pow(-47.506249999999994 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 47.75 && t < 50.45){
    g_out = std::sin(4*M_PI*(-47.599999999999994 + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 50.45 && t < 50.9){
    g_out = -0.9510565162951591 - 3.88322207745072*(-50.449999999999996 + t) + 75.09241263129132*pow(-50.449999999999996 + t,2) + 102.2023085493733*pow(-50.449999999999996 + t,3) - 2313.2562185913985*pow(-50.449999999999996 + t,4) + 7816.12883051807*pow(-50.449999999999996 + t,5) - 11139.02159066553*pow(-50.449999999999996 + t,6) + 5955.635780728658*pow(-50.449999999999996 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 50.9 && t < 56.3){
    g_out = std::sin(2*M_PI*(-50.599999999999994 + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 56.3 && t < 57.2){
    g_out = -0.9510565162951591 - 1.94161103872536*(-56.3 + t) + 18.77310315782283*pow(-56.3 + t,2) + 12.775288568671662*pow(-56.3 + t,3) - 144.57851366196132*pow(-56.3 + t,4) + 244.25402595368644*pow(-56.3 + t,5) - 174.04721235414576*pow(-56.3 + t,6) + 46.52840453694161*pow(-56.3 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 57.2 && t < 58){
    g_out = std::sin(3.141592653589793*(-56.599999999999994 + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 58 && t < 60.1){
    g_out = -0.9510565162951522 - 0.9708055193627463*(-57.99999999999999 + t) + 4.693275789455673*pow(-57.99999999999999 + t,2) + 1.5969110710840666*pow(-57.99999999999999 + t,3) - 8.435470359523158*pow(-57.99999999999999 + t,4) + 6.575748796886896*pow(-57.99999999999999 + t,5) - 2.113178453197536*pow(-57.99999999999999 + t,6) + 0.25088845758123324*pow(-57.99999999999999 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 60.1 && t < 62.1){
    g_out = std::sin(1.2566370614359172*(-58.599999999999994 + t));
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t > 62.1 && t < 63.6){
    g_out = -0.9510565162951535 - 0.38832220774509346*(-62.099999999999994 + t) + 0.7509241263129091*pow(-62.099999999999994 + t,2) + 0.10220230854937896*pow(-62.099999999999994 + t,3) + 5.2663941323903405*pow(-62.099999999999994 + t,4) - 9.249621591358798*pow(-62.099999999999994 + t,5) + 5.3394730254414045*pow(-62.099999999999994 + t,6) - 1.0384395655145722*pow(-62.099999999999994 + t,7);
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
  }
  else if (t >= 63.6){
    g_out = 0;
    update_arm_three((amplitude*g_out));
    ROS_INFO("time: %f, g_out: %f", t, g_out);
    if (counter < 2) {
      counter++;
    } else {
      counter = 0;
    }
    elapsed_time_ = elapsed_time_ - duration;
  }

  // if (t >= t0 && t <= t1){
  //   g_out = amplitude * g_function(t, (0.5/f_1), 0, 0, 0, 2*f_1*M_PI*std::cos(2*M_PI*0.5), 0, 0, 0, 0);
  //   update_arm(g_out, 4);
  //   ROS_INFO("time: %f, g_out: %f", t, g_out);
  // } 
  // else if (t > t1 && t <= t2){
  //   g_out = amplitude * std::sin(2*M_PI*f_1*t);
  //   update_arm(g_out, 4);
  //   ROS_INFO("time: %f, g_out: %f", t, g_out);
  // }
  // else if (t > t2 && t < t3){
  //   g_out = amplitude * g_function(t-2.5/f_1, (0.5/f_1+0.5/f_2), 0, 0, 2*f_1*M_PI*std::cos(2*M_PI*0.5), 2*f_2*M_PI*std::cos(2*M_PI*0.5), 0, 0, 0, 0);
  //   update_arm(g_out, 4);
  //   ROS_INFO("time: %f, g_out: %f", t, g_out);
  // }
  // else if (t > t3 && t <= t4){
  //   g_out = amplitude * std::sin(2*M_PI*f_1*t);
  //   update_arm(g_out, 4);
  //   ROS_INFO("time: %f, g_out: %f", t, g_out);
  // }

}

double JointPositionExampleController::g_function(const double t, const double T,
                                                  const double x0, const double xf,
                                                  const double v0, const double vf,
                                                  const double a0, const double af,
                                                  const double j0, const double jf) {
  double g;

  g = (3*a0*pow(t,2) + j0*pow(t,3) + 6*t*v0 + 6*x0 + (pow(t,7)*(T*(T*(12*a0 - 12*af + (j0 + jf)*T) + 60*(v0 + vf)) + 120*(x0 - xf)))/pow(T,7) +
     (3*pow(t,5)*(T*(T*(20*a0 - 14*af + (2*j0 + jf)*T) + 90*v0 + 78*vf) + 168*(x0 - xf)))/pow(T,5) - (pow(t,4)*(T*(T*(30*a0 - 15*af + (4*j0 + jf)*T) + 30*(4*v0 + 3*vf)) + 210*(x0 - xf)))/pow(T,4) -
     (pow(t,6)*(T*(T*(45*a0 - 39*af + 4*j0*T + 3*jf*T) + 216*v0 + 204*vf) + 420*(x0 - xf)))/pow(T,6))/6;

  return g;
}

void JointPositionExampleController::update_arm(const double g, const double joint) {
  for (size_t i = 0; i < 7; ++i) {
      if (i == joint) {
        position_joint_handles_[i].setCommand(end_pose_[i] + g);
      } else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
}

void JointPositionExampleController::centre_arm(void) {
  for (size_t i = 0; i < 7; ++i) {
      position_joint_handles_[i].setCommand(end_pose_[i]);
    }
}

void JointPositionExampleController::update_arm_three(const double g) {
  for (size_t i = 0; i < 7; ++i) {
      if (i == 4) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + g);
      } 
      else if (i == 5) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + g);
      } 
      else if (i == 6) {
        position_joint_handles_[i].setCommand(initial_pose_[i] + g);
      } 
      else {
        position_joint_handles_[i].setCommand(initial_pose_[i]);
      }
    }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
