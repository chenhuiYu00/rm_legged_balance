//
// Created by yuchen on 23-9-2.
//

#include "rm_legged_balance_control_ros/LeggedBalanceController.h"
#include "rm_legged_balance_control_ros/synchronized_module/RosReferenceManager.h"

#include <angles/angles.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ipm/IpmMpc.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <rm_common/math_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_legged_balance_control/definitions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_list_macros.hpp>

namespace rm {

bool LeggedBalanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  std::string taskFile;
  std::string libFolder;
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/libFolder", libFolder);

  balanceInterface_ = std::make_shared<rm::LeggedBalanceInterface>(taskFile, libFolder);
  balanceInterface_->getLeggedBalanceControlCmd()->setLegCmd(0.12);

  setupMpc(controller_nh);
  setupMrt();

  // Visualization
  ros::NodeHandle nh;
  visualizer_ = std::make_shared<LeggedBalanceVisualization>(nh, false);

  currentObservation_.time = 0.0;
  currentObservation_.state.setZero(STATE_DIM);
  currentObservation_.input.setZero(INPUT_DIM);

  // Hardware interface
  auto* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
  robotStateHandle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  jointHandles_.push_back(effortJointInterface->getHandle("left_wheel_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_wheel_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("left_front_first_leg_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_front_first_leg_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("left_back_first_leg_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_back_first_leg_joint"));
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

  // Odom TF
  odom2base_.header.frame_id = "odom";
  odom2base_.header.stamp = ros::Time::now();
  odom2base_.child_frame_id = "base_link";
  odom2base_.transform.rotation.w = 1;
  tfRtBroadcaster_.init(root_nh);
  tfRtBroadcaster_.sendTransform(odom2base_);

  // Pub leg info
  legLengthPublisher_ = controller_nh.advertise<std_msgs::Float64MultiArray>("pendulum_length", 1);
  legPendulumSupportForce_ = controller_nh.advertise<std_msgs::Float64MultiArray>("support_force", 1);

  // For sit-down
  ros::NodeHandle left = ros::NodeHandle(controller_nh, "left_wheel");
  ros::NodeHandle right = ros::NodeHandle(controller_nh, "right_wheel");
  if (!leftWheelController_.init(effortJointInterface, left) || !rightWheelController_.init(effortJointInterface, right)) {
    return false;
  }
  if (controller_nh.hasParam("pid_follow")) {
    if (!pidFollow_.init(ros::NodeHandle(controller_nh, "pid_follow"))) {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_left_leg")) {
    if (!pidLeftLeg_.init(ros::NodeHandle(controller_nh, "pid_left_leg"))) {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_right_leg")) {
    if (!pidRightLeg_.init(ros::NodeHandle(controller_nh, "pid_right_leg"))) {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_theta_diff")) {
    if (!pidThetaDiff_.init(ros::NodeHandle(controller_nh, "pid_theta_diff"))) {
      return false;
    }
  }
  if (controller_nh.hasParam("pid_roll")) {
    if (!pidRoll_.init(ros::NodeHandle(controller_nh, "pid_roll"))) {
      return false;
    }
  }
  return true;
}

void LeggedBalanceController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);
  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  /*todo: check block
  // Check block
  scalar_t blockAngle = 0.25, blockEffort = 1.5, blockVelocity = 3.0, blockDuration = 0.5;//todo: use param

  if (!blockState_) {
    if (std::abs(currentObservation_.state(1)) > blockAngle &&
        (std::abs(jointHandles_[0].getEffort()) > blockEffort || std::abs(jointHandles_[1].getEffort()) > blockEffort) &&
        (std::abs(jointHandles_[0].getVelocity()) < blockVelocity || std::abs(jointHandles_[1].getVelocity()) < blockVelocity)) {
      if (!maybeBlock_) {
        maybeBlockTime_ = time;
        maybeBlock_ = true;
      }
      if ((time - maybeBlockTime_).toSec() >= blockDuration) {
        blockState_ = true;
        blockStateChanged_ = true;
      }
    } else {
      maybeBlock_ = false;
    }
  }
   */

  // Move joints
  if (!blockState_) {
    normal(time, period);
  } else {
    block(time, period);
  }

  /*todo: check sit down
  // Sit down
  if (balanceInterface_->getLeggedBalanceControlCmd()->getSitDown()) {
    sitDown(time, period);
  }
   */

  // Power limit
  /*
  double limit = balanceInterface_->getLeggedBalanceControlCmd()->getPowerLimit();
  double a = 0., b = 0., c = 0.;  // Three coefficients of a quadratic equation in one variable
  for (const auto& joint : jointHandles_) {
    double cmd_effort = joint.getCommand();
    double real_vel = joint.getVelocity();
    a += square(cmd_effort);
    b += std::abs(cmd_effort * real_vel);
    c += square(real_vel);
  }
  a *= params_.powerCoeffEffort_;
  c = c * params_.powerCoeffVel_ + params_.powerOffset_ - limit;  // offset different from rm_chassis_controller
  double zoom = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
  for (auto joint : jointHandles_) {
    joint.setCommand(zoom > 1 ? joint.getCommand() : joint.getCommand() * zoom);
  }
   */

  // Visualization
  // visualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

  updateTfOdom(time, period);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ocs2::ros_msg_conversions::createObservationMsg(currentObservation_));
}

void LeggedBalanceController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(CONTROL_DIM), jointVel(CONTROL_DIM);
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    jointPos(i) = jointHandles_[i].getPosition();
    jointVel(i) = jointHandles_[i].getVelocity();
  }
  geometry_msgs::Vector3 gyro;
  gyro.x = imuSensorHandle_.getAngularVelocity()[0];
  gyro.y = imuSensorHandle_.getAngularVelocity()[1];
  gyro.z = imuSensorHandle_.getAngularVelocity()[2];
  try {
    tf2::doTransform(gyro, gyro, robotStateHandle_.lookupTransform("base_link", imuSensorHandle_.getFrameId(), time));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2imu, imu2base, odom2base;
  try {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robotStateHandle_.lookupTransform(imuSensorHandle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    jointHandles_[0].setCommand(0.);
    jointHandles_[1].setCommand(0.);
    return;
  }
  tf2::Quaternion odom2imuQuaternion;
  tf2::Vector3 odom2imu_origin;
  odom2imuQuaternion.setValue(imuSensorHandle_.getOrientation()[0], imuSensorHandle_.getOrientation()[1],
                              imuSensorHandle_.getOrientation()[2], imuSensorHandle_.getOrientation()[3]);
  odom2imu_origin.setValue(0, 0, 0);
  odom2imu.setOrigin(odom2imu_origin);
  odom2imu.setRotation(odom2imuQuaternion);
  odom2base = odom2imu * imu2base;
  scalar_t roll = 0, pitch = 0, yaw = 0;
  quatToRPY(toMsg(odom2base).rotation, roll_, pitch, yaw);

  //  try {
  //    scalar_t pitchUnsed = 0;
  //    odom2base_ = robotStateHandle_.lookupTransform("odom", "base_link", ros::Time(0));
  //    //    if (abs((odom2base_.header.stamp - time).toSec()) < 0.001) {
  //    quatToRPY(odom2base_.transform.rotation, roll, pitchUnsed, yaw);
  //    //    }
  //  } catch (tf2::TransformException& ex) {
  //  }

  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(4);
  scalar_t left_pos[2], left_spd[2], right_pos[2], right_spd[2], left_angle[2], right_angle[2];
  left_angle[0] = 3.49 + jointPos[4];  // [0]:back_vmc_joint [1]:front_vmc_joint
  left_angle[1] = jointPos[2] + M_PI - 3.49;
  right_angle[0] = 3.49 + jointPos[5];
  right_angle[1] = jointPos[3] + M_PI - 3.49;
  leg_pos(left_angle[0], left_angle[1], left_pos);
  leg_pos(right_angle[0], right_angle[1], right_pos);
  leg_spd(jointVel[4], jointVel[2], left_angle[0], left_angle[1], left_spd);
  leg_spd(jointVel[5], jointVel[3], right_angle[0], right_angle[1], right_spd);

  ocs2::vector_t pendulumLength(2);
  pendulumLength[0] = left_pos[0];
  pendulumLength[1] = right_pos[0];
  balanceInterface_->getLeggedBalanceControlCmd()->setPendulumLength(pendulumLength);

  std_msgs::Float64MultiArray legLength;
  legLength.data.push_back(pendulumLength(0));
  legLength.data.push_back(pendulumLength(1));
  legLengthPublisher_.publish(legLength);

  currentObservation_.state(9) = gyro.z;  // may need change
  currentObservation_.state(8) = gyro.y;
  currentObservation_.state(7) = right_spd[1] + gyro.y;
  currentObservation_.state(6) = left_spd[1] + gyro.y;
  currentObservation_.state(5) = (jointVel(0) + jointVel(1)) / 2. * params_.r_;
  currentObservation_.state(4) = yawLast + angles::shortest_angular_distance(yawLast, yaw);
  currentObservation_.state(3) = pitch;
  currentObservation_.state(2) = right_pos[1] + pitch;
  currentObservation_.state(1) = left_pos[1] + pitch;
  currentObservation_.state(0) = currentObservation_.state(0) += currentObservation_.state(5) * period.toSec();
}

void LeggedBalanceController::starting(const ros::Time& time) {
  updateStateEstimation(time, ros::Duration(0.001));
  //  currentObservation_.state(1) = 0;  // Why is this needed?
  //  currentObservation_.state(2) = 0;
  //  currentObservation_.state(3) = 0;
  currentObservation_.input.setZero(INPUT_DIM);

  ocs2::TargetTrajectories targetTrajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
  targetTrajectories.stateTrajectory[0](1) = 0.0;
  targetTrajectories.stateTrajectory[0](2) = 0.0;
  targetTrajectories.stateTrajectory[0](3) = 0.0;

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(targetTrajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(balanceInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;
}

LeggedBalanceController::~LeggedBalanceController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
}

void LeggedBalanceController::setupMpc(ros::NodeHandle& nh) {
  const std::string robotName = "legged_balance";

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<rm::RosReferenceManager>(balanceInterface_->getReferenceManagerPtr(),
                                                                          balanceInterface_->getLeggedBalanceControlCmd());
  rosReferenceManagerPtr->subscribe(nh);

  mpc_ = std::make_shared<ocs2::IpmMpc>(balanceInterface_->mpcSettings(), balanceInterface_->ipmSettings(),
                                        balanceInterface_->getOptimalControlProblem(), balanceInterface_->getInitializer());
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void LeggedBalanceController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<ocs2::MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&balanceInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        ocs2::executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            balanceInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  ocs2::setThreadPriority(balanceInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedBalanceController::updateTfOdom(const ros::Time& time, const ros::Duration& period) {
  try {
    odom2base_ = robotStateHandle_.lookupTransform("odom", "base_link", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    tfRtBroadcaster_.sendTransform(odom2base_);  // TODO: For some reason, the sendTransform in init sometime not work?
    ROS_WARN("%s", ex.what());
    return;
  }

  vector_t position(3);
  position(0) = odom2base_.transform.translation.x;
  position(1) = odom2base_.transform.translation.y;

  position(0) += currentObservation_.state(5) * period.toSec() * cos(currentObservation_.state(4));
  position(1) += currentObservation_.state(5) * period.toSec() * sin(currentObservation_.state(4));
  position(2) = params_.r_;

  odom2base_.header.stamp = time;
  odom2base_.transform.translation.x = position(0);
  odom2base_.transform.translation.y = position(1);
  odom2base_.transform.translation.z = position(2);

  robotStateHandle_.setTransform(odom2base_, "rm_legged_balance_controllers");
  tfRtBroadcaster_.sendTransform(odom2base_);
}

void LeggedBalanceController::normal(const ros::Time& time, const ros::Duration& period) {
  if (blockStateChanged_) {
    ROS_INFO("[balance] Enter BLOCK");
    blockStateChanged_ = false;
  }
  vector_t jointPos(CONTROL_DIM), jointVel(CONTROL_DIM);
  for (size_t i = 0; i < CONTROL_DIM; ++i) {
    jointPos(i) = jointHandles_[i].getPosition();
    jointVel(i) = jointHandles_[i].getVelocity();
  }

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  currentObservation_.input = optimizedInput;

  // Leg control
  ocs2::matrix_t F_bl(2, 1), J(2, 3), p(3, 1), F_leg(2, 1), legLength(2, 1);
  scalar_t F_roll, F_gravity, F_inertial;
  // clang-format off
  J << 1, 1, -1,
      -1, 1, 1;
  // clang-format on
  legLength << balanceInterface_->getLeggedBalanceControlCmd()->getPendulumLength()(0),
      balanceInterface_->getLeggedBalanceControlCmd()->getPendulumLength()(1);
  F_roll = pidRoll_.computeCommand(0 - roll_, period);  // todo: dynamic roll angle
  F_leg(0) = pidLeftLeg_.computeCommand(balanceInterface_->getLeggedBalanceControlCmd()->getLegCmd() -
                                            balanceInterface_->getLeggedBalanceControlCmd()->getPendulumLength()(0),
                                        period);
  F_leg(1) = pidLeftLeg_.computeCommand(balanceInterface_->getLeggedBalanceControlCmd()->getLegCmd() -
                                            balanceInterface_->getLeggedBalanceControlCmd()->getPendulumLength()(1),
                                        period);
  F_gravity = (1. / 2 * params_.massBody_) * params_.g_;
  F_inertial = (1. / 2 * params_.massBody_) * (legLength(0) + legLength(1)) / 2 / (2 * params_.d_) * optimizedState(5) * optimizedState(9);
  // F_gravity = 0.5;
  F_inertial = 0;
  p << F_roll, F_gravity, F_inertial;
  F_bl = J * p + F_leg;

  std_msgs::Float64MultiArray legForce;
  legForce.data.push_back(F_bl(0));
  legForce.data.push_back(F_bl(1));
  legPendulumSupportForce_.publish(legForce);

  scalar_t T_theta_diff = pidThetaDiff_.computeCommand(optimizedState(1) - optimizedState(2), period);
  scalar_t left_T[2], right_T[2], left_angle[2], right_angle[2];
  left_angle[0] = 3.49 + jointPos[4];  // [0]:back_vmc_joint [1]:front_vmc_joint
  left_angle[1] = jointPos[2] + M_PI - 3.49;
  right_angle[0] = 3.49 + jointPos[5];
  right_angle[1] = jointPos[3] + M_PI - 3.49;
  leg_conv(F_bl(0), optimizedInput(0) - T_theta_diff, left_angle[0], left_angle[1], left_T);
  leg_conv(F_bl(1), optimizedInput(1) + T_theta_diff, right_angle[0], right_angle[1], right_T);

  //  Tracking
  scalar_t kp = 0., kd = 1.0;
  scalar_t leftWheelPosRef =
      optimizedState(0) / params_.r_ - legLength(0) * sin(optimizedState(1)) - optimizedState(4) * params_.d_ / 2 / params_.r_;
  scalar_t leftWheelVelRef = optimizedState(5) / params_.r_ - legLength(0) * optimizedState(6) * cos(optimizedState(1)) -
                             optimizedState(9) * params_.d_ / 2 / params_.r_;
  scalar_t rightWheelPosRef =
      optimizedState(0) / params_.r_ - legLength(1) * sin(optimizedState(2)) + optimizedState(4) * params_.d_ / 2 / params_.r_;
  scalar_t rightWheelVelRef = optimizedState(5) / params_.r_ - legLength(1) * optimizedState(7) * cos(optimizedState(2)) +
                              optimizedState(9) * params_.d_ / 2 / params_.r_;

  //  jointHandles_[0].setCommand(kp * (leftWheelPosRef - jointHandles_[0].getPosition()) +
  //                              kd * (leftWheelVelRef - jointHandles_[0].getVelocity()) + optimizedInput(2));
  //  jointHandles_[1].setCommand(kp * (rightWheelPosRef - jointHandles_[1].getPosition()) +
  //                              kd * (rightWheelVelRef - jointHandles_[1].getVelocity()) + optimizedInput(3));
  jointHandles_[0].setCommand(optimizedInput(2));
  jointHandles_[1].setCommand(optimizedInput(3));
  jointHandles_[2].setCommand(left_T[1]);
  jointHandles_[3].setCommand(right_T[1]);
  jointHandles_[4].setCommand(left_T[0]);
  jointHandles_[5].setCommand(right_T[0]);
}

void LeggedBalanceController::block(const ros::Time& time, const ros::Duration& period) {
  // todo: use param
  scalar_t antiBlockEffort = 3.0, antiBlockTime = 0.3;
  //
  if (blockStateChanged_) {
    ROS_INFO("[balance] Enter BLOCK");
    blockStateChanged_ = false;

    lastBlockTime_ = time;
  }
  if ((time - lastBlockTime_).toSec() > antiBlockTime) {
    blockState_ = false;
    blockStateChanged_ = true;
    ROS_INFO("[balance] Exit BLOCK");
  } else {
    jointHandles_[0].setCommand(currentObservation_.state(1) > 0 ? -antiBlockEffort : antiBlockEffort);
    jointHandles_[1].setCommand(currentObservation_.state(1) > 0 ? -antiBlockEffort : antiBlockEffort);
  }
}

void LeggedBalanceController::sitDown(const ros::Time& time, const ros::Duration& period) {
  scalar_t wheelSpeed = balanceInterface_->getLeggedBalanceControlCmd()->getForwardVel() / params_.r_;
  scalar_t followSpeed =
      pidFollow_.computeCommand(balanceInterface_->getLeggedBalanceControlCmd()->getYawError(), period) * params_.d_ / 2 / params_.r_;
  leftWheelController_.setCommand(wheelSpeed - followSpeed);
  rightWheelController_.setCommand(wheelSpeed + followSpeed);
  leftWheelController_.update(time, period);
  rightWheelController_.update(time, period);
}

}  // namespace rm

PLUGINLIB_EXPORT_CLASS(rm::LeggedBalanceController, controller_interface::ControllerBase)
