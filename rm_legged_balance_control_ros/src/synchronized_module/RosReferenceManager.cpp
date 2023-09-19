//
// Created by qiayuan on 23-5-26.
//
#include "rm_legged_balance_control_ros/synchronized_module/RosReferenceManager.h"

#include <angles/angles.h>
#include <rm_legged_balance_control/definitions.h>
#include <rm_common/ori_tool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rm {

RosReferenceManager::RosReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
                                         std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr)
    : ReferenceManagerDecorator(std::move(referenceManagerPtr)),
      balanceControlCmdPtr_(std::move(balanceControlCmdPtr)),
      chassisCmdUpdated_(false),
      cmdVelUpdated_(false),
      transformListener_(buffer_) {}

void RosReferenceManager::preSolverRun(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t& initState) {
  if (cmdVelUpdated_) {
    std::lock_guard<std::mutex> lock(cmdVelMutex_);
    cmdVelUpdated_ = false;

    geometry_msgs::Vector3 vel;
    vel.x = cmdVel_.linear.x;
    vel.y = cmdVel_.linear.y;
    vel.z = 0;

    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    ocs2::vector_array_t inputTrajectory;
    scalar_t horizon = finalTime - initTime;

    if (chassisCmd_.mode == rm_msgs::ChassisCmd::RAW) {
      const size_t sample = 20;
      const scalar_t dt = horizon / sample;
      if (tfVel(chassisCmd_.command_source_frame, "odom", vel)) {
        for (size_t i = 0; i < sample + 1; ++i) {
          scalar_t time = dt * static_cast<scalar_t>(i);
          const vector_t targetState = [&]() {
            vector_t targetState = vector_t::Zero(rm::STATE_DIM);
            // TODO: targetState(0)?
            targetState(2) = initState(2) + cmdVel_.angular.z * time;
            targetState(3) = cos(targetState(2)) * vel.x + sin(targetState(2)) * vel.y;
            targetState(5) = cmdVel_.angular.z;
            return targetState;
          }();
          timeTrajectory.push_back(initTime + time);
          stateTrajectory.push_back(targetState);
          inputTrajectory.emplace_back(vector_t::Zero(INPUT_DIM));
        }
        referenceManagerPtr_->setTargetTrajectories({timeTrajectory, stateTrajectory, inputTrajectory});
      }
      balanceControlCmdPtr_->setSitDown(false);
    } else if (chassisCmd_.mode == rm_msgs::ChassisCmd::FOLLOW || chassisCmd_.mode == 3 || chassisCmd_.mode == 4) {
      vector_t targetState = vector_t::Zero(rm::STATE_DIM);
      if (chassisCmd_.follow_source_frame.empty()) {
        chassisCmd_.follow_source_frame = "yaw";
      }
      try {
        double roll{}, pitch{}, yaw{};
        quatToRPY(buffer_.lookupTransform("odom", chassisCmd_.follow_source_frame, ros::Time(0)).transform.rotation, roll, pitch, yaw);
        double yawError = angles::shortest_angular_distance(initState(2), yaw);
        targetState(2) = initState(2) + yawError;
        balanceControlCmdPtr_->setYawError(yawError);
      } catch (tf2::TransformException& ex) {
        targetState(2) = initState(2);
        ROS_WARN("%s", ex.what());
      }
      if (tfVel(chassisCmd_.command_source_frame, "base_link", vel)) {
        targetState(0) = initState(0) + vel.x * horizon;
        targetState(3) = vel.x;
        timeTrajectory = {initTime, finalTime};
        stateTrajectory.assign(2, targetState);
        inputTrajectory.assign(2, vector_t::Zero(INPUT_DIM));
        referenceManagerPtr_->setTargetTrajectories({timeTrajectory, stateTrajectory, inputTrajectory});
        balanceControlCmdPtr_->setForwardVel(vel.x);
      }
      if (chassisCmd_.mode == rm_msgs::ChassisCmd::FOLLOW) {
        balanceControlCmdPtr_->setMaxPitch(0.50, initTime);
        balanceControlCmdPtr_->setSitDown(false);
      } else if (chassisCmd_.mode == 3) {
        balanceControlCmdPtr_->setMaxPitch(0.36, initTime);
        balanceControlCmdPtr_->setSitDown(false);
      } else if (chassisCmd_.mode == 4) {
        balanceControlCmdPtr_->setSitDown(true);
      }
    }
  }

  if (chassisCmdUpdated_) {
    std::lock_guard<std::mutex> lock(chassisCmdMutex_);
    chassisCmdUpdated_ = false;
    balanceControlCmdPtr_->setPowerLimit(chassisCmd_.power_limit);
  }

  referenceManagerPtr_->preSolverRun(initTime, finalTime, initState);
}

void RosReferenceManager::subscribe(ros::NodeHandle& nodeHandle) {
  // Velocity command
  auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmdVelMutex_);
    cmdVelUpdated_ = true;
    cmdVel_ = *msg;
  };
  cmdVelSubscriber_ = nodeHandle.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmdVelCallback);

  // Chassis command
  auto chassisCmdCallback = [this](const rm_msgs::ChassisCmd::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(chassisCmdMutex_);
    chassisCmdUpdated_ = true;
    chassisCmd_ = *msg;
  };
  chassisCmdSubscriber_ = nodeHandle.subscribe<rm_msgs::ChassisCmd>("/cmd_chassis", 1, chassisCmdCallback);
}

bool RosReferenceManager::tfVel(std::string from, std::string to, geometry_msgs::Vector3& vel) {
  if (from.empty()) {
    from = "yaw";
  }
  try {
    tf2::doTransform(vel, vel, buffer_.lookupTransform(to, from, ros::Time(0)));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  return true;
}

}  // namespace rm
