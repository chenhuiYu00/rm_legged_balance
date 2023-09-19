//
// Created by qiayuan on 23-5-26.
//
#pragma once

#include <memory>
#include <string>
#include <utility>

#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>

#include <geometry_msgs/Twist.h>
#include <rm_msgs/ChassisCmd.h>
#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include "rm_legged_balance_control/LeggedBalanceControlCmd.h"

namespace rm {
using ocs2::scalar_t;
using ocs2::vector_t;

class RosReferenceManager : public ocs2::ReferenceManagerDecorator {
 public:
  RosReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
                      std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr);
  ~RosReferenceManager() override = default;

  void subscribe(ros::NodeHandle& nodeHandle);
  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState) override;

 private:
  bool tfVel(std::string from, std::string to, geometry_msgs::Vector3& vel);

  ::ros::Subscriber chassisCmdSubscriber_;
  std::mutex chassisCmdMutex_;
  std::atomic_bool chassisCmdUpdated_;
  rm_msgs::ChassisCmd chassisCmd_;

  ::ros::Subscriber cmdVelSubscriber_;
  std::mutex cmdVelMutex_;
  std::atomic_bool cmdVelUpdated_;
  geometry_msgs::Twist cmdVel_;

  std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener transformListener_;
};

}  // namespace rm
