//
// Created by yuchen on 23-9-2.
//
#include <std_msgs/Float64.h>
#include <tf/tf.h>

#include "rm_legged_balance_control_ros/LeggedBalanceVisualization.h"

namespace rm {
LeggedBalanceVisualization::LeggedBalanceVisualization(ros::NodeHandle nodeHandle, bool isDummy)
    : isDummy_(isDummy), lastObservation_(), position_(ocs2::vector_t::Zero(2)) {
  powerPublisher_ = nodeHandle.advertise<std_msgs::Float64>("balance_mpc_power", 1);
}

void LeggedBalanceVisualization::update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& policy,
                                        const ocs2::CommandData& /*command*/) {
  ros::Time timeMsg = ros::Time::now();
  geometry_msgs::TransformStamped world_transform;
  world_transform.header.stamp = timeMsg;

  scalar_t vel = observation.state(3);
  scalar_t dt = observation.time - lastObservation_.time;
  position_(0) += vel * dt * cos(observation.state(2));
  position_(1) += vel * dt * sin(observation.state(2));
  lastObservation_ = observation;

  // todo: leg tf
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(position_(0) + param_.l_1 * sin(observation.state(1)) * cos(observation.state(3)),
                                  position_(1) + param_.l_1 * sin(observation.state(1)) * sin(observation.state(3)),
                                  param_.l_1 * cos(observation.state(1)) + param_.r_));
  tf::Quaternion q = tf::createQuaternionFromRPY(0, observation.state(1), observation.state(3));
  transform.setRotation(q);
  odomTransform_ = transform;

  if (isDummy_) {  // for dummy node visualization
    tfBroadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    transform.setOrigin(tf::Vector3(0, param_.d_ / 2, -param_.l_1));
    transform.setRotation(tf::createQuaternionFromRPY(
        0, observation.state(0) / param_.r_ - observation.state(1) - observation.state(2) * param_.d_ / 2 / param_.r_, 0));
    tfBroadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "left_wheel"));

    transform.setOrigin(tf::Vector3(position_(0) + param_.d_ / 2 * sin(observation.state(2)),
                                    position_(1) - param_.d_ / 2 * cos(observation.state(2)), param_.r_));
    transform.setRotation(tf::createQuaternionFromRPY(
        0, observation.state(0) / param_.r_ + observation.state(2) * param_.d_ / 2 / param_.r_, observation.state(2)));
    tfBroadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "right_wheel"));
  }

  // Power
  ocs2::vector_t input = policy.inputTrajectory_[0];

  scalar_t wheel_left_speed = observation.state(3) / param_.r_ - observation.state(4) - observation.state(5) * param_.d_ / 2 / param_.r_;
  scalar_t wheel_right_speed = observation.state(3) / param_.r_ - observation.state(4) + observation.state(5) * param_.d_ / 2 / param_.r_;
  scalar_t power = abs(input(0) * wheel_left_speed) + param_.powerCoeffEffort_ * input(0) * input(0) +
                   param_.powerCoeffVel_ * wheel_left_speed * wheel_left_speed + abs(input(1) * wheel_right_speed) +
                   param_.powerCoeffEffort_ * input(1) * input(1) + param_.powerCoeffVel_ * wheel_right_speed * wheel_right_speed +
                   param_.powerOffset_;
  //  scalar_t power = abs(input(0) * wheel_left_speed) + coeff_vel * wheel_left_speed * wheel_left_speed +
  //  abs(input(1) * wheel_right_speed) +
  //                   coeff_vel * wheel_right_speed * wheel_right_speed + offset;
  std_msgs::Float64 msg;
  msg.data = power;
  powerPublisher_.publish(msg);
}

}  // namespace rm
