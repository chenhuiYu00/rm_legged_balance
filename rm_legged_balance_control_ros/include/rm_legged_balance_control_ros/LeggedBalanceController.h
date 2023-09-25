//
// Created by yuchen on 23-9-2.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/tf_rt_broadcaster.h>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <rm_legged_balance_control/LeggedBalanceInterface.h>

#include "rm_legged_balance_control_ros/LeggedBalanceVisualization.h"
#include "rm_legged_balance_control_ros/VMC.h"

namespace rm {

class LeggedBalanceController
    : public controller_interface::MultiInterfaceController<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface,
                                                            hardware_interface::ImuSensorInterface> {
 public:
  LeggedBalanceController() = default;
  ~LeggedBalanceController() override;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);
  void updateTfOdom(const ros::Time& time, const ros::Duration& period);
  void normal(const ros::Time& time, const ros::Duration& period);
  void block(const ros::Time& time, const ros::Duration& period);
  void sitDown(const ros::Time& time, const ros::Duration& period);
  virtual void setupMpc(ros::NodeHandle& nh);
  virtual void setupMrt();

  // Interface
  std::shared_ptr<LeggedBalanceInterface> balanceInterface_;
  rm_control::RobotStateHandle robotStateHandle_;
  std::vector<hardware_interface::JointHandle> jointHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  // State Estimation
  ocs2::SystemObservation currentObservation_;

  // Nonlinear MPC
  std::shared_ptr<ocs2::MPC_BASE> mpc_;
  std::shared_ptr<ocs2::MPC_MRT_Interface> mpcMrtInterface_;
  LeggedBalanceParameters params_;

  // Visualization
  std::shared_ptr<LeggedBalanceVisualization> visualizer_;
  ros::Publisher observationPublisher_;

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  ocs2::benchmark::RepeatedTimer mpcTimer_;

  // Odom TF
  rm_common::TfRtBroadcaster tfRtBroadcaster_;
  geometry_msgs::TransformStamped odom2base_{};

  // Block Protect:
  bool blockState_{false}, blockStateChanged_{false}, maybeBlock_{false};
  ros::Time maybeBlockTime_, lastBlockTime_;

  // Sit down
  effort_controllers::JointVelocityController leftWheelController_, rightWheelController_;
  control_toolbox::Pid pidFollow_;

  // Leg control
  scalar_t roll_;
  control_toolbox::Pid pidLeg_, pidRoll_;

  // VMC
  std::shared_ptr<VMC> vmc_;
};

}  // namespace rm