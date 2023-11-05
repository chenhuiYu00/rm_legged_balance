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
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <rm_legged_balance_control/LeggedBalanceInterface.h>

#include "rm_legged_balance_control_ros/LeggedBalanceVisualization.h"
#include "rm_legged_balance_control_ros/vmc/leg_conv.h"
#include "rm_legged_balance_control_ros/vmc/leg_pos.h"
#include "rm_legged_balance_control_ros/vmc/leg_spd.h"

namespace rm {

class LeggedBalanceController
    : public controller_interface::MultiInterfaceController<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface,
                                                            hardware_interface::ImuSensorInterface> {
  enum BalanceState { NORMAL, BLOCK, SIT_DOWN, STOP };

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
  void stop(const ros::Time& time, const ros::Duration& period);
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
  int balanceState_;
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  ocs2::benchmark::RepeatedTimer mpcTimer_;

  // Odom TF
  rm_common::TfRtBroadcaster tfRtBroadcaster_;
  geometry_msgs::TransformStamped odom2base_{};

  // Protect
  scalar_t legProtectAngle_, pitchProtectAngle_, rollProtectAngle_, legProtectLength_;
  bool blockStateChanged_{false}, maybeBlock_{false};
  ros::Time maybeBlockTime_, lastBlockTime_;

  // Sit down
  effort_controllers::JointVelocityController leftWheelController_, rightWheelController_;
  control_toolbox::Pid pidFollow_;

  // Leg control
  void unstickDetection(const ros::Time& time, const ros::Duration& period, const scalar_t& leftLegDLength, const scalar_t& leftLegSupport,
                        const scalar_t& leftLegTorque, const scalar_t& rightLegDLength, const scalar_t& rightLegSupport,
                        const scalar_t& rightLegTorque, const scalar_t& ddz);
  bool leftIsUnStick_, rightIsUnstick_;
  scalar_t left_pos_[2], left_spd_[2], right_pos_[2], right_spd_[2];
  scalar_t roll_, z_acc_, groundSupportThreshold_;
  control_toolbox::Pid pidLeftLeg_, pidRightLeg_, pidThetaDiff_, pidRoll_;
  ros::Publisher legLengthPublisher_, legPendulumSupportForcePublisher_, legGroundSupportForcePublisher_, leftUnStickPublisher_,
      rightUnStickPublisher_;
};

}  // namespace rm