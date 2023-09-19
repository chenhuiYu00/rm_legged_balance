//
// Created by yuchen on 23-9-2.
//

#pragma once

#include <tf/transform_broadcaster.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <rm_legged_balance_control/LeggedBalanceParameters.h>
#include <rm_legged_balance_control/definitions.h>

namespace rm {

class LeggedBalanceVisualization final : public ocs2::DummyObserver {
 public:
  LeggedBalanceVisualization(ros::NodeHandle nodeHandle, bool isDummy);
  ~LeggedBalanceVisualization() override = default;

  void update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& policy, const ocs2::CommandData& command) override;
  tf::Transform getOdomTransform() { return odomTransform_; }

 private:
  bool isDummy_;
  LeggedBalanceParameters param_;
  tf::TransformBroadcaster tfBroadcaster_;
  ros::Publisher powerPublisher_;

  ocs2::SystemObservation lastObservation_;
  ocs2::vector_t position_;
  tf::Transform odomTransform_;
};

}  // namespace rm
