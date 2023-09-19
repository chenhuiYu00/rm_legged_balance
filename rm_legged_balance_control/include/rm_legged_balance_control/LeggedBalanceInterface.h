//
// Created by yuchen on 23-9-2.
//
#pragma once
// OCS2
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

// Balance
#include "rm_legged_balance_control/LeggedBalanceControlCmd.h"
#include "rm_legged_balance_control/LeggedBalanceSystemDynamics.h"

namespace rm {
using ocs2::Initializer;
using ocs2::matrix_t;
using ocs2::OptimalControlProblem;
using ocs2::ReferenceManager;
using ocs2::ReferenceManagerInterface;
using ocs2::RobotInterface;
using ocs2::RolloutBase;
using ocs2::vector_t;

class LeggedBalanceInterface : public RobotInterface {
 public:
  LeggedBalanceInterface(const std::string& taskFile, const std::string& libraryFolder);

  /** Destructor */
  ~LeggedBalanceInterface() override = default;

  const vector_t& getInitialState() { return initialState_; }

  ocs2::ipm::Settings& ipmSettings() { return ipmSettings_; }
  ocs2::sqp::Settings& sqpSettings() { return sqpSettings_; }
  ocs2::mpc::Settings& mpcSettings() { return mpcSettings_; }

  const OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }

  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  const Initializer& getInitializer() const override { return *initializerPtr_; }

  std::shared_ptr<LeggedBalanceControlCmd> getLeggedBalanceControlCmd() const { return balanceControlCmdPtr_; }

 private:
  ocs2::sqp::Settings sqpSettings_;
  ocs2::mpc::Settings mpcSettings_;
  ocs2::ipm::Settings ipmSettings_;

  OptimalControlProblem problem_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;

  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> initializerPtr_;

  vector_t initialState_{STATE_DIM};

  std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr_;
};
}  // namespace rm
