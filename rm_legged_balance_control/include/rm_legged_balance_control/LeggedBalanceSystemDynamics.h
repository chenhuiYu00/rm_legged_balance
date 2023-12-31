//
// Created by yuchen on 23-9-2.
//
#pragma once
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "rm_legged_balance_control/LeggedBalanceControlCmd.h"
#include "rm_legged_balance_control/LeggedBalanceParameters.h"
#include "rm_legged_balance_control/definitions.h"

#include <ocs2_core/misc/LoadData.h>

namespace rm {
using ocs2::ad_matrix_t;
using ocs2::ad_scalar_t;
using ocs2::ad_vector_t;
using ocs2::PreComputation;
using ocs2::SystemDynamicsBaseAD;
using ocs2::vector_t;
using ocs2::VectorFunctionLinearApproximation;

class LeggedBalanceSystemDynamics final : public SystemDynamicsBaseAD {
 public:
  LeggedBalanceSystemDynamics(const std::string& filename, const std::string& libraryFolder, bool recompileLibraries,
                              std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr)
      : SystemDynamicsBaseAD(), balanceControlCmdPtr_(std::move(balanceControlCmdPtr)) {
    initialize(STATE_DIM, INPUT_DIM, "legged_balance_dynamics", libraryFolder, recompileLibraries, true);
    loadDynamicsParams(filename);
  }

  /** Destructor */
  ~LeggedBalanceSystemDynamics() override = default;
  LeggedBalanceSystemDynamics(const LeggedBalanceSystemDynamics& rhs) = default;

  LeggedBalanceSystemDynamics* clone() const override { return new LeggedBalanceSystemDynamics(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override;

  void loadDynamicsParams(const std::string& filename);
  ocs2::ad_matrix_t generateA(ad_scalar_t l_l, ad_scalar_t l_r) const;
  ocs2::ad_matrix_t generateB(ad_scalar_t l_l, ad_scalar_t l_r) const;

 protected:
  // For dynamic pendulum length
  vector_t getFlowMapParameters(scalar_t time, const PreComputation& preComputation) const override;
  size_t getNumFlowMapParameters() const override { return 2; }

 private:
  LeggedBalanceParameters param_;
  std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr_;
};

}  // namespace rm
