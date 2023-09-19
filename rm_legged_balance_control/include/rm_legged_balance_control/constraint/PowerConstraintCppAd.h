//
// Created by yuchen on 23-9-2.
//

#pragma once

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

#include <utility>

#include "rm_legged_balance_control/LeggedBalanceParameters.h"
#include "rm_legged_balance_control/definitions.h"
namespace rm {

class PowerConstraintCppAd : public ocs2::StateInputConstraintCppAd {
 public:
  explicit PowerConstraintCppAd(LeggedBalanceParameters param, std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr)
      : StateInputConstraintCppAd(ocs2::ConstraintOrder::Quadratic), param_(param), balanceControlCmdPtr_(std::move(balanceControlCmdPtr)) {
    initialize(STATE_DIM, INPUT_DIM, 1, "PowerLimit", "/tmp/rm", true, false);
  }

  PowerConstraintCppAd* clone() const override { return new PowerConstraintCppAd(*this); }

  size_t getNumConstraints(scalar_t /*time*/) const override { return 1; }

  vector_t getParameters(scalar_t /*time*/, const PreComputation& /*unused*/) const override {
    return (vector_t(1) << balanceControlCmdPtr_->getPowerLimit()).finished();
  }

  ad_vector_t constraintFunction(ad_scalar_t /*time*/, const ad_vector_t& state, const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override {
    const ad_scalar_t& limit = parameters(0);
    ad_scalar_t wheel_left_speed = state(3) / param_.r_ - state(4) - state(5) * param_.d_ / 2 / param_.r_;
    ad_scalar_t wheel_right_speed = state(3) / param_.r_ - state(4) + state(5) * param_.d_ / 2 / param_.r_;
    ad_scalar_t power = abs(input(0) * wheel_left_speed) + param_.powerCoeffEffort_ * input(0) * input(0) +
                        param_.powerCoeffVel_ * wheel_left_speed * wheel_left_speed + abs(input(1) * wheel_right_speed) +
                        param_.powerCoeffEffort_ * input(1) * input(1) + param_.powerCoeffVel_ * wheel_right_speed * wheel_right_speed +
                        param_.powerOffset_;
    //    ad_scalar_t power = abs(input(0) * wheel_left_speed) + coeff_vel * wheel_left_speed * wheel_left_speed +
    //                        abs(input(1) * wheel_right_speed) + coeff_vel * wheel_right_speed * wheel_right_speed + offset;
    ad_vector_t ret(1);
    ret(0) = limit - power;
    return ret;
  }

 private:
  LeggedBalanceParameters param_;
  std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr_;
};

}  // namespace rm
