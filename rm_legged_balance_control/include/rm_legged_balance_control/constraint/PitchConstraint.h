//
// Created by yuchen on 23-9-2.
//

#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

#include <utility>

namespace rm {

class PitchConstraint : public ocs2::StateConstraint {
 public:
  PitchConstraint(std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr)
      : StateConstraint(ocs2::ConstraintOrder::Linear), balanceControlCmdPtr_(std::move(balanceControlCmdPtr)) {}
  PitchConstraint* clone() const override { return new PitchConstraint(*this); }

  size_t getNumConstraints(scalar_t /*time*/) const override { return 4; }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& /*preComp*/) const override {
    scalar_t alpha = 0.5;

    vector_t ret(4);
    scalar_t pitchDiff = balanceControlCmdPtr_->getLastMaxPitch() - balanceControlCmdPtr_->getMaxPitch();
    scalar_t limit = balanceControlCmdPtr_->getMaxPitch() + exp(-alpha * (time - balanceControlCmdPtr_->getMaxPitchSetTime())) * pitchDiff;
    if (time - balanceControlCmdPtr_->getMaxPitchSetTime() > 0.5) {
      limit = balanceControlCmdPtr_->getMaxPitch();
    }
    // clang-format off
    ret << state(1) + limit,
          -state(1) + limit,
           state(2) + limit,
          -state(2) + limit;
    // clang-format on
    return ret;
  }
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation ret(4, STATE_DIM);
    ret.f = getValue(time, state, preComp);
    ret.dfdu.setZero();
    // clang-format off
    ret.dfdx << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                0,-1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                0, 0,-1, 0, 0, 0, 0, 0, 0, 0;
    // clang-format on
    return ret;
  }

 private:
  std::shared_ptr<LeggedBalanceControlCmd> balanceControlCmdPtr_;
};

}  // namespace rm
