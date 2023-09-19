//
// Created by yuchen on 23-9-2.
//

#pragma once
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "rm_legged_balance_control/definitions.h"

namespace rm {

class TorqueConstraint : public ocs2::StateInputConstraint {
 public:
  TorqueConstraint() : StateInputConstraint(ocs2::ConstraintOrder::Linear) {}
  TorqueConstraint* clone() const override { return new TorqueConstraint(*this); }

  size_t getNumConstraints(scalar_t /*time*/) const override { return 8; }

  vector_t getValue(scalar_t time, const vector_t& /*state*/, const vector_t& input, const PreComputation& /*preComp*/) const override {
    vector_t ret(getNumConstraints(time));
    scalar_t wheel_limit = 9.473, leg_limit = 9.473;
    // clang-format off
    ret << input(0) + wheel_limit,
          -input(0) + wheel_limit,
           input(1) + wheel_limit,
          -input(1) + wheel_limit,
           input(2) + leg_limit,
          -input(2) + leg_limit,
           input(3) + leg_limit,
          -input(3) + leg_limit;
    // clang-format on
    return ret;
  }
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation ret(getNumConstraints(time), STATE_DIM, INPUT_DIM);
    ret.f = getValue(time, state, input, preComp);
    ret.dfdx.setZero();
    // clang-format off
    ret.dfdu << 1, 0, 0, 0,
               -1, 0, 0, 0,
                0, 1, 0, 0,
                0,-1, 0, 0,
                0, 0, 1, 0,
                0, 0,-1, 0,
                0, 0, 0, 1,
                0, 0, 0,-1;
    // clang-format on
    return ret;
  }
};

}  // namespace rm
