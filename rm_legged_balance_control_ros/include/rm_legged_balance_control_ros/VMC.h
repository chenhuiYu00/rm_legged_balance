//
// Created by yuchen on 23-9-19.
//

#pragma once
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <iostream>

namespace rm {

class VMC {
 public:
  VMC(ocs2::scalar_t l_a, ocs2::scalar_t l_u, ocs2::scalar_t l_d);
  ~VMC() = default;

  ocs2::matrix_t pendulumEff2JointEff(ocs2::scalar_t F_bl, ocs2::scalar_t T_bl, ocs2::scalar_t front_joint_angle,
                                      ocs2::scalar_t back_joint_angle) const;
  ocs2::matrix_t jointPos2Pendulum(ocs2::scalar_t front_joint_angle, ocs2::scalar_t back_joint_angle, ocs2::scalar_t front_joint_vel,
                                   ocs2::scalar_t back_joint_vel) const;

 private:
  ocs2::scalar_t l_a_, l_u_, l_d_;
};

}  // namespace rm
