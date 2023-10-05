//
// Created by yuchen on 23-9-2.
//

#include "rm_legged_balance_control/LeggedBalanceSystemDynamics.h"

namespace rm {
// On the dynamic model of a two-wheeled inverted pendulum robot
// State = [x, theta_l, theta_r, alpha,, psi, x_dot, theta_l_dot, theta_r_dot, alpha_dot, psi_dot]^T
// Input = [tau_l, tau_r,tau_l_l,tau_l_r]^T
ad_vector_t LeggedBalanceSystemDynamics::systemFlowMap(ad_scalar_t /*time*/, const ad_vector_t& state, const ad_vector_t& input,
                                                       const ad_vector_t& parameters) const {
  // todo: add linear equation
  ad_scalar_t theta_l = state(1), theta_r = state(2), theta = state(3), psi = state(4), x_dot = state(5), theta_l_dot = state(6),
              theta_r_dot = state(7), theta_dot = state(8), psi_dot = state(9);

  ad_matrix_t A = generateA(parameters(0), parameters(1)), B = generateB(parameters(0), parameters(1));

  ad_vector_t result(6);
  result = A * state + B * input;

  return result;
}

vector_t LeggedBalanceSystemDynamics::getFlowMapParameters(scalar_t /*time*/, const PreComputation& /* preComputation */) const {
  vector_t v(2);
  // Left pendulum length # Right pendulum length
  v[0] = balanceControlCmdPtr_->getPendulumLength()[0];
  v[1] = balanceControlCmdPtr_->getPendulumLength()[1];
  return v;
}

void LeggedBalanceSystemDynamics::loadDynamicsParams(const std::string& filename) {
  ocs2::loadData::loadCppDataType(filename, "Dynamics.d", param_.d_);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.l_c", param_.l_c);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.r", param_.r_);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.massBody", param_.massBody_);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.massLeg", param_.massLeg_);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.massWheel", param_.massWheel_);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.jWheel", param_.jWheel_);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.i1", param_.i1);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.i2", param_.i2);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.i3", param_.i3);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.powerCoeffEffort", param_.powerCoeffEffort_);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.powerCoeffVel", param_.powerCoeffVel_);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.powerOffset", param_.powerOffset_);
}

ocs2::ad_matrix_t LeggedBalanceSystemDynamics::generateA(ad_scalar_t l_l, ad_scalar_t l_r) const {
  // Polynomial fitting
  ad_scalar_t O = ad_scalar_t(0);
  ad_scalar_t a_12 = ad_scalar_t(1), a_34 = ad_scalar_t(1), a_56 = ad_scalar_t(1), a_78 = ad_scalar_t(1), a_910 = ad_scalar_t(1);
  ad_scalar_t a_25 = 1.556 - 24.58 * l_l - 0.08178 * l_r - 15.16 * l_l * l_l - 1.048 * l_l * l_r + 0.1123 * l_r * l_r;
  ad_scalar_t a_27 = 1.556 - 0.08178 * l_l - 24.58 * l_r + 0.1123 * l_l * l_l - 1.048 * l_l * l_r - 15.16 * l_r * l_r;
  ad_scalar_t a_45 = 0.3376 - 6.09 * l_l + 0.01922 * l_r - 2.845 * l_l * l_l + 0.2355 * l_l * l_r - 0.02613 * l_r * l_r;
  ad_scalar_t a_47 = -0.3376 - 0.01922 * l_l + 6.09 * l_r + 0.2613 * l_l * l_l - 0.2355 * l_l * l_r + 2.845 * l_r * l_r;
  ad_scalar_t a_65 = 7.991 + 286.6 * l_l + 0.005895 * l_r - 434.2 * l_l * l_l + 0.04345 * l_l * l_r - 0.008873 * l_r * l_r;
  ad_scalar_t a_67 = -0.02577 + 0.1381 * l_l + 3.643 * l_r - 0.412 * l_l * l_l - 0.7317 * l_l * l_r - 2.644 * l_r * l_r;
  ad_scalar_t a_85 = -0.02577 + 3.643 * l_l + 0.1381 * l_r - 2.644 * l_l * l_l - 0.7317 * l_l * l_r - 0.412 * l_r * l_r;
  ad_scalar_t a_87 = 7.991 + 0.005895 * l_l + 286.6 * l_r - 0.008873 * l_l * l_l + 0.04345 * l_l * l_r - 434.2 * l_r * l_r;
  ad_scalar_t a_105 = 0.1876 - 9.279 * l_l - 0.0184 * l_r + 3.627 * l_l * l_l - 0.1974 * l_l * l_r + 0.02865 * l_r * l_r;
  ad_scalar_t a_107 = 0.1876 - 0.0184 * l_l - 9.279 * l_r + 0.2865 * l_l * l_l - 0.1974 * l_l * l_r + 3.627 * l_r * l_r;
  ad_scalar_t a_109 = ad_scalar_t(555219 / 30700.);

  ad_matrix_t A = (ad_matrix_t(10, 10) <<  // clang-format off
                       O,a_12,   O,   O,   O,   O,   O,   O,   O,   O,
                       O,   O,   O,   O,a_25,   O,a_27,   O,   O,   O,
                       O,   O,   O,a_34,   O,   O,   O,   O,   O,   O,
                       O,   O,   O,   O,a_45,   O,a_47,   O,   O,   O,
                       O,   O,   O,   O,   O,a_56,   O,   O,   O,   O,
                       O,   O,   O,   O,a_65,   O,a_67,   O,   O,   O,
                       O,   O,   O,   O,   O,   O,   O,a_78,   O,   O,
                       O,   O,   O,   O,a_85,   O,a_87,   O,   O,   O,
                       O,   O,   O,   O,   O,   O,   O,   O,   O,a_910,
                       O,   O,   O,   O,a_105,  O,a_107,  O,a_109,  O
                    ).finished();  // clang-format on

  return A;
}

ocs2::ad_matrix_t LeggedBalanceSystemDynamics::generateB(ad_scalar_t l_l, ad_scalar_t l_r) const {
  // Polynomial fitting
  ad_scalar_t O = ad_scalar_t(0);
  ad_scalar_t b_21 = 1.082 + 9.273 * l_l + 1.047 * l_r + 0.5413 * l_l * l_l + 0.2766 * l_l * l_r - 0.4251 * l_r * l_r;
  ad_scalar_t b_22 = 1.082 + 1.047 * l_l + 9.278 * l_r - 0.4251 * l_l * l_l + 0.2766 * l_l * l_r + 0.5413 * l_r * l_r;
  ad_scalar_t b_23 = -0.08052 - 2.891 * l_l - 0.04324 * l_r + 4.375 * l_l * l_l + 0.03729 * l_l * l_r + 0.01381 * l_r * l_r;
  ad_scalar_t b_24 = -0.08502 - 0.04324 * l_l - 2.891 * l_r + 0.01381 * l_l * l_l + 0.03729 * l_l * l_r + 4.375 * l_r * l_r;
  ad_scalar_t b_41 = -13.68 + 2.201 * l_l - 0.2399 * l_r + 0.01163 * l_l * l_l - 0.05459 * l_l * l_r + 0.09893 * l_r * l_r;
  ad_scalar_t b_42 = 13.68 + 0.2399 * l_l - 2.201 * l_r - 0.09893 * l_l * l_l - 0.05459 * l_l * l_r - 0.01163 * l_r * l_r;
  ad_scalar_t b_43 = -0.03804 - 0.6017 * l_l + 0.009886 * l_r + 0.928 * l_l * l_l - 0.008715 * l_l * l_r - 0.003214 * l_r * l_r;
  ad_scalar_t b_44 = 0.03804 - 0.009886 * l_l + 0.6017 * l_r + 0.003214 * l_l * l_l + 0.008715 * l_l * l_r - 0.928 * l_r * l_r;
  ad_scalar_t b_61 = -11.89 - 47.85 * l_l - 0.05541 * l_r + 80.04 * l_l * l_l + 0.0009459 * l_l * l_r + 0.03362 * l_r * l_r;
  ad_scalar_t b_62 = -2.096 - 0.006312 * l_l - 0.881 * l_r + 1.559 * l_l * l_l + 0.1807 * l_l * l_r + 0.6006 * l_r * l_r;
  ad_scalar_t b_63 = 11.5 - 37.99 * l_l + 0.002215 * l_r + 48.6 * l_l * l_l - 0.002104 * l_l * l_r - 0.001092 * l_r * l_r;
  ad_scalar_t b_64 = 0.09802 - 0.006269 * l_l - 0.1583 * l_r - 0.05063 * l_l * l_l + 0.02658 * l_l * l_r + 0.1612 * l_r * l_r;
  ad_scalar_t b_81 = -2.096 - 0.881 * l_l - 0.006312 * l_r + 0.6006 * l_l * l_l + 0.1807 * l_l * l_r + 1.559 * l_r * l_r;
  ad_scalar_t b_82 = -11.89 + 0.05541 * l_l - 47.85 * l_r + 0.03362 * l_l * l_l + 0.0009459 * l_l * l_r + 80.04 * l_r * l_r;
  ad_scalar_t b_83 = 0.09802 - 0.1583 * l_l - 0.006269 * l_r + 0.1612 * l_l * l_l + 0.02658 * l_l * l_r - 0.05063 * l_r * l_r;
  ad_scalar_t b_84 = 11.5 + 0.002215 * l_l - 37.99 * l_r - 0.001092 * l_l * l_l - 0.002104 * l_l * l_r + 48.6 * l_r * l_r;
  ad_scalar_t b_101 = -2.578 + 2.578 * l_l + 0.2085 * l_r - 1.117 * l_l * l_l + 0.05208 * l_l * l_r - 0.1085 * l_r * l_r;
  ad_scalar_t b_102 = -2.572 + 0.2085 * l_l + 2.578 * l_r - 0.1085 * l_l * l_l + 0.05208 * l_l * l_r - 1.117 * l_r * l_r;
  ad_scalar_t b_103 = -8.335 + 0.005946 * l_l - 0.00851 * l_r + 0.1271 * l_l * l_l + 0.007021 * l_l * l_r + 0.003523 * l_r * l_r;
  ad_scalar_t b_104 = -8.335 - 0.00851 * l_l + 0.005946 * l_r + 0.003523 * l_l * l_l + 0.007021 * l_l * l_r + 0.1271 * l_r * l_r;

  ad_matrix_t B = (ad_matrix_t(10, 4) <<  // clang-format off
                       O,   O,   O,   O,
                    b_21,b_22,b_23,b_24,
                       O,   O,   O,   O,
                    b_41,b_42,b_43,b_44,
                       O,   O,   O,   O,
                    b_61,b_62,b_63,b_64,
                       O,   O,   O,   O,
                    b_81,b_82,b_83,b_84,
                       O,   O,   O,   O,
                    b_101,b_102,b_103,b_104
                    ).finished();  // clang-format on

  return B;
}

}  // namespace rm
