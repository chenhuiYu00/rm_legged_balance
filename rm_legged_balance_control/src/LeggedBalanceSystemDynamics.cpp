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
  ad_scalar_t a_16 = ad_scalar_t(1), a_27 = ad_scalar_t(1), a_38 = ad_scalar_t(1), a_49 = ad_scalar_t(1), a_510 = ad_scalar_t(1);
  ad_scalar_t a_62 =
      -0.03218 - 0.11809246 * l_l - 0.08837277 * l_r + 0.08147009 * l_l * l_l + 0.01512841 * l_l * l_r + 0.11946842 * l_r * l_r;
  ad_scalar_t a_63 =
      -0.016560 + 0.00484958 * l_l - 0.064722 * l_r - 0.00457059 * l_l * l_l + 0.01565593 * l_l * l_r + 0.01267579 * l_r * l_r;
  ad_scalar_t a_64 =
      -0.4561021 - 0.1488225 * l_l - 0.19647707 * l_r + 0.09755714 * l_l * l_l + 0.04557856 * l_l * l_r + 0.15340679 * l_r * l_r;
  ad_scalar_t a_72 =
      -5.2986744 + 7.16471817 * l_l + 4.71747812 * l_r - 2.89144876 * l_l * l_l - 9.11851375 * l_l * l_r + 1.29256768 * l_r * l_r;
  ad_scalar_t a_73 =
      0.0237555 + 0.366526 * l_l - 0.41890653 * l_r - 0.64768334 * l_l * l_l + 0.8430833 * l_l * l_r - 0.14636845 * l_r * l_r;
  ad_scalar_t a_74 =
      -5.82844783 + 7.79268178 * l_l + 3.49073496 * l_r - 3.49226094 * l_l * l_l - 7.91481553 * l_l * l_r + 2.15180379 * l_r * l_r;
  ad_scalar_t a_82 =
      0.025970435 + 0.38864448 * l_l - 0.44986877 * l_r - 0.25955702 * l_l * l_l + 0.01947447 * l_l * l_r + 0.29290284 * l_r * l_r;
  ad_scalar_t a_83 =
      -5.15910343 + -2.01256235e-3 * l_l + 6.82319497 * l_r - 2.0728509e-2 * l_l * l_l + 4.43084507e-2 * l_l * l_r - 5.814509 * l_r * l_r;
  ad_scalar_t a_84 = -5.38599 - 0.44977 * l_l + 6.255285 * l_r - 0.31738057 * l_l * l_l + 0.10541621 * l_l * l_r - 5.37627405 * l_r * l_r;
  ad_scalar_t a_92 =
      0.0507036 + 0.186040 * l_l + 0.13922033 * l_r - 0.12834599 * l_l * l_l - 0.02383292 * l_l * l_r - 0.18820767 * l_r * l_r;
  ad_scalar_t a_93 =
      0.0260886 - 0.0076399 * l_l + 0.10196273 * l_r + 0.00720039 * l_l * l_l - 0.02466397 * l_l * l_r - 0.01996913 * l_r * l_r;
  ad_scalar_t a_94 = 16.157215 + 0.23445098 * l_l + 0.309525 * l_r - 0.15368 * l_l * l_l - 0.07180336 * l_l * l_r - 0.24167335 * l_r * l_r;
  ad_scalar_t a_102 =
      0.17629847 + 1.73458918 * l_l - 1.4810232 * l_r - 1.194995 * l_l * l_l + 0.20866224 * l_l * l_r + 0.6922007 * l_r * l_r;
  ad_scalar_t a_103 =
      -0.1557644 - 0.01884951 * l_l - 0.5443118 * l_r - 0.0696846 * l_l * l_l + 0.13368658 * l_l * l_r + 0.16891895 * l_r * l_r;
  ad_scalar_t a_104 =
      0.02348896 + 2.03423561 * l_l - 2.2203314 * l_r - 1.4573463 * l_l * l_l + 0.50555285 * l_l * l_r + 0.75491052 * l_r * l_r;

  ad_matrix_t A = (ad_matrix_t(10, 10) <<  // clang-format off
                         O,   O,   O,   O,   O,a_16,   O,   O,   O,   O,
                         O,   O,   O,   O,   O,   O,a_27,   O,   O,   O,
                         O,   O,   O,   O,   O,   O,   O,a_38,   O,   O,
                         O,   O,   O,   O,   O,   O,   O,   O,a_49,   O,
                         O,   O,   O,   O,   O,   O,   O,   O,   O,a_510,
                         O,a_62,a_63,a_64,   O,   O,   O,   O,   O,   O,
                         O,a_72,a_73,a_74,   O,   O,   O,   O,   O,   O,
                         O,a_82,a_83,a_84,   O,   O,   O,   O,   O,   O,
                         O,a_92,a_93,a_94,   O,   O,   O,   O,   O,   O,
                         O,a_102,a_103,a_104,O,   O,   O,   O,   O,   O
                      ).finished();  // clang-format on

  return A;
}

ocs2::ad_matrix_t LeggedBalanceSystemDynamics::generateB(ad_scalar_t l_l, ad_scalar_t l_r) const {
  // Polynomial fitting
  ad_scalar_t O = ad_scalar_t(0);
  ad_scalar_t b_61 = 0.19639 - 0.03374 * l_l + 0.14396 * l_r - 0.01304 * l_l * l_l - 0.06308 * l_l * l_r - 0.1707 * l_r * l_r;
  ad_scalar_t b_62 = 0.14699 - 0.00116552 * l_l + 0.030625 * l_r + 0.00074787 * l_l * l_l - 0.01067047 * l_l * l_r - 0.029384 * l_r * l_r;
  ad_scalar_t b_63 = -0.48126 - 0.6427468 * l_l - 0.408710 * l_r + 0.39800877 * l_l * l_l + 0.06777286 * l_l * l_r + 0.422876 * l_r * l_r;
  ad_scalar_t b_64 = 0.495914 - 0.1144039 * l_l - 0.234377 * l_r + 0.05907991 * l_l * l_l - 0.01663537 * l_l * l_r - 0.073463 * l_r * l_r;
  ad_scalar_t b_71 = 10.92329 - 27.372703 * l_l - 8.194668 * l_r + 20.7009335 * l_l * l_l + 19.00684 * l_l * l_r - 2.0176599 * l_r * l_r;
  ad_scalar_t b_72 = 0.092901 - 0.6885225 * l_l + 1.000315 * l_r + 0.85263388 * l_l * l_l - 0.732619 * l_l * l_r - 0.6726392 * l_r * l_r;
  ad_scalar_t b_73 = -14.4437 + 7.2189129 * l_l + 8.924072 * l_r + 5.63648 * l_l * l_l - 19.44009 * l_l * l_r + 7.532362 * l_r * l_r;
  ad_scalar_t b_74 = -0.571972 - 6.480518 * l_l + 8.604025 * l_r + 8.38546 * l_l * l_l - 4.495299 * l_l * l_r - 6.313617 * l_r * l_r;
  ad_scalar_t b_81 = 0.0150799 - 0.637193 * l_l + 0.821757 * l_r + 0.87436 * l_l * l_l - 0.650049 * l_l * l_r - 0.449953 * l_r * l_r;
  ad_scalar_t b_82 =
      11.021698 - 1.1473e-2 * l_l - 2.9612e1 * l_r + 3.615209e-2 * l_l * l_l - 7.3457e-02 * l_l * l_r + 2.93378e1 * l_r * l_r;
  ad_scalar_t b_83 = -0.8934116 + 1.284348 * l_l - 1.59693 * l_r - 0.468298 * l_l * l_l - 0.29046 * l_l * l_r + 1.3832024 * l_r * l_r;
  ad_scalar_t b_84 = -13.655 + 0.055609 * l_l - 6.032097 * l_r + 0.25345224 * l_l * l_l - 0.62251 * l_l * l_r + 12.064728 * l_r * l_r;
  ad_scalar_t b_91 = -4.30578 + 0.053165 * l_l - 0.226803 * l_r + 0.02055693 * l_l * l_l + 0.09938 * l_l * l_r + 0.2689769 * l_r * l_r;
  ad_scalar_t b_92 = -4.22795 + 0.001836 * l_l - 0.0482462 * l_r - 0.00117817 * l_l * l_l + 0.01681 * l_l * l_r + 0.04629 * l_r * l_r;
  ad_scalar_t b_93 = -0.75816 + 1.012567 * l_l + 0.6438723 * l_r - 0.6270134 * l_l * l_l - 0.10676773 * l_l * l_r - 0.666189 * l_r * l_r;
  ad_scalar_t b_94 = -0.78125 + 0.180229 * l_l + 0.3692322 * l_r - 0.093073 * l_l * l_l + 0.02620696 * l_l * l_r - 0.115732 * l_r * l_r;
  ad_scalar_t b_101 = -0.4095269 - 2.319868 * l_l + 2.72256 * l_r + 3.40986 * l_l * l_l - 2.58956 * l_l * l_r - 0.954152 * l_r * l_r;
  ad_scalar_t b_102 = 0.3904 - 0.039623 * l_l + 0.089357 * l_r + 0.138409 * l_l * l_l - 0.267257 * l_l * l_r - 0.073397 * l_r * l_r;
  ad_scalar_t b_103 = -4.57234 + 6.309787 * l_l - 4.4699 * l_r - 2.74989 * l_l * l_l - 0.64275015 * l_l * l_r + 1.562874 * l_r * l_r;
  ad_scalar_t b_104 = 4.671049 + 0.480128 * l_l - 3.81003 * l_r + 0.84256 * l_l * l_l - 2.355031 * l_l * l_r + 2.57757 * l_r * l_r;

  ad_matrix_t B = (ad_matrix_t(10, 4) <<  // clang-format off
                         O,   O,   O,   O,
                         O,   O,   O,   O,
                         O,   O,   O,   O,
                         O,   O,   O,   O,
                         O,   O,   O,   O,
                      b_61,b_62,b_63,b_64,
                      b_71,b_72,b_73,b_74,
                      b_81,b_82,b_83,b_84,
                      b_91,b_92,b_93,b_94,
                      b_101,b_102,b_103,b_104
                      ).finished();  // clang-format on

  return B;
}

}  // namespace rm
