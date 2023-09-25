//
// Created by yuchen on 23-9-2.
//

#include "rm_legged_balance_control/LeggedBalanceSystemDynamics.h"

namespace rm {
// On the dynamic model of a two-wheeled inverted pendulum robot
// State = [x, theta, psi, x_dot, theta_dot, psi_dot]^T
// Input = [tau_l, tau_r]^T
ad_vector_t LeggedBalanceSystemDynamics::systemFlowMap(ad_scalar_t /*time*/, const ad_vector_t& state, const ad_vector_t& input,
                                                       const ad_vector_t& parameters) const {
  // todo: add linear equation
  ad_scalar_t theta = state(1), psi = state(2), x_dot = state(3), theta_dot = state(4), psi_dot = state(5);
  ad_scalar_t m_11 = ad_scalar_t(1);
  ad_scalar_t m_12 = ad_scalar_t(1);
  ad_scalar_t m_22 = ad_scalar_t(1);
  ad_scalar_t m_33 = ad_scalar_t(1);
  ad_scalar_t v_1 = ad_scalar_t(1);
  ad_scalar_t v_2 = ad_scalar_t(1);
  ad_scalar_t v_3 = ad_scalar_t(1);
  ad_scalar_t g_2 = ad_scalar_t(1);
  ad_scalar_t b_11 = ad_scalar_t(1);
  ad_scalar_t b_12 = ad_scalar_t(1);
  ad_scalar_t b_21 = ad_scalar_t(1);
  ad_scalar_t b_22 = ad_scalar_t(1);
  ad_scalar_t b_31 = ad_scalar_t(1);
  ad_scalar_t b_32 = ad_scalar_t(1);

  ad_matrix_t b = (ad_matrix_t(3, 2) <<  // clang-format off
                       b_11, b_12,
                       b_21, b_22,
                       b_31, b_32).finished();  // clang-format on
  ad_vector_t v = (ad_vector_t(3) <<  // clang-format off
                       v_1,
                       v_2,
                       v_3).finished();  // clang-format on
  ad_vector_t g = (ad_vector_t(3) <<  // clang-format off
                       ad_scalar_t(0),
                       g_2,
                       ad_scalar_t(0)).finished();  // clang-format on

  ad_matrix_t m = (ad_matrix_t(3, 3) <<  // clang-format off
                       m_11, m_12, ad_scalar_t(0),
                       m_12, m_22, ad_scalar_t(0),
                       ad_scalar_t(0), ad_scalar_t(0), m_33).finished();  // clang-format on

  ad_scalar_t det = m_11 * m_22 * m_33 - m_12 * m_12 * m_33;
  ad_matrix_t m_inv =
      1 / det *
      (ad_matrix_t(3, 3) <<  // clang-format off
                m_33 * m_22, -m_33 * m_12, ad_scalar_t(0),
                -m_33 * m_12, m_33 * m_11, ad_scalar_t(0),
                ad_scalar_t(0), ad_scalar_t(0), m_22 * m_11 - m_12 * m_12).finished();  // clang-format on
  ad_vector_t qdd(3);
  qdd = m_inv * (b * input - g - v);

  ad_vector_t result(6);
  result(0) = state(3);
  result(1) = state(4);
  result(2) = state(5);
  result.tail(3) = qdd;
  return result;
}

vector_t LeggedBalanceSystemDynamics::getFlowMapParameters(scalar_t /*time*/, const PreComputation& /* preComputation */) const {
  vector_t v(2);
  v << balanceControlCmdPtr_->getPendulumLength()(0), balanceControlCmdPtr_->getPendulumLength()(1);
  return v;
}

void LeggedBalanceSystemDynamics::loadDynamicsParams(const std::string& filename) {
  ocs2::loadData::loadCppDataType(filename, "Dynamics.d", param_.d_);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.l_1", param_.l_1);
  ocs2::loadData::loadCppDataType(filename, "Dynamics.l_2", param_.l_2);
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

}  // namespace rm
