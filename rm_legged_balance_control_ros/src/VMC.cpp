//
// Created by yuchen on 23-9-20.
//
#include "rm_legged_balance_control_ros/VMC.h"

namespace rm {
VMC::VMC(ocs2::scalar_t l_a, ocs2::scalar_t l_u, ocs2::scalar_t l_d) {
  l_a_ = l_a;
  l_u_ = l_u;
  l_d_ = l_d;
}

ocs2::matrix_t VMC::pendulumEff2JointEff(ocs2::scalar_t F_bl, ocs2::scalar_t T_bl, ocs2::scalar_t front_joint_angle,
                                         ocs2::scalar_t back_joint_angle) const {
  ocs2::scalar_t xe, ye, x1, y1, x2, y2, e;
  ocs2::matrix_t joint(2, 1), virtual_eff(2, 1);
  ocs2::scalar_t l(0), theta(0);
  ocs2::matrix_t J(2, 2), J_inv(2, 2), J_inv_T(2, 2);
  ocs2::scalar_t j11(0), j12(0), j21(0), j22(0);

  x1 = l_a_ - l_u_ * cos(front_joint_angle);
  x2 = l_a_ - l_u_ * cos(back_joint_angle);
  y1 = l_u_ * sin(front_joint_angle);
  y2 = l_u_ * sin(back_joint_angle);
  e = sqrt(-(-l_d_ * l_d_ - 2 * l_d_ * l_d_ - l_d_ * l_d_ + x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2) *
           (-l_d_ * l_d_ + 2 * l_d_ * l_d_ - l_d_ * l_d_ + x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));

  xe = (y1 * e - y2 * e + x1 * x2 * x2 - x1 * x1 * x2 - x1 * y1 * y1 - x1 * y2 * y2 + x2 * y1 * y1 + x2 * y2 * y2 - x1 * x1 * x1 +
        x2 * x2 * x2 + 2 * x1 * y1 * y2 - 2 * x2 * y1 * y2) /
       (2 * (x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));
  ye = (x1 * e + x2 * e + x1 * x1 * y1 + x1 * x1 * y2 + x2 * x2 * y1 + x2 * x2 * y2 - y1 * y2 * y2 - y1 * y1 * y2 + y1 * y1 * y1 +
        y2 * y2 * y2 + 2 * x1 * x2 * y1 + 2 * x1 * x2 * y2) /
       (2 * (x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));

  l = sqrt(xe * xe + ye * ye);
  theta = atan(xe / ye);

  j11 = 1 / l_u_ * ((xe + x1) * sin(theta) + (ye - y1) * cos(theta)) /
        (-(xe + x1) * sin(front_joint_angle) + (ye - y1) * cos(front_joint_angle));
  j12 = l / l_u_ * ((xe + x1) * cos(theta) - (ye - y1) * sin(theta)) /
        (-(xe + x1) * sin(front_joint_angle) + (ye - y1) * cos(front_joint_angle));
  j21 = 1 / l_u_ * ((xe - x2) * sin(theta) + (ye - y2) * cos(theta)) /
        ((xe - x2) * sin(back_joint_angle) + (ye - y2) * cos(back_joint_angle));
  j22 = l / l_u_ * ((xe - x2) * cos(theta) - (ye - y2) * sin(theta)) /
        ((xe - x2) * sin(back_joint_angle) + (ye - y2) * cos(back_joint_angle));

  J << j11, j12, j21, j22;
  ocs2::scalar_t det = j11 * j22 - j12 * j21;
  J_inv = 1 / det * (ocs2::matrix_t(2, 2) << j22, -j12, -j21, j22).finished();
  J_inv_T = (ocs2::matrix_t(2, 2) << J_inv(0, 0), J_inv(1, 0), J_inv(0, 1), J_inv(1, 1)).finished();

  virtual_eff << F_bl, T_bl;
  joint = J_inv_T * virtual_eff;

  joint(0) = -joint(0);

  return joint;
}

ocs2::matrix_t VMC::jointPos2Pendulum(ocs2::scalar_t front_joint_angle, ocs2::scalar_t back_joint_angle, ocs2::scalar_t front_joint_vel,
                                      ocs2::scalar_t back_joint_vel) const {
  ocs2::scalar_t xe(0), ye(0), x1(0), y1(0), x2(0), y2(0), e(0);
  ocs2::scalar_t l(0), theta(0);
  ocs2::scalar_t j11(0), j12(0), j21(0), j22(0);

  x1 = l_a_ - l_u_ * cos(front_joint_angle);
  x2 = l_a_ - l_u_ * cos(back_joint_angle);
  y1 = l_u_ * sin(front_joint_angle);
  y2 = l_u_ * sin(back_joint_angle);
  e = sqrt(-(-l_d_ * l_d_ - 2 * l_d_ * l_d_ - l_d_ * l_d_ + x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2) *
           (-l_d_ * l_d_ + 2 * l_d_ * l_d_ - l_d_ * l_d_ + x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));

  xe = (y1 * e - y2 * e + x1 * x2 * x2 - x1 * x1 * x2 - x1 * y1 * y1 - x1 * y2 * y2 + x2 * y1 * y1 + x2 * y2 * y2 - x1 * x1 * x1 +
        x2 * x2 * x2 + 2 * x1 * y1 * y2 - 2 * x2 * y1 * y2) /
       (2 * (x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));
  ye = (x1 * e + x2 * e + x1 * x1 * y1 + x1 * x1 * y2 + x2 * x2 * y1 + x2 * x2 * y2 - y1 * y2 * y2 - y1 * y1 * y2 + y1 * y1 * y1 +
        y2 * y2 * y2 + 2 * x1 * x2 * y1 + 2 * x1 * x2 * y2) /
       (2 * (x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));

  l = sqrt(xe * xe + ye * ye);
  theta = atan(xe / ye);

  j11 = 1 / l_u_ * ((xe + x1) * sin(theta) + (ye - y1) * cos(theta)) /
        (-(xe + x1) * sin(front_joint_angle) + (ye - y1) * cos(front_joint_angle));
  j12 = l / l_u_ * ((xe + x1) * cos(theta) - (ye - y1) * sin(theta)) /
        (-(xe + x1) * sin(front_joint_angle) + (ye - y1) * cos(front_joint_angle));
  j21 = 1 / l_u_ * ((xe - x2) * sin(theta) + (ye - y2) * cos(theta)) /
        ((xe - x2) * sin(back_joint_angle) + (ye - y2) * cos(back_joint_angle));
  j22 = l / l_u_ * ((xe - x2) * cos(theta) - (ye - y2) * sin(theta)) /
        ((xe - x2) * sin(back_joint_angle) + (ye - y2) * cos(back_joint_angle));

  ocs2::matrix_t J(2, 2), J_inv(2, 2);
  J << j11, j12, j21, j22;
  ocs2::scalar_t det = j11 * j22 - j12 * j21;
  J_inv = 1 / det * (ocs2::matrix_t(2, 2) << j22, -j12, -j21, j22).finished();

  ocs2::matrix_t joint_dot(2, 1), pendulum_dot(2, 1);
  joint_dot << front_joint_vel, back_joint_vel;
  pendulum_dot = J_inv * joint_dot;

  ocs2::matrix_t m(4, 1);
  m << l, theta, pendulum_dot(0), pendulum_dot(1);

  return m;
}

}  // namespace rm