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
  ad_scalar_t a_62 = -0.24724045539306494 + (9.447623529293201) * l_l + (0.06812119732616839) * l_r + (3.94254240885233) * l_l * l_l +
                     (-1.9141221364444747) * l_l * l_r + (-0.03956669731274465) * l_r * l_r;
  ad_scalar_t a_63 = -0.022166488571220444 + (0.22758277197823715) * l_l + (-0.02461282034650958) * l_r +
                     (-0.06151804979328948) * l_l * l_l + (0.7140215626015767) * l_l * l_r + (-0.3670949205534176) * l_r * l_r;
  ad_scalar_t a_64 = -0.4065914942601602 + (-0.02550524662612847) * l_l + (-0.025505246626128394) * l_r +
                     (-0.003631921659981067) * l_l * l_l + (0.00944309981904222) * l_l * l_r + (-0.003631921659981153) * l_r * l_r;
  ad_scalar_t a_72 = 17.563500684716544 + (1092.7157958985156) * l_l + (-0.3624717848470027) * l_r + (-1949.5838327306492) * l_l * l_l +
                     (6.745152655211655) * l_l * l_r + (0.12081521354821234) * l_r * l_r;
  ad_scalar_t a_73 = 0.036482980071579754 + (-0.15695330598767016) * l_l + (-0.05980602949571537) * l_r +
                     (-1.5450422108990476) * l_l * l_l + (-1.68294536922974) * l_l * l_r + (1.1575322206506211) * l_r * l_r;
  ad_scalar_t a_74 = -0.15313827240843528 + (-2.1524246879005666) * l_l + (0.04250975593915829) * l_r + (3.952371535915845) * l_l * l_l +
                     (0.13534889333744077) * l_l * l_r + (0.009993875096188598) * l_r * l_r;
  ad_scalar_t a_82 = 1.285812323705684 + (-14.605889539758245) * l_l + (-12.672013823675613) * l_r + (-10.855667228907453) * l_l * l_l +
                     (-61.613259768341955) * l_l * l_r + (42.94350504652487) * l_r * l_r;
  ad_scalar_t a_83 = -4.0637597930871525 + (61.29598435795789) * l_l + (-10.77394207243573) * l_r + (0.17524201695610842) * l_l * l_l +
                     (-134.59764057058482) * l_l * l_r + (50.259883058316596) * l_r * l_r;
  ad_scalar_t a_84 = -0.1531382724084352 + (0.04250975593915851) * l_l + (-2.152424687900563) * l_r + (0.009993875096189625) * l_l * l_l +
                     (0.13534889333744077) * l_l * l_r + (3.9523715359158396) * l_r * l_r;
  ad_scalar_t a_92 = 0.3894966450966182 + (-14.883558044517315) * l_l + (-0.10731648983708553) * l_r + (-6.2109861388088685) * l_l * l_l +
                     (3.0154618072718677) * l_l * l_r + (0.062332419815219975) * l_r * l_r;
  ad_scalar_t a_93 = 0.03492055100099553 + (-0.3585284051770047) * l_l + (0.038774443025881206) * l_r + (0.09691405061230501) * l_l * l_l +
                     (-1.1248523334007707) * l_l * l_r + (0.578312476250098) * l_r * l_r;
  ad_scalar_t a_94 = 16.079217855208334 + (0.04018034984382026) * l_l + (0.04018034984382027) * l_r + (0.00572164170936371) * l_l * l_l +
                     (-0.014876431500604432) * l_l * l_r + (0.005721641709363705) * l_r * l_r;
  ad_scalar_t a_102 = 2.2013492171699287 + (-86.02233419621933) * l_l + (0.7970196059245538) * l_r + (-38.640850484272455) * l_l * l_l +
                      (-19.780990501849566) * l_l * l_r + (-0.394745519379061) * l_r * l_r;
  ad_scalar_t a_103 = -0.20137501719002096 + (1.9036930861551788) * l_l + (-0.1785326860396046) * l_r + (0.6059016266073536) * l_l * l_l +
                      (6.898595406819867) * l_l * l_r + (-3.682428873513967) * l_r * l_r;
  ad_scalar_t a_104 = 4.1066923104757096e-18 + (0.23407594217239253) * l_l + (-0.23407594217239208) * l_r +
                      (0.035592597346284574) * l_l * l_l + (9.367506770274758e-17) * l_l * l_r + (-0.03559259734628466) * l_r * l_r;

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
  ad_scalar_t b_61 = 0.1304111001576438 + (0.3602745011975263) * l_l + (-0.0003827527551188703) * l_r + (-0.6484876653842095) * l_l * l_l +
                     (-0.024684041604198147) * l_l * l_r + (-0.0007019854127603281) * l_r * l_r;
  ad_scalar_t b_62 = 0.1304111001576438 + (-0.0003827527551187593) * l_l + (0.36027450119752646) * l_r +
                     (-0.0007019854127606456) * l_l * l_l + (-0.024684041604198667) * l_l * l_r + (-0.64848766538421) * l_r * l_r;
  ad_scalar_t b_63 = 0.5487094330064273 + (-0.41430753532309267) * l_l + (-0.09931990703780416) * l_r + (-0.19658342071762983) * l_l * l_l +
                     (0.1032174384033582) * l_l * l_r + (-0.012410005346130404) * l_r * l_r;
  ad_scalar_t b_64 = 0.5487094330064273 + (-0.09931990703780413) * l_l + (-0.41430753532309283) * l_r +
                     (-0.012410005346130445) * l_l * l_l + (0.10321743840335858) * l_l * l_r + (-0.19658342071762963) * l_r * l_r;
  ad_scalar_t b_71 = 12.633366364121917 + (-45.50031168582462) * l_l + (-0.001674041984454533) * l_r + (62.239950184802694) * l_l * l_l +
                     (0.09051869562527326) * l_l * l_r + (0.002106340813647023) * l_r * l_r;
  ad_scalar_t b_72 = 0.05216187366144019 + (-0.27509024794901676) * l_l + (-0.8432609796401014) * l_r + (0.7624074299020027) * l_l * l_l +
                     (-0.7988037884331599) * l_l * l_r + (1.7829133527267544) * l_r * l_r;
  ad_scalar_t b_73 = -0.4181294383074379 + (-50.381076368422356) * l_l + (0.17435420679628777) * l_r + (89.12078733992048) * l_l * l_l +
                     (0.27676439054641744) * l_l * l_r + (0.03372942074201024) * l_r * l_r;
  ad_scalar_t b_74 = -0.6244738447194612 + (-7.733810806919665) * l_l + (0.6341733176058044) * l_r + (13.508975691499966) * l_l * l_l +
                     (2.6819539982950062) * l_l * l_r + (0.5410929744818938) * l_r * l_r;
  ad_scalar_t b_81 = 0.05216187366144022 + (-0.8432609796401005) * l_l + (-0.27509024794901926) * l_r + (1.7829133527267529) * l_l * l_l +
                     (-0.7988037884331596) * l_l * l_r + (0.7624074299020069) * l_r * l_r;
  ad_scalar_t b_82 = 12.633366364121919 + (-0.001674041984429664) * l_l + (-45.500311685824684) * l_r +
                     (0.0021063408136337003) * l_l * l_l + (0.09051869562519554) * l_l * l_r + (62.239950184802815) * l_r * l_r;
  ad_scalar_t b_83 = -0.6244738447194612 + (0.6341733176058064) * l_l + (-7.733810806919674) * l_r + (0.5410929744818944) * l_l * l_l +
                     (2.6819539982950067) * l_l * l_r + (13.508975691499986) * l_r * l_r;
  ad_scalar_t b_84 = -0.41812943830743965 + (0.17435420679625935) * l_l + (-50.3810763684224) * l_r + (0.03372942074210217) * l_l * l_l +
                     (0.2767643905464785) * l_l * l_r + (89.12078733992055) * l_r * l_r;
  ad_scalar_t b_91 = -4.201833765005014 + (-0.5675677522402356) * l_l + (0.000602979450847807) * l_r + (1.0216118136982388) * l_l * l_l +
                     (0.03888664327598085) * l_l * l_r + (0.0011058908734895845) * l_r * l_r;
  ad_scalar_t b_92 = -4.201833765005013 + (0.0006029794508467246) * l_l + (-0.5675677522402378) * l_r +
                     (0.0011058908734910104) * l_l * l_l + (0.038886643275982256) * l_l * l_r + (1.0216118136982468) * l_r * l_r;
  ad_scalar_t b_93 = -0.8644235950346255 + (0.6526900898562241) * l_l + (0.156466183986885) * l_r + (0.3096927755184884) * l_l * l_l +
                     (-0.16260626081463805) * l_l * l_r + (0.0195504228475243) * l_r * l_r;
  ad_scalar_t b_94 = -0.8644235950346254 + (0.1564661839868854) * l_l + (0.6526900898562216) * l_r + (0.01955042284752498) * l_l * l_l +
                     (-0.16260626081463958) * l_l * l_r + (0.30969277551849267) * l_r * l_r;
  ad_scalar_t b_101 = -0.22319818037202083 + (-3.445331243900265) * l_l + (-0.0016611782587174773) * l_r + (6.354275711437241) * l_l * l_l +
                      (-0.257553107575101) * l_l * l_r + (-0.006969894111447161) * l_r * l_r;
  ad_scalar_t b_102 = 0.22319818037202083 + (0.0016611782587176993) * l_l + (3.445331243900267) * l_r + (0.00696989411144619) * l_l * l_l +
                      (0.25755310757509814) * l_l * l_r + (-6.3542757114372455) * l_r * l_r;
  ad_scalar_t b_103 = -5.0465793155039185 + (3.7689647437321603) * l_l + (-0.9175845790159323) * l_r + (1.9266014036934025) * l_l * l_l +
                      (0.6959807006530497) * l_l * l_r + (-0.12137111793094524) * l_r * l_r;
  ad_scalar_t b_104 = 5.046579315503919 + (0.917584579015932) * l_l + (-3.768964743732158) * l_r + (0.12137111793094277) * l_l * l_l +
                      (-0.695980700653051) * l_l * l_r + (-1.9266014036934143) * l_r * l_r;

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
