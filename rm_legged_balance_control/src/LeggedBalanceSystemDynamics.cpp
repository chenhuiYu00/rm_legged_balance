//
// Created by yuchen on 23-9-2.
//

#include "rm_legged_balance_control/LeggedBalanceSystemDynamics.h"

namespace rm {
// On the dynamic model of a two-wheeled inverted pendulum robot
// State = [x, theta_l, theta_r, alpha, psi, x_dot, theta_l_dot, theta_r_dot, alpha_dot, psi_dot]^T
// Input = [tau_l_l, tau_l_r, tau_l, tau_r]^T
ad_vector_t LeggedBalanceSystemDynamics::systemFlowMap(ad_scalar_t /*time*/, const ad_vector_t& state, const ad_vector_t& input,
                                                       const ad_vector_t& parameters) const {
  // todo: add linear equation
  ad_scalar_t theta_l = state(1), theta_r = state(2), theta = state(3), psi = state(4), x_dot = state(5), theta_l_dot = state(6),
              theta_r_dot = state(7), theta_dot = state(8), psi_dot = state(9);

  ad_matrix_t A = generateA(parameters(0), parameters(0)), B = generateB(parameters(0), parameters(0));

  ad_vector_t result(10);
  result = A * state + B * input;

  return result;
}

vector_t LeggedBalanceSystemDynamics::getFlowMapParameters(scalar_t /*time*/, const PreComputation& /* preComputation */) const {
  vector_t v(1);
  // Left pendulum length # Right pendulum length
  v[0] = balanceControlCmdPtr_->getLegCmd();
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
  ad_scalar_t a_62 = -0.2620966751169891 + (9.57793177731217) * l_l + (0.06874741419898811) * l_r + (3.7253737974547585) * l_l * l_l +
                     (-1.9322328259917458) * l_l * l_r + (-0.03466225401535561) * l_r * l_r;
  ad_scalar_t a_63 = -0.022523325575806948 + (0.23197858391932916) * l_l + (-0.025804372346710985) * l_r +
                     (-0.062261252746585345) * l_l * l_l + (0.703208741836252) * l_l * l_r + (-0.3624336556693663) * l_r * l_r;
  ad_scalar_t a_64 = -0.40653044321465986 + (-0.025777007085373158) * l_l + (-0.025777007085373067) * l_r +
                     (-0.0031840388270436386) * l_l * l_l + (0.009523232137388522) * l_l * l_r + (-0.0031840388270437292) * l_r * l_r;
  ad_scalar_t a_72 = 19.03007313695322 + (1080.6988583654243) * l_l + (-0.3728389741152114) * l_r + (-1931.8112405000848) * l_l * l_l +
                     (6.832666224339661) * l_l * l_r + (0.10546165664612772) * l_r * l_r;
  ad_scalar_t a_73 = 0.03849322091884316 + (-0.18554642311004133) * l_l + (-0.04568833543440948) * l_r + (-1.517161109761889) * l_l * l_l +
                     (-1.6710189650507599) * l_l * l_r + (1.1341745944084027) * l_r * l_r;
  ad_scalar_t a_74 = -0.1564142644967046 + (-2.1270122848588446) * l_l + (0.0440434426173868) * l_r + (3.91502880815082) * l_l * l_l +
                     (0.13337249813827537) * l_l * l_r + (0.008826844020950797) * l_r * l_r;
  ad_scalar_t a_82 = 1.3398277063931623 + (-15.24789896395535) * l_l + (-12.463794568851107) * l_r + (-10.329382652979568) * l_l * l_l +
                     (-60.95226204931455) * l_l * l_r + (42.449798798416055) * l_r * l_r;
  ad_scalar_t a_83 = -4.032748070806581 + (60.924395530280975) * l_l + (-10.659442955262612) * l_r + (0.1777221473532702) * l_l * l_l +
                     (-133.74706488270547) * l_l * l_r + (49.89034436610246) * l_r * l_r;
  ad_scalar_t a_84 = -0.15641426449670492 + (0.04404344261738524) * l_l + (-2.1270122848588406) * l_r + (0.008826844020946703) * l_l * l_l +
                     (0.13337249813828866) * l_l * l_r + (3.915028808150809) * l_r * l_r;
  ad_scalar_t a_92 = 0.4129007750238469 + (-15.088842512834255) * l_l + (-0.10830301678179133) * l_r + (-5.868863950815158) * l_l * l_l +
                     (3.043992950396644) * l_l * l_r + (0.054606078236680844) * l_r * l_r;
  ad_scalar_t a_93 = 0.03548270340405506 + (-0.36545346119507754) * l_l + (0.04065158528319823) * l_r + (0.0980848746041737) * l_l * l_l +
                     (-1.107818076586169) * l_l * l_r + (0.5709692320736615) * l_r * l_r;
  ad_scalar_t a_94 = 16.079121676865007 + (0.04060847471107735) * l_l + (0.04060847471107743) * l_r + (0.005016057906133275) * l_l * l_l +
                     (-0.015002670020531295) * l_l * l_r + (0.005016057906132911) * l_r * l_r;
  ad_scalar_t a_102 = 2.3401191646209494 + (-87.23397560050579) * l_l + (0.8073048699106522) * l_r + (-36.51031145785336) * l_l * l_l +
                      (-19.961080299333673) * l_l * l_r + (-0.34527539413986474) * l_r * l_r;
  ad_scalar_t a_103 = -0.20436267665752839 + (1.9439615078105494) * l_l + (-0.19390977448401236) * l_r + (0.6128365295533835) * l_l * l_l +
                      (6.785771812023782) * l_l * l_r + (-3.627805247180045) * l_r * l_r;
  ad_scalar_t a_104 = -5.5100368712146515e-17 + (0.2366026879320562) * l_l + (-0.23660268793205572) * l_r +
                      (0.031203985318330556) * l_l * l_l + (1.7694179454963432e-16) * l_l * l_r + (-0.03120398531833087) * l_r * l_r;

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
  ad_scalar_t b_61 = 0.1309335863287238 + (0.35616924822120993) * l_l + (-0.0005644114319595384) * l_r + (-0.6424676898122653) * l_l * l_l +
                     (-0.02438003597469933) * l_l * l_r + (-0.000626163418879) * l_r * l_r;
  ad_scalar_t b_62 = 0.13093358632872384 + (-0.0005644114319592608) * l_l + (0.3561692482212103) * l_r +
                     (-0.0006261634188786131) * l_l * l_l + (-0.024380035974701342) * l_l * l_r + (-0.6424676898122651) * l_r * l_r;
  ad_scalar_t b_63 = 0.5495090403768209 + (-0.42028963959846494) * l_l + (-0.1003857665955703) * l_r + (-0.18668463126308077) * l_l * l_l +
                     (0.10419291159378304) * l_l * l_r + (-0.010881205107289155) * l_r * l_r;
  ad_scalar_t b_64 = 0.549509040376821 + (-0.10038576659557064) * l_l + (-0.42028963959846477) * l_r + (-0.010881205107289037) * l_l * l_l +
                     (0.10419291159378397) * l_l * l_r + (-0.18668463126308102) * l_r * l_r;
  ad_scalar_t b_71 = 12.59740025353221 + (-45.21305533539096) * l_l + (-0.0014864311526707752) * l_r + (61.84154905363865) * l_l * l_l +
                     (0.09125339988247294) * l_l * l_r + (0.001869102631267161) * l_r * l_r;
  ad_scalar_t b_72 = 0.05233677043454847 + (-0.28106000764233596) * l_l + (-0.8430496035887196) * l_r + (0.7677146098508235) * l_l * l_l +
                     (-0.7747126090867456) * l_l * l_r + (1.7788562566192405) * l_r * l_r;
  ad_scalar_t b_73 = -0.48563666575768316 + (-49.833830811667696) * l_l + (0.18056267878854015) * l_r + (88.31401334414645) * l_l * l_l +
                     (0.2661554422557417) * l_l * l_r + (0.02983606990895371) * l_r * l_r;
  ad_scalar_t b_74 = -0.638846525203375 + (-7.645192999398526) * l_l + (0.6631251761945922) * l_r + (13.385295625176516) * l_l * l_l +
                     (2.652168186735336) * l_l * l_r + (0.5174576868430275) * l_r * l_r;
  ad_scalar_t b_81 = 0.05233677043454857 + (-0.8430496035887192) * l_l + (-0.28106000764233696) * l_r + (1.7788562566192392) * l_l * l_l +
                     (-0.7747126090867487) * l_l * l_r + (0.7677146098508277) * l_r * l_r;
  ad_scalar_t b_82 = 12.5974002535322 + (-0.0014864311526920915) * l_l + (-45.213055335391) * l_r + (0.0018691026312152026) * l_l * l_l +
                     (0.09125339988263237) * l_l * l_r + (61.84154905363866) * l_r * l_r;
  ad_scalar_t b_83 = -0.6388465252033767 + (0.6631251761945891) * l_l + (-7.645192999398533) * l_r + (0.5174576868430132) * l_l * l_l +
                     (2.6521681867353784) * l_l * l_r + (13.38529562517651) * l_r * l_r;
  ad_scalar_t b_84 = -0.48563666575769915 + (0.1805626787885295) * l_l + (-49.83383081166772) * l_r + (0.02983606990885823) * l_l * l_l +
                     (0.26615544225599397) * l_l * l_r + (88.3140133441464) * l_r * l_r;
  ad_scalar_t b_91 = -4.202656877106233 + (-0.5611004358012412) * l_l + (0.000889160145142176) * l_r + (1.012128089503072) * l_l * l_l +
                     (0.03840772014589469) * l_l * l_r + (0.0009864427346570614) * l_r * l_r;
  ad_scalar_t b_92 = -4.202656877106233 + (0.0008891601451413433) * l_l + (-0.5611004358012407) * l_r +
                     (0.0009864427346579392) * l_l * l_l + (0.0384077201458982) * l_l * l_r + (1.0121280895030695) * l_r * l_r;
  ad_scalar_t b_93 = -0.8656832771835998 + (0.6621141525249776) * l_l + (0.15814531340457805) * l_r + (0.29409846156637065) * l_l * l_l +
                     (-0.16414299773112856) * l_l * l_r + (0.017141987856150856) * l_r * l_r;
  ad_scalar_t b_94 = -0.8656832771835997 + (0.15814531340457816) * l_l + (0.6621141525249763) * l_r + (0.017141987856151134) * l_l * l_l +
                     (-0.16414299773112917) * l_l * l_r + (0.29409846156637065) * l_r * l_r;
  ad_scalar_t b_101 = -0.2278068204757711 + (-3.405843259740182) * l_l + (-0.0032791001004284226) * l_r + (6.294982832040107) * l_l * l_l +
                      (-0.2551843854860277) * l_l * l_r + (-0.006207387239410889) * l_r * l_r;
  ad_scalar_t b_102 = 0.22780682047577172 + (0.0032791001004324194) * l_l + (3.405843259740185) * l_r + (0.006207387239412082) * l_l * l_l +
                      (0.2551843854860024) * l_l * l_r + (-6.2949828320401044) * l_r * l_r;
  ad_scalar_t b_103 = -5.05186591186714 + (3.8246702768923373) * l_l + (-0.9276508221143689) * l_r + (1.8294946535474788) * l_l * l_l +
                      (0.7026139141415749) * l_l * l_r + (-0.10644072261697324) * l_r * l_r;
  ad_scalar_t b_104 = 5.05186591186714 + (0.9276508221143656) * l_l + (-3.8246702768923426) * l_r + (0.10644072261697612) * l_l * l_l +
                      (-0.7026139141415758) * l_l * l_r + (-1.8294946535474716) * l_r * l_r;

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
