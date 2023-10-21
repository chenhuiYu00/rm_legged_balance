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
  v[0] = balanceControlCmdPtr_->getLegCmd();
  v[1] = balanceControlCmdPtr_->getLegCmd();
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
  ad_scalar_t a_62 = -0.2977253987041173 - 0.2977253987041173 + (9.96995726878815) * l_l + (0.06548610026435941) * l_r +
                     (2.7783730165865053) * l_l * l_l + (-1.9573862749515512) * l_l * l_r + (-0.014835308409558268) * l_r * l_r;
  ad_scalar_t a_63 = -0.0238056325815261 + (0.24300336436578124) * l_l + (-0.022605905388387554) * l_r +
                     (-0.06362839300432058) * l_l * l_l + (0.6564106023682916) * l_l * l_r + (-0.3531457601277728) * l_r * l_r;
  ad_scalar_t a_64 = -0.40638600542804043 - 0.40638600542804043 + (-0.026565458236246488) * l_l + (-0.02656545823624658) * l_r +
                     (-0.0012976288901899351) * l_l * l_l + (0.0096151621032908) * l_l * l_r + (-0.001297628890189943) * l_r * l_r;
  ad_scalar_t a_72 = 22.825588391237957 + (1039.4529514776366) * l_l + (-0.3810296712182435) * l_r + (-1831.5360995788765) * l_l * l_l +
                     (6.991833928909955) * l_l * l_r + (0.045310069660985164) * l_r * l_r;
  ad_scalar_t a_73 = 0.04517931533201555 + (-0.26516327930143) * l_l + (-0.03800988226208024) * l_r + (-1.3805579537700077) * l_l * l_l +
                     (-1.5938560502179715) * l_l * l_r + (1.1001375171761811) * l_r * l_r;
  ad_scalar_t a_74 = -0.16496967092459935 + (-2.038285300951328) * l_l + (0.04836375609510668) * l_r + (3.702514837188131) * l_l * l_l +
                     (0.1237832654356632) * l_l * l_r + (0.0036800310748435483) * l_r * l_r;
  ad_scalar_t a_82 = 1.521556114451272 + (-17.108987803608606) * l_l + (-12.587127240729608) * l_r + (-7.848057765157497) * l_l * l_l +
                     (-57.38821029021776) * l_l * l_r + (41.534250246908066) * l_r * l_r;
  ad_scalar_t a_83 = -3.907769184662282 + (59.93894971942578) * l_l + (-11.056836447167633) * l_r + (0.1836482662821961) * l_l * l_l +
                     (-129.42689827266634) * l_l * l_r + (49.31744629460262) * l_r * l_r;
  ad_scalar_t a_84 = -0.16496967092462037 + (0.04836375609510857) * l_l + (-2.0382853009513275) * l_r +
                     (0.0036800310748383025) * l_l * l_l + (0.1237832654356685) * l_l * l_r + (3.7025148371881227) * l_r * l_r;
  ad_scalar_t a_92 = 0.46902940609353916 + (-15.706430008697279) * l_l + (-0.10316522153657637) * l_r + (-4.376981781023584) * l_l * l_l +
                     (3.083619086689229) * l_l * l_r + (0.023371186747355388) * l_r * l_r;
  ad_scalar_t a_93 = 0.037502818906259096 + (-0.38282163417466997) * l_l + (0.0356127976473366) * l_r + (0.10023863436374818) * l_l * l_l +
                     (-1.034093417365026) * l_l * l_r + (0.5563373056451697) * l_r * l_r;
  ad_scalar_t a_94 = 16.07889413306267 + (0.04185058162112803) * l_l + (0.041850581621128286) * l_r + (0.002044253229132152) * l_l * l_l +
                     (-0.015147494269649044) * l_l * l_r + (0.002044253229132079) * l_r * l_r;
  ad_scalar_t a_102 = 2.6820131563079386 + (-90.9522469658351) * l_l + (0.7912693175846641) * l_r + (-27.27995538561555) * l_l * l_l +
                      (-20.282647467675474) * l_l * l_r + (-0.14784012658074275) * l_r * l_r;
  ad_scalar_t a_103 = -0.21589881509737974 + (2.042923567307077) * l_l + (-0.16641974103289336) * l_r + (0.6269518673875466) * l_l * l_l +
                      (6.329719771946127) * l_l * l_r + (-3.531828235016009) * l_r * l_r;
  ad_scalar_t a_104 = -1.4590346708587277e-15 + (0.24415244025104021) * l_l + (-0.24415244025104138) * l_r +
                      (0.01275013814047386) * l_l * l_l + (1.6479873021779667e-15) * l_l * l_r + (-0.012750138140473648) * l_r * l_r;

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
  ad_scalar_t b_61 = 0.13230196617909035 + (0.34179430614089235) * l_l + (-0.001070202177059193) * l_r + (-0.608037093266667) * l_l * l_l +
                     (-0.022828192625752815) * l_l * l_r + (-0.0002687803034695513) * l_r * l_r;
  ad_scalar_t b_62 = 0.13230196617909384 + (-0.0010702021770594705) * l_l + (0.3417943061408924) * l_r +
                     (-0.00026878030346864056) * l_l * l_l + (-0.022828192625753585) * l_l * l_r + (-0.6080370932666651) * l_r * l_r;
  ad_scalar_t b_63 = 0.551426445119832 + (-0.4383034053438054) * l_l + (-0.10329394189681385) * l_r + (-0.14325720250157817) * l_l * l_l +
                     (0.10554534414696201) * l_l * l_r + (-0.004409875147106362) * l_r * l_r;
  ad_scalar_t b_64 = 0.5514264451198299 + (-0.10329394189681246) * l_l + (-0.4383034053438073) * l_r + (-0.004409875147106793) * l_l * l_l +
                     (0.10554534414696295) * l_l * l_r + (-0.14325720250157792) * l_r * l_r;
  ad_scalar_t b_71 = 12.502093485430827 + (-44.17964949446687) * l_l + (-0.000527492653688455) * l_r + (59.33248960988332) * l_l * l_l +
                     (0.08919814648345614) * l_l * l_r + (0.0008043236602404136) * l_r * l_r;
  ad_scalar_t b_72 = 0.052480401629393686 + (-0.2940656024468905) * l_l + (-0.8342060721780021) * l_r + (0.7573687012998123) * l_l * l_l +
                     (-0.677367691683652) * l_l * l_r + (1.7148322301501016) * l_r * l_r;
  ad_scalar_t b_73 = -0.6607794248679433 + (-47.94681409019035) * l_l + (0.19732215969128575) * l_r + (83.73872525230058) * l_l * l_l +
                     (0.2238850492835267) * l_l * l_r + (0.012392335330189574) * l_r * l_r;
  ad_scalar_t b_74 = -0.6778652813441739 + (-7.3046762527809115) * l_l + (0.7470937158337096) * l_r + (12.610694575695797) * l_l * l_l +
                     (2.493567644059617) * l_l * l_r + (0.40454424146804935) * l_r * l_r;
  ad_scalar_t b_81 = 0.052480401629399154 + (-0.8342060721780005) * l_l + (-0.2940656024468933) * l_r + (1.7148322301501027) * l_l * l_l +
                     (-0.6773676916836537) * l_l * l_r + (0.7573687012998159) * l_r * l_r;
  ad_scalar_t b_82 = 12.502093485430418 + (-0.0005274926536316116) * l_l + (-44.17964949446703) * l_r +
                     (0.0008043236601285031) * l_l * l_l + (0.08919814648360558) * l_l * l_r + (59.33248960988338) * l_r * l_r;
  ad_scalar_t b_83 = -0.6778652813442511 + (0.7470937158337075) * l_l + (-7.304676252780927) * l_r + (0.4045442414680385) * l_l * l_l +
                     (2.4935676440596426) * l_l * l_r + (12.61069457569579) * l_r * l_r;
  ad_scalar_t b_84 = -0.6607794248684353 + (0.19732215969132127) * l_l + (-47.94681409019037) * l_r + (0.01239233533009454) * l_l * l_l +
                     (0.2238850492836464) * l_l * l_r + (83.7387252523004) * l_r * l_r;
  ad_scalar_t b_91 = -4.204812589690116 + (-0.5384544990558136) * l_l + (0.001685970675296633) * l_r + (0.9578869588520786) * l_l * l_l +
                     (0.03596298359511682) * l_l * l_r + (0.0004234300017889048) * l_r * l_r;
  ad_scalar_t b_92 = -4.204812589690121 + (0.0016859706752969383) * l_l + (-0.5384544990558128) * l_r +
                     (0.0004234300017876974) * l_l * l_l + (0.03596298359511868) * l_l * l_r + (0.9578869588520761) * l_r * l_r;
  ad_scalar_t b_93 = -0.8687039103300151 + (0.6904926042318875) * l_l + (0.1627267825714508) * l_r + (0.225683938623973) * l_l * l_l +
                     (-0.1662735873279845) * l_l * l_r + (0.006947210853346486) * l_r * l_r;
  ad_scalar_t b_94 = -0.868703910330012 + (0.162726782571449) * l_l + (0.69049260423189) * l_r + (0.006947210853347055) * l_l * l_l +
                     (-0.16627358732798553) * l_l * l_r + (0.22568393862397218) * l_r * l_r;
  ad_scalar_t b_101 = -0.2395581168884729 + (-3.2735551320005856) * l_l + (-0.007887315228176428) * l_r + (5.968430995548482) * l_l * l_l +
                      (-0.24163475034696685) * l_l * l_r + (-0.0026653555789573624) * l_r * l_r;
  ad_scalar_t b_102 = 0.23955811688850698 + (0.007887315228172653) * l_l + (3.27355513200059) * l_r + (0.0026653555789639127) * l_l * l_l +
                      (0.2416347503469572) * l_l * l_r + (-5.968430995548477) * l_r * l_r;
  ad_scalar_t b_103 = -5.064915736293888 + (3.9955426878511915) * l_l + (-0.95603004066832) * l_r + (1.4065248253552383) * l_l * l_l +
                      (0.7150451500850263) * l_l * l_r + (-0.04325711443541891) * l_r * l_r;
  ad_scalar_t b_104 = 5.064915736293857 + (0.9560300406683218) * l_l + (-3.99554268785121) * l_r + (0.04325711443542085) * l_l * l_l +
                      (-0.715045150085007) * l_l * l_r + (-1.4065248253552407) * l_r * l_r;

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
