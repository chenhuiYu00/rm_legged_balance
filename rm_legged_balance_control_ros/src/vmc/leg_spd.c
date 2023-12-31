/*
 * File: leg_spd.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 20-Oct-2023 20:45:56
 */

/* Include Files */
#include "rm_legged_balance_control_ros/vmc/leg_spd.h"
#include <math.h>

/* Function Definitions */
/*
 * LEG_SPD
 *     SPD = LEG_SPD(DPHI1,DPHI4,PHI1,PHI4)
 *
 * Arguments    : double dphi1
 *                double dphi4
 *                double phi1
 *                double phi4
 *                double spd[2]
 * Return Type  : void
 */
void leg_spd(double dphi1, double dphi4, double phi1, double phi4, double spd[2]) {
  double t10;
  double t11;
  double t12;
  double t13;
  double t19;
  double t2;
  double t20;
  double t23;
  double t24;
  double t3;
  double t35;
  double t38;
  double t4;
  double t47;
  double t5;
  double t6;
  double t61;
  double t62;
  double t63;
  double t7;
  double t8;
  double t9;
  /*     This function was generated by the Symbolic Math Toolbox version 8.7.
   */
  /*     21-Oct-2023 02:24:11 */
  t2 = cos(phi1);
  t3 = cos(phi4);
  t4 = sin(phi1);
  t5 = sin(phi4);
  t6 = t2 * 0.15;
  t7 = t3 * 0.15;
  t8 = t2 * 0.075;
  t9 = t3 * 0.075;
  t10 = t4 * 0.15;
  t11 = t5 * 0.15;
  t12 = t4 * 0.075;
  t13 = t5 * 0.075;
  t19 = t10 + -t11;
  t20 = t12 + -t13;
  t23 = (t7 + -t6) + 0.108;
  t24 = (t9 + -t8) + 0.054;
  t35 = t19 * t19 + t23 * t23;
  t38 = t2 * t19 * 0.3 + t4 * t23 * 0.3;
  t3 = t3 * t19 * 0.3 + t5 * t23 * 0.3;
  t19 = 1.0 / (t24 + t35);
  t23 = t19 * t19;
  t5 = sqrt((t20 * t20 + t24 * t24) + -(t35 * t35));
  t47 = 1.0 / t5;
  t4 = (t13 + -t12) + t5;
  t2 = atan(t19 * t4) * 2.0;
  t61 = cos(t2);
  t62 = sin(t2);
  t63 = 1.0 / (t23 * (t4 * t4) + 1.0);
  t5 = (t12 + t38) * t23 * t4 + t19 * (t8 + -(t47 * ((t6 * t20 + t10 * t24) + -(t35 * t38 * 2.0)) / 2.0));
  t38 = (t13 + t3) * t23 * t4 + t19 * (t9 + -(t47 * ((t7 * t20 + t11 * t24) + -(t35 * t3 * 2.0)) / 2.0));
  t8 = t10 + t62 / 4.0;
  t12 = (t6 + t61 / 4.0) - 0.054;
  t2 = t8 * t8;
  t7 = 1.0 / t8;
  t23 = t12 * t12;
  t9 = 1.0 / t2;
  t47 = t61 * t63;
  t19 = t6 + -(t47 * t5 / 2.0);
  t3 = t62 * t63;
  t5 = t10 + -(t3 * t5 / 2.0);
  t4 = 1.0 / sqrt(t2 + t23);
  t2 = 1.0 / (t9 * t23 + 1.0);
  spd[0] = dphi1 * t4 * (t8 * t19 * 2.0 - t12 * t5 * 2.0) / 2.0 + dphi4 * t4 * (t47 * t8 * t38 - t3 * t12 * t38) / 2.0;
  spd[1] = dphi4 * t2 * (t3 * t7 * t38 / 2.0 + t47 * t9 * t12 * t38 / 2.0) + dphi1 * t2 * (t7 * t5 + t9 * t12 * t19);
}

/*
 * File trailer for leg_spd.c
 *
 * [EOF]
 */
