//
// Created by yuchen on 23-9-2.
//
#pragma once
#include <iostream>

#include <ocs2_core/Types.h>

namespace rm {
using ocs2::scalar_t;

struct LeggedBalanceParameters {
  //  void display() const {
  //    std::cerr << "Balance parameters: " << std::endl;
  //    std::cerr << "mass:   " << mass_ << std::endl;
  //    std::cerr << "wheelRadius:   " << wheelRadius_ << std::endl;
  //    std::cerr << "heightBallCenterToBase:   " << heightBallCenterToBase_ << std::endl;
  //  }
  // Distance between the two wheels
  scalar_t d_ = 0.419;  // [m]
  // Distance of shoulder axle to center of mass
  scalar_t l_c = 0.03;  // [m]
  // Wheel radius
  scalar_t r_ = 0.125;  // [m]
  // Mass of the pendulum body (except wheels and legs) [kg]
  scalar_t massBody_ = 13.14;
  // Mass of the leg [kg]
  scalar_t massLeg_ = 13.14;
  // Mass of each wheel [kg]
  scalar_t massWheel_ = 0.465756;
  // Single wheel moment of inertia (MOI) w.r.t. the wheel axis [kg]
  scalar_t jWheel_ = 0.005773;  // [kg*m^2]
  // Earth gravitation [m/s^2]
  scalar_t g_ = 9.81;
  // MOI of each wheel w.r.t. the vertical axis
  scalar_t kWheel_ = 0.002907;  // [kg*m^2]
  // MOI of the pendulum body w.r.t.{B} [kg*m^2]
  scalar_t i1 = 2.918e-1;
  scalar_t i2 = 2.384e-1;
  scalar_t i3 = 2.515e-1;

  scalar_t powerCoeffEffort_ = 1.339;
  scalar_t powerCoeffVel_ = 0.016;
  scalar_t powerOffset_ = 9.8;
};

}  // namespace rm