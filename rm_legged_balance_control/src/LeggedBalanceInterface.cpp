//
// Created by yuchen on 23-9-2.
//

#include <iostream>
#include <memory>
#include <string>

#include "rm_legged_balance_control/LeggedBalanceInterface.h"
#include "rm_legged_balance_control/constraint/PitchConstraint.h"
#include "rm_legged_balance_control/constraint/PowerConstraintCppAd.h"
#include "rm_legged_balance_control/constraint/TorqueConstraint.h"

#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace rm {

LeggedBalanceInterface::LeggedBalanceInterface(const std::string& taskFile, const std::string& libraryFolder) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[LeggedBalanceInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedBalanceInterface] Task file not found: " + taskFilePath.string());
  }
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[LeggedBalanceInterface] Generated library path: " << libraryFolderPath << std::endl;

  // Default initial condition
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  // Solver settings
  sqpSettings_ = ocs2::sqp::loadSettings(taskFile, "sqp");
  ipmSettings_ = ocs2::ipm::loadSettings(taskFile, "ipm");
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc");

  referenceManagerPtr_.reset(new ReferenceManager);

  /*
   * Optimal control problem
   */
  // Cost
  matrix_t Q(STATE_DIM, STATE_DIM);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  matrix_t Qf(STATE_DIM, STATE_DIM);
  ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q);
  ocs2::loadData::loadEigenMatrix(taskFile, "R", R);
  std::cerr << "Q:  \n" << Q << "\n";
  std::cerr << "R:  \n" << R << "\n";
  problem_.costPtr->add("cost", std::make_unique<ocs2::QuadraticStateInputCost>(Q, R));

  balanceControlCmdPtr_ = std::make_shared<LeggedBalanceControlCmd>();

  // Dynamics
  bool recompileLibraries = false;  // load the flag to generate library files from taskFile
  ocs2::loadData::loadCppDataType(taskFile, "legged_balance_interface.recompileLibraries", recompileLibraries);
  problem_.dynamicsPtr = std::make_unique<LeggedBalanceSystemDynamics>(taskFile, libraryFolder, recompileLibraries, balanceControlCmdPtr_);

  // Rollout
  auto rolloutSettings = ocs2::rollout::loadSettings(taskFile, "rollout");
  rolloutPtr_ = std::make_unique<ocs2::TimeTriggeredRollout>(*problem_.dynamicsPtr, rolloutSettings);

  // Initialization
  vector_t initialInput = vector_t::Zero(INPUT_DIM);
  initializerPtr_ = std::make_unique<ocs2::DefaultInitializer>(INPUT_DIM);

  // Constraint
  double power_limit = 60, max_pitch = 0.36;
  ocs2::loadData::loadCppDataType(taskFile, "Constraint.power_limit", power_limit);
  ocs2::loadData::loadCppDataType(taskFile, "Constraint.max_pitch", max_pitch);
  /* todo: add limits
  balanceControlCmdPtr_->setPowerLimit(power_limit);
  balanceControlCmdPtr_->setMaxPitch(max_pitch, 0.);
  problem_.stateInequalityConstraintPtr->add("pitchConstraint", std::make_unique<PitchConstraint>(balanceControlCmdPtr_));
  problem_.inequalityConstraintPtr->add("torqueConstraint", std::make_unique<TorqueConstraint>());
  problem_.inequalityConstraintPtr->add("powerConstraint",
                                        std::make_unique<PowerConstraintCppAd>(LeggedBalanceParameters(), balanceControlCmdPtr_));
                                        */
}

}  // namespace rm
