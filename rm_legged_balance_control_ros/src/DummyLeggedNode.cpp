//
// Created by yuchen on 23-9-2.
//
#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <rm_legged_balance_control/LeggedBalanceInterface.h>
#include <rm_legged_balance_control/definitions.h>

#include "rm_legged_balance_control_ros/LeggedBalanceVisualization.h"

int main(int argc, char** argv) {
  const std::string robotName = "balance";

  // task file
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Interface
  const std::string taskFile = ros::package::getPath("rm_legged_balance_control") + "/config/" + taskFileFolderName + "/task.info";
  const std::string libFolder = ros::package::getPath("rm_legged_balance_control") + "/auto_generated";
  rm::LeggedBalanceInterface balanceInterface(taskFile, libFolder);

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&balanceInterface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto balanceDummyVisualization = std::make_shared<rm::LeggedBalanceVisualization>(nodeHandle, true);

  // Dummy balance
  ocs2::MRT_ROS_Dummy_Loop dummyBalance(mrt, balanceInterface.mpcSettings().mrtDesiredFrequency_,
                                        balanceInterface.mpcSettings().mpcDesiredFrequency_);
  dummyBalance.subscribeObservers({balanceDummyVisualization});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = balanceInterface.getInitialState();
  initObservation.input.setZero(rm::INPUT_DIM);
  initObservation.time = 0.0;

  // initial command
  const ocs2::TargetTrajectories initTargetTrajectories({initObservation.time}, {initObservation.state}, {initObservation.input});

  // Run dummy (loops while ros is ok)
  dummyBalance.run(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}
