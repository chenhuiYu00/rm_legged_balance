//
// Created by yuchen on 23-9-2.
//

#include "rm_legged_balance_control_ros/synchronized_module/RosReferenceManager.h"

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ipm/IpmMpc.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

#include <rm_legged_balance_control/LeggedBalanceInterface.h>

int main(int argc, char** argv) {
  const std::string robotName = "legged_balance";

  // task file
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nodeHandle;

  // Robot interface
  const std::string taskFile = ros::package::getPath("rm_legged_balance_control") + "/config/" + taskFileFolderName + "/task.info";
  const std::string libFolder = ros::package::getPath("rm_legged_balance_control") + "/auto_generated";
  rm::LeggedBalanceInterface balanceInterface(taskFile, libFolder);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr =
      std::make_shared<rm::RosReferenceManager>(balanceInterface.getReferenceManagerPtr(), balanceInterface.getLeggedBalanceControlCmd());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // MPC
  ocs2::IpmMpc mpc(balanceInterface.mpcSettings(), balanceInterface.ipmSettings(), balanceInterface.getOptimalControlProblem(),
                   balanceInterface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Launch MPC ROS node
  ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  // Successful exit
  return 0;
}
