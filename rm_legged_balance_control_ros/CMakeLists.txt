cmake_minimum_required(VERSION 3.0.2)
project(rm_legged_balance_control_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
        roslib
        visualization_msgs
        ocs2_ros_interfaces
        controller_interface
        rm_common
        angles
        tf2_ros
        tf2_geometry_msgs
        effort_controllers

        rm_legged_balance_control
        )

find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        LIBRARIES
        ${PROJECT_NAME}
        DEPENDS
        Boost
)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# main library
add_library(${PROJECT_NAME}
        src/LeggedBalanceVisualization.cpp
        src/synchronized_module/RosReferenceManager.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Controller library
add_library(legged_balance_controller
        src/LeggedBalanceController.cpp
        src/vmc/leg_conv.c
        src/vmc/leg_pos.c
        src/vmc/leg_spd.c
        )
add_dependencies(legged_balance_controller
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(legged_balance_controller
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(legged_balance_controller PUBLIC ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(legged_balance_dummy_node
        src/DummyLeggedNode.cpp
        )
add_dependencies(legged_balance_dummy_node
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(legged_balance_dummy_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(legged_balance_dummy_node PRIVATE ${OCS2_CXX_FLAGS})

## Ipm node
add_executable(legged_balance_ipm
        src/LeggedBalanceIpmMpcNode.cpp
        )
add_dependencies(legged_balance_ipm
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(legged_balance_ipm
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(legged_balance_ipm PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target rm_balance_control_ros")
    add_clang_tooling(
            TARGETS
            ${PROJECT_NAME}
            balance_ipm
            balance_dummy_node
            balance_target
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
install(TARGETS legged_balance_dummy_node legged_balance_ipm
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY launch rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
