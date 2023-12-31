cmake_minimum_required(VERSION 3.0.2)
project(rm_legged_balance_control)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_PACKAGE_DEPENDENCIES
        ocs2_core
        ocs2_mpc
        ocs2_mpc
        ocs2_sqp
        ocs2_ipm
        ocs2_robotic_tools
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
        DEPENDS
        Boost
        LIBRARIES
        ${PROJECT_NAME}
)

###########
## Build ##
###########
# Resolve for the package path at compile time.
configure_file(
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# Add directories for all targets
include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# balance interface library
add_library(${PROJECT_NAME}
        src/LeggedBalanceSystemDynamics.cpp
        src/LeggedBalanceInterface.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ${PROJECT_NAME}")
    add_clang_tooling(
            TARGETS
            ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
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
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
