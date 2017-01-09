#
# This is based on orocos sample cmakelists, modified slightly
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.8.3
#
cmake_minimum_required(VERSION 2.8.3)
 
#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(LWRControllerContained)
 
# current source and include:
#include_directories(${PROJECT_SOURCE_DIR}/include)




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
 
 
# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(rtt-core-extensions REQUIRED)
include_directories(${RTT-CORE-EXTENSIONS_INCLUDE_DIRS})
link_directories(${RTT-CORE-EXTENSIONS_LIBRARY_DIRS})


set (Eigen_INCLUDE_DIRS "/usr/include/eigen3")
find_package(orocos_kdl REQUIRED)
link_directories(${orocos_kdl_LIBRARY_DIR} ${orocos_kdl_LIBRARY_DIRS})
find_package(Boost COMPONENTS thread REQUIRED)
find_package(srdfdom_advr REQUIRED)

link_directories(${srdfdom_advr_LIBRARY_DIRS})


find_package(XBotCoreModel REQUIRED)

find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
        find_path("/usr/include" tinyxml.h)
        find_library(TINYXML_LIBRARY NAMES tinyxml)
        set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIR})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")
message(STATUS  ${orocos_kdl_LIBRARY_DIR} ${orocos_kdl_LIBRARY_DIRS} )
include_directories(
    include
    src
    ${orocos_kdl_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${RST-RT_INCLUDE_DIRS}
    ${TINYXML_INCLUDE_DIRS}
    ${srdfdom_advr_INCLUDE_DIRS}
    ${XBotCoreModel_INCLUDE_DIRS}
    
)

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#
 
 
# Creates a component library libexample-<target>.so
# and installs in the directory lib/orocos/example/
#
# orocos_component(${CMAKE_PROJECT_NAME} simple-rtt-component.hpp simple-rtt-component.cpp) # ...you may add multiple source files
orocos_component(${CMAKE_PROJECT_NAME}
                                        ${PROJECT_SOURCE_DIR}/KDLParser.hpp
                                        ${PROJECT_SOURCE_DIR}/KDLParser.cpp
                                        ${PROJECT_SOURCE_DIR}/kukaLWRModel.h
                                        ${PROJECT_SOURCE_DIR}/kukaLWRModel.cpp
                                        ${PROJECT_SOURCE_DIR}/controller-contained/ControllerContained.hpp
                                        ${PROJECT_SOURCE_DIR}/controller-contained/ControllerContained.cpp
                                        ${PROJECT_SOURCE_DIR}/kinematic-chain/ChainBase.h
                                        ${PROJECT_SOURCE_DIR}/kinematic-chain/ChainBase.cpp
                                        ${PROJECT_SOURCE_DIR}/kinematic-task-space/KTSController.h
                                        ${PROJECT_SOURCE_DIR}/kinematic-task-space/KTSController.cpp
                                        ${PROJECT_SOURCE_DIR}/joint-position-ctrl/JointPositionCtrl.hpp
                                        ${PROJECT_SOURCE_DIR}/joint-position-ctrl/JointPositionCtrl.cpp
                                        ${PROJECT_SOURCE_DIR}/torque-transition/TorqueTransition.hpp
                                        ${PROJECT_SOURCE_DIR}/torque-transition/TorqueTransition.cpp
                                        ${PROJECT_SOURCE_DIR}/trajectory-gen/TrajectoryGen.hpp
                                        ${PROJECT_SOURCE_DIR}/trajectory-gen/TrajectoryGen.cpp
) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.
 
#
# Building a typekit (recommended):
#
# Creates a typekit library libexample-types-<target>.so
# and installs in the directory lib/orocos/example/types/
#
#orocos_typegen_headers(example-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement !
 
#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.
 
 
#
# Building a Plugin or Service (optional):
#
# Creates a plugin library libexample-service-<target>.so or libexample-plugin-<target>.so
# and installs in the directory lib/orocos/example/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_service(example-service example-service.cpp) # ...only one service per library !
#orocos_plugin(example-plugin example-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin/orocos_service statements.

# target_link_libraries()

target_link_libraries(${CMAKE_PROJECT_NAME} ${orocos_kdl_LIBRARIES} ${RST-RT_LIBRARIES} ${RTT-CORE-EXTENSIONS_LIBRARIES}
   ${USE_OROCOS_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${OROCOS-RTT_LIBRARIES}
   ${Boost_LIBRARIES}
   ${TINYXML_LIBRARIES}
   ${srdfdom_advr_LIBRARIES}
   ${XBotCoreModel_LIBRARIES}
   )
 
#
# Additional headers (not in typekit):
#
# Installs in the include/orocos/example/ directory
#
# orocos_install_headers( example-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.
 
#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
