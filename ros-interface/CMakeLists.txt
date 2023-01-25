cmake_minimum_required(VERSION 3.1)

SET(PROJECT_NAMESPACE ComePerrot)
SET(PROJECT_NAME ros-interface)
SET(PROJECT_DESCRIPTION "Ros interface for Talos manipulation MPC")
SET(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/talos-manipulation)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  if(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nPlease run the following command first:\ngit submodule update --init\n"
    )
  else()
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_compile_options(-Wno-error)

# Project dependencies
add_project_dependency(Eigen3 REQUIRED)
add_project_dependency(sobec REQUIRED)
add_project_dependency(mpc-pointing REQUIRED)

set(CATKIN_COMPONENTS
  roscpp
  roslib
  realtime_tools
  eigen_conversions
  linear_feedback_controller_msgs
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_SOURCES
    src/ros-mpc-interface.cpp
)
SET(${PROJECT_NAME}_HEADERS
    include/${PROJECT_NAME}/ros-mpc-interface.h
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC sobec::sobec Eigen3::Eigen)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

add_executable(ros_mpc_pointing src/ros-mpc-pointing.cpp)
target_link_libraries(ros_mpc_pointing ${PROJECT_NAME} mpc-pointing::mpc-pointing sobec::sobec Eigen3::Eigen ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/ros-mpc-test.test)
endif()

# Installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
install(TARGETS ros_mpc_pointing RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY test DESTINATION share/${PROJECT_NAME})
  
  

