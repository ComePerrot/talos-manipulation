cmake_minimum_required(VERSION 3.1)

SET(PROJECT_NAMESPACE ComePerrot)
SET(PROJECT_NAME talos_manipulation)
SET(PROJECT_DESCRIPTION "Manipulation for Talos")
SET(PROJECT_URL https://gitlab.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})

# Project configuration
set(DCMAKE_EXPORT_COMPILE_COMMANDS=1)


# JRL-cmakemodule setup
include("${JRL_CMAKE_MODULES}/base.cmake")

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_compile_options(-Wno-error)

# Project dependencies
add_project_dependency(sobec REQUIRED)

set(CATKIN_COMPONENTS
  roscpp
  realtime_tools
  message_generation
  eigen_conversions
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

SET(${PROJECT_NAME}_SOURCES
    src/main.cpp
)
SET(${PROJECT_NAME}_HEADERS

)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC sobec::sobec)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

