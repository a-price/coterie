cmake_minimum_required(VERSION 2.8.3)
project(coterie)

set(BUILD_FLAGS "-std=c++11 -Wshadow -Wall -Wextra -Wpedantic")
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE RelWithDebInfo ... FORCE)
   message(STATUS "Setting CMAKE_BUILD_TYPE to ${CMAKE_BUILD_TYPE} (default).")
endif()
#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Release")

add_definitions(-DBOOST_DISABLE_ASSERTS)

# Add additional CMake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Set build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  coterie_msgs
  message_generation
  roscpp
  roslib
  std_msgs
  visualization_msgs
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
  filesystem
  iostreams
  serialization
  system
)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(Qhull REQUIRED)
include_directories(SYSTEM ${QHULL_INCLUDE_DIR})
link_libraries(${QHULL_LIBRARIES})

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} ${QHULL_LIBRARIES}
 CATKIN_DEPENDS cmake_modules roscpp roslib std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/template_instantiations.cpp
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(coterie ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(visualize_rasterset src/nodes/visualize_rasterset.cpp)
add_dependencies(visualize_rasterset ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_rasterset
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Notify any IDE that headers are part of project
file(GLOB_RECURSE IncludeFiles "include/*")
add_custom_target(${PROJECT_NAME}_headers SOURCES ${IncludeFiles})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/nodes/visualize_raster_isosurface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS coterie coterie_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Turn on testing
enable_testing()

if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
  find_package( Threads )

  # Add all unit tests
  file(GLOB UTestFiles "test/unit/*.cpp")
  foreach(utest_file ${UTestFiles})
    get_filename_component(utest_base ${utest_file} NAME_WE)

#    add_executable(${utest_base} test/unit/${utest_base}.cpp)
    catkin_add_gtest(${utest_base} test/unit/${utest_base}.cpp)
    target_link_libraries(${utest_base}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      ${GTEST_LIBRARIES}
      ${CMAKE_THREAD_LIBS_INIT}
    )
  endforeach()

endif()
