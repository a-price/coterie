cmake_minimum_required(VERSION 2.8.11)
project(coterie)

# Add additional CMake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
include(AIMMacros)

use_cxx_standard(11)

use_strict_compile_flags()

use_default_build_type(RelWithDebInfo)

enable_coverage_build()

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#set(CMAKE_BUILD_TYPE "Release")


add_definitions(-DBOOST_DISABLE_ASSERTS)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  coterie_msgs
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

find_eigen3_ros()
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

# Include OpenMP
use_openmp()

find_package(Qhull REQUIRED)
include_directories(SYSTEM ${QHULL_INCLUDE_DIR})

find_package( CGAL QUIET COMPONENTS
  Core
)
include(${CGAL_USE_FILE})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

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
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
 LIBRARIES ${PROJECT_NAME} ${QHULL_LIBRARIES}
 CATKIN_DEPENDS coterie_msgs roscpp roslib std_msgs visualization_msgs
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
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${QHULL_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
if (CGAL_FOUND)
  target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
endif()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(coterie ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(visualize_raster_set src/nodes/visualize_raster_set.cpp)
add_dependencies(visualize_raster_set ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_raster_set
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(visualize_ellipsoidal_set src/nodes/visualize_ellipsoidal_set.cpp)
add_dependencies(visualize_ellipsoidal_set ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_ellipsoidal_set
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(visualize_polytope_set src/nodes/visualize_polytope_set.cpp)
add_dependencies(visualize_polytope_set ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visualize_polytope_set
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
#enable_testing()

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
