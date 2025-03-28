cmake_minimum_required(VERSION 2.8.3)
project(slam_karto_g2o)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    visualization_msgs
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")


find_package(Eigen REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)

include_directories(
    include/${PROJECT_NAME}
    ${catkin_INCLUDE_DIRS} 
    ${EIGEN_INCLUDE_DIRS}
    ${CHOLMOD_INCLUDE_DIR}
)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package()

add_executable(slam_karto_g2o src/slam_karto_g2o.cpp src/G2OSolver.cpp)

target_link_libraries(slam_karto_g2o ${catkin_LIBRARIES} ${CSPARSE_LIBRARY} ${CHOLMOD_LIBRARIES})

target_link_libraries(slam_karto_g2o
  g2o_core
  g2o_stuff
  g2o_types_slam2d
  g2o_solver_cholmod
  g2o_solver_csparse
  g2o_csparse_extension
)

#install(TARGETS slam_karto_g2o
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
