cmake_minimum_required(VERSION 3.7)
project(mpcc_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        carla_ros_bridge
        ad_challenge_msgs
        std_msgs
        geometry_msgs
        nav_msgs
        ackermann_msgs
)
# find_package(Python COMPONENTS Development)


catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES mpcc_carla_ros_bridge
        CATKIN_DEPENDS roscpp rospy carla_ros_bridge ad_challenge_msgs std_msgs geometry_msgs nav_msgs ackermann_msgs
        # DEPENDS system_lib
        )

include_directories(.)
include_directories(
        External/blasfeo/lib/include
        External/hpipm/lib/include
        External/matplotlib
        External/Eigen
        External/Json/include
        )
# include_directories(
#         include
# )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
# add_message_files( DIRECTORY msg )
# generate_messages( DEPENDENCIES std_msgs)

add_executable(${PROJECT_NAME}
        src/main.cpp
        # types.cpp
        # types.h
        # config.h
        # Params/params.cpp
        # Spline/cubic_spline.cpp
        # Spline/arc_length_spline.cpp
        # Interfaces/hpipm_interface.cpp
        # Interfaces/solver_interface.cpp
        # Constraints/constraints.cpp
        # Constraints/bounds.cpp
        # Cost/cost.cpp
        # Model/model.cpp
        # Model/integrator.cpp
        # Tests/spline_test.cpp
        # Tests/cost_test.cpp
        # Tests/model_integrator_test.cpp
        # Tests/constratins_test.cpp
        # MPC/mpc.cpp
        # Params/track.cpp
        # Params/track.h
        # Plotting/plotting.cpp
        # Plotting/plotting.h
        )
add_executable(mpcc_exp1 src/mpcc_exp1.cpp)
add_executable(mpcc_exp2 src/mpcc_exp2.cpp)
        
# target_include_directories(${PROJECT_NAME} PRIVATE ${Python_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME} ${Python_LIBRARIES})
target_link_libraries(${PROJECT_NAME} 
        ${CMAKE_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a 
        ${CMAKE_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a 
        ${catkin_LIBRARIES}
        m
        )
target_link_libraries(mpcc_exp1 ${catkin_LIBRARIES})
target_link_libraries(mpcc_exp2 ${catkin_LIBRARIES})
