cmake_minimum_required(VERSION 2.8.3)
project(auto_circle_generator)

## Added from moveit
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
    ##
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
    ##
    raven_2
    pc_analysis
)

find_package(Boost REQUIRED
  COMPONENTS
    system
    filesystem
    date_time
    thread
)


## Add in message and service files
#add_message_files(FILES raven_automove.msg raven_state.msg)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    #
    raven_2
    pc_analysis
    ##
    message_runtime
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${catkin_auto_INCLUDE_DIRS}) ## what's this?
#link_directories(${catkin_LIBRARY_DIRS}) # is not recommended by tutorial


## 
#add_executable(talkerAutoCircle src/talker.cpp  src/Raven_Controller.cpp src/Raven_PathPlanner.cpp)
#target_link_libraries(talkerAutoCircle ${catkin_LIBRARIES})
#add_dependencies(talkerAutoCircle AutoCircle_test_generate_messages_cpp)

##
#add_executable(listenerAutoCircle src/listener.cpp)
#target_link_libraries(listenerAutoCircle ${catkin_LIBRARIES})
#add_dependencies(listenerAutoCircle AutoCircle_test_generate_messages_cpp)


##
#add_executable(motion_path_publish src/motion_path_publish_v1.cpp)
#target_link_libraries(motion_path_publish ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(motion_path_publish ${catkin_EXPORTED_TARGETS})

## just test the mapping for movement
#add_executable(move src/move_v2.cpp)
#target_link_libraries(move ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(move ${catkin_EXPORTED_TARGETS})

## kinematics
#add_executable(moveit_kine src/moveit_kine.cpp)
#target_link_libraries(moveit_kine ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(moveit_kine ${catkin_EXPORTED_TARGETS})

## 
#add_executable(auto_deb src/motion_path_publish_v3.cpp)
#target_link_libraries(auto_deb ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(auto_deb ${catkin_EXPORTED_TARGETS})

## move to a position
add_executable(move_to_pos src/motion_path_publish_v5.cpp)
target_link_libraries(move_to_pos ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(move_to_pos ${catkin_EXPORTED_TARGETS})

add_executable(dsn_path src/dsn/dsn_path_v2.cpp)
target_link_libraries(dsn_path ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(dsn_path ${catkin_EXPORTED_TARGETS})

add_executable(dsn_automove src/dsn/dsn_automove_v3_v2.cpp)
target_link_libraries(dsn_automove ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(dsn_automove ${catkin_EXPORTED_TARGETS})

#add_executable(moveit_rm_rs src/tutorial/robot_model_and_robot_state_tutorial.cpp)
#target_link_libraries(moveit_rm_rs ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(moveit_rm_rs ${catkin_EXPORTED_TARGETS})

add_executable(add_label_ravenstate src/dsn/add_label_ravenstate.cpp)
target_link_libraries(add_label_ravenstate ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(add_label_ravenstate ${catkin_EXPORTED_TARGETS})
