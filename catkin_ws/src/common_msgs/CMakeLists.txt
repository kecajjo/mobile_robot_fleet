cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

find_package(catkin REQUIRED 
              COMPONENTS
              std_msgs
              message_generation
              message_runtime
              roscpp
              rospy)

add_message_files(DIRECTORY
                  msg
                  FILES
                  RobotStatus.msg
                  RobotState.msg
                  WarehouseLocation.msg)

add_service_files(DIRECTORY
                  srv
                  FILES
                  SetValue.srv)

generate_messages(DEPENDENCIES std_msgs)

add_dependencies(tests common_msgs_generate_messages)

catkin_package(CATKIN_DEPENDS
               message_generation
               message_runtime
               roscpp
               rospy
               std_msgs)
