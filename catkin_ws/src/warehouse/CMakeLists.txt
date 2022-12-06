cmake_minimum_required(VERSION 3.15)
project(warehouse)

find_package(catkin REQUIRED
             COMPONENTS std_msgs
                        rospy
                        common_msgs)

catkin_python_setup()       
catkin_package(CATKIN_DEPENDS
                common_msgs
                std_msgs)
                        
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


catkin_install_python(PROGRAMS
                      scripts/warehouse_node.py
                      DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
