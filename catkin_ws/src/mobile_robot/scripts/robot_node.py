#!/usr/bin/env python

import rospy
from common_msgs.srv import CommandRobot, CommandRobotResponse
from common_msgs.msg import RobotState

from mobile_robot.robot import Robot

class RobotsHandler(object):
    def __init__(self):
        robots_qty =  rospy.get_param('~robots_qty')
        capacity = rospy.get_param('~robot_capacity')
        self._service_move_to = rospy.Service('robot/move_to', CommandRobot, self._response_move_to)
        self._service_get_parts = rospy.Service('/robot/get_parts', CommandRobot, self._response_get_parts)
        self._service_drop_parts = rospy.Service('/robot/drop_parts', CommandRobot, self._response_drop_parts)
        self._robots = {}
        for id in range(robots_qty):
            self._robots[id] = Robot(id, capacity)
        

    def _response_move_to(self, req):
        robot = self._robots.get(req.robot_id)
        if robot._state is not RobotState.READY:
            return CommandRobotResponse(False)
        robot._move_to(req.value)
        return CommandRobotResponse(True)

    def _response_get_parts(self, req):
        robot = self._robots.get(req.robot_id)
        if (req.value > robot._capacity - len(robot._carried_parts.queue)) or robot._state is not RobotState.READY:
            return CommandRobotResponse(False)
        robot._get_parts(req.value)
        return CommandRobotResponse(True)

    def _response_drop_parts(self, req):
        robot = self._robots.get(req.robot_id)
        if robot._carried_parts.empty() or robot._state is not RobotState.READY:
            return CommandRobotResponse(False)
        robot._drop_parts()
        return CommandRobotResponse(True)


if __name__ == '__main__':
    rospy.init_node('mobile_robot')
    rospy.loginfo("Starting robots...")

    robot = RobotsHandler()
    rospy.spin()
