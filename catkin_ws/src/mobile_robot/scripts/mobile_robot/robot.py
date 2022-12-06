#!/usr/bin/env python

import rospy
import queue
from common_msgs.msg import RobotState, RobotStatus, WarehouseLocation

WAIT_TIME_PER_PART = 2.0
WAIT_TIME_MOVE = 5.0

class Robot(object):
    def __init__(self, id, capacity):
        self._id = id
        self._capacity = capacity

        self._carried_parts = queue.Queue(maxsize=self._capacity)
        self._state = RobotState.READY
        self._location = WarehouseLocation.RESTING_AREA
        
        self._pub_robot_status = rospy.Publisher('/robot/status', RobotStatus, queue_size=10)
        rospy.sleep(2)
        self._publish_robot_status()
        
        rospy.loginfo("Robot_{} has started succesfuly!".format(self._id))
        
    def _move_to(self, location):
        self._location = location
        self._state = RobotState.MOVING
        self._publish_robot_status()
        rospy.loginfo("Robot_{} is moving!".format(self._id))

        rospy.Timer(rospy.Duration(WAIT_TIME_MOVE), self._timed_task_finished, oneshot=True)

    def _get_parts(self, parts_qty):
        self._state = RobotState.WORKING
        self._publish_robot_status()
        
        for _ in range(parts_qty):
            self._carried_parts.put(self._location)  # put part of ID inherited from current location

        rospy.loginfo("Robot_{} is loading!".format(self._id))
        rospy.Timer(rospy.Duration(WAIT_TIME_PER_PART * parts_qty), self._timed_task_finished, oneshot=True)

    def _drop_parts(self):
        self._state = RobotState.WORKING
        self._publish_robot_status()

        rospy.Timer(rospy.Duration(WAIT_TIME_PER_PART * len(self._carried_parts.queue)),
                    self._timed_task_finished, oneshot=True)
        rospy.loginfo("Robot_{} is unloading!".format(self._id))
        self._carried_parts.queue.clear()

    def _publish_robot_status(self):
        msg = RobotStatus()
        msg.state = RobotState(self._state)
        msg.location = WarehouseLocation(self._location)
        msg.carried_parts = list(self._carried_parts.queue)
        msg.id = self._id
        msg.capacity = self._capacity
        self._pub_robot_status.publish(msg)

    def _timed_task_finished(self, event):
        self._state = RobotState.READY
        self._publish_robot_status()
