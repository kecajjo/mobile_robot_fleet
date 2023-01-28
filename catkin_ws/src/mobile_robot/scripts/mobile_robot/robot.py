#!/usr/bin/env python

import rospy
import queue
from common_msgs.msg import RobotState, RobotStatus, WarehouseLocation
from common_msgs.srv import MovementOrder

WAIT_TIME_PER_PART = 2.0
WAIT_TIME_MOVE = 5.0
_MOVE_ORDER_SERVICE = "/robot/movement_order"

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
        # TODO:
        # current_location = self._location
        # self._location = location
        self._state = RobotState.MOVING
        self._publish_robot_status()
        rospy.loginfo("Robot_{} is moving to node {}!".format(self._id, location))
        _movement_order = rospy.ServiceProxy(_MOVE_ORDER_SERVICE,MovementOrder)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # next_move, status = _movement_order(WarehouseLocation(self._location),WarehouseLocation(location))
            rospy.wait_for_service(_MOVE_ORDER_SERVICE)
            try:
                resp = _movement_order(self._id, WarehouseLocation(self._location),WarehouseLocation(location))
                if resp.task_finished:
                    break
                else:
                    self._location = resp.next_move.location
                    rospy.loginfo("id:{} loc: {}".format(self._id,self._location))
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            rate.sleep()
        self._task_finished()
        # TODO:
        # Remove timer below and call service that should inform movement_controller
        # that this robot wants to move from "current_location" to "location", example:
        #
        # self._movement_order.call(current_location, location)
        #
        # Once robot reaches its destination "location" - movement_controller should
        # inform robot that robot reached location, so self._task_finished should be executed
        # (for example) by using service where robot is a server and movement_controller a client
        #rospy.Timer(rospy.Duration(WAIT_TIME_MOVE), self._timed_task_finished, oneshot=True)

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

    def _task_finished(self):
        self._state = RobotState.WAITING
        self._publish_robot_status()

    def _timed_task_finished(self, event):
        # self._task_finished()
        self._state = RobotState.READY
        self._publish_robot_status()
