#!/usr/bin/env python

import rospy
import queue
from std_srvs.srv import Trigger, TriggerResponse
from common_msgs.srv import SetValue, SetValueResponse
from common_msgs.msg import RobotState, RobotStatus, WarehouseLocation

WAIT_TIME_PER_PART = 2.0
WAIT_TIME_MOVE = 5.0


class Robot(object):
    def __init__(self):
        self._id = rospy.get_param('~robot_id')
        self._capacity = rospy.get_param('~robot_capacity')
        PREFIX = "/robot_{}".format(self._id)

        self._carried_parts = queue.Queue(maxsize=self._capacity)
        self._state = RobotState.READY
        self._location = WarehouseLocation.RESTING_AREA

        self._service_move_to = rospy.Service(PREFIX + '/move_to', SetValue, self._response_move_to)
        self._service_get_parts_from_stand = rospy.Service(PREFIX + '/get_parts', SetValue,
                                                           self._response_get_parts_from_stand)
        self._service_drop_parts = rospy.Service(PREFIX + '/drop_parts', Trigger, self._response_drop_parts)

        self._pub_robot_state = rospy.Publisher(PREFIX + '/status', RobotStatus, queue_size=10, latch=True)
        self._publish_robot_status()

    def _response_move_to(self, req):
        if self._state is not RobotState.READY:
            return SetValueResponse(False)

        self._location = req.value
        self._state = RobotState.MOVING
        self._publish_robot_status()

        rospy.Timer(rospy.Duration(WAIT_TIME_MOVE), self._timed_task_finished, oneshot=True)
        return SetValueResponse(True)

    def _response_get_parts_from_stand(self, req):
        if (req.value > self._capacity - len(self._carried_parts.queue)) or self._state is not RobotState.READY:
            return SetValueResponse(False)

        self._state = RobotState.WORKING
        self._publish_robot_status()

        # TODO: here we should (probably) call WareHouse service "/load_part"
        #       this service will remove part from stand and provide it to robot
        # response_load_part = self._load_part.call(self._location, req.value)

        for _ in range(req.value):
            self._carried_parts.put(self._location)  # put part of ID inherited from current location

        rospy.Timer(rospy.Duration(WAIT_TIME_PER_PART * req.value), self._timed_task_finished, oneshot=True)
        return SetValueResponse(True)

    def _response_drop_parts(self, req):
        if self._carried_parts.empty() or self._state is not RobotState.READY:
            return TriggerResponse(False, "")

        self._state = RobotState.WORKING
        self._publish_robot_status()

        rospy.Timer(rospy.Duration(WAIT_TIME_PER_PART * len(self._carried_parts.queue)),
                    self._timed_task_finished, oneshot=True)
        self._carried_parts.queue.clear()
        return TriggerResponse(True, "")

    def _publish_robot_status(self):
        msg = RobotStatus()
        msg.state = RobotState(self._state)
        msg.location = WarehouseLocation(self._location)
        msg.carried_parts = list(self._carried_parts.queue)
        self._pub_robot_state.publish(msg)

    def _timed_task_finished(self, event):
        self._state = RobotState.READY
        self._publish_robot_status()


if __name__ == '__main__':
    rospy.init_node('mobile_robot')
    rospy.loginfo("Starting")

    robot = Robot()
    rospy.loginfo("Robot_{} has started succesfuly!".format(robot._id))
    rospy.spin()
