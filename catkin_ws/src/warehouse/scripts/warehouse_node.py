#!/usr/bin/env python

import rospy
from common_msgs.msg import Part, Order, WarehouseLocation
import random

_ORDER_TOPIC="/orders"

class Warehouse(object):
    def __init__(self):
        self._pub_robot_status = rospy.Publisher(_ORDER_TOPIC, Order, queue_size=10)
        rospy.Timer(rospy.Duration(2), self._my_callback, True)

    def _my_callback(self, event):
        self.order_parts()
        rospy.Timer(rospy.Duration(random.randint(1,5)), self._my_callback,True)

    def order_parts(self):
        order = Order()
        line_ids = [WarehouseLocation.DUMP_ID_0, WarehouseLocation.DUMP_ID_1]
        part_ids = [vars(WarehouseLocation)["STATION_ID_{}".format(x)] for x in range(1,5)]
        order.line = random.choice(line_ids)
        part = Part()
        part.part_id = random.choice(part_ids)
        part.amount = random.randint(1,4)
        order.parts = [part]
        self._pub_robot_status.publish(order)
        

if __name__ == '__main__':
    rospy.init_node('Warehouse')

    warehouse = Warehouse()

    rospy.spin()
