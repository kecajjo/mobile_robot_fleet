#!/usr/bin/env python

import rospy
from common_msgs.msg import Part, Order
from warehouse.factory_map import FactoryMap
import random

_ORDER_TOPIC="/orders"

class Warehouse(object):
    def __init__(self):
        self._factory_map = FactoryMap()
        self._pub_robot_status = rospy.Publisher(_ORDER_TOPIC, Order, queue_size=10)
        rospy.Timer(rospy.Duration(2), self._my_callback, True)

    def _my_callback(self, event):
        self.order_parts()
        rospy.Timer(rospy.Duration(random.randint(1,5)), self._my_callback, True)

    def order_parts(self):
        order = Order()
        line_ids = self._factory_map._get_dump_areas()
        part_ids = self._factory_map._get_station_areas()
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
