#!/usr/bin/env python

import rospy
from common_msgs.msg import Part, Order, WarehouseLocation
import random

_ORDER_TOPIC="/orders"

class FactoryMap(object):
    STATION_OFFSET = 101
    DUMP_OFFSET = 201
    RESTING_AREA = WarehouseLocation.RESTING_AREA

    def __init__(self):
        rows = rospy.get_param('/factory_map_rows', default=3)
        columns = rospy.get_param('/factory_map_columns', default=5)

        self.rows = rows
        self.columns = columns
        self.graph = dict()
        self.graph[0] = [x for x in range(1, columns+1)]  # RESTING AREA
        station = [x for x in range(FactoryMap.STATION_OFFSET, FactoryMap.STATION_OFFSET+rows+1)]
        dump = [x for x in range(FactoryMap.DUMP_OFFSET, FactoryMap.DUMP_OFFSET+rows+1)]
        for i in range(1, rows+1):
            self.graph[station[i-1]] = [1+(i-1)*columns]
            self.graph[dump[i-1]] = [i*columns]
            for j in range(1, columns+1):
                edges = []
                if j != 1:
                    edges.append(j-1+(i-1)*columns)
                else:
                    edges.append(station[i-1])
                if j != columns:
                    edges.append(j+1+(i-1)*columns)
                else:
                    edges.append(dump[i-1])
                if i != 1:
                    edges.append(j+(i-2)*columns)
                else:
                    edges.append(FactoryMap.RESTING_AREA)
                if i != rows:
                    edges.append(j+i*columns)
                self.graph[j+(i-1)*columns] = edges

    def _get_move_areas(self):
        return([area for area in self.graph.keys()
                if area > FactoryMap.RESTING_AREA and area < FactoryMap.STATION_OFFSET])

    def _get_station_areas(self):
        return([area for area in self.graph.keys()
               if area >= FactoryMap.STATION_OFFSET and area < FactoryMap.DUMP_OFFSET])

    def _get_dump_areas(self):
        return([area for area in self.graph.keys() if area >= FactoryMap.DUMP_OFFSET])


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
