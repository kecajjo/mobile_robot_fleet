#!/usr/bin/env python

import sys
import rospy
from warehouse.factory_map import FactoryMap

class Dijkstra(object):
    def __init__(self):

        factory_map = FactoryMap()
        self.rows=factory_map.rows
        self.columns= factory_map.columns

        self.station=factory_map.station
        self.dump=factory_map.dump

        #I've considered graph as the list of all unvisited nodes
        self.graph=factory_map.graph
        self._move_to=robot._move_to()
        start_node=0
        path=[]
        #prev_nodes=[]

        max_value=sys.maxsize

        for node in self.graph:
            path[node]=max_value
        path[start_node]=0

        while len(path)>=0:
            self.current_location=self._move_to().location
            for node in path:
             if max_value > self.current_location:
                max_value=self.current_location+ len(self.graph)


if __name__ == '__main__':
    rospy.init_node('movement_controller')
    rospy.loginfo("Starting robots...")

    dijkstra = Dijkstra()
    rospy.spin()
