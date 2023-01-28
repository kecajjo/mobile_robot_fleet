#!/usr/bin/env python

import sys
import rospy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from warehouse.factory_map import FactoryMap
from common_msgs.srv import MovementOrder, MovementOrderResponse
from common_msgs.msg import WarehouseLocation
 

class Node(object):
        def __init__(self, id =sys.maxsize, edges=[], dist=sys.maxsize):
            self.id = id
            self.edges = edges
            self.distance = dist
            self.prev_node = sys.maxsize


class PriorityQueue(object):
    
    def __init__(self):
        self.items = dict()

    def is_empty(self):
        return not self.items

    def insert(self,key,item):
        self.items[key]=item

    def remove(self):
        keys = list(self.items.keys())
        min = keys[0]
        for i in keys:
            if self.items[i].distance < self.items[min].distance:
                min = i
        return self.items.pop(min), min


class Dijkstra(object):

    class Move(object):
        def __init__(self, id, prev_loc, loc):
            self.id = id
            self.prev_loc = prev_loc
            self.loc = loc

    def __init__(self):
        self.factory_map = FactoryMap()
        self._movement_order = rospy.Service('/robot/movement_order', MovementOrder, self._response_movement_order)
        self.rows=self.factory_map.rows
        self.columns= self.factory_map.columns
        self.graph=self.factory_map.graph
        self.coord_x = self.factory_map.coord_x
        self.coord_y = self.factory_map.coord_y
        self.robots_locations = dict()
        self.last_moves = []
        self._visualize_movement()
        # factory_map._visualize()

    def _response_movement_order(self, req):
        self.robots_locations[req.id] = req.current_location.location
        if req.current_location != req.destination:
            path = self.find_shortest_path(req.current_location.location, req.destination.location)
            self.last_moves.append(self.Move(req.id,req.current_location.location,path[0]))
            return MovementOrderResponse(WarehouseLocation(path[0]),False)
        else:
            return MovementOrderResponse(WarehouseLocation(0),True)


    def find_shortest_path(self,start_node, end_node):
        nodes = PriorityQueue()  
        removed_nodes = dict()
        path = []
        for k in self.graph.keys():
            if k != start_node:
                nodes.insert(k,Node(k,self.graph[k]))
            else:
                nodes.insert(k,Node(k,self.graph[k],0))
        while not nodes.is_empty():
            n,k = nodes.remove()
            if k == end_node:
                path.append(k)
                while n.prev_node != start_node:
                    path.append(n.prev_node)
                    n = removed_nodes[n.prev_node]
                path.reverse()
                return path
            
            removed_nodes[k]=n
            
            for e in n.edges:                                                                                                                                                                               
                if e in nodes.items.keys():
                    if e and nodes.items[e].distance > n.distance + 1:
                        nodes.items[e].distance = n.distance + 1
                        nodes.items[e].prev_node = k
                    elif e==0 and nodes.items[e].distance > n.distance + self.columns:
                        nodes.items[e].distance = n.distance + self.columns
                        nodes.items[e].prev_node = k

    def _visualize_movement(self):
        ann = dict()
        for n in self.graph.keys():
            plt.scatter(self.coord_x[n], self.coord_y[n], s=300, c='white', edgecolors='black', zorder=100)
            for i in self.graph[n]:
                plt.plot([self.coord_x[n], self.coord_x[i]], [self.coord_y[n], self.coord_y[i]], c='black')
        for n in self.graph.keys():
            if n >= self.factory_map.DUMP_OFFSET:
                plt.scatter(self.coord_x[n], self.coord_y[n], s=300, c='red', edgecolors='black', zorder=100)
            elif n >= self.factory_map.STATION_OFFSET:
                plt.scatter(self.coord_x[n], self.coord_y[n], s=300, c='green', edgecolors='black',zorder=100)
            elif n == 0:
                plt.scatter(self.coord_x[n], self.coord_y[n], s=300, c='blue', edgecolors='black',zorder=100)
        while not rospy.is_shutdown():
            while self.last_moves:
                m = self.last_moves.pop()
                if m.id in ann.keys():
                    ann[m.id].remove()
                a = plt.annotate(m.id, (self.coord_x[m.loc], self.coord_y[m.loc]), color='black', ha="center", va="center", zorder=101)
                ann[m.id]=a
            plt.pause(0.3)


if __name__ == '__main__':
    rospy.init_node('movement_controller')
    rospy.loginfo("Starting robots...")
    dijkstra = Dijkstra()
    rospy.spin()
