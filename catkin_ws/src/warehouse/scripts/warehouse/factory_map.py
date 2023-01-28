import rospy
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from common_msgs.msg import WarehouseLocation

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
        self.coord_x = dict()
        self.coord_y = dict()
        self.graph[0] = [x for x in range(1, columns+1)]  # RESTING AREA
        self.coord_x[0] = (columns+1)/2
        self.coord_y[0] = 0
        station = [x for x in range(FactoryMap.STATION_OFFSET, FactoryMap.STATION_OFFSET+rows+1)]
        dump = [x for x in range(FactoryMap.DUMP_OFFSET, FactoryMap.DUMP_OFFSET+rows+1)]
        for i in range(1, rows+1):
            self.graph[station[i-1]] = [1+(i-1)*columns]
            self.coord_x[station[i-1]] = 0
            self.coord_y[station[i-1]] = i
            self.graph[dump[i-1]] = [i*columns]
            self.coord_x[dump[i-1]] = columns+1
            self.coord_y[dump[i-1]] = i
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
                self.coord_x[j+(i-1)*columns] = j
                self.coord_y[j+(i-1)*columns] = i
        # self._visualize()
        # print(self.graph)

    def _get_move_areas(self):
        return([area for area in self.graph.keys()
                if area > FactoryMap.RESTING_AREA and area < FactoryMap.STATION_OFFSET])

    def _get_station_areas(self):
        return([area for area in self.graph.keys()
               if area >= FactoryMap.STATION_OFFSET and area < FactoryMap.DUMP_OFFSET])

    def _get_dump_areas(self):
        return([area for area in self.graph.keys() if area >= FactoryMap.DUMP_OFFSET])

    def _visualize(self):
        for n in self.graph.keys():
            for i in self.graph[n]:
                plt.plot([self.coord_x[n], self.coord_x[i]], [self.coord_y[n], self.coord_y[i]], c='black')
        plt.scatter(self.coord_x.values(), self.coord_y.values(), s=300, c='white', edgecolors='black', zorder=100)
        for n in self.graph.keys():
            if n >= self.DUMP_OFFSET:
                plt.scatter(self.coord_x[n], self.coord_y[n], s=300, c='red', edgecolors='black', zorder=100)
                plt.annotate('1', (1, 1), color='black', ha="center", va="center", zorder=101)
                plt.annotate('2', (1, 1), color='black', ha="center", va="center", zorder=101)
            elif n >= self.STATION_OFFSET:
                plt.scatter(self.coord_x[n], self.coord_y[n], s=300, c='green', edgecolors='black',zorder=100)
            elif n == 0:
                plt.scatter(self.coord_x[n], self.coord_y[n], s=300, c='blue', edgecolors='black',zorder=100)
        while not rospy.is_shutdown():
            plt.pause
        plt.show()
