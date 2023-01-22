import rospy
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
