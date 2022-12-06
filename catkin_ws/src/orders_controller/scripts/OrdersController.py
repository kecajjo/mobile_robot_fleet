import rospy
import queue
from common_msgs.msg import Part, Order, RobotStatus, WarehouseLocation, RobotState

class RobotOrderType(object):
    MOVE_TO = 0
    LOAD = 1
    UNLOAD = 2

class RobotOrder(object):
    def __init__(self, order_type, order_data):
        self.order_type = order_type
        self.data = order_data

class WarehouseOrder(object):
    def __init__(self, part_id, amount, line):
        self.warehouse_id = part_id
        self.part_id = part_id
        self.amount = amount
        self.line_id = line

class MoveTo(object):
    def __init__(self, location_id):
        self.location_id = location_id

class Load(object):
    def __init__(self, parts, amount):
        self.parts = parts
        self.amount = amount

class Unload(object):
    def __init__(self, parts, amount):
        self.parts = parts
        self.amount = amount

class RobotInfo(object):
    def __init__(self, state, position, load, capacity):
        self.state = state
        self.position = position
        self.load = load
        self.capacity = capacity
        self.tasks = queue.Queue(30)

class OrdersController(object):
    def __init__(self):
        self._transit_orders = queue.Queue(30)
        self._robots_states = {}

    def _proceed_next_order(self):
        if self._transit_orders.empty():
            return
        warehouse_order = self._transit_orders.get()
        robot_orders = self._warehouse_order_to_robot_orders(warehouse_order)
        if not self._assign_orders_to_robot(robot_orders):
            self._transit_orders.put(warehouse_order)

    def _update_robot_status_received(self, msg):
        self._robots_states[msg.id] = RobotInfo(msg.state, msg.location, msg.carried_parts, msg.capacity)
        if msg.state == RobotState.READY:
            self._order_robot(msg.id)
    
    def _order_robot(self, id):
        robot_info = self._robots_states[id]
        if robot_info.state != RobotState.READY:
            return
        if robot_info.tasks.empty():
            if robot_info.position == WarehouseLocation.RESTING_AREA:
                return
            else:
                robot_info.tasks.put(RobotOrder(RobotOrderType.MOVE_TO, MoveTo(WarehouseLocation.RESTING_AREA)))
        order = self._robots_states[id].tasks.get()
        if order.order_type == RobotOrderType.MOVE_TO:
            pass
        elif order.order_type == RobotOrderType.LOAD:
            pass
        elif order.order_type == RobotOrderType.UNLOAD:
            pass
        else:
            raise Exception

    def _transit_order_received(self, order):
        processed_orders_list = self._process_order_msg(order)
        for order in processed_orders_list:
            self._transit_orders.put(order)

    def _assign_orders_to_robot(self, orders):
        to_load = 0
        for order in orders:
            if order.order_type == RobotOrderType.LOAD:
                to_load = to_load + order.data.amount

        capable_robots = []
        for info in self._robots_states.values():
            if info.capacity <= to_load and info.tasks.qsize() < 12:
                capable_robots.append(info)

        if capable_robots == []:
            return False
        
        robot = min(capable_robots, key = lambda robot: robot.tasks.qsize())
        for order in orders:
            robot.tasks.put(order)
        return True

    @staticmethod
    def _process_order_msg(order):
        return [WarehouseOrder(part.part_id, part.amount, order.line) for part in order.parts]

    @staticmethod
    def _warehouse_order_to_robot_orders(order):
        ret_list = []
        ret_list.append(RobotOrder(RobotOrderType.MOVE_TO, MoveTo(order.warehouse_id)))
        ret_list.append(RobotOrder(RobotOrderType.LOAD, Load(order.part_id, order.amount)))
        ret_list.append(RobotOrder(RobotOrderType.MOVE_TO, MoveTo(order.line_id)))
        ret_list.append(RobotOrder(RobotOrderType.UNLOAD, Unload(order.part_id, order.amount)))
        return ret_list


