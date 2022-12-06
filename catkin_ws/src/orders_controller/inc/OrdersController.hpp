#pragma once

#include <map>
#include <memory>
#include <queue>
#include <utility>

enum PositionType { ROAD = 0, LINE = 1, WAREHOUSE = 2, PARKING };

class OrdersController {
  private:
    struct Order {
        float amount;
        unsigned int partId;
        unsigned int lineId;
    };
    struct RobotPosition {
        PositionType type;
        unsigned int id;
    };
    struct RobotInfo {
        float capacity;
        float load;
        RobotState state;
        std::queue<Order> tasks;
        RobotPosition position;
    };

  public:
    void loadRobot(std::pair<unsigned int, unsigned int> part);
    void registerRobot(unsigned int id, std::unique_ptr<RobotInfo> robotInfo);
    void updateRobotInfo(unsigned int id, std::unique_ptr<RobotInfo> robotInfo);
    void orderRobot();

  private:
    void transitOrderReceived();
    void registerRobotReceived();
    void updateRobotStateReceived();
    unsigned int getRobotAtWarehouseId();
    void proceedNextOrder();

    std::map<unsigned int, std::unique_ptr<RobotInfo>> robotsStates;
    std::queue<Order> transitOrders;
};