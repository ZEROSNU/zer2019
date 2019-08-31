#include "Model.h"
#include <string>
#include <iostream>
#include <vector>
#include "core_msgs/Control.h"
#include "core_msgs/VehicleState.h"

core_msgs::ControlConstPtr Model::GetControl(core_msgs::VehicleStateConstPtr vehiclestate) {
    std::cout << "fuck" << std::endl;
}