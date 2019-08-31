#ifndef MODEL
#define MODEL

#include <string>
#include <iostream>
#include <vector>
#include "core_msgs/Control.h"
#include "core_msgs/VehicleState.h"

class Model{
    private :
    std::string _name;
    std::map<std::string, double> _coefficients;
    core_msgs::VehicleStateConstPtr (*_function) (std::map<std::string, double>, core_msgs::ControlConstPtr);
    double (*_updater) (std::map<std::string, double>, std::vector<core_msgs::ControlConstPtr>, std::vector<core_msgs::VehicleStateConstPtr>);
    public :
    Model(std::string name = "noname") {
        this->_name = name;
    }
    double GetCoefficients(std::string coeff) {
        return this->_coefficients[coeff];
    }
    double ModelUpdate(std::vector<core_msgs::ControlConstPtr> control, std::vector<core_msgs::VehicleStateConstPtr> vehiclestate) {
        double confidence = this->_updater (this->_coefficients, control, vehiclestate);
        return confidence;
    }

    core_msgs::VehicleStateConstPtr GetVehicleState(core_msgs::ControlConstPtr control) {
        return this->_function(this->_coefficients, control);
    }
    core_msgs::ControlConstPtr GetControl(core_msgs::VehicleStateConstPtr vehiclestate);
};

#endif