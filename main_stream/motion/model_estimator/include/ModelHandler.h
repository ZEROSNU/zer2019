#ifndef MODELHANDLER
#define MODELHANDLER

#include <string>
#include <iostream>
#include <vector>
#include "Model.h"
#include "core_msgs/Control.h"
#include "core_msgs/VehicleState.h"

class ModelHandler{
    private :
    int _buffsize;
    std::string _name;
    std::map<std::string, Model> _model;
    std::map<std::string, double> _confidence;
    std::string _mostconf;
    std::vector<core_msgs::ControlConstPtr> _controlbuff;
    std::vector<core_msgs::VehicleStateConstPtr> _vehiclestatebuff;
    public :
    ModelHandler(std::string name = "noname") {
        this->_name = name;
        this->_buffsize = 100;
    }
    void SetBuffSize(int size) {
        this->_buffsize = size;
    }
    int GetBuffSize(void) {
        return this->_buffsize;
    }
    void SetModel(std::string name, Model model) {
        this->_model[name];
    }
    void ModelUpdate();
    core_msgs::VehicleStateConstPtr GetVehicleState(core_msgs::ControlConstPtr control) {
        return this->_model[this->_mostconf].GetVehicleState(control);
    }
    core_msgs::ControlConstPtr GetControl(core_msgs::VehicleStateConstPtr vehiclestate) {
        return this->_model[this->_mostconf].GetControl(vehiclestate);
    }
    void AddData(core_msgs::ControlConstPtr control);
    void AddData(core_msgs::VehicleStateConstPtr vehiclestate);
};

#endif