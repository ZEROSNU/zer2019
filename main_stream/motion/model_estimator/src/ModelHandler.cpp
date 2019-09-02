
#include <string>
#include <iostream>
#include <vector>
#include "Model.h"
#include "ModelHandler.h"
#include "core_msgs/Control.h"
#include "core_msgs/VehicleState.h"


void ModelHandler::ModelUpdate() {
    double maxconf = 0;
    for (std::map<std::string, Model>::iterator it = this->_model.begin() ; it!= this->_model.end() ; it++) {
        this->_confidence[it->first] = it->second.ModelUpdate(this->_controlbuff, this->_vehiclestatebuff);
        if (maxconf < _confidence[it->first]) {
            maxconf = _confidence[it->first];
            this->_mostconf = it->first;
        }
    }
    return;
}


void ModelHandler::AddData(core_msgs::ControlConstPtr control) {
    if(this->_controlbuff.size() > this->_buffsize) {
        this->_controlbuff.erase(this->_controlbuff.begin());
    }
    this->_controlbuff.push_back(control);
}
void ModelHandler::AddData(core_msgs::VehicleStateConstPtr vehiclestate) {
    if(this->_vehiclestatebuff.size() > this->_buffsize) {
        this->_vehiclestatebuff.erase(this->_vehiclestatebuff.begin());
    }
    this->_vehiclestatebuff.push_back(vehiclestate);
}