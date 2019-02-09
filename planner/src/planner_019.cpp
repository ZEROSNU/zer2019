#include "ros/ros.h"
#include "carsetupcomhandle.h"

#include <string>
#include <iostream>
#include <functional>
#include <cmath>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main (int argc, char **argv) {
    std::string nn = "planner_019";
    ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>());
    og::SimpleSetup ss(space);
    ros::init(argc, argv, nn);
    
    ros::NodeHandle nh;
    std::string map_id;
    nh.getParam("/map_id", map_id);

    CarSetupComHandle comh = CarSetupComHandle(argc, argv, nn);
    comh.SimpleSetup();


    int seq = 0;
    while(ros::ok()) {
        if(seq<CarSetupComHandle::GetLatestSeq()) {
            ss.clear();
            seq = CarSetupComHandle::GetLatestSeq();
            std::cout << "planning sequence number : " << seq << std::endl;

            nav_msgs::OccupancyGridConstPtr mp = CarSetupComHandle::GetMap(map_id, seq);
            if (mp == NULL)
                continue;
            ob::RealVectorBounds bounds(2);
            float l = sqrt(mp->info.width * mp->info.width + mp->info.height * mp->info.height) * mp->info.resolution;
            bounds.setLow(0, mp->info.origin.position.x - l);
            bounds.setHigh(0, mp->info.origin.position.x + l);
            bounds.setLow(1, mp->info.origin.position.y - l);
            bounds.setHigh(1, mp->info.origin.position.y + l);
            
            space->as<ob::SE2StateSpace>() -> setBounds(bounds);
            const ob::SpaceInformation *si = ss.getSpaceInformation().get();
            
            ss.setStateValidityChecker([map_id, seq, si](const ob::State *state) {return CarSetupComHandle::isStateValid(map_id, seq, si, state);});
            ob::ScopedStatePtr st = CarSetupComHandle::GetStart(space, map_id, seq);
            ob::ScopedStatePtr gl = CarSetupComHandle::GetGoal(space, map_id, seq);

            if (st == NULL || gl == NULL)
                continue;
            ss.setStartAndGoalStates(*st,*gl);            
            ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
            ss.setup();
            ss.print();
            ob::PlannerStatus solved = ss.solve(0.03);
            if (solved) {
 
                std::cout << "Found solution:" << std::endl;
                ss.simplifySolution();
                og::PathGeometric path = ss.getSolutionPath();
                path.interpolate(1000);
                comh.PublishPath(map_id, seq, path);
            }
            else
                std::cout << "No solution found" << std::endl;
        }
        
        ros::spinOnce();
    }
    
    return 0;
}
