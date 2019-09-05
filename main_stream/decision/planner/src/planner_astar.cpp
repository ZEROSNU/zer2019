// 2019_planner.cpp

// libraries
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <cmath>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/PathControl.h>

#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/SimpleSetup.h"
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "carsetupcomhandle.h"
#include "hybrid_astar.h"
#include "core_msgs/ActiveNode.h"
#include "geometry_msgs/PoseArray.h"
#include "core_msgs/MotionState.h"

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

// activeness
bool nodeactivation = true;
bool parking = false;
// callback for handling activeness
void motioncb(core_msgs::MotionState::ConstPtr msg) {
    if (msg->motion_state.compare("PARKING") == 0) {
        parking = true;
    }
    else {
        parking = false;
    }
    return;
}
void activecb(core_msgs::ActiveNode::ConstPtr msg) {
    int length = msg->active_nodes.size();
    bool kill = true;
    std::string nn = "path_planner";
    std::string monitor = "zero_monitor";
    for (int i = 0; i < length ; i++) {
        if (nn.compare(msg->active_nodes[i]) == 0) {
            nodeactivation = true;
            std::cout << "node activated" << std::endl;
            return;
        }
        if (monitor.compare(msg->active_nodes[i]) == 0) {
            kill = false;
        }
    }
    nodeactivation = false;
    if (kill) {
        ros::shutdown();
    }
    std::cout << "node deactivated" << std::endl;
    return;
}

int main(int argc, char **argv){
    std::string node_name = "hybrid_astar_planner";
    ob::StateSpacePtr space(std::make_shared<ob::SE2StateSpace>());
    //oc::ControlSpacePtr cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2)); 
    //oc::SpaceInformationPtr space_info(std::make_shared<oc::SpaceInformation>(space, cspace));

    ob::SpaceInformationPtr space_info(std::make_shared<ob::SpaceInformation>(space));
    og::SimpleSetup ss(space_info);
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;   

    //Get activeness from active_nodes
    ros::Subscriber isparking = nh.subscribe("/motion_state", 1000, motioncb);
    ros::Subscriber activenode = nh.subscribe("/active_nodes", 1000, activecb);

    
    //Get map date from Nodehandle
    std::string map_id = "car_frame";
    //nh.getParam("/map_id", map_id);
    bool withgear = false;
    //Date input from Rviz
    CarSetupComHandle comh = CarSetupComHandle(argc, argv, node_name);
    comh.SimpleSetup();
    comh.SetTopicPub<geometry_msgs::PoseArray>("/hybrid_astar");
    //setup planner
    ompl::hybridASTARPtr planner(std::make_shared<ompl::hybridASTAR>(space_info));
    int seq = 0;


    while(ros::ok()){
        if(parking) {
            ros::spinOnce();
//            std::cout << "parking" << std::endl;
            continue;
        }
        //Check if the message was updated
        if(CarSetupComHandle::isUpdatedMap()){
            //Clear the previous state space, update seq, acquire map
            ss.clear();
            int gseq = CarSetupComHandle::GetLatestGoalSeq();
            int mseq = CarSetupComHandle::GetLatestMapSeq();
            int sseq = CarSetupComHandle::GetLatestStartSeq();

            // //DEBUG: not sure if these values are accurate (especially the start)
            // std::cout << "-------------------debug-----------------" << std::endl;
            // std::cout << "goal sequence: " << gseq << std::endl;
            // std::cout << "map sequence: " << mseq << std::endl;
            // std::cout << "start sequence: " << sseq << std::endl;
            // std::cout << "----------------debug end-----------------" << std::endl;

            nav_msgs::OccupancyGridConstPtr input_map = CarSetupComHandle::GetMap(map_id, mseq); 
            
            //Error exception for empty map
            if(input_map == NULL){
              ROS_ERROR_STREAM("map input is empty"); //current map is empty
              break;
            } 
            
            //Set dimensions and bounds for the input map
            int map_length = input_map->info.height;
            int map_width = input_map->info.width;
            float map_len_res = input_map->info.resolution;
            float map_len_fix = input_map->info.height * input_map->info.resolution;
            float map_wid_fix = input_map->info.width * input_map->info.resolution;
            //std::cout << "--info.resolution--" << std::endl;
            //std::cout << map_len_res << std::endl;
            //std::cout << "--info.origin.position.x--" << std::endl;
            //std::cout << input_map->info.origin.position.x << std::endl;
            //std::cout << "--info.origin.position.y--" << std::endl;
            //std::cout << input_map->info.origin.position.y << std::endl;

            // //DEBUG: length and width inputs are working fine
            // std::cout << "-------------------debug-----------------" << std::endl;
            // std::cout << "map length: " << map_length << std::endl;
            // std::cout << "map width: " << map_width << std::endl;
            // std::cout << "----------------debug end-----------------" << std::endl;

            ob::RealVectorBounds map_bounds(2);
            map_bounds.setLow(0, input_map->info.origin.position.x);
            map_bounds.setLow(1, input_map->info.origin.position.y);
            map_bounds.setHigh(0, input_map->info.origin.position.x + map_wid_fix);
            map_bounds.setHigh(1, input_map->info.origin.position.y + map_len_fix);
            //sbounds.setLow(0, mp->info.origin.position.x);
            //sbounds.setHigh(0, mp->info.origin.position.x + mp->info.width * mp->info.resolution);
            //sbounds.setLow(1, mp->info.origin.position.y);
            //sbounds.setHigh(1, mp->info.origin.position.y + mp->info.height * mp->info.resolution);
            //*/
            //map_bounds.setLow(0, -3);
            //map_bounds.setLow(1, -3);
            //map_bounds.setHigh(0, 3);
            //map_bounds.setHigh(1, 3);

            //std::cout << "X Bounds: " << map_bounds.low[0] << ", " << map_bounds.high[0] << std::endl;
            //std::cout << "Y Bounds: " << map_bounds.low[1] << ", " << map_bounds.high[1] << std::endl;

            space->as<ob::SE2StateSpace>()->setBounds(map_bounds);
            ob::OptimizationObjectivePtr obj = std::make_shared<ob::PathLengthOptimizationObjective>(space_info);
            

            ob::ScopedStatePtr start = CarSetupComHandle::GetStart(space, map_id, sseq);
            ob::ScopedStatePtr goal = CarSetupComHandle::GetGoal(space, map_id, gseq);
        
            space_info->setStateValidityChecker([map_id, mseq, space](const ob::State *state) {return CarSetupComHandle::isStateValid(map_id, mseq, space, state);});
            space_info->setStateValidityCheckingResolution(0.005);
            //setting up rest of the planner and the goal points
            ss.setOptimizationObjective(obj);
            planner->setup();
            ss.setPlanner(planner);
            ss.setStartAndGoalStates(*start, *goal);
            ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
            ss.setup();
            //std::cout << "-------------------------------------SimpleSetup----------------------"<<std::endl;
            //ss.print();
            ob::PlannerStatus solved = ss.solve(1);
            if(solved){
                //std::cout << "Path found" << std::endl;
                og::PathGeometric path = ss.getSolutionPath();
                if(nodeactivation){
                    comh.PublishPath(map_id, mseq, path, withgear);
                }
            } else {
                og::PathGeometric path(space_info);
                if(nodeactivation){
                    comh.PublishPath(map_id, mseq, path, withgear);
                }
                std::cout << "NO PATH FOUND" << std::endl;
            }
        }
        
        ros::spinOnce();
    }

    return 0;
}
