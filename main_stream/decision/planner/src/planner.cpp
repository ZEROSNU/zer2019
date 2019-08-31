#include "ros/ros.h"
#include "carsetupcomhandle.h"
#include "DORRT.h"
#include "magneticmodel.h"
#include "potentialmodel.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"
#include <string>
#include <iostream>
#include <functional>
#include <cmath>
#include <ompl/base/ScopedState.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include "core_msgs/ActiveNode.h"



namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;
bool nodeactivation = true;

void simplecarmodel(const ob::State *state, const oc::Control *control, const double time, ob::State *sp, double length) {
    const auto *s = state->as<ob::SE2StateSpace::StateType>();
    const auto *c = control->as<oc::RealVectorControlSpace::ControlType>();
    double x = s->getX();
    double y = s->getY();
    double yaw = s->getYaw();
    double u1 = c->values[0];
    double u2 = c->values[1];
    x += u1*cos(yaw);
    y += u1*sin(yaw);
    yaw += tan(u2)/length;
    sp->as<ob::SE2StateSpace::StateType>()->setXY(x,y);
    sp->as<ob::SE2StateSpace::StateType>()->setYaw(yaw);
}

void activecb(core_msgs::ActiveNode::ConstPtr msg) {
    int length = msg->active_nodes.size();
    bool kill = true;
    std::string nn = "path_planner";
    std::string monitor = "zero_monitor";
    for (int i = 0; i < length ; i++) {
        if ( nn.compare(msg->active_nodes[i]) == 0 ) {
            nodeactivation = true;
            std::cout << "node activated" << std::endl;
            return;
        }
        if ( monitor.compare(msg->active_nodes[i]) == 0 ) {
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

int main (int argc, char **argv) {

    std::string nn = "path_planner";
    ob::StateSpacePtr sspace(std::make_shared<ob::SE2StateSpace>());
    oc::ControlSpacePtr cspace(std::make_shared<oc::RealVectorControlSpace>(sspace, 2));
    oc::SpaceInformationPtr si(std::make_shared<oc::SpaceInformation>(sspace, cspace));
    oc::SimpleSetup ss(si);
    //for calculating forces
    MagneticModelPtr mm(std::make_shared<MagneticModel>());
    PotentialModelPtr pm(std::make_shared<PotentialModel>());
    //ob::ProblemDefinitionPtr pdef(std::make_shared<ob::ProblemDefinition>(si));
    ros::init(argc, argv, nn);

    double MaxVel = 1.0;
    double MaxSteer = 1.0;
    double AxlesLength = 1.0;
    double StepSize = 0.01;
    ros::NodeHandle nh;


    ros::Subscriber activenode = nh.subscribe("/active_nodes", 1000, activecb);
    
    std::string map_id = "car_frame";
    bool withgear = false;
    CarSetupComHandle comh = CarSetupComHandle(argc, argv, nn);
    comh.SimpleSetup();
    comh.SetTopicPub <geometry_msgs::PoseArray> ("/tree");
    /*<--Testing Fields
    std::string MagneticFieldTopic = "/MagneticField";
    std::string PotentialFieldTopic = "/PotentialField";
    comh.SetTopicPub <visualization_msgs::MarkerArray> (MagneticFieldTopic);
    comh.SetTopicPub <visualization_msgs::MarkerArray> (PotentialFieldTopic);
    //*/
    ompl::doRRTPtr pl(std::make_shared<ompl::doRRT>(si, mm, pm));
    while(ros::ok()) {
        if(CarSetupComHandle::isUpdatedMap()) {
            ss.clear();
            int gseq = CarSetupComHandle::GetLatestGoalSeq();
            int mseq = CarSetupComHandle::GetLatestMapSeq();
            int sseq = CarSetupComHandle::GetLatestStartSeq();
            std::cout << "planning sequence number : " << gseq << std::endl;

            nav_msgs::OccupancyGridConstPtr mp = CarSetupComHandle::GetMap(map_id, mseq);
            if (mp == NULL)
                continue;
            ob::RealVectorBounds sbounds(2);
            /*
            float l = sqrt(mp->info.width * mp->info.width + mp->info.height * mp->info.height) * mp->info.resolution;
            sbounds.setLow(0, mp->info.origin.position.x - l);
            sbounds.setHigh(0, mp->info.origin.position.x + l);
            sbounds.setLow(1, mp->info.origin.position.y - l);
            sbounds.setHigh(1, mp->info.origin.position.y + l);
            //*/
            ///*
            sbounds.setLow(0, mp->info.origin.position.x);
            sbounds.setHigh(0, mp->info.origin.position.x + mp->info.width * mp->info.resolution);
            sbounds.setLow(1, mp->info.origin.position.y);
            sbounds.setHigh(1, mp->info.origin.position.y + mp->info.height * mp->info.resolution);
            //*/
            sspace->as<ob::SE2StateSpace>() -> setBounds(sbounds);

            ob::RealVectorBounds cbounds(2);
            cbounds.setLow(0, 0);
            cbounds.setHigh(0, MaxVel);
            cbounds.setLow(1, -MaxSteer);
            cbounds.setHigh(1, MaxSteer);
            cspace ->as<oc::RealVectorControlSpace>() -> setBounds(cbounds);

            ob::OptimizationObjectivePtr obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);

            ob::ScopedStatePtr st = CarSetupComHandle::GetStart(sspace, map_id, sseq);
            ob::ScopedStatePtr gl = CarSetupComHandle::GetGoal(sspace, map_id, gseq);
            if (st == NULL || gl == NULL)
                continue;
            
            si->setStateValidityChecker([map_id, mseq, sspace](const ob::State *state) {return CarSetupComHandle::isStateValid(map_id, mseq, sspace, state);});
            si->setStatePropagator([AxlesLength](const ob::State *state, const oc::Control *control, const double time, ob::State *sp) {return simplecarmodel(state, control, time, sp, AxlesLength);});
            si->setPropagationStepSize(StepSize);
            si->setStateValidityCheckingResolution(0.005);

            //Testing magnetic field
            /*
            visualization_msgs::MarkerArray bma;
            visualization_msgs::MarkerArray pma;
            //mm->magneticTest(st->get(), gl->get(), si.get(), comh.GetMap(map_id, seq)->info.resolution*10, map_id, seq, bma);
            mm->magneticTest(st->get(), gl->get(), si.get(), comh.GetMap(map_id, mseq)->info.resolution, map_id, mseq, bma);
            pm->potentialTest(gl->get(), si.get(), comh.GetMap(map_id, mseq)->info.resolution, map_id, mseq, pma);
            comh.PublishTopicPub(MagneticFieldTopic, bma);
            comh.PublishTopicPub(PotentialFieldTopic, pma);
            //*/
            //<----------------------->

            ss.setOptimizationObjective(obj);
            pl->setup();
            /*
            pl->setPotMax(0.15);
            pl->setResolution(0.50);
            pl->setFieldNorm(10.0);
            pl->setMaxSample(100);
            pl->setExtendLength(6);
            pl->setApproxLength(5);
            pl->setR_RRT(50.0);
            pl->setReedsSheppRadius(8.0);
            //pl->setCheckTime(false);
            */
            ss.setPlanner(pl);
            //comh.ShowAll();
            //*/
            ss.setStartAndGoalStates(*st,*gl);
            ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
            ss.setup();
            std::cout << "------------------------SimpleSetup-------------------------"<<std::endl;
            ss.print();
            ob::PlannerStatus solved = ss.solve(1);


            ///*
            std::vector<ob::State *> tree = pl->getTree();
            geometry_msgs::PoseArray treemsg;
            treemsg.header.stamp = ros::Time::now();
            treemsg.header.seq = sseq;
            treemsg.header.frame_id = "car_frame";
            treemsg.poses.resize(tree.size());
            std::vector<ob::State*>::iterator it;
            int n;
            for(it = tree.begin(), n = 0; n < tree.size(); ++it, ++n) {
                const auto *s = (*it)->as<ob::SE2StateSpace::StateType>();
                treemsg.poses[n].position.x = s->getX();
                treemsg.poses[n].position.y = s->getY();
                tf2::Quaternion q;
                q.setEuler( 0, 0, s->getYaw());
                treemsg.poses[n].orientation.x = q[0];
                treemsg.poses[n].orientation.y = q[1];
                treemsg.poses[n].orientation.z = q[2];
                treemsg.poses[n].orientation.w = q[3];
            }
            comh.PublishTopicPub("/tree", treemsg);
            //pl->showTree();
            //pl->showCalcTime();
            //*/
            if (solved) {
                std::cout << "Found solution:" << std::endl;
                //ss.simplifySolution();
                oc::PathControl path = ss.getSolutionPath();
                //path.printAsMatrix(std::cout);
                if(nodeactivation) {
                    comh.PublishPath(map_id, mseq, path, withgear);
                }
                //if(comh.TransformPath("map", map_id, comh.GetLatestBaseSeq("map", map_id), 2, path)) {
//                    std::cout << "transformed successfully!!!!!!!!!!!!!" << std::endl;
                //}
                //comh.PublishPath("map", mseq, path, withgear);
            }
            else
                std::cout << "No solution found" << std::endl;
                
        }
        
        ros::spinOnce();
    }
    
    return 0;
}
