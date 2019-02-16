#include "ros/ros.h"
#include "carsetupcomhandle.h"
#include "DORRT.h"
#include "magneticmodel.h"
#include "potentialmodel.h"
#include "visualization_msgs/MarkerArray.h"
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



namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

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


int main (int argc, char **argv) {
    std::string nn = "planner_019";
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
    std::string map_id;
    bool withgear = false;
    nh.getParam("/map_id", map_id);
    CarSetupComHandle comh = CarSetupComHandle(argc, argv, nn);
    comh.SimpleSetup();
    /*<--Testing Fields
    std::string MagneticFieldTopic = "/MagneticField";
    std::string PotentialFieldTopic = "/PotentialField";
    comh.SetTopicPub <visualization_msgs::MarkerArray> (MagneticFieldTopic);
    comh.SetTopicPub <visualization_msgs::MarkerArray> (PotentialFieldTopic);
    //*/
    ompl::doRRTPtr pl(std::make_shared<ompl::doRRT>(si, mm, pm));
    int mseq = -1;
    while(ros::ok()) {
        if(mseq<CarSetupComHandle::GetLatestMapSeq()) {
            ss.clear();
            int gseq = CarSetupComHandle::GetLatestGoalSeq();
            mseq = CarSetupComHandle::GetLatestMapSeq();
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
            mm->magneticTest(st->get(), gl->get(), si.get(), 0.25, map_id, seq, bma);
            pm->potentialTest(gl->get(), si.get(), 0.25, map_id, seq, pma);
            comh.PublishTopicPub(MagneticFieldTopic, bma);
            comh.PublishTopicPub(PotentialFieldTopic, pma);
            //*/
            //<----------------------->

            ss.setOptimizationObjective(obj);
            pl->setup();
            ss.setPlanner(pl);
            comh.ShowAll();
            //*/
            ss.setStartAndGoalStates(*st,*gl);
            ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
            ss.setup();
            std::cout << "------------------------SimpleSetup-------------------------"<<std::endl;
            ss.print();
            ob::PlannerStatus solved = ss.solve(1);
            if (solved) {
                std::cout << "Found solution:" << std::endl;
                //ss.simplifySolution();
                oc::PathControl path = ss.getSolutionPath();
                //path.printAsMatrix(std::cout);
                //comh.PublishPath(map_id, mseq, path, withgear);
                if(comh.TransformPath("map", map_id, comh.GetLatestBaseSeq("map", map_id), 2, path)) {
                    std::cout << "transformed successfully!!!!!!!!!!!!!" << std::endl;
                }
                comh.PublishPath("map", mseq, path, withgear);
            }
            else
                std::cout << "No solution found" << std::endl;
                
        }
        
        ros::spinOnce();
    }
    
    return 0;
}