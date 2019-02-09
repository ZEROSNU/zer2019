#ifndef CARSETUPCOMHANDLE
#define CARSETUPCOMHANDLE
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <string>
#include <iostream>
#include <map>

#include <tf2/LinearMath/Quaternion.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class CarSetupComHandle {
    private:
    ros::Subscriber start_subs;
    ros::Subscriber goal_subs;
    ros::Subscriber map_subs;
    ros::Subscriber base_subs;
    ros::Subscriber point_subs;
    ros::Publisher path_pubs;
    std::map<std::string, bool> isinitialized;
    std::map<std::string, int> seqbias;
    static std::string nodename;
    static std::map<std::string,std::map<int, geometry_msgs::PoseWithCovarianceStampedConstPtr>> start;
    static std::map<std::string,std::map<int, geometry_msgs::PoseStampedConstPtr>> goal;
    static std::map<std::string,std::map<int, nav_msgs::OccupancyGridConstPtr>> map;
    static std::map<std::string,std::map<int, tf2_msgs::TFMessageConstPtr>> base;
    static std::map<std::string,std::map<int, geometry_msgs::PointStampedConstPtr>> point;
    static int latest_seq_start;
    static int latest_seq_goal;
    static int latest_seq_map;
    static int latest_seq_base;

    public:
    CarSetupComHandle(int argc, char **argv, std::string nodename);


    void SetStart(const std::string topic = "/initialpose");
    void StartCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init);
    static ob::ScopedStatePtr GetStart(const ob::StateSpacePtr& space, std::string id, int seq);

    void SetGoal(const std::string topic = "/goalpose");
    void GoalCB(const geometry_msgs::PoseStamped::ConstPtr& fin);
    static ob::ScopedStatePtr GetGoal(const ob::StateSpacePtr &space, std::string id, int seq);

    void SetBase(const std::string topic = "/tf");
    void BaseCB(const tf2_msgs::TFMessage::ConstPtr& bs);
    static tf2_msgs::TFMessageConstPtr GetBase(std::string id, int seq);

    void SetMap(const std::string topic = "/map");
    void MapCB(const nav_msgs::OccupancyGrid::ConstPtr& mp);
    static nav_msgs::OccupancyGridConstPtr GetMap(std::string id, int seq);

    void SetPoint(const std::string topic = "/clicked_point");
    void PointCB(const geometry_msgs::PointStamped::ConstPtr& pt);
    static geometry_msgs::PointStamped::ConstPtr GetPoint(std::string id, int seq);

    void SetPath(const std::string topic = "/planned_path");
    void PublishPath(const std::string id, int seq, og::PathGeometric& path);

    void SimpleSetup();

    static int GetLatestSeq();
    
    static bool CheckonMap (std::string id, int seq, float x, float y);
    static bool isStateValid (std::string id, int seq, const ob::SpaceInformation *si, const ob::State *state);

    ~CarSetupComHandle();
};
#endif