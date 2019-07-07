#ifndef CARSETUPCOMHANDLE
#define CARSETUPCOMHANDLE
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <string>
#include <iostream>
#include <map>

#include <tf2/LinearMath/Quaternion.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/control/PathControl.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

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
    std::map<std::string, ros::Publisher> topic_pubs;
    static std::string nodename;
    static std::map<std::string,std::map<int, geometry_msgs::PoseWithCovarianceStampedConstPtr>> start;
    static std::map<std::string,std::map<int, geometry_msgs::PoseStampedConstPtr>> goal;
    static std::map<std::string,std::map<int, nav_msgs::OccupancyGridConstPtr>> map;
    static std::map<std::string,std::map<std::string, std::map<int, geometry_msgs::TransformStampedConstPtr>>> base;
    static std::map<std::string,std::map<int, geometry_msgs::PointStampedConstPtr>> point;
    static std::map<std::string, int> latest_seq;
    static std::map<std::string, bool> is_updated;

    public:
    CarSetupComHandle(int argc, char **argv, std::string nodename);


    void SetStart(const std::string topic = "/initial_pose");
    void StartCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init);
    static ob::ScopedStatePtr GetStart(const ob::StateSpacePtr& space, std::string id, int seq);

    void SetGoal(const std::string topic = "/goal_pose");
    void GoalCB(const geometry_msgs::PoseStamped::ConstPtr& fin);
    static ob::ScopedStatePtr GetGoal(const ob::StateSpacePtr &space, std::string id, int seq);

    void SetBase(const std::string topic = "/tf");
    void BaseCB(const tf2_msgs::TFMessage::ConstPtr& bs);
    static geometry_msgs::TransformStampedConstPtr GetBase(std::string from_id, std::string to_id, int seq);

    void SetMap(const std::string topic = "/local_map");
    void MapCB(const nav_msgs::OccupancyGrid::ConstPtr& mp);
    static nav_msgs::OccupancyGridConstPtr GetMap(std::string id, int seq);

    void SetPoint(const std::string topic = "/clicked_point");
    void PointCB(const geometry_msgs::PointStamped::ConstPtr& pt);
    static geometry_msgs::PointStamped::ConstPtr GetPoint(std::string id, int seq);

    void SetPath(const std::string topic = "/path");
    void PublishPath(const std::string id, int seq, og::PathGeometric& path, bool withgear);
    void PublishPath(const std::string id, int seq, oc::PathControl& path, bool withgear);

    bool TransformPath(const std::string from_id, const std::string to_id, int tfseq, int pathon, og::PathGeometric& path);
    bool TransformPath(const std::string from_id, const std::string to_id, int tfseq, int pathon, oc::PathControl& path);
    bool TransformState(const std::string from_id, const std::string to_id, int tfseq, int stateon, ob::ScopedStatePtr state);

    template<class T> void SetTopicPub(const std::string topic) {
        std::cout << "Publishing somthing as topic name \"" << topic << "\"\n";
        ros::NodeHandle nh;
        this->topic_pubs[topic] = nh.advertise<T>(topic, 1000);
    }
    template<class T> void PublishTopicPub(const std::string topic, const T& msg) {
        this->topic_pubs[topic].publish(msg);
        std::cout << topic << " published" << std::endl;
    }
    
    void SimpleSetup();

    static int GetLatestMapSeq();
    static int GetLatestBaseSeq(const std::string from_id, const std::string to_id);
    static int GetLatestStartSeq();
    static int GetLatestGoalSeq();
    static int GetLatestPointSeq();

    static bool isUpdatedMap();
    static bool isUpdatedBase(const std::string from_id, const std::string to_id);
    static bool isUpdatedStart();
    static bool isUpdatedGoal();
    static bool isUpdatedPoint();

    static void ShowAll();

    static void ShowMap();
    static void ShowBase();
    static void ShowStart();
    static void ShowGoal();
    static void ShowPoint();
    
    static bool CheckonMap (std::string id, int seq, double x, double y);
    static bool isStateValid (std::string id, int seq, const ob::StateSpacePtr &space, const ob::State *state);

    ~CarSetupComHandle();
};
#endif
