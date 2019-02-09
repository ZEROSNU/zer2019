#include "carsetupcomhandle.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include <string>
#include <iostream>
#include <map>

#include <tf2/LinearMath/Quaternion.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <cmath>


std::string CarSetupComHandle::nodename;
std::map<std::string,std::map<int, geometry_msgs::PoseWithCovarianceStamped::ConstPtr>> CarSetupComHandle::start;
std::map<std::string,std::map<int, geometry_msgs::PoseStamped::ConstPtr>> CarSetupComHandle::goal;
std::map<std::string,std::map<int, nav_msgs::OccupancyGrid::ConstPtr>> CarSetupComHandle::map;
std::map<std::string,std::map<int, tf2_msgs::TFMessage::ConstPtr>> CarSetupComHandle::base;
std::map<std::string,std::map<int, geometry_msgs::PointStamped::ConstPtr>> CarSetupComHandle::point;

int CarSetupComHandle::latest_seq_start = 0;
int CarSetupComHandle::latest_seq_goal = 0;
int CarSetupComHandle::latest_seq_map = 0;
int CarSetupComHandle::latest_seq_base = 0;

CarSetupComHandle::CarSetupComHandle(int argc, char **argv, std::string nodename) {
    CarSetupComHandle::nodename = nodename;
    std::cout << "CarSetupComHandle with node name \"" << nodename << "\" created\n";
    this -> isinitialized["start"] = false;
    this -> isinitialized["goal"] = false;
    this -> isinitialized["base"] = false;
    this -> isinitialized["map"] = false;
    this -> isinitialized["point"] = false;

}
void CarSetupComHandle::SetStart(const std::string topic) {
    std::cout << "Subscribing StartPoint as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->start_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::StartCB, this);
}
void CarSetupComHandle::StartCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init) {
    if (!this->isinitialized["start"]) {
        this->seqbias["start"] = init->header.seq;
        this->isinitialized["start"] = true;
        std::cout << "start_initialized" << std::endl;
    }
    std::cout << "start_seq number : " << init-> header.seq - this->seqbias["start"] << ", map id : " << init->header.frame_id << std::endl;
    start[init->header.frame_id][init->header.seq - this->seqbias["start"]] = init;
    CarSetupComHandle::latest_seq_start = init->header.seq - this->seqbias["start"];
}
ob::ScopedStatePtr CarSetupComHandle::GetStart(const ob::StateSpacePtr& space, std::string id, int seq) {
    if (start.count(id) < 1) {
        std::cout << "Start:No id found\n";
        return NULL;
    }
    else if (start[id].count(seq) < 1) {
        std::cout << "Start:No sequence found\n";
        return NULL;
    }
    else {
        ob::ScopedState<> st(space);

        tf2::Quaternion q(start[id][seq] -> pose.pose.orientation.x, start[id][seq] -> pose.pose.orientation.y, start[id][seq] -> pose.pose.orientation.z, start[id][seq] -> pose.pose.orientation.w);
        st[0] = start[id][seq] -> pose.pose.position.x;
        st[1] = start[id][seq] -> pose.pose.position.y;
        st[2] = q.getAngle();
    
        return std::make_shared<ob::ScopedState<>> (st);
    }
}


void CarSetupComHandle::SetGoal(const std::string topic) {
    std::cout << "Subscribing GoalPoint as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->goal_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::GoalCB, this);
}
void CarSetupComHandle::GoalCB(const geometry_msgs::PoseStamped::ConstPtr& fin) {
    if (!this->isinitialized["goal"]) {
        this->seqbias["goal"] = fin->header.seq;
        this->isinitialized["goal"] = true;
        std::cout << "goal_initialized" << std::endl;
    }
    std::cout << "goal_seq number : " << fin->header.seq - this->seqbias["goal"]<< ", goal id : " << fin->header.frame_id << std::endl;
    goal[fin->header.frame_id][fin->header.seq - this->seqbias["goal"]] = fin;
    CarSetupComHandle::latest_seq_goal = fin->header.seq - this->seqbias["goal"];
}
ob::ScopedStatePtr CarSetupComHandle::GetGoal(const ob::StateSpacePtr &space, std::string id, int seq) {
    if (goal.count(id) < 1) {
        std::cout << "Goal:No id found\n";
        return NULL;
    }
    else if (goal[id].count(seq) < 1) {
        std::cout << "Goal:No sequence found\n";
        return NULL;
    }
    else {
        ob::ScopedState<> st(space);

        tf2::Quaternion q(goal[id][seq] -> pose.orientation.x, goal[id][seq] -> pose.orientation.y, goal[id][seq] -> pose.orientation.z, goal[id][seq] -> pose.orientation.w);
        st[0] = goal[id][seq] -> pose.position.x;
        st[1] = goal[id][seq] -> pose.position.y;
        st[2] = q.getAngle();
    
        return std::make_shared<ob::ScopedState<>> (st);
    }
}

void CarSetupComHandle::SetBase(const std::string topic) {
    std::cout << "Subscribing BaseFrame as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->base_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::BaseCB, this);
}
void CarSetupComHandle::BaseCB(const tf2_msgs::TFMessage::ConstPtr& bs) {
    if (!this->isinitialized["base"]) {
        this->seqbias["base"] = bs->transforms[0].header.seq;
        this->isinitialized["base"] = true;
        std::cout << "base_initialized" << std::endl;
    }
    std::cout << "base_seq number : " << bs->transforms[0].header.seq - this->seqbias["base"] << ", base id : " << bs->transforms[0].child_frame_id << std::endl;
    base[bs->transforms[0].child_frame_id][bs->transforms[0].header.seq - this->seqbias["base"]] = bs;
    CarSetupComHandle::latest_seq_base = bs->transforms[0].header.seq - this->seqbias["base"];
}
tf2_msgs::TFMessage::ConstPtr CarSetupComHandle::GetBase(std::string id, int seq) {
    if (base.count(id) < 1) {
        std::cout << "TF:No id found\n";
        return NULL;
    }
    else if (base[id].count(seq) < 1) {
        std::cout << "TF:No sequence found\n";
        return NULL;
    }
    else
        return base[id][seq];
}

void CarSetupComHandle::SetMap(const std::string topic) {
    std::cout << "Subscribing Map as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->map_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::MapCB, this);
}
void CarSetupComHandle::MapCB(const nav_msgs::OccupancyGrid::ConstPtr& mp) {
    if (!this->isinitialized["map"]) {
        this->seqbias["map"] = mp->header.seq;
        this->isinitialized["map"] = true;
        std::cout << "map_initialized" << std::endl;
    }
    std::cout << "map_seq number : " << mp->header.seq - this->seqbias["map"] << ", map id : " << mp->header.frame_id << std::endl;
    map[mp->header.frame_id][mp->header.seq - this->seqbias["map"]] = mp;
    CarSetupComHandle::latest_seq_map = mp->header.seq - this->seqbias["map"];
}
nav_msgs::OccupancyGrid::ConstPtr CarSetupComHandle::GetMap(std::string id, int seq) {
    if (map.count(id) < 1) {
        std::cout << "Map:No id found\n";
        return NULL;
    }
    else if (map[id].count(seq) < 1) {
        std::cout << "Map:No sequence found\n";
        return NULL;
    }
    else
        return map[id][seq];
}

void CarSetupComHandle::SetPoint(const std::string topic) {
    std::cout << "Subscribing Point as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->point_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::PointCB, this);
}
void CarSetupComHandle::PointCB(const geometry_msgs::PointStamped::ConstPtr& pt) {
    if (!this->isinitialized["point"]) {
        this->seqbias["point"] = pt->header.seq;
        this->isinitialized["point"] = true;
        std::cout << "point_initialized" << std::endl;
    }
    std::cout << "point_seq number : " << pt->header.seq - this->seqbias["point"] << ", point id : " << pt->header.frame_id << std::endl;
    point[pt->header.frame_id][pt->header.seq - this->seqbias["point"]] = pt;
    if (CarSetupComHandle::CheckonMap(pt->header.frame_id, pt->header.seq - this->seqbias["point"], pt->point.x, pt->point.y))
        std::cout << "point on empty" << std::endl;
    else
        std::cout << "point on obstacle" << std::endl;
}
geometry_msgs::PointStamped::ConstPtr CarSetupComHandle::GetPoint(std::string id, int seq) {
    if (point.count(id) < 1) {
        std::cout << "Point:No id found\n";
        return NULL;
    }
    else if (point[id].count(seq) < 1) {
        std::cout << "Point:No sequence found\n";
        return NULL;
    }
    else
        return point[id][seq];
}

void CarSetupComHandle::SetPath(const std::string topic) {
    std::cout << "Publishing Path as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->path_pubs = nh.advertise<nav_msgs::Path>(topic, 1000);
}
void CarSetupComHandle::PublishPath(std::string id, int seq, og::PathGeometric& path) {
    nav_msgs::Path navpath;
    std::vector<ob::State*> pathvec = path.getStates();
    navpath.header.stamp = ros::Time::now();
    navpath.header.seq = seq;
    navpath.header.frame_id = id;
    navpath.poses.resize(pathvec.size());
    
    std::vector<ob::State*>::iterator it;
    int n;
    for(it = pathvec.begin(), n =0; it != pathvec.end(); ++it, ++n) {
        const auto *s = (*it)->as<ob::SE2StateSpace::StateType>();
        navpath.poses[n].header.seq = seq;
        navpath.poses[n].header.frame_id = id;
        navpath.poses[n].pose.position.x = s->getX();
        navpath.poses[n].pose.position.y = s->getY();
        tf2::Quaternion q;
        q.setEuler(s->getYaw(), 0, 0);
        navpath.poses[n].pose.orientation.x = q[0];
        navpath.poses[n].pose.orientation.y = q[1];
        navpath.poses[n].pose.orientation.z = q[2];
        navpath.poses[n].pose.orientation.w = q[3];
    }
    this->path_pubs.publish(navpath);
}

void CarSetupComHandle::SimpleSetup() {
    this -> SetStart();
    this -> SetGoal();
    this -> SetBase();
    this -> SetMap();
    this -> SetPoint();
    this -> SetPath();
}

bool CarSetupComHandle::CheckonMap (const std::string id, int seq, float x, float y) {
    if (map.count(id) < 1) {
        std::cout << "Map:No id found\n";
        return false;
    }
    else if (map[id].count(seq) < 1) {
        std::cout << "Map:No sequence found\n";
        return false;
    }
    else {
        nav_msgs::OccupancyGrid::ConstPtr mp = map[id][seq];
        float res = mp -> info.resolution;
        float px = x - mp -> info.origin.position.x;
        float py = y - mp -> info.origin.position.y;
        tf2::Quaternion q(mp -> info.origin.orientation.x, mp -> info.origin.orientation.y, mp -> info.origin.orientation.z, mp -> info.origin.orientation.w);
        float yaw = q.getAngle();
        float fx = cos(yaw) * px + sin(yaw) * py;
        float fy = -sin(yaw) * px + cos(yaw) * py;
        int j = (int)fx/res;
        int i = (int)fy/res;
        int w = mp -> info.width;
        if (0 <= i && i < mp->info.height && 0 <= j && j < mp->info.width ) {
            if (mp->data[w*i+j] == 0)
                return true;
            else
                return false;
        }
        else
            return false;
    }
}
bool CarSetupComHandle::isStateValid (std::string id, int seq, const ob::SpaceInformation *si, const ob::State *state) {
    const auto *s = state->as<ob::SE2StateSpace::StateType>();
    double x = s->getX();
    double y = s->getY();
    return si->satisfiesBounds(s) && CarSetupComHandle::CheckonMap(id, seq, x, y);
}

int CarSetupComHandle::GetLatestSeq() {
    int l = std::min(CarSetupComHandle::latest_seq_start, CarSetupComHandle::latest_seq_goal );
    l = std::min(CarSetupComHandle::latest_seq_map, l);
    l = std::min(CarSetupComHandle::latest_seq_base, l);
    return l;
}
CarSetupComHandle::~CarSetupComHandle() {
    std::cout << "destructing CarSetupComHandle of \"" << this->nodename << "\" node\n";
}