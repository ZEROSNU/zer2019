// carsetupcomhandle.cpp

#include "carsetupcomhandle.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <string>
#include <iostream>
#include <map>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathGeometric.h>

std::string CarSetupComHandle::nodename;
std::map<std::string,std::map<int, geometry_msgs::PoseWithCovarianceStamped::ConstPtr>> CarSetupComHandle::start;
std::map<std::string,std::map<int, geometry_msgs::PoseStamped::ConstPtr>> CarSetupComHandle::goal;
std::map<std::string,std::map<int, nav_msgs::OccupancyGrid::ConstPtr>> CarSetupComHandle::map;
std::map<std::string,std::map<std::string, std::map<int, geometry_msgs::TransformStamped::ConstPtr>>> CarSetupComHandle::base; //from / to
std::map<std::string,std::map<int, geometry_msgs::PointStamped::ConstPtr>> CarSetupComHandle::point;
std::map<std::string, int> CarSetupComHandle::latest_seq;
std::map<std::string, bool> CarSetupComHandle::is_updated;



CarSetupComHandle::CarSetupComHandle(int argc, char **argv, std::string nodename) {
    CarSetupComHandle::nodename = nodename;
    //std::cout << "CarSetupComHandle with node name \"" << nodename << "\" created\n";
    this -> isinitialized["start"] = false;
    this -> isinitialized["goal"] = false;
    this -> isinitialized["map"] = false;
    this -> isinitialized["point"] = false;

}
void CarSetupComHandle::SetStart(const std::string topic) {
    //std::cout << "Subscribing StartPoint as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->start_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::StartCB, this);
    if(latest_seq.count("start") < 1) {
        latest_seq["start"] = -1;
    }
}
void CarSetupComHandle::StartCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init) {
    if (!this->isinitialized["start"]) {
        this->seqbias["start"] = init->header.seq;
        this->isinitialized["start"] = true;
        //std::cout << "start_initialized" << std::endl;
    }
    std::cout << "start_seq number : " << init-> header.seq - this->seqbias["start"] << ", map id : " << init->header.frame_id << std::endl;
    start[init->header.frame_id][init->header.seq - this->seqbias["start"]] = init;
    CarSetupComHandle::latest_seq["start"] = init->header.seq - this->seqbias["start"];
    CarSetupComHandle::is_updated["start"] = true;
}
ob::ScopedStatePtr CarSetupComHandle::GetStart(const ob::StateSpacePtr& space, std::string id, int seq) {
    ob::ScopedState<> st(space);
    if (start.count(id) < 1) {
        std::cout << "Start:No id found : " << id << std::endl;
        st[0] = 0;
        st[1] = 0;
        st[2] = M_PI/2;
    }
    else if (start[id].count(seq) < 1) {
        st[0] = 0;
        st[1] = 0;
        st[2] = M_PI/2;
        std::cout << "Start:No sequence found : " << seq << std::endl;
    }
    else {

        tf2::Quaternion q(start[id][seq] -> pose.pose.orientation.x, start[id][seq] -> pose.pose.orientation.y, start[id][seq] -> pose.pose.orientation.z, start[id][seq] -> pose.pose.orientation.w);
        st[0] = start[id][seq] -> pose.pose.position.x;
        st[1] = start[id][seq] -> pose.pose.position.y;
        st[2] = q.getAngle();
    }
    return std::make_shared<ob::ScopedState<>> (st);
}


void CarSetupComHandle::SetGoal(const std::string topic) {
    //std::cout << "Subscribing GoalPoint as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->goal_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::GoalCB, this);
    if(latest_seq.count("goal") < 1) {
        latest_seq["goal"] = -1;
    }
}
void CarSetupComHandle::GoalCB(const geometry_msgs::PoseStamped::ConstPtr& fin) {
    if (!this->isinitialized["goal"]) {
        this->seqbias["goal"] = fin->header.seq;
        this->isinitialized["goal"] = true;
        std::cout << "goal_initialized" << std::endl;
    }
    std::cout << "goal_seq number : " << fin->header.seq - this->seqbias["goal"]<< ", goal id : " << fin->header.frame_id << std::endl;
    goal[fin->header.frame_id][fin->header.seq - this->seqbias["goal"]] = fin;
    CarSetupComHandle::latest_seq["goal"] = fin->header.seq - this->seqbias["goal"];
    CarSetupComHandle::is_updated["goal"] = true;

}
ob::ScopedStatePtr CarSetupComHandle::GetGoal(const ob::StateSpacePtr &space, std::string id, int seq) {
    ob::ScopedState<> st(space);
    if (goal.count(id) < 1) {
        std::cout << "Goal:No id found : " << id << std::endl;
        st[0] = 0;
        st[1] = 0;
        st[2] = 0;
    }
    else if (goal[id].count(seq) < 1) {
        std::cout << "Goal:No sequence found : " << seq << std::endl;
        st[0] = 0;
        st[1] = 0;
        st[2] = 0;
    }
    else {

        tf2::Quaternion q(goal[id][seq] -> pose.orientation.x, goal[id][seq] -> pose.orientation.y, goal[id][seq] -> pose.orientation.z, goal[id][seq] -> pose.orientation.w);
        st[0] = goal[id][seq] -> pose.position.x;
        st[1] = goal[id][seq] -> pose.position.y;
        st[2] = q.getAngle();

    }
    return std::make_shared<ob::ScopedState<>> (st);
}

void CarSetupComHandle::SetBase(const std::string topic) {
    //std::cout << "Subscribing BaseFrame as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->base_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::BaseCB, this);
    if(latest_seq.count("base") < 1) {
        latest_seq["base"] = -1;
    }
}
void CarSetupComHandle::BaseCB(const tf2_msgs::TFMessage::ConstPtr& bs) {
    int size = bs->transforms.size();
    for(int i = 0 ; i < size ; i++) {
        std::string name = "basefrom_" + bs->transforms[i].header.frame_id + "_to_" + bs->transforms[i].child_frame_id;
        if(isinitialized.count(name) < 1) {
            this->seqbias[name] = bs->transforms[i].header.seq;
            this->isinitialized[name] = true;
            std::cout << name << "_initialized" << std::endl;
        }
        std::cout << name << "_seq number : " << bs->transforms[i].header.seq - this->seqbias[name] << ", base id : " << bs->transforms[i].child_frame_id << ", parent_id : " << bs->transforms[i].header.frame_id << std::endl;
        base[bs->transforms[i].header.frame_id][bs->transforms[i].child_frame_id][bs->transforms[i].header.seq - this->seqbias[name]] = boost::make_shared<geometry_msgs::TransformStamped>(bs->transforms[i]);
        CarSetupComHandle::latest_seq[name] = bs->transforms[i].header.seq - this->seqbias[name];
        CarSetupComHandle::is_updated[name] = true;

    }
}
geometry_msgs::TransformStamped::ConstPtr CarSetupComHandle::GetBase(std::string from_id, std::string to_id, int seq) {
    if (base.count(from_id) < 1) {
        std::cout << "TF:No parent id found : " << from_id << std::endl;
        return nullptr;
    }
    else if(base[from_id].count(to_id) < 1) {
        std::cout << "TF:No child id found : " << to_id << std::endl;
        return nullptr;
    }
    else if (base[from_id][to_id].count(seq) < 1) {
        std::cout << "TF:No sequence found : " << seq << std::endl;
        return nullptr;
    }
    else
        return base[from_id][to_id][seq];
}

void CarSetupComHandle::SetMap(const std::string topic) {
    //std::cout << "Subscribing Map as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->map_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::MapCB, this);
    if(latest_seq.count("map") < 1) {
        latest_seq["map"] = -1;
    }
}
void CarSetupComHandle::MapCB(const nav_msgs::OccupancyGrid::ConstPtr& mp) {
    if (!this->isinitialized["map"]) {
        this->seqbias["map"] = mp->header.seq;
        this->isinitialized["map"] = true;
        std::cout << "map_initialized" << std::endl;
    }
    std::cout << "map_seq number : " << mp->header.seq - this->seqbias["map"] << ", map id : " << mp->header.frame_id << std::endl;
    map[mp->header.frame_id][mp->header.seq - this->seqbias["map"]] = mp;
    CarSetupComHandle::latest_seq["map"] = mp->header.seq - this->seqbias["map"];
    CarSetupComHandle::is_updated["map"] = true;

}
nav_msgs::OccupancyGrid::ConstPtr CarSetupComHandle::GetMap(std::string id, int seq) {
    if (map.count(id) < 1) {
        std::cout << "Map:No id found : " << id << std::endl;
        return nullptr;
    }
    else if (map[id].count(seq) < 1) {
        std::cout << "Map:No sequence found : " << seq << std::endl;
        return nullptr;
    }
    else
        return map[id][seq];
}

void CarSetupComHandle::SetPoint(const std::string topic) {
    //std::cout << "Subscribing Point as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->point_subs = nh.subscribe(topic, 1000, &CarSetupComHandle::PointCB, this);
    if(latest_seq.count("point") < 1) {
        latest_seq["point"] = -1;
    }
}
void CarSetupComHandle::PointCB(const geometry_msgs::PointStamped::ConstPtr& pt) {
    if (!this->isinitialized["point"]) {
        this->seqbias["point"] = pt->header.seq;
        this->isinitialized["point"] = true;
        std::cout << "point_initialized" << std::endl;
    }
    std::cout << "point_seq number : " << pt->header.seq - this->seqbias["point"] << ", point id : " << pt->header.frame_id << std::endl;
    point[pt->header.frame_id][pt->header.seq - this->seqbias["point"]] = pt;
    CarSetupComHandle::latest_seq["point"] = pt->header.seq - this->seqbias["point"];
    CarSetupComHandle::is_updated["point"] = true;
    std::cout << "(" << pt->point.x << ", " << pt->point.y << ") : ";
    if (CarSetupComHandle::CheckonMap(pt->header.frame_id, pt->header.seq - this->seqbias["point"], pt->point.x, pt->point.y))
        std::cout << "point on empty" << std::endl;
    else
        std::cout << "point on obstacle" << std::endl;
}
geometry_msgs::PointStamped::ConstPtr CarSetupComHandle::GetPoint(std::string id, int seq) {
    if (point.count(id) < 1) {
        std::cout << "Point:No id found : " << id << std::endl;
        return nullptr;
    }
    else if (point[id].count(seq) < 1) {
        std::cout << "Point:No sequence found : " << seq << std::endl;
        return nullptr;
    }
    else
        return point[id][seq];
}

void CarSetupComHandle::SetPath(const std::string topic) {
    //std::cout << "Publishing Path as topic name \"" << topic << "\"\n";
    ros::NodeHandle nh;
    this->path_pubs = nh.advertise<nav_msgs::Path>(topic, 1000);
}
void CarSetupComHandle::PublishPath(const std::string id, int seq, og::PathGeometric& path, bool withgear) {
    nav_msgs::Path navpath;
    std::vector<ob::State*> pathvec = path.getStates();
    navpath.header.stamp = ros::Time::now();
    navpath.header.seq = seq;
    navpath.header.frame_id = id;
    navpath.poses.resize(pathvec.size());
    
    std::vector<ob::State*>::iterator it;
    int n;
    double pastyaw;
    for(it = pathvec.begin(), n = 0; n < pathvec.size(); ++it, ++n) {
        const auto *s = (*it)->as<ob::SE2StateSpace::StateType>();
        navpath.poses[n].header.seq = seq;
        navpath.poses[n].header.frame_id = id;
        navpath.poses[n].header.stamp = navpath.header.stamp;
        navpath.poses[n].pose.position.x = s->getX();
        navpath.poses[n].pose.position.y = s->getY();
        if(withgear && n>0) {
            double deltax = s->getX() - navpath.poses[n-1].pose.position.x;
            double deltay = s->getY() - navpath.poses[n-1].pose.position.y;
            if(deltax * cos(pastyaw) + deltay * sin(pastyaw) > 0) {
                navpath.poses[n-1].pose.position.z = 0.1;
            }
            else {
                navpath.poses[n-1].pose.position.z = -0.1;
            }
        }
        tf2::Quaternion q;
        pastyaw = s->getYaw();
        q.setEuler( 0, 0, pastyaw);
        navpath.poses[n].pose.orientation.x = q[0];
        navpath.poses[n].pose.orientation.y = q[1];
        navpath.poses[n].pose.orientation.z = q[2];
        navpath.poses[n].pose.orientation.w = q[3];
    }
    this->path_pubs.publish(navpath);
    std::cout << "Path published : " << seq << std::endl;
}
void CarSetupComHandle::PublishPath(const std::string id, int seq, oc::PathControl& path, bool withgear) {
    nav_msgs::Path navpath;
    std::vector<ob::State*> pathvec = path.getStates();
    navpath.header.stamp = ros::Time::now();
    navpath.header.seq = seq;
    navpath.header.frame_id = id;
    navpath.poses.resize(pathvec.size());
    
    std::vector<ob::State*>::iterator it;
    int n;
    double pastyaw;
    for(it = pathvec.begin(), n = 0; n < pathvec.size(); ++it, ++n) {
        const auto *s = (*it)->as<ob::SE2StateSpace::StateType>();
        navpath.poses[n].header.seq = seq;
        navpath.poses[n].header.frame_id = id;
        navpath.poses[n].header.stamp = navpath.header.stamp;
        navpath.poses[n].pose.position.x = s->getX();
        navpath.poses[n].pose.position.y = s->getY();
        if(withgear && n>0) {
            double deltax = s->getX() - navpath.poses[n-1].pose.position.x;
            double deltay = s->getY() - navpath.poses[n-1].pose.position.y;
            if(deltax * cos(pastyaw) + deltay * sin(pastyaw) > 0) {
                navpath.poses[n-1].pose.position.z = 0.1;
            }
            else {
                navpath.poses[n-1].pose.position.z = -0.1;
            }
        }
        tf2::Quaternion q;
        pastyaw = s->getYaw();
        q.setEuler(0, 0, pastyaw);
        navpath.poses[n].pose.orientation.x = q[0];
        navpath.poses[n].pose.orientation.y = q[1];
        navpath.poses[n].pose.orientation.z = q[2];
        navpath.poses[n].pose.orientation.w = q[3];
    }
    this->path_pubs.publish(navpath);
    std::cout << "Path published : " << seq << std::endl;
}
bool CarSetupComHandle::TransformState(const std::string from_id, const std::string to_id, int tfseq, int stateon, ob::ScopedStatePtr state) {
    if (base.count(from_id) < 1) {
        std::cout << "TF:No parent id found : " << from_id << std::endl;
        return false;
    }
    else if(base[from_id].count(to_id) < 1) {
        std::cout << "TF:No child id found : " << to_id << std::endl;
        return false;
    }
    else if (base[from_id][to_id].count(tfseq) < 1) {
        std::cout << "TF:No sequence found : " << tfseq << std::endl;
        return false;
    }
    geometry_msgs::TransformStampedConstPtr tf = GetBase(from_id, to_id, tfseq);

    if(stateon==1) {
        double x = (*state)[0];
        double y = (*state)[1];
        double syaw = (*state)[2];
        
        x -= tf->transform.translation.x;
        y -= tf->transform.translation.y;

        tf2::Quaternion q;
        q[0] = tf->transform.rotation.x;
        q[1] = tf->transform.rotation.y;
        q[2] = tf->transform.rotation.z;
        q[3] = tf->transform.rotation.w;
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        (*state)[2] = (syaw - yaw);
        double fx = cos(yaw) * x + sin(yaw) * y;
        double fy = -sin(yaw) * x + cos(yaw) * y;
        (*state)[0] = fx;
        (*state)[1] = fy;
        return true;
    }
    else if (stateon == 2) {
        tf2::Quaternion q;
        q[0] = tf->transform.rotation.x;
        q[1] = tf->transform.rotation.y;
        q[2] = tf->transform.rotation.z;
        q[3] = tf->transform.rotation.w;
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double x = (*state)[0];
        double y = (*state)[1];
        double syaw = (*state)[2];
        double fx = cos(yaw) * x - sin(yaw) * y;
        double fy = sin(yaw) * x + cos(yaw) * y;

        fx += tf->transform.translation.x;
        fy += tf->transform.translation.y;

        (*state)[2] = syaw + yaw;
        (*state)[0] = fx;
        (*state)[1] = fy;
        return true;

    }
}


bool CarSetupComHandle::TransformPath(const std::string from_id, const std::string to_id, int tfseq, int pathon, og::PathGeometric& path) {
    if (base.count(from_id) < 1) {
        std::cout << "TF:No parent id found : " << from_id << std::endl;
        return false;
    }
    else if(base[from_id].count(to_id) < 1) {
        std::cout << "TF:No child id found : " << to_id << std::endl;
        return false;
    }
    else if (base[from_id][to_id].count(tfseq) < 1) {
        std::cout << "TF:No sequence found : " << tfseq << std::endl;
        return false;
    }
    geometry_msgs::TransformStampedConstPtr tf = GetBase(from_id, to_id, tfseq);
    std::vector<ob::State*> pathvec = path.getStates();
    
    if(pathon == 1) {
        for(std::vector<ob::State*>::iterator it = pathvec.begin() ; it != pathvec.end() ; ++it) {
            auto *s = (*it)->as<ob::SE2StateSpace::StateType>();
            double x = s->getX();
            double y = s->getY();
            double syaw = s->getYaw();
            x -= tf->transform.translation.x;
            y -= tf->transform.translation.y;

            tf2::Quaternion q;
            q[0] = tf->transform.rotation.x;
            q[1] = tf->transform.rotation.y;
            q[2] = tf->transform.rotation.z;
            q[3] = tf->transform.rotation.w;
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            s->setYaw(syaw - yaw);
            double fx = cos(yaw) * x + sin(yaw) * y;
            double fy = -sin(yaw) * x + cos(yaw) * y;
            s->setXY(fx, fy);
        }
        return true;
    }
    else if(pathon == 2) {
        for(std::vector<ob::State*>::iterator it = pathvec.begin() ; it != pathvec.end() ; ++it) {
            auto *s = (*it)->as<ob::SE2StateSpace::StateType>();
            tf2::Quaternion q;
            q[0] = tf->transform.rotation.x;
            q[1] = tf->transform.rotation.y;
            q[2] = tf->transform.rotation.z;
            q[3] = tf->transform.rotation.w;
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            double x = s->getX();
            double y = s->getY();
            double syaw = s->getYaw();
            double fx = cos(yaw) * x - sin(yaw) * y;
            double fy = sin(yaw) * x + cos(yaw) * y;

            fx += tf->transform.translation.x;
            fy += tf->transform.translation.y;

            s->setYaw(syaw + yaw);
            s->setXY(fx, fy);
        }
        return true;
    }
    else
        return false;
}
bool CarSetupComHandle::TransformPath(const std::string from_id, const std::string to_id, int tfseq, int pathon, oc::PathControl& path) {
    if (base.count(from_id) < 1) {
        std::cout << "TF:No parent id found : " << from_id << std::endl;
        return false;
    }
    else if(base[from_id].count(to_id) < 1) {
        std::cout << "TF:No child id found : " << to_id << std::endl;
        return false;
    }
    else if (base[from_id][to_id].count(tfseq) < 1) {
        std::cout << "TF:No sequence found : " << tfseq << std::endl;
        return false;
    }
    geometry_msgs::TransformStampedConstPtr tf = GetBase(from_id, to_id, tfseq);
    std::vector<ob::State*> pathvec = path.getStates();
    
    if(pathon == 1) {
        for(std::vector<ob::State*>::iterator it = pathvec.begin() ; it != pathvec.end() ; ++it) {
            auto *s = (*it)->as<ob::SE2StateSpace::StateType>();
            double x = s->getX();
            double y = s->getY();
            double syaw = s->getYaw();
            x -= tf->transform.translation.x;
            y -= tf->transform.translation.y;

            tf2::Quaternion q;
            q[0] = tf->transform.rotation.x;
            q[1] = tf->transform.rotation.y;
            q[2] = tf->transform.rotation.z;
            q[3] = tf->transform.rotation.w;
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            s->setYaw(syaw - yaw);
            double fx = cos(yaw) * x + sin(yaw) * y;
            double fy = -sin(yaw) * x + cos(yaw) * y;
            s->setXY(fx, fy);
        }
        return true;
    }
    else if(pathon == 2) {
        for(std::vector<ob::State*>::iterator it = pathvec.begin() ; it != pathvec.end() ; ++it) {
            auto *s = (*it)->as<ob::SE2StateSpace::StateType>();
            tf2::Quaternion q;
            q[0] = tf->transform.rotation.x;
            q[1] = tf->transform.rotation.y;
            q[2] = tf->transform.rotation.z;
            q[3] = tf->transform.rotation.w;
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            double x = s->getX();
            double y = s->getY();
            double syaw = s->getYaw();
            double fx = cos(yaw) * x - sin(yaw) * y;
            double fy = sin(yaw) * x + cos(yaw) * y;
            fx += tf->transform.translation.x;
            fy += tf->transform.translation.y;

            s->setYaw(syaw + yaw);
            s->setXY(fx, fy);
        }
        return true;
    }
    else
        return false;
}


void CarSetupComHandle::SimpleSetup() {
    this -> SetStart();
    this -> SetGoal();
    this -> SetBase();
    this -> SetMap();
    this -> SetPoint();
    this -> SetPath();
}

bool CarSetupComHandle::CheckonMap (const std::string id, int seq, double x, double y) {
    if (map.count(id) < 1) {
        std::cout << "Map:No id found : " << id << std::endl;
        return false;
    }
    else if (map[id].count(seq) < 1) {
        std::cout << "Map:No sequence found : " << seq << std::endl;
        return false;
    }
    else {
        nav_msgs::OccupancyGrid::ConstPtr mp = map[id][seq];
        double res = mp -> info.resolution;
        double px = x - mp -> info.origin.position.x;
        double py = y - mp -> info.origin.position.y;
        tf2::Quaternion q(mp -> info.origin.orientation.x, mp -> info.origin.orientation.y, mp -> info.origin.orientation.z, mp -> info.origin.orientation.w);
        double yaw = q.getAngle();
        double fx = cos(yaw) * px + sin(yaw) * py;
        double fy = -sin(yaw) * px + cos(yaw) * py;
        //if (fx < 0 || fy < 0)
            //return false;
        int j = mp->info.width/2 + (int)(std::floor(x/res));
        int i = mp->info.height/2 + (int)(std::floor(y/res));
        int w = mp -> info.width;
        if (i < mp->info.height && j < mp->info.width ) {
            if (mp->data[w*i+j] == 0){
                return true;
            }
            else {
                //std::cout << "OCCUPIED STATE" << std::endl;
                return false;
            }
        }
        else{
            return false;
        }
    }
}
bool CarSetupComHandle::isStateValid (std::string id, int seq, const ob::StateSpacePtr &space, const ob::State *state) {
    const auto *s = state->as<ob::SE2StateSpace::StateType>();
    double x = s->getX();
    double y = s->getY();
    ob::RealVectorBounds bounds = space->as<ob::SE2StateSpace>()->getBounds();
    if(x<bounds.low[0] || x>bounds.high[0]){
        //std::cout <<" OUT OF BOUNDS (X)" << std::endl;
        return false;
    }
    if(y<bounds.low[1] || y>bounds.high[1]){
        //std::cout <<" OUT OF BOUNDS (Y)" << std::endl;
        return false;
    }
    return CarSetupComHandle::CheckonMap(id, seq, x, y);
}

bool CarSetupComHandle::isUpdatedMap() {
    bool up = CarSetupComHandle::is_updated["map"];
    CarSetupComHandle::is_updated["map"] = false;
    return up;
}
bool CarSetupComHandle::isUpdatedBase(const std::string from_id, const std::string to_id) {
    std::string name = "basefrom_" + from_id + "_to_" + to_id;
    bool up = CarSetupComHandle::is_updated[name];
    CarSetupComHandle::is_updated[name] = false;
    return up;
}
bool CarSetupComHandle::isUpdatedStart() {
    bool up = CarSetupComHandle::is_updated["start"];
    CarSetupComHandle::is_updated["start"] = false;
    return up;
}
bool CarSetupComHandle::isUpdatedGoal() {
    bool up = CarSetupComHandle::is_updated["goal"];
    CarSetupComHandle::is_updated["goal"] = false;
    return up;
}
bool CarSetupComHandle::isUpdatedPoint() {
    bool up = CarSetupComHandle::is_updated["point"];
    CarSetupComHandle::is_updated["point"] = false;
    return up;
}

int CarSetupComHandle::GetLatestMapSeq() {
    return CarSetupComHandle::latest_seq["map"];
}
int CarSetupComHandle::GetLatestBaseSeq(const std::string from_id, const std::string to_id) {
    return CarSetupComHandle::latest_seq["basefrom_" + from_id + "_to_" + to_id];
}
int CarSetupComHandle::GetLatestStartSeq() {
    return CarSetupComHandle::latest_seq["start"];
}
int CarSetupComHandle::GetLatestGoalSeq() {
    return CarSetupComHandle::latest_seq["goal"];
}
int CarSetupComHandle::GetLatestPointSeq() {
    return CarSetupComHandle::latest_seq["point"];
}

void CarSetupComHandle::ShowAll() {
    ShowMap();
    ShowBase();
    ShowStart();
    ShowGoal();
    ShowPoint();
}
void CarSetupComHandle::ShowMap() {
    std::cout << "=========================" << std::endl;
    std::cout << "Maps : " << std::endl;
    for(auto it = map.begin() ; it!=map.end() ; it++) {
        for(auto it2 = it->second.begin() ; it2 != it->second.end() ; it2++) {
            std::cout << "at [" << it->first << "][" << it2->first << "]" << std::endl;
            std::cout << "  frame id : " << it2->second->header.frame_id << std::endl;
            std::cout << "  seq : " << it2->second->header.seq << std::endl;
            std::cout << "  time : " << it2->second->header.stamp.toSec() << std::endl;
            std::cout << "  load time : " << it2->second->info.map_load_time.toSec() << std::endl;
            std::cout << "  resolution : " << it2->second->info.resolution << std::endl;
            std::cout << "  width : " << it2->second->info.width << std::endl;
            std::cout << "  height : " << it2->second->info.height << std::endl;
            std::cout << "  origin : (" << it2->second->info.origin.position.x << ", " << it2->second->info.origin.position.y << ", " << it2->second->info.origin.position.z << ")" << std::endl;
            std::cout << "  orientation : (" << it2->second->info.origin.orientation.x << ", "<< it2->second->info.origin.orientation.y << ", " << it2->second->info.origin.orientation.z << ", " << it2->second->info.origin.orientation.w << ")" << std::endl;
            std::cout << "-------------------------" << std::endl;
        }
    }
}
void CarSetupComHandle::ShowBase() {
    std::cout << "=========================" << std::endl;
    std::cout << "Bases : " << std::endl;
    for(auto itf = base.begin() ; itf!=base.end() ; itf++) {
        for(auto itt = itf->second.begin() ; itt!=itf->second.end() ; itt++) {
            for(auto it2 = itt->second.begin() ; it2 != itt->second.end() ; it2++) {
                std::cout << "at [" << itf->first << "][" << itt->first << "][" << it2->first << "]" << std::endl;
                std::cout << "  frame id : " << it2->second->header.frame_id << std::endl;
                std::cout << "  seq : " << it2->second->header.seq << std::endl;
                std::cout << "  time : " << it2->second->header.stamp.toSec() << std::endl;
                std::cout << "  child frame id : " << it2->second->child_frame_id << std::endl;
                std::cout << "  translation : (" << it2->second->transform.translation.x << ", " << it2->second->transform.translation.y << ", " << it2->second->transform.translation.z << ")" << std::endl;
                std::cout << "  rotation : (" << it2->second->transform.rotation.x << ", " << it2->second->transform.rotation.y << ", " << it2->second->transform.rotation.z << ", " << it2->second->transform.rotation.w << ")" << std::endl;
                std::cout << "-------------------------" << std::endl;
            }
        }
    }
}
void CarSetupComHandle::ShowStart() {
    std::cout << "=========================" << std::endl;
    std::cout << "Starts : " << std::endl;
    for(auto it = start.begin() ; it!=start.end() ; it++) {
        for(auto it2 = it->second.begin() ; it2 != it->second.end() ; it2++) {
            std::cout << "at [" << it->first << "][" << it2->first << "]" << std::endl;
            std::cout << "  frame id : " << it2->second->header.frame_id << std::endl;
            std::cout << "  seq : " << it2->second->header.seq << std::endl;
            std::cout << "  time : " << it2->second->header.stamp.toSec() << std::endl;
            std::cout << "  position : (" << it2->second->pose.pose.position.x << ", " << it2->second->pose.pose.position.y << ", " << it2->second->pose.pose.position.z << ")" << std::endl ;
            std::cout << "  orientaton : (" << it2->second->pose.pose.orientation.x << ", " << it2->second->pose.pose.orientation.y << ", " << it2->second->pose.pose.orientation.z  << ", " << it2->second->pose.pose.orientation.w << ")" << std::endl;
            std::cout << "-------------------------" << std::endl;
        }
    }
}
void CarSetupComHandle::ShowGoal() {
    std::cout << "=========================" << std::endl;
    std::cout << "Goals : " << std::endl;
    for(auto it = goal.begin() ; it!=goal.end() ; it++) {
        for(auto it2 = it->second.begin() ; it2 != it->second.end() ; it2++) {
            std::cout << "at [" << it->first << "][" << it2->first << "]" << std::endl;
            std::cout << "  frame id : " << it2->second->header.frame_id << std::endl;
            std::cout << "  seq : " << it2->second->header.seq << std::endl;
            std::cout << "  time : " << it2->second->header.stamp.toSec() << std::endl;
            std::cout << "  position : (" << it2->second->pose.position.x << ", " << it2->second->pose.position.y << ", " << it2->second->pose.position.z << ")" << std::endl ;
            std::cout << "  orientaton : (" << it2->second->pose.orientation.x << ", " << it2->second->pose.orientation.y << ", " << it2->second->pose.orientation.z  << ", " << it2->second->pose.orientation.w << ")" << std::endl;
            std::cout << "-------------------------" << std::endl;
        }
    }
}
void CarSetupComHandle::ShowPoint() {
    std::cout << "=========================" << std::endl;
    std::cout << "Points : " << std::endl;
    for(auto it = point.begin() ; it!=point.end() ; it++) {
        for(auto it2 = it->second.begin() ; it2 != it->second.end() ; it2++) {
            std::cout << "at [" << it->first << "][" << it2->first << "]" << std::endl;
            std::cout << "  frame id : " << it2->second->header.frame_id << std::endl;
            std::cout << "  seq : " << it2->second->header.seq << std::endl;
            std::cout << "  time : " << it2->second->header.stamp.toSec() << std::endl;
            std::cout << "  point : (" << it2->second->point.x << ", " << it2->second->point.y << ", " << it2->second->point.z << ")" << std::endl;
            std::cout << "-------------------------" << std::endl;
        }
    }
}
CarSetupComHandle::~CarSetupComHandle() {
    std::cout << "destructing CarSetupComHandle of \"" << this->nodename << "\" node\n";
}
