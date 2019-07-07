#include "potentialmodel.h"
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <limits>

PotentialModel::PotentialModel() {
    AttractiveK_=0.01;
    RepulsiveK_=1.0;
    InterestR_=2.0;
}

void PotentialModel::repulsiveForce(const oc::SpaceInformation *si, double res, const ob::State *target, Eigen::Vector3d &pvector) {
    if(!si->isValid(target)) {
        pvector(0)=0;
        pvector(1)=0;
        pvector(2)=0;
        return;
    }
    auto const *t = target->as<ob::SE2StateSpace::StateType>();
    auto *s = si->allocState()->as<ob::SE2StateSpace::StateType>();
    double fx = 0;
    double fy = 0;
    for ( double ix = t->getX() - ((int)(InterestR_ / res) * res) ; ix <= t -> getX() + InterestR_ ; ix+=res ) {
        for ( double iy = t->getY() - ((int)(InterestR_ / res) * res) ; iy <= t->getY() + InterestR_ ; iy+=res ) {
            s->setX(ix);
            s->setY(iy);
            double d = sqrt( (ix-t->getX()) * (ix-t->getX()) + (iy-t->getY()) * (iy-t->getY()));
            if(d > InterestR_ || si->isValid(s)) continue;
            fx += RepulsiveK_ * (1.0f/d - 1.0f/InterestR_) * (1.0f/d/d/d) * (t->getX() - ix) * res * res;
            fy += RepulsiveK_ * (1.0f/d - 1.0f/InterestR_) * (1.0f/d/d/d) * (t->getY() - iy) * res * res;
        }
    }
    pvector(0) = fx;
    pvector(1) = fy;
    pvector(2) = 0;
}
void PotentialModel::attractiveForce(const ob::State *goal, const ob::State *target, Eigen::Vector3d &pvector) {
    auto const *g = goal->as<ob::SE2StateSpace::StateType>();
    auto const *t = target->as<ob::SE2StateSpace::StateType>();
    double dx = g->getX() - t->getX();
    double dy = g->getY() - t->getY();
    pvector(0) = AttractiveK_ * dx;
    pvector(1) = AttractiveK_ * dy;
    pvector(2) = 0;
}
void PotentialModel::totalForce (const oc::SpaceInformation *si, double res, const ob::State *goal, const ob::State *target, Eigen::Vector3d &pvector) {
    if(!si->isValid(target)) {
        pvector(0)=0;
        pvector(1)=0;
        pvector(2)=0;
        return;
    }
    Eigen::Vector3d avector;
    Eigen::Vector3d rvector;
    PotentialModel::attractiveForce(goal, target, avector);
    PotentialModel::repulsiveForce(si, res, target, rvector);
    pvector = avector + rvector;
}
double PotentialModel::attractivePotential(const ob::State *goal, const ob::State *target) {
    auto const *t = target->as<ob::SE2StateSpace::StateType>();
    auto const *g = goal->as<ob::SE2StateSpace::StateType>();
    double p = 1.0/2 * AttractiveK_ * ( (t->getX()-g->getX()) * (t->getX()-g->getX()) + (t->getY()-g->getY()) * (t->getY()-g->getY()) );

    return p;
}
double PotentialModel::repulsivePotential(const oc::SpaceInformation *si, double res, const ob::State *target) {
    if(!si->isValid(target)) {
        return std::numeric_limits<double>::infinity();
    }
    double p = 0;
    ob::RealVectorBounds bounds = si->getStateSpace()->as<ob::SE2StateSpace>()->getBounds();
    auto const *t = target->as<ob::SE2StateSpace::StateType>();
    auto *s = si->allocState()->as<ob::SE2StateSpace::StateType>();
    for ( double ix = t->getX() - ((int)(InterestR_ / res) * res) ; ix <= t -> getX() + InterestR_ ; ix+=res ) {
        for ( double iy = t->getY() - ((int)(InterestR_ / res) * res) ; iy <= t->getY() + InterestR_ ; iy+=res ) {
            s->setX(ix);
            s->setY(iy);
            double d = sqrt( (ix-t->getX()) * (ix-t->getX()) + (iy-t->getY()) * (iy-t->getY()));
            if(d > InterestR_ || si->isValid(s)) continue;
            p += 1.0/2 * RepulsiveK_ * (1/d - 1/InterestR_) * (1/d - 1/InterestR_) * res * res;
        }
    }
    return p;
}
double PotentialModel::totalPotential(const oc::SpaceInformation *si, double res, const ob::State *goal, const ob::State *target) {
    if(!si->isValid(target)) {
        return std::numeric_limits<double>::infinity();
    }
    return (PotentialModel::repulsivePotential(si, res, target) + PotentialModel::attractivePotential(goal, target));
}

void PotentialModel::potentialForceTest(const ob::State *goal,
                 const oc::SpaceInformation *si,
                 double stepsize,
                 const std::string map_id,
                 const int seq,
                 visualization_msgs::MarkerArray& ma) {
    ob::RealVectorBounds bounds = si->getStateSpace()->as<ob::SE2StateSpace>()->getBounds();
    int xsize = (int) (bounds.getDifference()[0] / stepsize) + 1;
    int ysize = (int) (bounds.getDifference()[1] / stepsize) + 1;
    ma.markers.resize( xsize * ysize );
    ob::State *s = si->allocState();
    Eigen::Vector3d tot;
    for (int i = 0 ; i < xsize ; i++) {
        for (int j = 0 ; j < ysize ; j++) { 
            int n = i*ysize+j; 
            double x = bounds.low[0] + stepsize * i;
            double y = bounds.low[1] + stepsize * j;
            s->as<ob::SE2StateSpace::StateType>()->setXY(x, y);
            s->as<ob::SE2StateSpace::StateType>()->setYaw(0);
            PotentialModel::totalForce(si, stepsize, goal, s, tot);
            double size = sqrt(tot.dot(tot));
            /*
            if (size > 2)
                size = 0;
                //*/
            ma.markers[n].header.seq = seq;
            ma.markers[n].header.stamp = ros::Time::now();
            ma.markers[n].header.frame_id = map_id;
            ma.markers[n].ns = "PotentialForceField";
            ma.markers[n].id = n;
            ma.markers[n].type = visualization_msgs::Marker::ARROW;
            ma.markers[n].action = visualization_msgs::Marker::ADD;
            ma.markers[n].pose.position.x = x;
            ma.markers[n].pose.position.y = y;
            ma.markers[n].pose.position.z = 0;
            tf2::Quaternion q;
            if(tot(0) == 0) {
                (tot(1) > 0) ? q.setEuler(0, 0, M_PI / 2) : q.setEuler(0, 0, -M_PI / 2);
            }
            else {
                q.setEuler(0, 0, (atan(tot(1)/tot(0)) + ((tot(0) < 0) ? M_PI : 0)));
            }
            ma.markers[n].pose.orientation.x = q[0];
            ma.markers[n].pose.orientation.y = q[1];
            ma.markers[n].pose.orientation.z = q[2];
            ma.markers[n].pose.orientation.w = q[3];
            ma.markers[n].scale.x = size/10;
            //ma.markers[n].scale.x = 0.5;
            ma.markers[n].scale.y = 0.05;
            ma.markers[n].scale.z = 0.05;
            ma.markers[n].color.a = 1;
            ma.markers[n].color.r = 0.0;
            ma.markers[n].color.g = 0.0;
            ma.markers[n].color.b = 1.0;
            ma.markers[n].mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        }
    }
}
void PotentialModel::potentialTest(const ob::State *goal,
                 const oc::SpaceInformation *si,
                 double stepsize,
                 const std::string map_id,
                 const int seq,
                 visualization_msgs::MarkerArray& ma) {
    ob::RealVectorBounds bounds = si->getStateSpace()->as<ob::SE2StateSpace>()->getBounds();
    int xsize = (int) (bounds.getDifference()[0] / stepsize) + 1;
    int ysize = (int) (bounds.getDifference()[1] / stepsize) + 1;
    ma.markers.resize( xsize * ysize );
    ob::State *s = si->allocState();
    Eigen::Vector3d tot;
    for (int i = 0 ; i < xsize ; i++) {
        for (int j = 0 ; j < ysize ; j++) { 
            int n = i*ysize+j; 
            double x = bounds.low[0] + stepsize * i;
            double y = bounds.low[1] + stepsize * j;
            s->as<ob::SE2StateSpace::StateType>()->setXY(x, y);
            s->as<ob::SE2StateSpace::StateType>()->setYaw(0);
            //double p = PotentialModel::repulsivePotential(si, stepsize, s);
            double p = PotentialModel::totalPotential(si, stepsize, goal, s);
            /*
            if (size > 2)
                size = 0;
                //*/
            ma.markers[n].header.seq = seq;
            ma.markers[n].header.stamp = ros::Time::now();
            ma.markers[n].header.frame_id = map_id;
            ma.markers[n].ns = "PotentialField";
            ma.markers[n].id = n;
            ma.markers[n].type = visualization_msgs::Marker::ARROW;
            ma.markers[n].action = visualization_msgs::Marker::ADD;
            ma.markers[n].pose.position.x = x;
            ma.markers[n].pose.position.y = y;
            ma.markers[n].pose.position.z = 0;
            tf2::Quaternion q;
            q.setEuler(-M_PI/2,0,0);
            ma.markers[n].pose.orientation.x = q[0];
            ma.markers[n].pose.orientation.y = q[1];
            ma.markers[n].pose.orientation.z = q[2];
            ma.markers[n].pose.orientation.w = q[3];
            ma.markers[n].scale.x = p;
            //ma.markers[n].scale.x = 0.5;
            ma.markers[n].scale.y = 0.05;
            ma.markers[n].scale.z = 0.05;
            ma.markers[n].color.a = 1;
            ma.markers[n].color.r = 0.0;
            ma.markers[n].color.g = 0.0;
            ma.markers[n].color.b = 1.0;
            ma.markers[n].mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        }
    }
}
PotentialModel::~PotentialModel() {

}