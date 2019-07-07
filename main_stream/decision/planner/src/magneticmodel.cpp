#include "magneticmodel.h"
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <ompl/base/spaces/SE2StateSpace.h>

MagneticModel::MagneticModel() {
    MagneticK_ = 1;// k*I/r 2e-7
    MagneticR_ = 0.15; // the distance between a pair of two parallel line current
    MagneticI_ = 1; // current
}

void MagneticModel::magneticForce(const ob::State *start, const ob::State *goal, const ob::State *target, Eigen::Vector3d &bvector) {
    auto const *s = start->as<ob::SE2StateSpace::StateType>();
    auto const *g = goal->as<ob::SE2StateSpace::StateType>();
    auto const *t = target->as<ob::SE2StateSpace::StateType>();
    double dx = 0;
    double dy = 0;
    double fx = 0;
    double fy = 0;
    
    dx = t->getX() - s->getX() + MagneticR_/2*sin(s->getYaw());
    dy = t->getY() - s->getY() - MagneticR_/2*cos(s->getYaw());
    fx += MagneticK_ / (dx*dx+dy*dy) * MagneticI_ * (-dy);
    fy += MagneticK_ / (dx*dx+dy*dy) * MagneticI_ * dx;
    dx = t->getX() - s->getX() - MagneticR_/2*sin(s->getYaw());
    dy = t->getY() - s->getY() + MagneticR_/2*cos(s->getYaw());
    fx -= MagneticK_ / (dx*dx+dy*dy) * MagneticI_ * (-dy);
    fy -= MagneticK_ / (dx*dx+dy*dy) * MagneticI_ * dx;

    dx = t->getX() - g->getX() + MagneticR_/2*sin(g->getYaw());
    dy = t->getY() - g->getY() - MagneticR_/2*cos(g->getYaw());
    fx += MagneticK_ / (dx*dx+dy*dy) * MagneticI_ * (-dy);
    fy += MagneticK_ / (dx*dx+dy*dy) * MagneticI_ * dx;
    dx = t->getX() - g->getX() - MagneticR_/2*sin(g->getYaw());
    dy = t->getY() - g->getY() + MagneticR_/2*cos(g->getYaw());
    fx -= MagneticK_ / (dx*dx+dy*dy) * MagneticI_ * (-dy);
    fy -= MagneticK_ / (dx*dx+dy*dy) * MagneticI_ * dx;
    bvector(0) = fx;
    bvector(1) = fy;

}
void MagneticModel::magneticTest(const ob::State *start,
                                 const ob::State *goal,
                                 const oc::SpaceInformation *si,
                                 double stepsize,
                                 const std::string map_id,
                                 const int seq,
                                 visualization_msgs::MarkerArray& ma
                                 ) {
    ob::RealVectorBounds bounds = si->getStateSpace()->as<ob::SE2StateSpace>()->getBounds();
    int xsize = (int) (bounds.getDifference()[0] / stepsize) + 1;
    int ysize = (int) (bounds.getDifference()[1] / stepsize) + 1;
    ma.markers.resize( xsize * ysize );
    Eigen::Vector3d bvector;

    ob::State *s = si->allocState();
    
    for (int i = 0 ; i < xsize ; i++) {
        for (int j = 0 ; j < ysize ; j++) { 
            int n = i*ysize+j; 
            double x = bounds.low[0] + stepsize * i;
            double y = bounds.low[1] + stepsize * j;
            s->as<ob::SE2StateSpace::StateType>()->setXY(x, y);
            s->as<ob::SE2StateSpace::StateType>()->setYaw(0);
            MagneticModel::magneticForce(start, goal, s, bvector);
            double size = sqrt(bvector(0) * bvector(0) + bvector(1) * bvector(1));
            ma.markers[n].header.seq = seq;
            ma.markers[n].header.stamp = ros::Time::now();
            ma.markers[n].header.frame_id = map_id;
            ma.markers[n].ns = "MagneticForce";
            ma.markers[n].id = n;
            ma.markers[n].type = visualization_msgs::Marker::ARROW;
            ma.markers[n].action = visualization_msgs::Marker::ADD;
            ma.markers[n].pose.position.x = x;
            ma.markers[n].pose.position.y = y;
            ma.markers[n].pose.position.z = 0;
            tf2::Quaternion q;
            if(bvector(0) == 0) {
                (bvector(1) > 0) ? q.setEuler(0, 0, M_PI / 2) : q.setEuler(0, 0, -M_PI / 2);
            }
            else {
                q.setEuler(0, 0, (atan(bvector(1)/bvector(0)) + ((bvector(0) < 0) ? M_PI : 0)));
            }
            ma.markers[n].pose.orientation.x = q[0];
            ma.markers[n].pose.orientation.y = q[1];
            ma.markers[n].pose.orientation.z = q[2];
            ma.markers[n].pose.orientation.w = q[3];
            ma.markers[n].scale.x = size;
            //ma.markers[n].scale.x = 0.5;
            ma.markers[n].scale.y = 0.05;
            ma.markers[n].scale.z = 0.05;
            ma.markers[n].color.a = 1;
            ma.markers[n].color.r = 0.0;
            ma.markers[n].color.g = 1.0;
            ma.markers[n].color.b = 0.0;
            ma.markers[n].mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        }
    }
}
MagneticModel::~MagneticModel() {

}