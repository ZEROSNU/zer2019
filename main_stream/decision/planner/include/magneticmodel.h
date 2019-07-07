#ifndef MAGNETICMODEL
#define MAGNETICMODEL

#include <string>
#include <iostream>
#include "visualization_msgs/MarkerArray.h"
#include <ompl/base/State.h>
#include <ompl/control/SpaceInformation.h>
#include <eigen3/Eigen/Dense>

namespace ob = ompl::base;
/*
namespace og = ompl::geometric;
*/
namespace oc = ompl::control;

class MagneticModel {
    private:
    double MagneticK_;
    double MagneticR_;
    double MagneticI_;
    public:
    MagneticModel();
    void magneticForce(const ob::State *start, const ob::State *goal, const ob::State *target, Eigen::Vector3d &bvector);
    void magneticTest(const ob::State *start,
                     const ob::State *goal,
                     const oc::SpaceInformation *si,
                     double stepsize,
                     const std::string map_id,
                     const int seq,
                     visualization_msgs::MarkerArray& pa);
    void setMagneticK(double k) {
        MagneticK_ = k;
    }
    void setMagneticR(double r) {
        MagneticR_ = r;
    }
    void setMagneticI(double r) {
        MagneticI_ = r;
    }
    double getMagneticK(void) {
        return MagneticK_;
    }
    double getMagneticR(void) {
        return MagneticR_;
    }
    double getMagneticI(void) {
        return MagneticI_;
    }
    ~MagneticModel();
};
typedef std::shared_ptr<MagneticModel> MagneticModelPtr;

#endif