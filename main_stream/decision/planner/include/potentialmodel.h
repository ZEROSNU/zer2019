#ifndef POTENTIALMODEL
#define POTENTIALMODEL
#include <string>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"
#include <ompl/base/State.h>
#include <ompl/control/SpaceInformation.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
/*
namespace og = ompl::geometric;
*/
class PotentialModel {
    private:
    double AttractiveK_;
    double RepulsiveK_;
    double InterestR_;
    public:
    PotentialModel();
    void repulsiveForce(const oc::SpaceInformation *si, double res, const ob::State *target, Eigen::Vector3d &pvector);
    void attractiveForce(const ob::State *goal, const ob::State *target, Eigen::Vector3d &pvector);
    void totalForce (const oc::SpaceInformation *si, double res, const ob::State *goal, const ob::State *target, Eigen::Vector3d &pvector);
    void potentialForceTest(const ob::State *goal,
                     const oc::SpaceInformation *si,
                     double stepsize,
                     const std::string map_id,
                     const int seq,
                     visualization_msgs::MarkerArray& ma);
    void potentialTest(const ob::State *goal,
                     const oc::SpaceInformation *si,
                     double stepsize,
                     const std::string map_id,
                     const int seq,
                     visualization_msgs::MarkerArray& ma);
    double repulsivePotential(const oc::SpaceInformation *si, double res, const ob::State *target);
    double attractivePotential(const ob::State *goal, const ob::State *target);
    double totalPotential(const oc::SpaceInformation *si, double res, const ob::State *goal, const ob::State *target);
    void setAttractiveK(double k) {
        AttractiveK_ = k;
    }
    void setRepulsiveK(double r) {
        RepulsiveK_ = r;
    }
    void setInterestR(double r) {
        InterestR_ = r;
    }
    double getAttractiveK(void) {
        return AttractiveK_;
    }
    double getRepulsiveK(void) {
        return RepulsiveK_;
    }
    double getInterestR(void) {
        return InterestR_;
    }
    ~PotentialModel();
};
typedef std::shared_ptr<PotentialModel> PotentialModelPtr;

#endif