#include "DORRT.h"
#include <ompl/base/Planner.h>
#include <ompl/control/PathControl.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace ompl {
    doRRT::doRRT(const control::SpaceInformationPtr &si, const MagneticModelPtr &mm, const PotentialModelPtr &pm) : base::Planner(si, "doRRT") {
        specs_.approximateSolutions = true;
        specs_.optimizingPaths = true;
        specs_.canReportIntermediateSolutions = false;
        siC_ = si.get();
        mm_ = mm;
        pm_ = pm;
        r_rrt_ = 6.0;
        potmax_ = 0.700;
        fieldnorm_ = 1.0;
        res_ = 0.30;
        maxsample_ = 100;
        extendlength_ = 1.5;
        approxlength_ = 0.5;
        interpolate_ = true;
        checktime_ = true;
        reedssheppradius_ = 1.0;
        sp_ = std::make_shared<base::ReedsSheppStateSpace>(reedssheppradius_);
    }
    doRRT::~doRRT(void) {
        doRRT::freeMemory();
    }
    void doRRT::setup(void) {
        Planner::setup();
        mtree_.freeMemory();
        timeflags_.clear();
    }

    base::PlannerStatus doRRT::solve(const base::PlannerTerminationCondition &ptc) {
        checkValidity();
        bool arrived = false;
        auto path(std::make_shared<control::PathControl>(si_));
        base::State* goal = pdef_->getGoal().get()->as<base::GoalState>()->getState();
        if(pdef_->getStartStateCount() == 0) {
            OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
            return base::PlannerStatus::INVALID_START;
        }
        else if (pdef_->getStartStateCount() > 1) {
            OMPL_ERROR("%s: There are too many valid initial states!", getName().c_str());
            return base::PlannerStatus::INVALID_START;
        }
        if(checktime_) addFlag("get start and goal");
        base::State* start = pdef_->getStartState(pdef_->getStartStateCount()-1);
        Motion *startmotion = new Motion(siC_);
        Motion *goalmotion = new Motion(siC_);
        goalmotion->inGoal = true;
        std::vector<Motion *> neargoal;
        si_->copyState(goalmotion->state, goal);
        si_->copyState(startmotion->state, start);
        setRoot(startmotion);
        sampler_ = si_->allocStateSampler();
        std::vector<int> nbh;
        while(ptc == false) {
            if(checktime_) addFlag("new while");
            Motion *rmotion = new Motion(siC_); // random sampled motion
            if(!DOSampling(rmotion)) {
                std::cout << "broken at sampling" << std::endl;
                continue;
            }
            if(checktime_) addFlag("after sampling");
            Motion *nrst = mtree_.members[nearest(rmotion)]; // nearest motion from random sampled
            if(checktime_) addFlag("after finding nearest");
            Motion *nmotion = new Motion(siC_); // DOextended, or steered motion

            if(!DOExtend(nrst, rmotion, nmotion)) {
                std::cout << "broken at extending" << std::endl;
                continue;
            }
            if(checktime_) addFlag("after extending");
            Motion *minmotion = nrst; // xmin, or parent motion
            double cmin = combineCosts(nrst->cost, distanceMotion(minmotion, nmotion));

            getNeighbors(nmotion, nbh);
            if(checktime_) addFlag("after finding neighbors");
            ///*
            for(int i = 0 ; i<nbh.size() ; i++) {
                Motion* near = mtree_.members[nbh[i]];
                if((cmin > near->cost + distanceMotion(near, nmotion)) && collisionFree(near, nmotion)) {
                    minmotion = near;
                    cmin = near->cost + distanceMotion(near, nmotion);
                }
            }
            //*/
            addMotion(minmotion, nmotion);
            ///*
            for(int i = 0 ; i<nbh.size() ; i++) {
                Motion* near = mtree_.members[nbh[i]];
                if((near->cost > nmotion->cost + distanceMotion(near, nmotion)) && collisionFree(nmotion, near)) {
                    changeParent(nmotion, near);
                }
            }
            if(checktime_) addFlag("after rewiring");
            //*/
            if(distanceMotion(nmotion, goalmotion) < approxlength_ && collisionFree(nmotion, goalmotion)) {
                neargoal.push_back(nmotion);
                nmotion->inGoal = true;
                arrived = true;
            }
            delete rmotion;
        }
        if(checktime_) addFlag("after escaping the rrt");
        if(!arrived) {
            neargoal.push_back(mtree_.members[nearest(goalmotion)]);
        }
        int mincostind = 0;
        double mincost = neargoal[0]->cost;
        if(arrived) {
            std::cout << neargoal.size() << " paths are found" << std::endl;
        }
        else
            std::cout << "no path found" << std::endl;
        for(int i = 0 ; i < neargoal.size() ; i++) {
            if(neargoal[i]->cost < mincost) {
                mincostind = i;
                mincost = neargoal[i]->cost;
            }
        }
        base::State* temp(si_->allocState());
        for(Motion* it = neargoal[mincostind] ; (interpolate_ ? it->parent!=nullptr : it!=nullptr) ; it = it->parent) {
            path->append(it->state);
            if(interpolate_) {
                double d = distanceMotion(it, it->parent);
                double deltat = res_/d/2;
                for(int i = 1; i * deltat < 1 ; i++) {
                    sp_->interpolate(it->state, it->parent->state, i*deltat, temp);
                    path->append(temp);
                }
            }
        }
        path->as<geometric::PathGeometric>()->reverse();
        if(interpolate_ && arrived) {
            double d = distanceMotion(neargoal[mincostind], goalmotion);
            double deltat = res_/d/2;
            for(int i = 1; i * deltat < 1 ; i++) {
                sp_->interpolate(neargoal[mincostind]->state, goalmotion->state, i*deltat, temp);
                path->append(temp);
            }
        }
        si_->freeState(temp);
        delete goalmotion;
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());
        pdef_->addSolutionPath(psol);
        if(checktime_) addFlag("after making a path");
        return (arrived ? base::PlannerStatus(true, false) : base::PlannerStatus(true, true));
    }
    bool doRRT::DOSampling(Motion *node) {
        base::State* goal = pdef_->getGoal().get()->as<base::GoalState>()->getState();
        base::State* start = pdef_->getStartState(0);
        int attempt = 0;
        do {
            sampler_->sampleUniform(node->state);
            node->potential = pm_->repulsivePotential(siC_, res_, node->state);
            attempt++;
            if(attempt > maxsample_) {
                std::cout << node->potential;
                return false;
            }
        }
        while(node->potential > potmax_);
        if(!si_->isValid(node->state)) return false;
        auto *state = node->state->as<base::SE2StateSpace::StateType>();
        pm_->totalForce(siC_, res_, goal, node->state, node->pvector);
        mm_->magneticForce(start, goal, node->state, node->fvector);
        Eigen::Vector3d rvec(cos(state->getYaw()), sin(state->getYaw()), 0);
        Eigen::Vector3d zaxis(0,0,1);
        Eigen::Vector3d ptan = zaxis.cross(node->pvector)/sqrt(node->pvector.dot(node->pvector));
        Eigen::Vector3d fvec = node->fvector/sqrt(node->fvector.dot(node->fvector));
        ptan = (ptan.dot(rvec) > 0) ? ptan : -ptan;
        rvec = rvec * (1-node->potential/potmax_) * 0 + node->potential/potmax_ * ptan;
        rvec = rvec * (1-fieldNormMag(node->fvector)) + fieldNormMag(node->fvector)*fvec;
        double yaw;
        if (rvec(0) == 0) yaw = ((rvec(1)>0) ? M_PI/2 : -M_PI/2);
        else yaw = atan(rvec(1)/rvec(0)) + ((rvec(0) < 0) ? M_PI : 0);
        state->setYaw(yaw);
        return true;
    }
    double doRRT::fieldNormMag(Eigen::Vector3d field) {
        return atan(sqrt(field.dot(field)) * M_PI/2 * fieldnorm_) / M_PI * 2;
    }
    bool doRRT::DOExtend(Motion *nearest, Motion *rand, Motion *node) {
        base::State* goal = pdef_->getGoal().get()->as<base::GoalState>()->getState();
        double d1 = sp_->distance(nearest->state, rand->state);
        double deltat1 = res_/d1/2;
        base::State* temp(si_->allocState());
        bool isvalid = true;
        for (int i = 0 ; (deltat1 * i < 1) && (i * res_ < extendlength_) ; i++) {
            sp_->interpolate(nearest->state, rand->state, deltat1 * i, temp);
            if(!si_->isValid(temp)) {
                break;
            }
            si_->copyState(node->state, temp);
        }
        pm_->totalForce(siC_, res_, goal, node->state, node->pvector);
        node->potential = pm_->repulsivePotential(siC_, res_, node->state);

        for(int i = 0 ; i < maxsample_ ; i++ ) {
            isvalid = true;
            if(isvalid) {
                if(node->potential > potmax_) {
                    isvalid = false;
                }
            }
            if(isvalid) {
                double d2 = sp_->distance(nearest->state, node->state);
                double delta2 = res_/d2/2;
                for(int j = 0 ; delta2 * j < 1; j++) {
                    sp_->interpolate(nearest->state, node->state, j*delta2, temp);
                    if(!si_->isValid(temp)) {
                        isvalid = false;
                        break;
                    }
                }
            }
            
            if(!isvalid) {
                double x = node->state->as<base::SE2StateSpace::StateType>()->getX();
                double y = node->state->as<base::SE2StateSpace::StateType>()->getY();
                double yaw;

                x += DOMoveLength(node->potential, node->pvector) * node->pvector(0) / node->pvector.dot(node->pvector);
                y += DOMoveLength(node->potential, node->pvector) * node->pvector(1) / node->pvector.dot(node->pvector);

                node->state->as<base::SE2StateSpace::StateType>()->setXY(x, y);

                mm_->magneticForce(nearest->state, rand->state, node->state, node->fvector);
                if (node->fvector(0) == 0) yaw = ((node->fvector(1)>0) ? M_PI/2 : -M_PI/2);
                else yaw = atan(node->fvector(1)/node->fvector(0)) + ((node->fvector(0) < 0) ? M_PI : 0);
                node->state->as<base::SE2StateSpace::StateType>()->setYaw(yaw);

                pm_->totalForce(siC_, res_, goal, node->state, node->pvector);
                node->potential = pm_->repulsivePotential(siC_, res_, node->state);
                continue;
            }
            else {
                break;
            }
        }
        si_->freeState(temp);
        if(isvalid) {
            return true;
        }
        else
            return false;
    }
    void doRRT::addFlag(std::string name) {
        TimeFlag tf(name);
        timeflags_.push_back(tf);
    }
    void doRRT::showCalcTime(void) {
        if(timeflags_.size() == 0) {
            std::cout << "no flags" << std::endl;
        }
        else {
            std::cout << "at \"" << timeflags_[0].name << "\" : " << std::chrono::duration_cast<std::chrono::nanoseconds>(timeflags_[0].tp.time_since_epoch()).count() << std::endl;
            if(timeflags_.size() > 1) {
                for(int i = 1 ; i < timeflags_.size() ; i++) {
                    std::cout << "from \"" << timeflags_[i-1].name << "\" to \"" << timeflags_[i].name << "\" is " << std::chrono::duration_cast<std::chrono::nanoseconds>(timeflags_[i].tp-timeflags_[i-1].tp).count() << std::endl;
                }
            }
        }
    }

    bool doRRT::collisionFree(Motion *m1, Motion *m2) {
        double d = sp_->distance(m1->state, m2->state);
        double deltat = res_/d/2;

        base::State* temp(si_->allocState());
        for (int i = 0 ; deltat * i < 1 ; i++) {
            sp_->interpolate(m1->state, m2->state, deltat * i, temp);
            if(!si_->isValid(temp)) {
                si_->freeState(temp);
                return false;
            }
        }
        si_->freeState(temp);
        return true;
    }

    
    double doRRT::DOMoveLength(double potential, Eigen::Vector3d pvector) {
        return res_;
    }
    double doRRT::distanceMotion(const Motion *a, const Motion *b) {
        double d = sp_->distance(a->state, b->state);
        return d;
    }
    double doRRT::distanceMotionEuc(const Motion *a, const Motion *b) {
        const auto * astate = a->state->as<base::SE2StateSpace::StateType>();
        const auto * bstate = b->state->as<base::SE2StateSpace::StateType>();
        double edist = sqrt( (astate->getX()-bstate->getX()) * (astate->getX()-bstate->getX()) + (astate->getY()-bstate->getY()) * (astate->getY()-bstate->getY()) );
        return edist;
    }
    void doRRT::removeFromParent(Motion *m) {
        for(std::vector<Motion *>::iterator it = m->parent->children.begin() ; it!=m->parent->children.end();++it) {
            if(*it == m) {
                m->parent->children.erase(it);
                break;
            }
        }
    }
    void doRRT::updateChildCosts(Motion *m) {
        for (std::size_t i = 0 ; i < m->children.size() ; ++i) {
            m->children[i]->cost = combineCosts(m->cost, m->children[i]->incCost);
            updateChildCosts(m->children[i]);
        }
    }
    double doRRT::combineCosts(double c1, double c2) {
        return (c1 + c2);
    }
    void doRRT::getNeighbors(Motion *motion, std::vector<int> &nbhind) {
        nbhind.clear();
        double cardDbl = static_cast<double>(mtree_.size + 1u);
        double r = r_rrt_*std::pow(log(cardDbl)/cardDbl, 1.0/2);
        for(int i = 0 ; i < mtree_.size ; i++) {
            if(distanceMotionEuc(mtree_.members[i], motion) > r) continue;
            double dist = distanceMotion(mtree_.members[i], motion);
            if(r > dist) {
                nbhind.push_back(i);
            }
        }
    }
    void doRRT::changeParent(Motion *np, Motion *m) {
        removeFromParent(m);
        m->parent = np;
        np->children.push_back(m);
        m->incCost = distanceMotion(np, m);
        m->cost = combineCosts(m->parent->cost, m->incCost);
        updateChildCosts(m);
    }
    int doRRT::nearest(Motion *from) {
        Motion* dest = mtree_.members[0];
        double mincost = distanceMotion(dest, from);
        int minindex = 0;
        for(int i = 0 ; i < mtree_.size ; i++) {
            if(distanceMotionEuc(mtree_.members[i], from) > mincost) continue;
            double cost = distanceMotion(mtree_.members[i], from);
            if(mincost > cost) {
                mincost = cost;
                minindex = i;
            }
        }
        return minindex;
    }
    void doRRT::addMotion(Motion *p, Motion *c) {
        mtree_.members.push_back(c);
        mtree_.size++;
        c->parent = p;
        p->children.push_back(c);
        c->incCost = distanceMotion(p, c);
        c->cost = combineCosts(p->cost, c->incCost);
    }
    void doRRT::setRoot(Motion *h) {
        mtree_.freeMemory();
        mtree_.members.push_back(h);
        h->parent = nullptr;
        h->cost = 0;
        mtree_.head = h;
        mtree_.size = 1;
    }
    void doRRT::showTree(void) {
        for(int i = 0 ; i < mtree_.size ; i++) {
            std::cout << i << "th | position : " << mtree_.members[i]->state->as<base::SE2StateSpace::StateType>()->getX() << ", " << mtree_.members[i]->state->as<base::SE2StateSpace::StateType>()->getY() << std::endl;
            if(mtree_.members[i]->parent) {
                std::cout << "       parent : " << mtree_.members[i]->parent->state->as<base::SE2StateSpace::StateType>()->getX() << ", " << mtree_.members[i]->parent->state->as<base::SE2StateSpace::StateType>()->getY() << std::endl;
                if(collisionFree(mtree_.members[i], mtree_.members[i]->parent)) {
                    std::cout << "       collision free from parent" << std::endl;
                }
                else {
                    std::cout << "       collision from parent" << std::endl;
                }
            }
            std::cout << "       cost : " << mtree_.members[i]->cost << std::endl;
        }
    }
    std::vector<base::State *> doRRT::getTree(void) {
        std::vector<base::State *> res;
        for(int i = 0 ; i < mtree_.size ; i++) {
            res.push_back(mtree_.members[i]->state);
        }
        return res;
    }
    void doRRT::showMotion(const Motion *m, const std::string tag) {
        std::cout << tag << std::endl;
        if(m->state) {
            auto *s = m->state->as<base::SE2StateSpace::StateType>();
            std::cout << "   position : " << s->getX() << ", " << s->getY() << std::endl;
        }
        if(m->parent) {
            auto *p = m->parent->state->as<base::SE2StateSpace::StateType>();
            std::cout << "   parent : " << p->getX() << ", " << p->getY() << std::endl << std::endl;
        }
        std::cout << "   cost : " << m->cost << std::endl;
    }
    void doRRT::clear(void) {

    }
    void doRRT::freeMemory() {
        mtree_.freeMemory();
        timeflags_.clear();
    }
}