// hybrid_astar.h

#ifndef HYBRIDASTAR
#define HYBRIDASTAR

#include <ompl/base/Planner.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/State.h>
#include <vector>
#include <cmath>


namespace ompl {
    
    class hybridASTAR : public base::Planner {
        public :

        hybridASTAR(const base::SpaceInformationPtr &si);
        ~hybridASTAR(void) override;
        
        base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
       
        double drive_distance = sqrt(2)*0.09+0.015;
        double cost;
        double current_cost;
        double pi = 3.14159265;
        
        void clear(void) override;
        void setup(void) override;
        void freeMemory();

        double euclidean_distance(base::State *state, base::State *goal);

        std::vector<double> return_discrete(double x, double y);
        std::vector<double> heading_changes = {-3*pi/12, -2*pi/12, -pi/12, 0, pi/12, 2*pi/12, 3*pi/12};

        int return_lowest_cost_path(std::vector<double> input);
        
        bool vector_contains(std::vector<std::vector<double>> closed, std::vector<double> input);
        bool state_compare(base::State* input, base::State* goal);
    
        float heuristic;
       
        
    };
    typedef std::shared_ptr<hybridASTAR> hybridASTARPtr;
}

#endif
