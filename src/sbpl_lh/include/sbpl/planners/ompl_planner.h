#pragma once
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/goals/GoalState.h>
#include <memory>
#include <vector>

#include <sbpl/headers.h>

// ROS includes
#include <ros/ros.h>
#include <angles/angles.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#define RRT_NUM 0
#define PRM_P_NUM 1
#define RRTCONNECT_NUM 2 
#define RRTSTAR_NUM 3
#define RRTSTARFIRSTSOL_NUM 4

typedef ompl::base::ScopedState<ompl::base::SE2StateSpace> SE2State;

struct DiscreteEnv {
	int width_cells;
	int height_cells;
	int numthetadirs;
	int obsthresh;
	int cost_inscribed_thresh;
	int cost_possibly_circumscribed_thresh;
	
	double cellsize;
	double nominalvel_mpersecs;
	double timetoturn45degsinplace_secs;

	double start_x;
	double start_y;
	double start_theta;
	double goal_x;
	double goal_y;
	double goal_theta;

	std::vector<std::vector<int>> grid_2d;
};

class CustomStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
     CustomStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
     							DiscreteEnv env) :
     ompl::base::StateValidityChecker(si),
     m_env(env) {

     }
    
     bool checkEnvBounds(int x, int y, int theta) const {
     	if(x >= 0 && x < m_env.width_cells &&
     	   y >= 0 && y < m_env.height_cells &&
     	   theta >= 0 && theta < m_env.numthetadirs)
     		return true;
     	else {
     		ROS_DEBUG("Out of bounds");
     		return false;
     	}
     }

     virtual bool isValid(const ompl::base::State* state) const
     {
    	const ompl::base::SE2StateSpace::StateType* se2_state = 
    	dynamic_cast<const ompl::base::SE2StateSpace::StateType*> (state);

     	int x = (int)(se2_state->getX()/m_env.cellsize);
     	int y = (int)(se2_state->getY()/m_env.cellsize);

     	double angular_bin = 2.0*M_PI/m_env.numthetadirs;
    	int theta = (int)((se2_state->getYaw()/angular_bin) + 0.5f);

    	if(theta >= m_env.numthetadirs)
        	theta -= m_env.numthetadirs;

    	if(theta < 0)
        	theta += m_env.numthetadirs;

     	if(!checkEnvBounds(x, y, theta))
     		return false;

     	if(m_env.grid_2d[x][y] == 1)
     		return false;

     	return true;
     }

 private:
     DiscreteEnv m_env;
};

class OMPLPlanner {
public:
	OMPLPlanner(int planner_id, double planning_time);
	bool initEnv(const char* sEnvFile);
	void readConfiguration(FILE* fCfg);
	bool initOMPL();
	bool initViz();
	bool plan();
	bool createStartGoal(SE2State& ompl_start, SE2State& ompl_goal);
	void printState(const ompl::base::State* state);
	void visualizeMap();
	void visualizePath(ompl::geometric::PathGeometric& geo_path);

private:
    ompl::base::SpaceInformationPtr m_si;
    ompl::base::StateSpacePtr m_SE2Space;
    ompl::base::ProblemDefinition* m_pdef;
    ompl::base::Planner* m_planner;
    ompl::geometric::PathSimplifier* m_pathSimplifier;
    ompl::base::StateValidityChecker* m_collision_checker;
    int m_planner_id;
    double m_allocated_planning_time;
    DiscreteEnv m_env;

    //visualization
    ros::NodeHandle nh;  
    ros::Publisher map_publisher;
    ros::Publisher path_publisher;
    nav_msgs::OccupancyGrid map;
    nav_msgs::Path ompl_path;
};

