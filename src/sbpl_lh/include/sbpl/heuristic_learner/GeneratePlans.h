#pragma once

// Standard includes
#include <memory>
#include <vector>
#include <algorithm>

// ROS INCLUDES
#include <ros/ros.h>
#include <ros/package.h>

// SBPL includes
#include <sbpl/headers.h>
#include <sbpl/heuristic_learner/plan_data.h>

class GeneratePlans {
public:
	GeneratePlans(int num_plans, int planner_mode,
				  double time_per_plan,
				  const char* envCfgFilename = NULL);
	~GeneratePlans();
	void runSetup();
	std::vector<PlanData>& getFullPlanData();
	void writePlansToFile(std::string folder = "training_plans");

private:
	int m_num_plans;
	int m_planner_mode;
	double m_time_per_plan;
	const char* m_config_file;

	std::vector<PlanData> m_training_data;
};