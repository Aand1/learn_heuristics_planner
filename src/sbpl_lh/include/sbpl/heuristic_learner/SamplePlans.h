#pragma once

// Standard includes
#include <memory>
#include <vector>
#include <algorithm>

// SBPL includes
#include <sbpl/headers.h>
#include <sbpl/plan_data.h>

class SamplePlans {
public:
	SamplePlans(int num_plans, int planner_mode,
				const char* envCfgFilename);
	~SamplePlans();


private:
	int m_num_trials;
	int m_planner_mode;
	const char* m_config_file;
};