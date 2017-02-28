#include <sbpl/heuristic_learner/GeneratePlans.h>

GeneratePlans::GeneratePlans(int num_plans, int planner_mode,
						     double time_per_plan,
						     const char* envCfgFilename) :
m_num_plans(num_plans),
m_planner_mode(planner_mode),
m_time_per_plan(time_per_plan),
m_config_file(envCfgFilename) {

	m_training_data.resize(m_num_plans);
}

GeneratePlans::~GeneratePlans() {
	m_training_data.clear();
}

void GeneratePlans::runSetup() {

	switch(m_planner_mode) {
		case 0:
		case 1:
		case 2:
		case 3:
		{
			for(int i = 0; i < m_num_plans; ++i) {

			    OMPLPlanner ompl_planner(m_planner_mode, m_time_per_plan);

			    if (!ompl_planner.initEnv(m_config_file)) {
			        printf("ERROR: InitializeEnv failed\n");
			        throw new SBPL_Exception();
			    }

			    if (!ompl_planner.initOMPL()) {
			        printf("ERROR: Initializing OMPL failed\n");
			        throw new SBPL_Exception();
			    }

			    bool bRet = ompl_planner.plan(m_training_data[i]);
			}
			break;
		}
		default:
		{
			ROS_ERROR("Not implemented other demo methods");
			return;
		}
	}
}

std::vector<PlanData>& GeneratePlans::getFullPlanData() {
	return m_training_data;
}

void GeneratePlans::writePlansToFile(std::string folder) {
	
	if(m_training_data.size() == 0) {
		ROS_ERROR("No plans to write");
		return;
	}

	std::string lib_path = ros::package::getPath("sbpl_lh") +
						   "/" + 
						   folder +
						   "/";

	for (int i = 0; i < m_num_plans; ++i) {
		std::string filename = "trial" + std::to_string(i) + ".plan";
		std::string full_path = lib_path + filename;

		std::ofstream file;
		file.open(full_path);
		file << m_training_data[i].succ << "\n"; 
		file << m_training_data[i].plan_time << "\n"; 
		file << m_training_data[i].goal.x << "," <<
				m_training_data[i].goal.y << "," <<
				m_training_data[i].goal.yaw << "\n"; 

		for(int j = 0; j < m_training_data[i].path.size(); ++j) {
			file << m_training_data[i].path[j].x << "," <<
					m_training_data[i].path[j].y << "," <<
					m_training_data[i].path[j].yaw << "," <<
					m_training_data[i].cost[j] << "\n";  			
		}
	}
}
