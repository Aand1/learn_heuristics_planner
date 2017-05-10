#include <sbpl/heuristic_learner/GeneratePlans.h>

GeneratePlans::GeneratePlans(int num_plans, int planner_mode,
						     double time_per_plan,
						     const char* envCfgFilename,
						     const char* motPrimFilename) :
m_num_plans(num_plans),
m_planner_mode(planner_mode),
m_time_per_plan(time_per_plan),
m_config_file(envCfgFilename),
m_mprim_file(motPrimFilename) {

	// m_training_data.resize(m_num_plans);
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

			    PlanData pdata;
			    bool bRet = ompl_planner.plan(pdata);
			    if(bRet) m_training_data.push_back(pdata);
			}
			break;
		}
		case 4:
		{
		    int bRet = 0;
		    double allocated_time_secs = m_time_per_plan; // in seconds
		    double initialEpsilon = 10;//atof(eps);
		    MDPConfig MDPCfg;
		    bool bsearchuntilfirstsolution = false;
		    bool bforwardsearch = true;

		    // set the perimeter of the robot 
		    // (it is given with 0,0,0 robot ref. point for which planning is done)
		    std::vector<sbpl_2Dpt_t> perimeterptsV;
		    sbpl_2Dpt_t pt_m;
		    double halfwidth = 0.1;
		    double halflength = 0.1;
		    pt_m.x = -halflength;
		    pt_m.y = -halfwidth;
		    perimeterptsV.push_back(pt_m);
		    pt_m.x = halflength;
		    pt_m.y = -halfwidth;
		    perimeterptsV.push_back(pt_m);
		    pt_m.x = halflength;
		    pt_m.y = halfwidth;
		    perimeterptsV.push_back(pt_m);
		    pt_m.x = -halflength;
		    pt_m.y = halfwidth;
		    perimeterptsV.push_back(pt_m);

		    // perimeterptsV.clear();

			for(int i = 0; i < m_num_plans; ++i) {

			    // Initialize Environment (should be called before initializing anything else)
			    EnvironmentNAVXYTHETALAT environment_navxythetalat;
			
			    if (!environment_navxythetalat.InitializeEnv(m_config_file, perimeterptsV, m_mprim_file)) {
			        printf("ERROR: InitializeEnv failed\n");
			        throw new SBPL_Exception();
			    }
				   
			    // Initialize MDP Info
			    if (!environment_navxythetalat.InitializeMDPCfg(&MDPCfg)) {
			        printf("ERROR: InitializeMDPCfg failed\n");
			        throw new SBPL_Exception();
			    }

			    // plan a path
			    std::vector<int> solution_stateIDs_V;

			    SBPLPlanner* planner = new ARAPlanner(&environment_navxythetalat, bforwardsearch);    
			    
			    // set planner properties
			    if (planner->set_start(MDPCfg.startstateid) == 0) {
			        printf("ERROR: failed to set start state\n");
			        throw new SBPL_Exception();
			    }
			    if (planner->set_goal(MDPCfg.goalstateid) == 0) {
			        printf("ERROR: failed to set goal state\n");
			        throw new SBPL_Exception();
			    }

			    planner->set_initialsolution_eps(initialEpsilon);
			    planner->set_search_mode(bsearchuntilfirstsolution);

			    environment_navxythetalat.InitViz();	
			    environment_navxythetalat.VisualizeMap();
	
			    // plan
			    printf("start planning...\n");
    			double t0 = ros::Time::now().toSec();
			    bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
			    double t1 = ros::Time::now().toSec();
    			double planning_time = t1-t0;
			    printf("done planning\n");
			    printf("size of solution=%d\n", (unsigned int)solution_stateIDs_V.size());

			    environment_navxythetalat.VisualizePath(solution_stateIDs_V);

				PlanData plan_data;
				plan_data.plan_time = planning_time;
				if(bRet) { 
				    plan_data.succ = true;
				    environment_navxythetalat.computePlanningData(plan_data,
					    						        solution_stateIDs_V);
				    m_training_data.push_back(plan_data);
				}
				else{
    			    plan_data.succ = false;
				}

			   	fflush(NULL);

		    	delete planner;	

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

	std::string filename = "SE2_training_plans.data";
	std::string full_path = lib_path + filename;
	std::ofstream file;
	file.open(full_path);
	for (int i = 0; i < m_training_data.size(); ++i) {

		for(int j = 0; j < m_training_data[i].path.size(); ++j) {
			file << m_training_data[i].path[j].x << "," <<
					m_training_data[i].path[j].y << "," <<
					m_training_data[i].path[j].yaw << "," <<
					m_training_data[i].goal.x << "," <<
					m_training_data[i].goal.y << "," <<
					m_training_data[i].goal.yaw << "," << 
					m_training_data[i].cost[j] << "\n";  			
		}
	}
}

void GeneratePlans::writePlansToFannData(std::string folder) {
	
	if(m_training_data.size() == 0) {
		ROS_ERROR("No plans to write");
		return;
	}

	std::string lib_path = ros::package::getPath("sbpl_lh") +
						   "/" + 
						   folder +
						   "/";
	std::string filename = "SE2_train_2.data";
	std::string full_path = lib_path + filename;

	// open file
	std::ofstream file;
	file.open(full_path);

	int num_training_data = 0;
	for (int i = 0; i < m_training_data.size(); ++i) {
		num_training_data += m_training_data[i].path.size();
	}

	file << num_training_data << " " << INPUT_FEATURE_SIZE <<
								 " " << OUTPUT_SIZE << "\n";
 
	for (int i = 0; i < m_training_data.size(); ++i) {

		for(int j = 0; j < m_training_data[i].path.size(); ++j) {
			file << m_training_data[i].path[j].x << " " <<
					m_training_data[i].path[j].y << " " <<
					m_training_data[i].path[j].yaw << " " <<
					m_training_data[i].goal.x << " " <<
				    m_training_data[i].goal.y << " " <<
					m_training_data[i].goal.yaw << "\n"; 
	
			file << m_training_data[i].cost[j] << "\n";
		}

	}
}
