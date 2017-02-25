#include <sbpl/planners/ompl_planner.h>

OMPLPlanner::OMPLPlanner(int planner_id, double planning_time) :
m_planner_id(planner_id),
m_allocated_planning_time(planning_time) {

}

bool OMPLPlanner::initEnv(const char* sEnvFile) {
	FILE* fCfg = fopen(sEnvFile, "r");
    if (fCfg == NULL) {
        SBPL_ERROR("ERROR: unable to open %s\n", sEnvFile);
        throw new SBPL_Exception();
    }
    readConfiguration(fCfg);
    fclose(fCfg);
    return true;
}

void OMPLPlanner::readConfiguration(FILE* fCfg)
{
    //read in the configuration of environment and initialize  EnvNAVXYTHETALATCfg structure
    char sTemp[1024], sTemp1[1024];
    int dTemp;
    int x, y;

    //discretization(cells)
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early (discretization)\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "discretization(cells):");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format (discretization)\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early (discretization)\n");
        throw new SBPL_Exception();
    }
    m_env.width_cells = atoi(sTemp);
    
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early (discretization)\n");
        throw new SBPL_Exception();
    }
    m_env.height_cells = atoi(sTemp);

    // Scan for optional NumThetaDirs parameter. Check for following obsthresh.
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "NumThetaDirs:");
    if (strcmp(sTemp1, sTemp) != 0) {
        // optional NumThetaDirs not available; default is NAVXYTHETALAT_THETADIRS (16)
        strcpy(sTemp1, "obsthresh:");
        if (strcmp(sTemp1, sTemp) != 0) {
            SBPL_ERROR("ERROR: configuration file has incorrect format\n");
            SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
            throw new SBPL_Exception();
        }
        else {
            m_env.numthetadirs = NAVXYTHETALAT_THETADIRS;
        }
    }
    else {
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            SBPL_ERROR("ERROR: ran out of env file early (NumThetaDirs)\n");
            throw new SBPL_Exception();
        }
        m_env.numthetadirs = atoi(sTemp);

        //obsthresh: 
        if (fscanf(fCfg, "%s", sTemp) != 1) {
            SBPL_ERROR("ERROR: ran out of env file early (obsthresh)\n");
            throw new SBPL_Exception();
        }
        strcpy(sTemp1, "obsthresh:");
        if (strcmp(sTemp1, sTemp) != 0) {
            SBPL_ERROR("ERROR: configuration file has incorrect format\n");
            SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
            SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
            throw new SBPL_Exception();
        }
    }

    // obsthresh
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.obsthresh = atoi(sTemp);
    SBPL_PRINTF("obsthresh = %d\n", m_env.obsthresh);

    //cost_inscribed_thresh: 
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "cost_inscribed_thresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.cost_inscribed_thresh = atoi(sTemp);
    SBPL_PRINTF("cost_inscribed_thresh = %d\n", m_env.cost_inscribed_thresh);

    //cost_possibly_circumscribed_thresh: 
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "cost_possibly_circumscribed_thresh:");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        SBPL_PRINTF("see existing examples of env files for the right format of heading\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.cost_possibly_circumscribed_thresh = atoi(sTemp);
    SBPL_PRINTF("cost_possibly_circumscribed_thresh = %d\n", m_env.cost_possibly_circumscribed_thresh);

    //cellsize
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "cellsize(meters):");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.cellsize = atof(sTemp);

    //speeds
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "nominalvel(mpersecs):");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.nominalvel_mpersecs = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    strcpy(sTemp1, "timetoturn45degsinplace(secs):");
    if (strcmp(sTemp1, sTemp) != 0) {
        SBPL_ERROR("ERROR: configuration file has incorrect format\n");
        SBPL_PRINTF("Expected %s got %s\n", sTemp1, sTemp);
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.timetoturn45degsinplace_secs = atof(sTemp);

    //start(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.start_x = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.start_y = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.start_theta = atof(sTemp);

    //end(meters,rads):
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.goal_x = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.goal_y = atof(sTemp);
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    m_env.goal_theta = atof(sTemp);

    (m_env.grid_2d).resize(m_env.width_cells, std::vector<int>(m_env.height_cells));

    //environment:
    if (fscanf(fCfg, "%s", sTemp) != 1) {
        SBPL_ERROR("ERROR: ran out of env file early\n");
        throw new SBPL_Exception();
    }
    for (int y = 0; y < m_env.height_cells; y++)
        for (int x = 0; x < m_env.width_cells; x++) {
            if (fscanf(fCfg, "%d", &dTemp) != 1) {
                SBPL_ERROR("ERROR: incorrect format of config file\n");
                throw new SBPL_Exception();
            }
            m_env.grid_2d[x][y] = dTemp;
        }
}

bool OMPLPlanner::initOMPL() {

	ROS_INFO("Initializing OMPL");

	ompl::base::SE2StateSpace* se2 = new ompl::base::SE2StateSpace();
    ompl::base::RealVectorBounds base_bounds(2);
    base_bounds.setLow(0,0);
    base_bounds.setHigh(0,10);
    base_bounds.setLow(1,0);
    base_bounds.setHigh(1,6);
    se2->setBounds(base_bounds);

    m_SE2Space.reset(se2);

    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(m_SE2Space));
    m_si = si;

    m_collision_checker = new CustomStateValidityChecker(si, m_env);
    si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(m_collision_checker));
    si->setStateValidityCheckingResolution(0.002/si->getMaximumExtent());
    si->setup();

    m_pdef = new ompl::base::ProblemDefinition(si);

    if (m_planner_id == RRT_NUM)
        m_planner = new ompl::geometric::RRT(si);
    else if (m_planner_id == RRTCONNECT_NUM)
        m_planner = new ompl::geometric::RRTConnect(si);
    else if (m_planner_id == PRM_P_NUM)
        m_planner = new ompl::geometric::PRM(si);
    else if (m_planner_id == RRTSTAR_NUM || m_planner_id == RRTSTARFIRSTSOL_NUM)
        m_planner = new ompl::geometric::RRTstar(si);
    else {
        ROS_ERROR("invalid planner id!");
    	return false;
    }

    m_planner->setProblemDefinition(ompl::base::ProblemDefinitionPtr(m_pdef));
    m_planner->setup();
    m_pathSimplifier = new ompl::geometric::PathSimplifier(si);
    ROS_INFO("finished initializing OMPL planner");

    if(!initViz()) {
        ROS_ERROR("Could not init viz");
        return true;
    }
    visualizeMap();
    ROS_INFO("Initialized visualization");

    return true;
}

bool OMPLPlanner::plan() {
	if (m_planner_id == PRM_P_NUM)
        ROS_INFO("running PRM planner!");
    if (m_planner_id == RRT_NUM)
        ROS_INFO("running RRT planner!");
    if (m_planner_id == RRTSTAR_NUM || m_planner_id == RRTSTARFIRSTSOL_NUM)
        ROS_INFO("running RRTStar planner!");
    if (m_planner_id == RRTCONNECT_NUM)
        ROS_INFO("running RRTConnect planner!");

    m_planner->clear();
    m_planner->getProblemDefinition()->clearSolutionPaths();
    m_planner->as<ompl::geometric::PRM>()->clearQuery();

    SE2State ompl_start(m_SE2Space);
    SE2State ompl_goal(m_SE2Space);

    if (!createStartGoal(ompl_start, ompl_goal))
        return false;

    m_pdef->clearGoal();
    m_pdef->clearStartStates();
    m_pdef->setStartAndGoalStates(ompl_start,ompl_goal);

    double t0 = ros::Time::now().toSec();
    ROS_INFO("Allocated planning time %f", m_allocated_planning_time);
    ompl::base::PlannerStatus ompl_res = m_planner->solve(m_allocated_planning_time);
    double t1 = ros::Time::now().toSec();
    double planning_time = t1-t0;
    ompl::base::PathPtr path = m_planner->getProblemDefinition()->getSolutionPath();

    ompl::geometric::PathGeometric geo_path(m_si);

    if (ompl_res.asString().compare("Exact solution") == 0 && path){
        
        ROS_INFO("OMPL Found exact solution");

        geo_path = static_cast<ompl::geometric::PathGeometric&>(*path);

        bool b1 = m_pathSimplifier->reduceVertices(geo_path);
        bool b2 = m_pathSimplifier->collapseCloseVertices(geo_path);
        bool b3 = m_pathSimplifier->shortcutPath(geo_path);

        for(unsigned int i = 0; i < geo_path.getStateCount(); i++){
            ompl::base::State* state = geo_path.getState(i);
            printState(state);
        }
    }

    visualizePath(geo_path);

    return true;
}

bool OMPLPlanner::initViz() {

    map.header.stamp = ros::Time::now();
    map.header.frame_id = "map";
    map.info.origin.orientation.w = 1.0;
    map.info.resolution = m_env.cellsize;
    map.info.width = m_env.width_cells;
    map.info.height = m_env.height_cells;

    for (int y = 0; y < m_env.height_cells; y++) {
        for (int x = 0; x < m_env.width_cells; x++) {
            (map.data).push_back(100*m_env.grid_2d[x][y]);
        }
    }
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("viz_map", 1000);

    ompl_path.header.stamp = ros::Time::now();
    ompl_path.header.frame_id = "map";
    path_publisher = nh.advertise<nav_msgs::Path>("viz_path", 1000);

    return true;
}

void OMPLPlanner::printState(const ompl::base::State* state) {
    
    const ompl::base::SE2StateSpace::StateType* se2_state = 
        dynamic_cast<const ompl::base::SE2StateSpace::StateType*> (state);


    ROS_INFO("State x : %f y : %f z : %f", se2_state->getX(),
                                           se2_state->getY(),
                                           se2_state->getYaw());
}

void OMPLPlanner::visualizeMap() {  
    // if(!m_use_visualization) return;  
    
    ros::Duration(1.0).sleep();
    map_publisher.publish(map);
}

void OMPLPlanner::visualizePath(ompl::geometric::PathGeometric& geo_path) {
    // if(!m_use_visualization) return;  

    for(unsigned int i = 0; i < geo_path.getStateCount(); i++) {

        ompl::base::State* state = geo_path.getState(i);

        const ompl::base::SE2StateSpace::StateType* se2_state = 
           dynamic_cast<const ompl::base::SE2StateSpace::StateType*> (state);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = se2_state->getX();
        pose_stamped.pose.position.y = se2_state->getY();
        pose_stamped.pose.position.z = 0.0; // random number 
        pose_stamped.pose.orientation.w = 1.0;

        (ompl_path.poses).push_back(pose_stamped);
    }

    path_publisher.publish(ompl_path);
}

bool OMPLPlanner::createStartGoal(SE2State& ompl_start, SE2State& ompl_goal) {

	ompl_start->setXY(m_env.start_x, m_env.start_y);
	ompl_start->setYaw(angles::normalize_angle(m_env.start_theta));

    if(!m_collision_checker->isValid(ompl_start.get())) {
        return false;
    }

    ROS_INFO("Start Valid : %f %f %f", ompl_start->getX(),
                                 ompl_start->getY(),
                                 ompl_start->getYaw() );

	ompl_goal->setXY(m_env.goal_x, m_env.goal_y);
	ompl_goal->setYaw(angles::normalize_angle(m_env.goal_theta));

    if(!m_collision_checker->isValid(ompl_goal.get())) {
        return false;
    }

    ROS_INFO("Goal Valid : %f %f %f", ompl_goal->getX(),
                                 ompl_goal->getY(),
                                 ompl_goal->getYaw() );

    return true;
}