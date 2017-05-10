#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

#include <ros/ros.h>

#include <sbpl/headers.h>

void generateTraningModel(int num_plans, double time_per_plan,
                         int planner_id, const char* envCfgFilename,
                         const char* motPrimFilename) {

    GeneratePlans plan_generator(num_plans, planner_id,
                                 time_per_plan, envCfgFilename,
                                 motPrimFilename);
    plan_generator.runSetup();
    plan_generator.writePlansToFile();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "learn_node");
    
    ros::NodeHandle ph("~");
    
    std::string r_config_path, r_prim_path;
    int num_plans;
    double time_per_plan;
    int planner_id;
    bool use_ompl;
    
    ph.getParam("config_path", r_config_path);
    ph.getParam("prim_path", r_prim_path);
    ph.getParam("num_plans", num_plans);
    ph.getParam("time_per_plan", time_per_plan);
    ph.getParam("use_ompl", use_ompl);
    ph.getParam("planner_id", planner_id);

    std::string lib_path = ros::package::getPath("sbpl_lh");
    std::string config_path = lib_path + r_config_path; 
    std::string prim_path = lib_path + r_prim_path; 
    
    generateTraningModel(num_plans, time_per_plan,
                        planner_id, config_path.c_str(),
                        prim_path.c_str());

    return 0;        
}