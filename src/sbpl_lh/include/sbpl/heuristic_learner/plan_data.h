#pragma once
#include <iostream>
#include <vector>
#include <fstream>

typedef struct {

	double x;
	double y;
	double yaw;

} PlanState;

typedef struct {

	bool succ;
	double plan_time;
	PlanState goal;
	std::vector<PlanState> path; // does not include goal
	std::vector<double> cost; // should it be int?

	void printPlanData() {
		std::cout << "Succ : " << succ << std::endl;	
		std::cout << "PlanTime : " << plan_time << std::endl;	
		std::cout << "Goal : " << goal.x << " " 
							   << goal.y << " "
							   << goal.yaw << std::endl;

		for(int i = 0 ; i < path.size(); ++i) {

			std::cout << "State" << i <<  ": " << path[i].x << " " 
								   			   << path[i].y << " "
								   			   << path[i].yaw << " "
								   			   << cost[i] << std::endl;
		}	
	}

} PlanData;