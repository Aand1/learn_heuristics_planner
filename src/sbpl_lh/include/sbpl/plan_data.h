#pragma once
#include <iostream>
#include <vector>
#include <iostream>

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

} PlanData;