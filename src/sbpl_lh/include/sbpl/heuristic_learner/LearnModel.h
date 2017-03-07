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

// Eigen includes
#include <eigen3/Eigen/Geometry> 

// Fann includes
#include <doublefann.h>
#include <fann_cpp.h>

class LearnModel {
public:
	LearnModel();
	~LearnModel();
	std::string setupFolder(std::string folder);
	void vfApprox();
	// int printCallback(FANN::neural_net &net, FANN::training_data &train,
    //unsigned int max_epochs, unsigned int epochs_between_reports,
    //float desired_error, unsigned int epochs, void *user_data);

private:
    std::string m_tdata_file;
    std::string m_net_file;
};
