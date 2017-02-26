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

#define FEATURE_SIZE 6

enum MathModel {
	Linear = 0,
	Exponential
};

class LearnModel {
public:
	LearnModel(std::vector<PlanData> training_data,
			   double alpha = 0.01, int model_num = 0);
	~LearnModel();
	void initTheta();
	void updateTheta(const Eigen::MatrixXd& del_theta);
	Eigen::MatrixXd getMathModel(const Eigen::MatrixXd& feature);
	Eigen::MatrixXd getMathModelGradient(const Eigen::MatrixXd& 
										 feature);
	Eigen::MatrixXd getFeatureVector(const PlanState& s, 
								     const PlanState& g);
	void vfApprox();
	Eigen::MatrixXd getTheta();

private:
	std::vector<PlanData> m_training_data;
    Eigen::MatrixXd m_theta;
    double m_alpha;
    int m_model_num;
};
