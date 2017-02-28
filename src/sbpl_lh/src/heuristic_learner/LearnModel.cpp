#include <sbpl/heuristic_learner/LearnModel.h>

LearnModel::LearnModel(std::vector<PlanData>& training_data,
					   double alpha, int model_num) :
m_training_data(training_data),
m_alpha(alpha),
m_model_num(model_num) {
	initTheta();
}

LearnModel::~LearnModel() {
	m_training_data.clear();
}

void LearnModel::initTheta() {
	// hardcode feature size for now
	m_theta = Eigen::MatrixXd::Zero(1, FEATURE_SIZE);
}

void LearnModel::updateTheta(const Eigen::MatrixXd& del_theta) {
	m_theta += m_alpha*del_theta;
}

Eigen::MatrixXd LearnModel::getMathModel(const Eigen::MatrixXd& feature) {
	
	if(m_model_num == MathModel::Linear) {
		return m_theta*feature;
	}	
}

Eigen::MatrixXd LearnModel::getMathModelGradient
				(const Eigen::MatrixXd& feature) {
	
	if(m_model_num == MathModel::Linear) {
		return feature;
	}
}

Eigen::MatrixXd LearnModel::getFeatureVector(const PlanState& s, 
											 const PlanState& g) {
	
	Eigen::MatrixXd feature = Eigen::MatrixXd::Zero(FEATURE_SIZE, 1);

	feature(0,0) = s.x;
	feature(1,0) = s.y;
	feature(2,0) = s.yaw;
	feature(3,0) = g.x;
	feature(4,0) = g.y;
	feature(5,0) = g.yaw;

	return feature;
}

void LearnModel::vfApprox() {

	for(int i = 0; i < m_training_data.size(); ++i) {
		
		for(int j = 0; j < m_training_data[i].path.size(); ++j) {
			Eigen::MatrixXd f = getFeatureVector(m_training_data[i].path[j],
												 m_training_data[i].goal);

			Eigen::MatrixXd del_theta = (m_training_data[i].cost[j] - 
										 getMathModel(f)(0,0))*
										 getMathModelGradient(f);

			updateTheta(del_theta);
		}
	}
}

Eigen::MatrixXd LearnModel::getTheta() {
	return m_theta;
}

void LearnModel::printTheta() {
	std::cout << "Theta : " << std::endl;
	std::cout << m_theta << std::endl;
}
