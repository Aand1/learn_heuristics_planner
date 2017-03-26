#include <sbpl/heuristic_learner/LearnModel.h>

LearnModel::LearnModel()
{
	std::string folder = setupFolder("training_plans");

	m_tdata_file = folder + "SE2_train_2.data";
	m_net_file = folder + "SE2.net";
	m_test_file = folder + "SE2_test_2.txt";
}

LearnModel::~LearnModel() {

}

std::string LearnModel::setupFolder(std::string folder) {
	return ros::package::getPath("sbpl_lh") +
					   	   "/" + 
					       folder +
					       "/";
}

void LearnModel::vfApprox() {

	const float learning_rate = 0.5f;
    const unsigned int num_input = INPUT_FEATURE_SIZE;
    const unsigned int num_output = OUTPUT_SIZE;
    const unsigned int num_layers = 4;
    const unsigned int num_neurons_hidden = 10;
    const float desired_error = (const float) 0.1;
    const unsigned int max_epochs = 35000;
    const unsigned int epochs_between_reports = 1000;
    const unsigned int layer_arr [num_layers] = 
    {num_input, 12, 12, num_output};

    FANN::neural_net net;
    // net.create_standard(num_layers, num_input, num_neurons_hidden, 
    // 					num_neurons_hidden, num_output);
    net.create_standard_array(num_layers, layer_arr); 

    net.set_learning_rate(learning_rate);

    net.set_activation_steepness_hidden(0.5);
    net.set_activation_steepness_output(0.5);

    net.set_activation_function_layer(FANN::SIGMOID, 1);
    net.set_activation_function_layer(FANN::GAUSSIAN, 2);
    net.set_activation_function_output(FANN::LINEAR);

    net.create_from_file(m_net_file);

    // net.set_training_algorithm(FANN::TRAIN_BATCH);

    net.print_parameters();

    FANN::training_data data;
    if (data.read_train_from_file(m_tdata_file))
    {
    	data.shuffle_train_data();

        fann_type **train_dat;
        fann_type **out_dat;
        train_dat = data.get_input();
        out_dat = data.get_output();

        // Initialize and train the network with the data
        net.init_weights(data);

        net.train_on_data(data, max_epochs,
        				  epochs_between_reports, desired_error);

        //////////////////////TESTING///////////////////////

		// open file
		std::ofstream file;
		file.open(m_test_file);

        for(double i = 0.0; i < 3.0; i += 0.1) {
        	for(double j = 0.0; j < 3.0; j += 0.1) {
        		double vec[6] = {j/3.0, i/3.0, 0.0, 2.9/3.0, 1.5/3.0, 0.0}; 
    	        // Run the network on the test data
	            fann_type *calc_out = net.run(vec);

	            file << *calc_out << " ";
        	}
        	file << "\n";
        }

        net.save(m_net_file);
    }
}

