#include <libaction/estimator.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>

int main()
{
	const int height = 368, width = 432, channels = 3;

	try {
		libaction::Estimator estimator("graph_tflite.tflite", 1,
			height, width, channels);

		std::cerr << "...." << std::endl;

		estimator.get_input();

		std::cerr << "...." << std::endl;

		estimator.estimate();

		std::cerr << "...." << std::endl;

		estimator.get_output();

	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
