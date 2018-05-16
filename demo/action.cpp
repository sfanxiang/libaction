#include <libaction/estimator.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>

int main()
{
	const int height = 256, width = 144, channels = 3;

	try {
		libaction::Estimator estimator("graph_tflite.tflite", 0,
			height, width, channels);

		estimator.get_input();

		auto time = std::chrono::steady_clock::now();

		for (int i = 0; i < 16; i++) {
			estimator.estimate();
			auto now = std::chrono::steady_clock::now();
			std::cerr << std::chrono::duration_cast<std::chrono::microseconds>(now - time).count() << std::endl;
			time = now;
		}

		estimator.get_output();

	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
