#include <boost/multi_array.hpp>
#include <libaction/human.hpp>
#include <libaction/still/multi/estimator.hpp>
#include <libaction/still/single/estimator.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

/// @example action_still.cpp

static std::unique_ptr<boost::multi_array<uint8_t, 3>> read_image(
	const std::string &file, size_t height, size_t width, size_t channels)
{
	std::ifstream f(file, f.binary);
	if (!f.is_open())
		throw std::runtime_error("failed to open image file");

	auto image = std::unique_ptr<boost::multi_array<uint8_t, 3>>(
		new boost::multi_array<uint8_t, 3>(
			boost::extents[height][width][channels]));
	f.read(reinterpret_cast<char*>(image->data()), image->num_elements());

	if (static_cast<size_t>(f.gcount()) < image->num_elements())
		throw std::runtime_error("image file too small");

	return image;
}

int main()
{
	const size_t height = 224, width = 128, channels = 3;

	try {
		libaction::still::single::Estimator<float> estimator(
			"posenet_tflite.tflite", 0, height, width, channels);

		const size_t im[] = {232, 217, 3};
		auto image = read_image("p1.raw", im[0], im[1], im[2]);

		for (int i = 0; i < 20; i++) {
			auto time = std::chrono::steady_clock::now();

			auto humans = estimator.estimate(*image);

			for (auto &human: *humans) {
				auto &body_parts = human.body_parts();
				for (auto &part: body_parts) {
					std::cout << static_cast<int>(part.first) << ": "
						<< part.second.x() * im[0] << ","
						<< part.second.y() * im[1] << std::endl;
				}
				std::cout << std::endl;
			}

			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<
				std::chrono::microseconds>(now - time).count();
			std::cout << "Elapsed: " << elapsed << std::endl << std::endl;
		}
	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
