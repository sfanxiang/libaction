#include <boost/multi_array.hpp>
#include <libaction/estimator.hpp>
#include <libaction/image.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

std::unique_ptr<boost::multi_array<uint8_t, 3>> read_image(
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
	const size_t height = 256, width = 144, channels = 3;

	try {
		libaction::Estimator<float> estimator("graph_tflite.tflite", 0,
			height, width, channels);

		const size_t im[] = {232, 217, 3};
		auto image = read_image("p1.raw", im[0], im[1], im[2]);
		estimator.estimate(*image);

	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
