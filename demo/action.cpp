#include <libaction/estimator.hpp>
#include <libaction/image.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

std::unique_ptr<uint8_t[]> read_image(const std::string &file,
	size_t height, size_t width, size_t channels)
{
	std::ifstream f(file, f.binary);
	if (!f.is_open())
		throw std::runtime_error("failed to open image file");

	const auto size = libaction::image::size(height, width, channels);

	auto image = std::unique_ptr<uint8_t[]>(new uint8_t[size]);
	f.read(reinterpret_cast<char*>(image.get()), size);

	if (static_cast<size_t>(f.gcount()) < size)
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
		estimator.estimate<uint8_t>(image.get(), im[0], im[1], im[2]);

	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
