/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#include <boost/multi_array.hpp>
#include <libaction/human.hpp>
#include <libaction/motion/single/estimator.hpp>
#include <libaction/still/single/estimator.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

/// @example action_motion.cpp

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

static std::unique_ptr<boost::multi_array<uint8_t, 3>> motion_callback(
	const std::string &image_file_prefix,
	const std::string &image_file_suffix,
	size_t image_height, size_t image_width, size_t channels, size_t pos
) {
	// read the image
	auto image = read_image(
		image_file_prefix + std::to_string(pos) + image_file_suffix,
		image_height, image_width, channels);
	return image;
}

int main(int argc, char *argv[])
{
	if (argc != 9) {
		std::cerr << "Usage: <raw image files prefix> <raw image files suffix> "
			"<number of images> <image height> <image width> "
			"<graph file> <graph height> <graph width>"
			<< std::endl << std::endl
			<< "For example, if <raw image files prefix> is \"image\", "
			"<raw image files suffix> is \".raw\" and <number of images> is 3, "
			"then the image sequence is image0.raw, image1.raw, and "
			"image2.raw." << std::endl;
		return EXIT_FAILURE;
	}

	try {
		const size_t channels = 3;
		const size_t fuzz_range = 4;
		const size_t fuzz_rate = 1;

		const std::string image_file_prefix = argv[1];
		const std::string image_file_suffix = argv[2];
		const unsigned long num_images = std::stoul(argv[3]);
		const size_t image_height = std::stoul(argv[4]);
		const size_t image_width = std::stoul(argv[5]);
		const std::string graph_file = argv[6];
		const size_t graph_height = std::stoul(argv[7]);
		const size_t graph_width = std::stoul(argv[8]);

		if (num_images == 0) {
			throw std::runtime_error("<number of images> is zero");
		}

		// initialize the single pose estimator
		libaction::still::single::Estimator<float> still_estimator(
			graph_file, 0, graph_height, graph_width, channels);

		// initialize the single motion estimator
		libaction::motion::single::Estimator motion_estimator;

		// initialize the callback
		using callback_type = std::function<
			std::unique_ptr<boost::multi_array<uint8_t, 3>>(size_t pos)>;
		callback_type callback(std::bind(
			&motion_callback,
			image_file_prefix, image_file_suffix,
			image_height, image_width, channels,
			std::placeholders::_1));

		auto time_before = std::chrono::steady_clock::now();

		for (size_t i = 0; i < num_images; i++) {
			// do estimation
			auto humans = motion_estimator.estimate(i, num_images,
				fuzz_range, fuzz_rate, still_estimator, callback);

			// show results
			std::cout << "======== Image #" << i << " ========" << std::endl;
			for (auto &human: *humans) {
				std::cout << "Human #" << human.first << std::endl;
				auto &body_parts = human.second.body_parts();
				for (auto &part: body_parts) {
					std::cout << static_cast<int>(part.first) << ": "
						<< part.second.x() * image_height << ","
						<< part.second.y() * image_width << std::endl;
				}
				std::cout << std::endl;
			}
		}

		auto time_after = std::chrono::steady_clock::now();

		// show elapsed time
		auto elapsed = std::chrono::duration_cast<
			std::chrono::microseconds>(time_after - time_before).count();
		std::cout << "Elapsed: " << elapsed << std::endl;
		std::cout << "Average: "
			<< static_cast<double>(elapsed) / static_cast<double>(num_images)
			<< std::endl;
	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
