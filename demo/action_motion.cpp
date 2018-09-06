/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

/**
 * @example action_motion.cpp
 */

#include <boost/multi_array.hpp>
#include <libaction/human.hpp>
#include <libaction/motion/multi/serialize.hpp>
#include <libaction/motion/single/estimator.hpp>
#include <libaction/still/single/estimator.hpp>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

static std::unique_ptr<const boost::multi_array<uint8_t, 3>> read_image(
	const std::string &file, size_t height, size_t width, size_t channels)
{
	FILE *f = std::fopen(file.c_str(), "rb");
	if (!f)
		throw std::runtime_error("failed to open image file");

	auto image = std::unique_ptr<boost::multi_array<uint8_t, 3>>(
		new boost::multi_array<uint8_t, 3>(
			boost::extents[height][width][channels]));
	auto count = std::fread(image->data(), image->num_elements(), 1, f);

	std::fclose(f);

	if (count < 1)
		throw std::runtime_error("image file too small");

	return image;
}

static std::unique_ptr<const boost::multi_array<uint8_t, 3>> motion_callback(
	const std::string &image_file_prefix,
	const std::string &image_file_suffix,
	size_t image_height, size_t image_width, size_t channels,
	size_t pos, bool /* last_image_access */
) {
	// read the image
	auto image = read_image(
		image_file_prefix + std::to_string(pos) + image_file_suffix,
		image_height, image_width, channels);
	return image;
}

int main(int argc, char *argv[])
{
	if (argc != 13) {
		std::cerr << "Usage: <raw image files prefix> <raw image files suffix> "
			"<number of images> <image height> <image width> "
			"<graph file> <graph height> <graph width> <zoom> "
			"<concurrent estimations> <threads per estimation> <save file>"
			<< std::endl << std::endl
			<< "For example, if <raw image files prefix> is \"image\", "
			"<raw image files suffix> is \".raw\" and <number of images> is 3, "
			"then the image sequence is image0.raw, image1.raw, and "
			"image2.raw." << std::endl << std::endl
			<< "If <threads per estimation> is 0, the number of threads per "
			"estimation will be automatically decided."
			<< std::endl << std::endl;
		return EXIT_FAILURE;
	}

	try {
		const size_t channels = 3;
		const size_t fuzz_range = 7;
		const size_t zoom_range = 3;
		const size_t zoom_rate = 1;

		const std::string image_file_prefix = argv[1];
		const std::string image_file_suffix = argv[2];
		const unsigned long num_images = std::stoul(argv[3]);
		const size_t image_height = std::stoul(argv[4]);
		const size_t image_width = std::stoul(argv[5]);
		const std::string graph_file = argv[6];
		const size_t graph_height = std::stoul(argv[7]);
		const size_t graph_width = std::stoul(argv[8]);
		const bool zoom = (std::stoul(argv[9]) != 0);
		const size_t concurrent_estimations = std::stoul(argv[10]);
		const size_t threads_per_estimation = std::stoul(argv[11]);
		const std::string save_file = argv[12];

		if (num_images == 0)
			throw std::runtime_error("<number of images> is 0");
		if (concurrent_estimations == 0)
			throw std::runtime_error("<concurrent estimations> is 0");

		std::vector<std::unique_ptr<libaction::still::single::Estimator<float>>>
			still_estimators;
		std::vector<libaction::still::single::Estimator<float>*>
			still_estimator_ptrs;

		// initialize the single pose estimators
		for (size_t i = 0; i < concurrent_estimations; i++) {
			still_estimators.push_back(
				std::unique_ptr<libaction::still::single::Estimator<float>>(
					new libaction::still::single::Estimator<float>(
						graph_file, threads_per_estimation,
						graph_height, graph_width, channels)));
			still_estimator_ptrs.push_back(still_estimators.back().get());
		}

		// initialize the single motion estimator
		libaction::motion::single::Estimator motion_estimator;

		// initialize the callback
		using callback_type = std::function<
			std::unique_ptr<const boost::multi_array<uint8_t, 3>>(
				size_t pos, bool last_image_access)>;
		callback_type callback(std::bind(
			&motion_callback,
			image_file_prefix, image_file_suffix,
			image_height, image_width, channels,
			std::placeholders::_1, std::placeholders::_2));

		std::list<std::unordered_map<std::size_t, libaction::Human>> action;

		auto time_before = std::chrono::steady_clock::now();

		for (size_t i = 0; i < num_images; i++) {
			// do estimation
			auto humans = motion_estimator.estimate(i, num_images,
				fuzz_range, { }, true,
				zoom, zoom_range, zoom_rate,
				still_estimator_ptrs, still_estimator_ptrs, callback);

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
			}
			std::cout << std::endl;

			action.push_back(std::move(*humans));
		}

		auto time_after = std::chrono::steady_clock::now();

		if (!save_file.empty()) {
			auto se = libaction::motion::multi::serialize::serialize(action);

			FILE *f = std::fopen(save_file.c_str(), "wb");
			if (f) {
				std::fwrite(se->data(), se->size(), 1, f);
				std::fclose(f);
			}
		}

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
