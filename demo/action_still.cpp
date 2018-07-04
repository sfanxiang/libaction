/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

/// @example action_still.cpp

#include <boost/multi_array.hpp>
#include <libaction/human.hpp>
#include <libaction/motion/multi/serialize.hpp>
#include <libaction/still/single/estimator.hpp>
#include <chrono>
#include <cstdlib>
#include <fstream>
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

int main(int argc, char *argv[])
{
	if (argc != 9) {
		std::cerr << "Usage: <raw image file> <image height> <image width> "
			"<graph file> <graph height> <graph width> <threads> <save file>"
			<< std::endl << std::endl
			<< "If <threads> is 0, the number of threads for the estimation "
			"will be automatically decided." << std::endl << std::endl;
		return EXIT_FAILURE;
	}

	try {
		const size_t channels = 3;

		const std::string image_file = argv[1];
		const size_t image_height = std::stoul(argv[2]);
		const size_t image_width = std::stoul(argv[3]);
		const std::string graph_file = argv[4];
		const size_t graph_height = std::stoul(argv[5]);
		const size_t graph_width = std::stoul(argv[6]);
		const size_t threads = std::stoul(argv[7]);
		const std::string save_file = argv[8];

		// initialize the single pose estimator
		libaction::still::single::Estimator<float> estimator(
			graph_file, threads, graph_height, graph_width, channels);

		// read the image
		auto image = read_image(image_file, image_height, image_width,
			channels);

		auto time_before = std::chrono::steady_clock::now();
		// do estimation
		auto humans = estimator.estimate(*image);
		auto time_after = std::chrono::steady_clock::now();

		// show results
		for (auto &human: *humans) {
			auto &body_parts = human.body_parts();
			for (auto &part: body_parts) {
				std::cout << static_cast<int>(part.first) << ": "
					<< part.second.x() * image_height << ","
					<< part.second.y() * image_width << std::endl;
			}
			std::cout << std::endl;
		}


		if (!save_file.empty()) {
			std::list<std::pair<std::size_t, libaction::Human>> human_map;
			size_t i = 0;
			for (auto &human: *humans) {
				human_map.push_back(std::make_pair(i, std::move(human)));
				i++;
			}

			auto se = libaction::motion::multi::serialize::serialize(
				std::vector<std::list<std::pair<
					std::size_t, libaction::Human>>>{ std::move(human_map) });

			std::ofstream ofs(save_file, std::ios::binary);
			ofs.write(reinterpret_cast<char*>(se->data()), se->size());
			ofs.close();
		}

		// show elapsed time
		auto elapsed = std::chrono::duration_cast<
			std::chrono::microseconds>(time_after - time_before).count();
		std::cout << "Elapsed: " << elapsed << std::endl;
	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
