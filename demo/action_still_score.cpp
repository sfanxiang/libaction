/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

/**
 * @example action_still_score.cpp
 */

#include <libaction/motion/multi/deserialize.hpp>
#include <libaction/still/single/score.hpp>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

static std::unique_ptr<std::vector<uint8_t>> read_file(
	const std::string &file, size_t max)
{
	FILE *f = std::fopen(file.c_str(), "rb");
	if (!f)
		throw std::runtime_error("failed to open file");

	auto data = std::unique_ptr<std::vector<uint8_t>>(
		new std::vector<uint8_t>());

	int c;
	while (data->size() < max && (c = fgetc(f)) != EOF) {
		data->push_back(static_cast<uint8_t>(c));
	}

	std::fclose(f);

	return data;
}

int main(int argc, char *argv[])
{
	if (argc != 3) {
		std::cerr << "Usage: <sample file> <standard file>"
			<< std::endl << std::endl;
		return EXIT_FAILURE;
	}

	try {
		const std::size_t max = 0x20000000;

		const std::string sample_file = argv[1];
		const std::string standard_file = argv[2];

		auto sample_data = read_file(sample_file, max);
		auto standard_data = read_file(standard_file, max);

		auto sample =
			libaction::motion::multi::deserialize::deserialize(*sample_data);
		auto standard =
			libaction::motion::multi::deserialize::deserialize(*standard_data);

		if (sample->empty() || standard->empty()) {
			throw std::runtime_error("data is empty");
		}

		for (auto &index_human: *sample->begin()) {
			auto human2_it = standard->begin()->find(index_human.first);

			if (human2_it != standard->begin()->end()) {
				auto &human1 = index_human.second;
				auto &human2 = human2_it->second;

				auto scores = libaction::still::single::score::score(
					human1, human2);

				uint32_t total = 0;
				std::cout << "Human #" << index_human.first << ":" << std::endl;
				for (auto &score: *scores) {
					std::cout << static_cast<int>(score.first.first) << ", "
						<< static_cast<int>(score.first.second) << ": "
						<< static_cast<uint32_t>(score.second) * 100 / 128
						<< std::endl;
					total += static_cast<uint32_t>(score.second);
				}
				if (!scores->empty()) {
					std::cout << "average: "
						<< total * 100 / 128 / scores->size() << std::endl;
				}
				std::cout << std::endl;
			}
		}
	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
