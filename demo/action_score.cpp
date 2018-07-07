/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

/**
 * @example action_still_score.cpp
 */

#include <libaction/body_part.hpp>
#include <libaction/motion/multi/deserialize.hpp>
#include <libaction/still/single/score.hpp>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
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

		if (sample->size() > standard->size()) {
			throw std::runtime_error("sample size too large");
		}

		std::map<std::pair<libaction::BodyPart::PartIndex,
			libaction::BodyPart::PartIndex>, uint64_t> part_sums;
		std::map<std::pair<libaction::BodyPart::PartIndex,
			libaction::BodyPart::PartIndex>, uint32_t> part_counts;
		uint64_t frame_sum = 0;
		uint32_t frame_count = 0;

		auto sample_it = sample->begin();
		auto standard_it = standard->begin();

		for (size_t i = 0; i < sample->size(); i++, sample_it++, standard_it++)
		{
			auto &sample_frame = *sample_it;
			auto &standard_frame = *standard_it;

			auto human1_it = sample_frame.find(0);
			if (human1_it == sample_frame.end())
				continue;

			auto human2_it = standard_frame.find(0);
			if (human2_it == standard_frame.end())
				continue;

			auto &human1 = human1_it->second;
			auto &human2 = human2_it->second;

			auto scores = libaction::still::single::score::score(
				human1, human2);

			uint32_t sum = 0;
			std::cout << "======== Image #" << i << " ========" << std::endl;
			for (auto &score: *scores) {
				std::cout << static_cast<int>(score.first.first) << ", "
					<< static_cast<int>(score.first.second) << ": "
					<< static_cast<uint32_t>(score.second) * 100 / 128
					<< std::endl;
				sum += static_cast<uint32_t>(score.second);
				part_sums[score.first] += score.second;
				part_counts[score.first]++;
			}
			if (!scores->empty()) {
				uint32_t average = sum * 100 / 128 / scores->size();
				std::cout << "average: " << average << std::endl;
				frame_sum += sum / scores->size();
				frame_count++;
			}
			std::cout << std::endl;
		}

		std::cout << "Part average:" << std::endl;
		for (auto &score: part_sums) {
			uint64_t current = score.second / part_counts[score.first];
			std::cout << static_cast<int>(score.first.first) << ", "
				<< static_cast<int>(score.first.second) << ": "
				<< current * 100 / 128
				<< std::endl;
		}
		std::cout << "Frame average: " <<
			(frame_count != 0 ? frame_sum * 100 / 128 / frame_count : 0)
			<< std::endl;
	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
