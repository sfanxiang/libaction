/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

/**
 * @example action_missed.cpp
 */

#include <libaction/body_part.hpp>
#include <libaction/motion/multi/deserialize.hpp>
#include <libaction/motion/single/missed_moves.hpp>
#include <libaction/still/single/score.hpp>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <list>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

static std::unique_ptr<std::vector<std::uint8_t>> read_file(
	const std::string &file, std::size_t max)
{
	FILE *f = std::fopen(file.c_str(), "rb");
	if (!f)
		throw std::runtime_error("failed to open file");

	auto data = std::unique_ptr<std::vector<std::uint8_t>>(
		new std::vector<std::uint8_t>());

	int c;
	while (data->size() < max && (c = std::fgetc(f)) != EOF) {
		if (std::ferror(f)) {
			std::fclose(f);
			throw std::runtime_error("failed to read file");
		}
		data->push_back(static_cast<std::uint8_t>(c));
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

		std::list<std::map<std::pair<
			libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
				std::uint8_t>> score_list;

		auto sample_it = sample->begin();
		auto standard_it = standard->begin();

		for (std::size_t i = 0; i < sample->size() && i < standard->size();
			i++, sample_it++, standard_it++)
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

			score_list.emplace_back(std::move(*scores));
		}

		auto result = libaction::motion::single::missed_moves::
			missed_moves(score_list, 108, 32);

		std::size_t i = 0;
		for (auto &frame: *result) {
			i++;

			if (frame.empty())
				continue;

			std::cout << "======== Image #" << i - 1 << " ========" << std::endl;
			for (auto &part: frame) {
				std::cout << i - 1 + 1 - part.second.first << " - " << i - 1
					<< ": "
					<< static_cast<int>(part.first.first) << ", "
					<< static_cast<int>(part.first.second) << ": "
					<< static_cast<std::uint32_t>(part.second.second)
						* 100 / 128
					<< std::endl;
			}
			std::cout << std::endl;
		}
	} catch (std::exception &e) {
		std::cerr << "Error: " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
