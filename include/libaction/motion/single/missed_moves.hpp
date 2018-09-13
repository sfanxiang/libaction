/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__SINGLE__MISSED_MOVES_HPP_
#define LIBACTION__MOTION__SINGLE__MISSED_MOVES_HPP_

#include "../../body_part.hpp"

#include <algorithm>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <tuple>
#include <utility>

namespace libaction
{
namespace motion
{
namespace single
{
namespace missed_moves
{

/// Find missed moves in a consecutive list of scores.

/// @param[in]  score_list  A consecutive list of scores of the form List<Map<
///                         std::pair<libaction::BodyPart::PartIndex,
///                         libaction::BodyPart::PartIndex>, Integer>>.
/// @param[in]  threshold   The threshold lower than which is a potentially
///                         missed move.
/// @return                 A list of missed moves. The value of the map
///                         (std::pair<std::uint32_t, std::uint8_t>) indicates
///                         the number of frames for a missed move and the mean
///                         score. Missed moves are recorded at their last
///                         frame and span over
///                         [current frame + 1 - number of frames,
///                         current frame].
/// @sa                     still::single::score
template<typename ScoreList>
std::unique_ptr<std::list<std::map<std::pair<
		libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
	std::pair<std::uint32_t, std::uint8_t>>>>
missed_moves(const ScoreList &score_list, std::uint8_t threshold)
{
	if (score_list.size() > std::numeric_limits<std::uint32_t>::max() - 4)
		throw std::runtime_error("score list too long");

	std::map<std::tuple<std::uint32_t, std::uint32_t, std::uint64_t>,
		std::pair<libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>
			> record;
	// tuple: end, start, score_sum_from_start_to_end
	std::map<std::pair<libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
			std::tuple<std::uint32_t, std::uint32_t, std::uint64_t, std::uint64_t>> track;
	// tuple: end, start, score_sum_from_start_to_end, score_sum_from_start_to_current

	std::uint32_t i = 0;
	for (auto &score: score_list) {
		for (auto it = track.begin(); it != track.end(); ) {
			if (score.find(it->first) == score.end()) {
				// missing part
				if (std::get<3>(it->second) + 128 <
					static_cast<std::uint64_t>(threshold) *
						(i - std::get<1>(it->second) + 1)) {
					std::get<3>(it->second) += 128;
				} else {
					record[std::make_tuple(std::get<0>(it->second),
							std::get<1>(it->second),
							std::get<2>(it->second))]
						= it->first;
					it = track.erase(it);
					continue;
				}
			}
			++it;
		}
		for (auto &part: score) {
			std::uint8_t part_score = 0;
			if (part.second >= 0 && part.second <= 128)
				part_score = part.second;
			else if (part.second > 128)
				part_score = 128;

			auto find = track.find(part.first);
			if (find != track.end()) {
				if (part_score < threshold) {
					std::get<0>(find->second) = i;
					std::get<3>(find->second) += part_score;
					std::get<2>(find->second) = std::get<3>(find->second);
				} else {
					if (std::get<3>(find->second) + part_score <
						static_cast<std::uint64_t>(threshold) *
							(i - std::get<1>(find->second) + 1)) {
						std::get<3>(find->second) += part_score;
					} else {
						record[std::make_tuple(std::get<0>(find->second),
								std::get<1>(find->second),
								std::get<2>(find->second))]
							= find->first;
						track.erase(find);
					}
				}
			} else {
				if (part_score < threshold) {
					track[part.first] = std::make_tuple(i, i, part_score, part_score);
				}
			}
		}
		i++;
	}
	for (auto &track_item: track) {
		record[std::make_tuple(std::get<0>(track_item.second),
				std::get<1>(track_item.second),
				std::get<2>(track_item.second))]
			= track_item.first;
	}

	using return_type = std::list<std::map<std::pair<
			libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
		std::pair<std::uint32_t, std::uint8_t>>>;
	auto moves = std::unique_ptr<return_type>(new return_type());

	for (std::uint32_t i = 0; i < score_list.size(); i++) {
		moves->emplace_back();
		while (!record.empty() && std::get<0>(record.begin()->first) == i) {
			std::uint32_t length =
				std::get<0>(record.begin()->first) - std::get<1>(record.begin()->first) + 1;
			std::uint64_t score = std::get<2>(record.begin()->first) / length;
			std::uint8_t score_mean = 128;
			if (score < 128)
				score_mean = score;

			moves->rbegin()->insert(std::make_pair(record.begin()->second,
				std::make_pair(length, score_mean)));
		}
	}

	return moves;
}

}
}
}
}

#endif
