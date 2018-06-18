/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__DETAIL__FUZZ_HPP_
#define LIBACTION__MOTION__DETAIL__FUZZ_HPP_

#include "../../body_part.hpp"
#include "../../human.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace libaction
{
namespace motion
{
namespace detail
{
namespace fuzz
{

namespace
{

inline std::vector<std::vector<libaction::BodyPart::PartIndex>>
other_ends(libaction::BodyPart::PartIndex end)
{
	using index = libaction::BodyPart::PartIndex;

	switch (end) {
	case index::nose:
		return { { index::neck }, { index::eye_r, index::eye_l } };
		break;
	case index::neck:
		return { { index::nose }, { index::shoulder_r, index::shoulder_l } };
		break;
	case index::shoulder_r:
		return { { index::shoulder_l }, { index::hip_r }, { index::nose } };
		break;
	case index::elbow_r:
		return { { index::shoulder_r }, { index::wrist_r } };
		break;
	case index::wrist_r:
		return { { index::elbow_r } };
		break;
	case index::shoulder_l:
		return { { index::shoulder_r }, { index::hip_l }, { index::nose } };
		break;
	case index::elbow_l:
		return { { index::shoulder_l }, { index::wrist_l } };
		break;
	case index::wrist_l:
		return { { index::elbow_l } };
		break;
	case index::hip_r:
		return { { index::hip_l }, { index::shoulder_r }, { index::knee_r } };
		break;
	case index::knee_r:
		return { { index::hip_r } };
		break;
	case index::ankle_r:
		return { { index::knee_r } };
		break;
	case index::hip_l:
		return { { index::hip_r }, { index::shoulder_l }, { index::knee_l } };
		break;
	case index::knee_l:
		return { { index::hip_l } };
		break;
	case index::ankle_l:
		return { { index::knee_l } };
		break;
	case index::eye_r:
		return { { index::eye_l }, { index::nose } };
		break;
	case index::eye_l:
		return { { index::eye_r }, { index::nose } };
		break;
	case index::ear_r:
		return { { index::ear_l }, { index::nose } };
		break;
	case index::ear_l:
		return { { index::ear_r }, { index::nose } };
		break;
	case index::end:
		return {};
		break;
	default:
		return {};
		break;
	}
}

inline bool has_part(const libaction::Human &human,
	libaction::BodyPart::PartIndex part_index)
{
	return human.body_parts().find(part_index) != human.body_parts().end();
}

inline bool has_end(const libaction::Human &human,
	const std::vector<libaction::BodyPart::PartIndex> &end)
{
	for (auto &end_part: end) {
		if (human.body_parts().find(end_part) ==
				human.body_parts().end()) {
			return false;
		}
	}
	return true;
}

template<typename HumanPtr>
inline std::tuple<size_t, size_t, size_t> search_for_ends(
	size_t fuzz_range, size_t fuzz_rate,
	libaction::BodyPart::PartIndex part_index,
	const std::vector<std::vector<libaction::BodyPart::PartIndex>> &ends,
	std::function<HumanPtr(size_t relative_pos, bool left)> &callback)
{
	if (fuzz_rate == 0) {
		throw std::runtime_error("fuzz_rate == 0");
	}

	std::tuple<size_t, size_t, size_t> result{0, 0, 0};

	if (fuzz_range < fuzz_rate || fuzz_range - fuzz_rate < fuzz_rate) {
		// impossible
		return result;
	}

	size_t ends_index_next = 0;

	// look into each end
	for (auto &end: ends) {
		size_t ends_index = ends_index_next;
		ends_index_next++;

		// try to find the first pose that contains this end on the left
		bool found = false;
		size_t loff = fuzz_rate;
		for (; loff < fuzz_range &&
				(std::get<0>(result) == 0 ||
				std::get<0>(result) + std::get<1>(result) > loff + fuzz_rate);
			loff += fuzz_rate)
		{
			auto human = callback(loff, true);
			if (!human) {
				// reached the left bound
				found = false;
				break;
			}
			if (has_part(*human, part_index) && has_end(*human, end)) {
				found = true;
				break;
			}
		}
		if (!found)
			continue;

		// try to find the first pose that contains this end on the right
		found = false;
		size_t roff = fuzz_rate;
		for(; roff <= fuzz_range - loff &&
				(std::get<0>(result) == 0 ||
				std::get<0>(result) + std::get<1>(result) > loff + roff);
			roff += fuzz_rate)
		{
			auto human = callback(roff, false);
			if (!human) {
				// reached the right bound
				found = false;
				break;
			}
			if (has_part(*human, part_index) && has_end(*human, end)) {
				found = true;
				break;
			}
		}
		if (!found)
			continue;

		result = std::make_tuple(loff, roff, ends_index);

		if (loff == fuzz_rate && roff == fuzz_rate) {
			// no better finding possible
			break;
		}
	}

	return result;
}

inline libaction::BodyPart get_fuzz_part(
	size_t left_offset,
	size_t right_offset,
	const libaction::Human &left,
	const libaction::Human &right,
	const libaction::Human &target,
	libaction::BodyPart::PartIndex part_index,
	const std::vector<libaction::BodyPart::PartIndex> &end)
{
	float score = 1.0;

	// left

	float x_left_end = 0, y_left_end = 0;
	for (auto end_index: end) {
		auto &body_part = left.body_parts().at(end_index);
		x_left_end += body_part.x() / static_cast<float>(end.size());
		y_left_end += body_part.y() / static_cast<float>(end.size());
		score *= body_part.score();
	}

	auto &left_body_part = left.body_parts().at(part_index);
	float x_left_part = left_body_part.x();
	float y_left_part = left_body_part.y();
	score *= left_body_part.score();

	auto x_left_diff = x_left_part - x_left_end;
	auto y_left_diff = y_left_part - y_left_end;

	float left_angle = 0.0f;
	if (y_left_diff != 0.0f || x_left_diff != 0.0f)
		left_angle = std::atan2(y_left_diff, x_left_diff);
	auto left_length = std::sqrt(
		x_left_diff * x_left_diff + y_left_diff * y_left_diff);

	// right

	float x_right_end = 0, y_right_end = 0;
	for (auto end_index: end) {
		auto &body_part = right.body_parts().at(end_index);
		x_right_end += body_part.x() / static_cast<float>(end.size());
		y_right_end += body_part.y() / static_cast<float>(end.size());
		score *= body_part.score();
	}

	auto &right_body_part = right.body_parts().at(part_index);
	float x_right_part = right_body_part.x();
	float y_right_part = right_body_part.y();
	score *= right_body_part.score();

	auto x_right_diff = x_right_part - x_right_end;
	auto y_right_diff = y_right_part - y_right_end;

	float right_angle = 0.0f;
	if (y_right_diff != 0.0f || x_right_diff != 0.0f)
		right_angle = std::atan2(y_right_diff, x_right_diff);
	auto right_length = std::sqrt(
		x_right_diff * x_right_diff + y_right_diff * y_right_diff);

	// fix angle

	if (y_left_diff == 0.0f && x_left_diff == 0.0f && (y_right_diff != 0.0f || x_right_diff != 0.0f))
		left_angle = right_angle;
	else if (y_right_diff == 0.0f && x_right_diff == 0.0f && (y_left_diff != 0.0f || x_left_diff != 0.0f))
		right_angle = left_angle;

	// calculate results

	auto loff = static_cast<float>(left_offset);
	auto roff = static_cast<float>(right_offset);
	auto toff = loff + roff;

	auto angle = left_angle / toff * roff + right_angle / toff * loff;
	auto length = left_length / toff * roff + right_length / toff * loff;

	float x_end = 0, y_end = 0;
	for (auto end_index: end) {
		auto &body_part = target.body_parts().at(end_index);
		x_end += body_part.x() / static_cast<float>(end.size());
		y_end += body_part.y() / static_cast<float>(end.size());
		score *= body_part.score();
	}

	float x = x_end + length * std::cos(angle);
	float y = y_end + length * std::sin(angle);

	return libaction::BodyPart(part_index, x, y, score);
}

}

template<typename Needed, typename HumanPtr>
inline std::unique_ptr<libaction::Human> fuzz(
	size_t fuzz_range, size_t fuzz_rate,
	const Needed &needed,
	std::function<HumanPtr(size_t relative_pos, bool left)> &callback)
{
	HumanPtr target = callback(0, false);
	if (!target) {
		throw std::runtime_error("fuzz target not found");
	}

	std::vector<libaction::BodyPart> add;

	for (auto &part_index: needed) {
		if (has_part(*target, part_index)) {
			// if part_index already exists in target, skip
			continue;
		}

		auto all_ends = other_ends(part_index);
		decltype(all_ends) ends;	// ends that exist in target
		for (auto &end: all_ends) {
			if (has_end(*target, end)) {
				ends.push_back(end);
			}
		}

		auto result = search_for_ends(fuzz_range, fuzz_rate, part_index, ends, callback);

		if (std::get<0>(result) != 0) {
			auto left = callback(std::get<0>(result), true);
			auto right = callback(std::get<1>(result), false);
			auto &end = ends[std::get<2>(result)];

			add.push_back(get_fuzz_part(
				std::get<0>(result), std::get<1>(result),
				*left, *right, *target, part_index, end));
		}
	}

	if (add.empty()) {
		// no part is added
		return std::unique_ptr<libaction::Human>(new libaction::Human(*target));
	}

	for (auto &part: target->body_parts()) {
		add.push_back(part.second);
	}

	return std::unique_ptr<libaction::Human>(new libaction::Human(add));
}

}
}
}
}

#endif
