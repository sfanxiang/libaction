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

// TODO: rel_recipe, abs_double_recipe, abs_single_recipe

inline const std::vector<std::pair<
	libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>> &
double_recipe()
{
	using id = libaction::BodyPart::PartIndex;

	// avoid the id:: prefix
	auto nose = id::nose;
	auto neck = id::neck;
	auto shoulder_r = id::shoulder_r;
	auto elbow_r = id::elbow_r;
	auto wrist_r = id::wrist_r;
	auto shoulder_l = id::shoulder_l;
	auto elbow_l = id::elbow_l;
	auto wrist_l = id::wrist_l;
	auto hip_r = id::hip_r;
	auto knee_r = id::knee_r;
	auto ankle_r = id::ankle_r;
	auto hip_l = id::hip_l;
	auto knee_l = id::knee_l;
	auto ankle_l = id::ankle_l;
	auto eye_r = id::eye_r;
	auto eye_l = id::eye_l;
	auto ear_r = id::ear_r;
	auto ear_l = id::ear_l;

	static const std::vector<std::pair<
		libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex
	>> recipe{
		// same name
		{ eye_r, eye_l },
		{ eye_l, eye_r },
		{ shoulder_r, shoulder_l },
		{ shoulder_l, shoulder_r },
		{ ear_r, ear_l },
		{ ear_l, ear_r },
		{ hip_r, hip_l },
		{ hip_l, hip_r },
		// same side / both no side
		{ eye_r, ear_r },
		{ eye_l, ear_l },
		{ knee_r, ankle_r },
		{ knee_l, ankle_l },
		{ shoulder_r, hip_r },
		{ shoulder_l, hip_l },
		{ hip_r, knee_r },
		{ hip_l, knee_l },
		{ knee_r, hip_r },
		{ knee_l, hip_l },
		{ hip_r, shoulder_r },
		{ hip_l, shoulder_l },
		{ ankle_r, knee_r },
		{ ankle_l, knee_l },
		{ ear_r, eye_r },
		{ ear_l, eye_l },
		{ shoulder_r, elbow_r },
		{ shoulder_l, elbow_l },
		{ elbow_r, shoulder_r },
		{ elbow_l, shoulder_l },
		{ nose, neck },
		{ neck, nose },
		{ elbow_r, wrist_r },
		{ elbow_l, wrist_l },
		{ wrist_r, elbow_r },
		{ wrist_l, elbow_l },
		// side -> no side
		{ eye_r, nose },
		{ eye_l, nose },
		{ ear_r, nose },
		{ ear_l, nose },
		{ shoulder_r, neck },
		{ shoulder_l, neck },
		{ eye_r, neck },
		{ eye_l, neck },
		{ ear_r, neck },
		{ ear_l, neck },
		{ hip_r, neck },
		{ hip_l, neck },
		// no side -> side
		{ neck, shoulder_r },
		{ neck, shoulder_l },
		{ nose, ear_r },
		{ nose, ear_l },
		{ nose, eye_r },
		{ nose, eye_l },
		{ neck, ear_r },
		{ neck, ear_l },
		{ neck, eye_r },
		{ neck, eye_l },
		// different sides
		{ eye_r, ear_l },
		{ eye_l, ear_r },
		{ shoulder_r, hip_l },
		{ shoulder_l, hip_r },
		{ hip_r, shoulder_l },
		{ hip_l, shoulder_r },
		{ ear_r, eye_l },
		{ ear_l, eye_r }
	};

	return recipe;
}

inline const std::vector<libaction::BodyPart::PartIndex> &
single_recipe()
{
	using id = libaction::BodyPart::PartIndex;

	// avoid the id:: prefix
	auto nose = id::nose;
	auto neck = id::neck;
	auto shoulder_r = id::shoulder_r;
	auto elbow_r = id::elbow_r;
	auto wrist_r = id::wrist_r;
	auto shoulder_l = id::shoulder_l;
	auto elbow_l = id::elbow_l;
	auto wrist_l = id::wrist_l;
	auto hip_r = id::hip_r;
	auto knee_r = id::knee_r;
	auto ankle_r = id::ankle_r;
	auto hip_l = id::hip_l;
	auto knee_l = id::knee_l;
	auto ankle_l = id::ankle_l;
	auto eye_r = id::eye_r;
	auto eye_l = id::eye_l;
	auto ear_r = id::ear_r;
	auto ear_l = id::ear_l;

	static const std::vector<libaction::BodyPart::PartIndex> recipe{
		ankle_r,
		ankle_l,
		neck,
		shoulder_r,
		shoulder_l,
		hip_r,
		hip_l,
		knee_r,
		knee_l,
		nose,
		eye_r,
		eye_l,
		ear_r,
		ear_l,
		elbow_r,
		elbow_l,
		wrist_r,
		wrist_l
	};

	return recipe;
}

inline bool has_part(const libaction::Human &human,
	libaction::BodyPart::PartIndex part_index)
{
	return human.body_parts().find(part_index) != human.body_parts().end();
}

inline bool has_parts(const libaction::Human &human,
	const std::vector<libaction::BodyPart::PartIndex> &parts)
{
	for (auto &part: parts) {
		if (human.body_parts().find(part) == human.body_parts().end()) {
			return false;
		}
	}
	return true;
}

template<typename HumanPtr>
inline std::pair<size_t, size_t> search_for_parts(
	size_t fuzz_range, size_t fuzz_rate,
	const std::vector<libaction::BodyPart::PartIndex> &parts,
	std::function<std::pair<bool, HumanPtr>(size_t relative_pos, bool left)>
		&callback)
{
	if (fuzz_rate == 0) {
		throw std::runtime_error("fuzz_rate == 0");
	}

	if (fuzz_range < fuzz_rate || fuzz_range - fuzz_rate < fuzz_rate) {
		// impossible
		return std::make_pair(0, 0);
	}

	// try to find the first pose that contains parts on the left
	bool found = false;
	size_t loff = fuzz_rate;
	for (; loff < fuzz_range; loff += fuzz_rate)
	{
		bool valid;
		HumanPtr human;
		std::tie(valid, human) = callback(loff, true);
		if (!valid) {
			// reached the left bound
			found = false;
			break;
		}
		if (human && has_parts(*human, parts)) {
			found = true;
			break;
		}
	}
	if (!found)
		return std::make_pair(0, 0);

	// try to find the first pose that contains parts on the right
	found = false;
	size_t roff = fuzz_rate;
	for(; roff <= fuzz_range - loff; roff += fuzz_rate)
	{
		bool valid;
		HumanPtr human;
		std::tie(valid, human) = callback(roff, false);
		if (!valid) {
			// reached the right bound
			found = false;
			break;
		}
		if (human && has_parts(*human, parts)) {
			found = true;
			break;
		}
	}
	if (!found)
		return std::make_pair(0, 0);

	return std::make_pair(loff, roff);
}

inline float get_double_fuzz_score(
	size_t left_offset,
	size_t right_offset,
	const libaction::Human &left,
	const libaction::Human &right,
	const libaction::Human &target,
	libaction::BodyPart::PartIndex source_part_index,
	libaction::BodyPart::PartIndex target_part_index,
	float initial_score = 1.0f)
{
	float &score = initial_score;

	// left
	score *= left.body_parts().at(source_part_index).score();
	score *= left.body_parts().at(target_part_index).score();

	// right
	score *= right.body_parts().at(source_part_index).score();
	score *= right.body_parts().at(target_part_index).score();

	// target
	score *= target.body_parts().at(source_part_index).score();

	// offset
	auto loff = static_cast<float>(left_offset);
	auto roff = static_cast<float>(right_offset);
	auto toff = loff + roff;
	score /= toff;

	return score;
}

inline float get_single_fuzz_score(
	size_t left_offset,
	size_t right_offset,
	const libaction::Human &left,
	const libaction::Human &right,
	libaction::BodyPart::PartIndex target_part_index,
	float initial_score = 1.0f / 3.0f)
{
	float &score = initial_score;

	// left
	score *= left.body_parts().at(target_part_index).score();

	// right
	score *= right.body_parts().at(target_part_index).score();

	// offset
	auto loff = static_cast<float>(left_offset);
	auto roff = static_cast<float>(right_offset);
	auto toff = loff + roff;
	score /= toff;

	return score;
}

inline libaction::BodyPart get_double_fuzz_part(
	size_t left_offset,
	size_t right_offset,
	const libaction::Human &left,
	const libaction::Human &right,
	const libaction::Human &target,
	libaction::BodyPart::PartIndex source_part_index,
	libaction::BodyPart::PartIndex target_part_index,
	float score)
{
	// left

	auto &left_source_body_part = left.body_parts().at(source_part_index);
	auto &left_target_body_part = left.body_parts().at(target_part_index);

	auto x_left_diff = left_target_body_part.x() - left_source_body_part.x();
	auto y_left_diff = left_target_body_part.y() - left_source_body_part.y();

	float left_angle = 0.0f;
	if (y_left_diff != 0.0f || x_left_diff != 0.0f)
		left_angle = std::atan2(y_left_diff, x_left_diff);
	auto left_length = std::sqrt(
		x_left_diff * x_left_diff + y_left_diff * y_left_diff);

	// right

	auto &right_source_body_part = right.body_parts().at(source_part_index);
	auto &right_target_body_part = right.body_parts().at(target_part_index);

	auto x_right_diff = right_target_body_part.x() - right_source_body_part.x();
	auto y_right_diff = right_target_body_part.y() - right_source_body_part.y();

	float right_angle = 0.0f;
	if (y_right_diff != 0.0f || x_right_diff != 0.0f)
		right_angle = std::atan2(y_right_diff, x_right_diff);
	auto right_length = std::sqrt(
		x_right_diff * x_right_diff + y_right_diff * y_right_diff);

	// fix angles

	if (y_left_diff == 0.0f && x_left_diff == 0.0f && (y_right_diff != 0.0f || x_right_diff != 0.0f))
		left_angle = right_angle;
	else if (y_right_diff == 0.0f && x_right_diff == 0.0f && (y_left_diff != 0.0f || x_left_diff != 0.0f))
		right_angle = left_angle;

	// use a reasonable angle for calculating the weighted average
	if (left_angle > 0 && right_angle < 0) {
		if (left_angle - right_angle > (std::acos(-1.0f) /* pi */ )) {
			right_angle += 2 * (std::acos(-1.0f) /* pi */ );
		}
	} else if (left_angle < 0 && right_angle > 0) {
		if (right_angle - left_angle > (std::acos(-1.0f) /* pi */ )) {
			left_angle += 2 * (std::acos(-1.0f) /* pi */ );
		}
	}

	// calculate results

	auto loff = static_cast<float>(left_offset);
	auto roff = static_cast<float>(right_offset);
	auto toff = loff + roff;

	auto angle = left_angle / toff * roff + right_angle / toff * loff;
	auto length = left_length / toff * roff + right_length / toff * loff;

	auto &source_body_part = target.body_parts().at(source_part_index);
	float x = source_body_part.x() + length * std::cos(angle);
	float y = source_body_part.y() + length * std::sin(angle);

	return libaction::BodyPart(target_part_index, x, y, score);
}

inline libaction::BodyPart get_single_fuzz_part(
	size_t left_offset,
	size_t right_offset,
	const libaction::Human &left,
	const libaction::Human &right,
	libaction::BodyPart::PartIndex target_part_index,
	float score)
{
	auto &left_body_part = left.body_parts().at(target_part_index);
	auto &right_body_part = right.body_parts().at(target_part_index);

	auto loff = static_cast<float>(left_offset);
	auto roff = static_cast<float>(right_offset);
	auto toff = loff + roff;

	float x = left_body_part.x() / toff * roff + right_body_part.x() / toff * loff;
	float y = left_body_part.y() / toff * roff + right_body_part.y() / toff * loff;

	return libaction::BodyPart(target_part_index, x, y, score);
}

}

template<typename HumanPtr>
inline std::unique_ptr<libaction::Human> fuzz(
	size_t fuzz_range, size_t fuzz_rate,
	std::function<std::pair<bool, HumanPtr>(size_t relative_pos, bool left)>
		&callback)
{
	std::unique_ptr<libaction::Human> target;

	{
		bool valid;
		HumanPtr original;
		std::tie(valid, original) = callback(0, false);
		if (!valid) {
			throw std::runtime_error("fuzz target not found");
		}

		if (original) {
			target = std::unique_ptr<libaction::Human>(new libaction::Human(
				*original));
		}
	}

	const auto &dbl = double_recipe();
	const auto &sing = single_recipe();

	while (true) {
		std::pair<
			std::pair<size_t, size_t>,
			std::pair<
				libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>
		> dbl_candidate;
		std::pair<
			std::pair<size_t, size_t>,
			libaction::BodyPart::PartIndex
		> sing_candidate;

		bool use_dbl = false, use_sing = false;
		float score = -1.0f;

		if (target) {
			// double recipe
			for (auto &rule: dbl) {
				if (has_part(*target, rule.second) || !has_part(*target, rule.first))
					continue;

				std::vector<libaction::BodyPart::PartIndex> search_for{
					rule.first, rule.second };
				auto search_result = search_for_parts(fuzz_range, fuzz_rate, search_for, callback);
				if (search_result.first == 0)	// not found
					continue;

				auto left = callback(search_result.first, true).second;
				auto right = callback(search_result.second, false).second;

				auto current_score = get_double_fuzz_score(
					search_result.first,
					search_result.second,
					*left,
					*right,
					*target,
					rule.first,
					rule.second
				);
				if (current_score > score) {
					score = current_score;
					use_dbl = true;
					dbl_candidate = std::make_pair(
						std::make_pair(search_result.first, search_result.second),
						rule
					);
				}
			}
		}

		// single recipe
		for (auto &rule: sing) {
			if (target && has_part(*target, rule))
				continue;

			auto search_result = search_for_parts(fuzz_range, fuzz_rate, { rule }, callback);
			if (search_result.first == 0)	// not found
				continue;

			auto left = callback(search_result.first, true).second;
			auto right = callback(search_result.second, false).second;

			auto current_score = get_single_fuzz_score(
				search_result.first,
				search_result.second,
				*left,
				*right,
				rule
			);
			if (current_score > score) {
				score = current_score;
				use_dbl = false;
				use_sing = true;
				sing_candidate = std::make_pair(
					std::make_pair(search_result.first, search_result.second),
					rule
				);
			}
		}

		if (target && use_dbl) {
			auto &pos = dbl_candidate.first;
			auto &rule = dbl_candidate.second;

			auto left = callback(pos.first, true).second;
			auto right = callback(pos.second, false).second;

			auto body_part = get_double_fuzz_part(
				pos.first,
				pos.second,
				*left,
				*right,
				*target,
				rule.first,
				rule.second,
				score
			);

			target->body_parts()[rule.second] = body_part;
		} else if (use_sing) {
			auto &pos = sing_candidate.first;
			auto &rule = sing_candidate.second;

			auto left = callback(pos.first, true).second;
			auto right = callback(pos.second, false).second;

			auto body_part = get_single_fuzz_part(
				pos.first,
				pos.second,
				*left,
				*right,
				rule,
				score
			);

			if (target) {
				target->body_parts()[rule] = body_part;
			} else {
				target = std::unique_ptr<libaction::Human>(
					new libaction::Human(std::vector<libaction::BodyPart>{
						body_part }));
			}
		} else {
			break;
		}
	}

	return target;
}

}
}
}
}

#endif
