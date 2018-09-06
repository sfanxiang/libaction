/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__SINGLE__FUZZ_HPP_
#define LIBACTION__MOTION__SINGLE__FUZZ_HPP_

#include "../../body_part.hpp"
#include "../../human.hpp"
#include "fuzz/detail.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace libaction
{
namespace motion
{
namespace single
{
namespace fuzz
{

/// Retrieve the widest possible range for fuzz estimation.

/// @param[in]  pos         The current index of the frame, starting from 0.
/// @param[in]  length      The total number of frames. Must be greater than
///                         `pos`.
/// @param[in]  fuzz_range  The range of images used for fuzz estimation. The
///                         left and right bound will be `(fuzz_range - 1)`
///                         frames away from `pos`, if the result is a valid
///                         frame number.
/// @return                 The left and the right bound, inclusively.
/// @exception              std::runtime_error
inline std::pair<std::size_t, std::size_t>
get_fuzz_lr(std::size_t pos, std::size_t length, std::size_t fuzz_range)
{
	if (length == 0)
		throw std::runtime_error("length == 0");
	if (length <= pos)
		throw std::runtime_error("length <= pos");

	std::size_t l = pos;
	std::size_t r = pos;

	if (fuzz_range != 0) {
		if (pos >= fuzz_range - 1)
			l = pos - (fuzz_range - 1);
		else
			l = 0;
		if (length - pos > fuzz_range - 1)
			r = pos + (fuzz_range - 1);
		else
			r = length - 1;
	}

	return std::make_pair(l, r);
}

/// Fuzz estimation for a single person.

/// @param[in]  fuzz_range  The range of images used for fuzz estimation.
///                         The distance between the right frame and the left
///                         frame used in each recipe is at most `fuzz_range`.
///                         This function has no effect if `fuzz_range` is 0.
/// @param[in]  callback    Callback for obtaining results from estimators.
///                         `relative_pos` and `left` locates the position of
///                         image to be obtained. `left` controls whether the
///                         frame to be obtained is to the left or to the right
///                         of the target frame, and `relative_pos` controls
///                         the distance from the target frame. If
///                         `relative_pos` is 0, the target frame should be
///                         returned. The first return value of `callback`
///                         should indicate whether the frame is in bound at
///                         `relative_pos`. The second return value is only
///                         used when the first value is true, in which case
///                         the second value should be a human estimation, or
///                         `nullptr` if the person does not exist at the
///                         location. The callback may be called multiple times
///                         with the same arguments, so you should consider
///                         caching the results, at least for all frames within
///                         the range returned by get_fuzz_lr().
/// @warning                The person must exist at the target frame.
/// @return                 A human inferred from the image.
/// @exception              std::runtime_error
template<typename HumanPtr>
inline std::unique_ptr<libaction::Human> fuzz(
	std::size_t fuzz_range,
	const std::function<std::pair<bool, HumanPtr>(std::size_t relative_pos, bool left)>
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

	const auto &relative = detail::relative_recipe();
	const auto &absolute = detail::absolute_recipe();

	while (true) {
		std::pair<
			std::pair<std::size_t, std::size_t>,
			std::pair<
				libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>
		> relative_candidate;
		std::pair<
			std::pair<std::size_t, std::size_t>,
			libaction::BodyPart::PartIndex
		> absolute_candidate;

		bool use_relative = false, use_absolute = false;
		float score = -1.0f;

		if (target) {
			// relative recipe
			for (auto &rule: relative) {
				if (detail::has_part(*target, rule.second) ||
						!detail::has_part(*target, rule.first))
					continue;

				std::vector<libaction::BodyPart::PartIndex> search_for{
					rule.first, rule.second };
				auto search_result = detail::search_for_parts(fuzz_range, search_for, callback);
				if (search_result.first == 0)	// not found
					continue;

				auto left = callback(search_result.first, true).second;
				auto right = callback(search_result.second, false).second;

				auto current_score = detail::get_relative_fuzz_score(
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
					use_relative = true;
					relative_candidate = std::make_pair(
						std::make_pair(search_result.first, search_result.second),
						rule
					);
				}
			}
		}

		if (!use_relative) {
			// absolute recipe
			for (auto &rule: absolute) {
				if (target && detail::has_part(*target, rule))
					continue;

				auto search_result = detail::search_for_parts(fuzz_range, { rule }, callback);
				if (search_result.first == 0)	// not found
					continue;

				auto left = callback(search_result.first, true).second;
				auto right = callback(search_result.second, false).second;

				auto current_score = detail::get_absolute_fuzz_score(
					search_result.first,
					search_result.second,
					*left,
					*right,
					rule
				);
				if (current_score > score) {
					score = current_score;
					use_relative = false;
					use_absolute = true;
					absolute_candidate = std::make_pair(
						std::make_pair(search_result.first, search_result.second),
						rule
					);
				}
			}
		}

		if (target && use_relative) {
			auto &pos = relative_candidate.first;
			auto &rule = relative_candidate.second;

			auto left = callback(pos.first, true).second;
			auto right = callback(pos.second, false).second;

			auto body_part = detail::get_relative_fuzz_part(
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
		} else if (use_absolute) {
			auto &pos = absolute_candidate.first;
			auto &rule = absolute_candidate.second;

			auto left = callback(pos.first, true).second;
			auto right = callback(pos.second, false).second;

			auto body_part = detail::get_absolute_fuzz_part(
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
