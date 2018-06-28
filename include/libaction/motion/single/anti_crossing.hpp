/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__SINGLE__ANTI_CROSSING_HPP_
#define LIBACTION__MOTION__SINGLE__ANTI_CROSSING_HPP_

#include "../../body_part.hpp"
#include "../../human.hpp"

#include <cmath>
#include <memory>

namespace libaction
{
namespace motion
{
namespace single
{
namespace anti_crossing
{

namespace
{

inline auto horizontal_dist(
	const libaction::BodyPart &x, const libaction::BodyPart &y)
-> decltype(x.y())
{
	return std::abs(x.y() - y.y());
}

}

/// Process an estimation to reduce crossing results.

/// @param[in]  target      The result from a previous estimation. Only a single
///                         human (with at least one body part) is supported.
/// @param[in]  left        The result of the estimation on the frame to the
///                         left of the target. It must contain the same person
///                         as found in target.
/// @param[in]  right       The result of the estimation on the frame to the
///                         right of the target. It must contain the same person
///                         as found in target.
/// @return                 The processed estimation.
inline std::unique_ptr<libaction::Human> anti_crossing(
	const libaction::Human &target,
	const libaction::Human *left,
	const libaction::Human *right
) {
	using id = libaction::BodyPart::PartIndex;

	auto result = std::unique_ptr<libaction::Human>(new libaction::Human(
		target));

	// notice the confusion between left frame / right frame and
	// left body part / right body part

	for (auto current: {
		std::tuple<id, id>
		{ id::eye_l, id::eye_r }, { id::ear_l, id::ear_r },
		{ id::shoulder_l, id::shoulder_r }, { id::elbow_l, id::elbow_r },
		{ id::wrist_l, id::wrist_r }, { id::hip_l, id::hip_r },
		{ id::knee_l, id::knee_r }, { id::ankle_l, id::ankle_r } })
	{
		bool left_cross = false, right_cross = false;

		auto target_0 = target.body_parts().find(std::get<0>(current));
		auto target_1 = target.body_parts().find(std::get<1>(current));

		for (auto side: { left, right }) {
			if (!side)
				continue;

			auto side_0 = side->body_parts().find(std::get<0>(current));
			auto side_1 = side->body_parts().find(std::get<1>(current));

			if (target_0 != target.body_parts().end() &&
					target_1 != target.body_parts().end()) {
				if (side_0 != side->body_parts().end()) {
					if (!left_cross &&
							horizontal_dist(target_0->second, side_0->second)
							> horizontal_dist(target_0->second, target_1->second)
								* 8.0f) {
						// left moved to right
						left_cross = true;
					}
				}
				if (side_1 != side->body_parts().end()) {
					if (!right_cross &&
							horizontal_dist(target_1->second, side_1->second)
							> horizontal_dist(target_1->second, target_0->second)
								* 8.0f) {
						// right moved to left
						right_cross = true;
					}
				}
			} else if (target_0 != target.body_parts().end()) {
				if (side_0 != side->body_parts().end() &&
						side_1 != side->body_parts().end()) {
					if (!left_cross &&
							horizontal_dist(target_0->second, side_0->second)
							> horizontal_dist(target_0->second, side_1->second)
								* 8.0f) {
						// left moved to right
						left_cross = true;
					}
				}
			} else if (target_1 != target.body_parts().end()) {
				if (side_0 != side->body_parts().end() &&
						side_1 != side->body_parts().end()) {
					if (!right_cross &&
							horizontal_dist(target_1->second, side_1->second)
							> horizontal_dist(target_1->second, side_0->second)
								* 8.0f) {
						// right moved to left
						right_cross = true;
					}
				}
			}

			if (left_cross && right_cross)
				break;
		}

		if (left_cross) {
			// remove left
			result->body_parts().erase(std::get<0>(current));
		}
		if (right_cross) {
			// remove right
			result->body_parts().erase(std::get<1>(current));
		}
	}

	return result;
}

}
}
}
}

#endif
