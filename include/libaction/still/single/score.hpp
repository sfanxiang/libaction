/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__STILL__SINGLE__SCORE_HPP_
#define LIBACTION__STILL__SINGLE__SCORE_HPP_

#include "../../body_part.hpp"
#include "../../human.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace libaction
{
namespace still
{
namespace single
{
namespace score
{

namespace detail
{

inline std::vector<std::pair<
	libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>>
score_connections()
{
	using id = libaction::BodyPart::PartIndex;

	return {
		{ id::shoulder_r, id::elbow_r },
		{ id::shoulder_l, id::elbow_l },
		{ id::shoulder_r, id::shoulder_l },
		{ id::shoulder_r, id::neck },
		{ id::shoulder_l, id::neck },
		{ id::shoulder_r, id::nose },
		{ id::shoulder_l, id::nose },
		{ id::shoulder_r, id::hip_r },
		{ id::shoulder_l, id::hip_l },
		{ id::neck, id::nose },
		{ id::elbow_r, id::wrist_r },
		{ id::elbow_l, id::wrist_l },
		{ id::nose, id::eye_r },
		{ id::nose, id::eye_l },
		{ id::nose, id::ear_r },
		{ id::nose, id::ear_l },
		{ id::eye_r, id::eye_l },
		{ id::ear_r, id::ear_l },
		{ id::hip_r, id::hip_l },
		{ id::hip_r, id::knee_r },
		{ id::hip_l, id::knee_l },
		{ id::knee_r, id::ankle_r },
		{ id::knee_l, id::ankle_l }
	};
}

inline float angle(float x, float y)
{
	return std::atan2(y, x);
}

inline float angle_diff(float x1, float y1, float x2, float y2)
{
	if ((y1 == 0.0f && x1 == 0.0f) || (y2 == 0.0f && x2 == 0.0f))
		return 0.0f;

	auto a1 = angle(x1, y1);
	auto a2 = angle(x2, y2);

	auto diff = std::abs(a1 - a2);

	return std::min(diff, 2 * (std::acos(-1.0f) /* pi */ ) - diff);
}

inline float angle_score(float x1, float y1, float x_range1, float y_range1,
	float x2, float y2, float x_range2, float y_range2)
{
	if (x_range1 == 0.0f || y_range1 == 0.0f ||
			x_range2 == 0.0f || y_range2 == 0.0f) {
		return 0.0f;
	}

	return angle_diff(x1 * 4.0f / x_range1, y1 / y_range1,
		x2 * 4.0f / x_range2, y2 / y_range2) / (std::acos(-1.0f) /* pi */ );
}

inline float angle_score(const libaction::BodyPart &connection1_from,
	const libaction::BodyPart &connection1_to,
	float x_range1, float y_range1,
	const libaction::BodyPart &connection2_from,
	const libaction::BodyPart &connection2_to,
	float x_range2, float y_range2)
{
	return angle_score(
		connection1_to.x() - connection1_from.x(),
		connection1_to.y() - connection1_from.y(),
		x_range1, y_range1,
		connection2_to.x() - connection2_from.x(),
		connection2_to.y() - connection2_from.y(),
		x_range2, y_range2);
}

inline float distance(float x, float y)
{
	return std::sqrt(x * x + y * y);
}

inline float distance_score(float x1, float y1, float x_range1, float y_range1,
	float x2, float y2, float x_range2, float y_range2)
{
	if (x_range1 == 0.0f || y_range1 == 0.0f ||
			x_range2 == 0.0f || y_range2 == 0.0f) {
		return 0.0f;
	}

	auto d1 = distance(x1 * 4.0f / x_range1, y1 / y_range1);
	auto d2 = distance(x2 * 4.0f / x_range2, y2 / y_range2);

	auto diff = std::abs(d2 - d1);
	auto sum = d1 + d2;

	if (sum > 0.0f)
		return diff / sum;
	else
		return 0.0f;
}

inline float distance_score(const libaction::BodyPart &connection1_from,
	const libaction::BodyPart &connection1_to,
	float x_range1, float y_range1,
	const libaction::BodyPart &connection2_from,
	const libaction::BodyPart &connection2_to,
	float x_range2, float y_range2)
{
	return distance_score(
		connection1_to.x() - connection1_from.x(),
		connection1_to.y() - connection1_from.y(),
		x_range1, y_range1,
		connection2_to.x() - connection2_from.x(),
		connection2_to.y() - connection2_from.y(),
		x_range2, y_range2);
}

inline float range(std::vector<float> &data)
{
	if (data.size() < 2)
		return 0.0f;

	float max = *data.begin();
	float min = *data.begin();

	for (float &value: data) {
		if (!(value <= max))
			max = value;
		if (!(value >= min))
			min = value;
	}

	return max - min;
}

inline std::pair<float, float> sig_range(const libaction::Human &human)
{
	std::vector<float> x, y;
	for (auto &part: human.body_parts()) {
		x.push_back(part.second.x());
		if (part.first != libaction::BodyPart::PartIndex::wrist_r &&
				part.first != libaction::BodyPart::PartIndex::wrist_l &&
				part.first != libaction::BodyPart::PartIndex::elbow_r &&
				part.first != libaction::BodyPart::PartIndex::elbow_l &&
				part.first != libaction::BodyPart::PartIndex::ankle_r &&
				part.first != libaction::BodyPart::PartIndex::ankle_l &&
				part.first != libaction::BodyPart::PartIndex::knee_r &&
				part.first != libaction::BodyPart::PartIndex::knee_l) {
			y.push_back(part.second.y());
		}
	}

	float x_range = range(x);
	float y_range = range(y);

	if (x_range == 0.0f)
		x_range = y_range;
	else if (y_range == 0.0f)
		y_range = x_range;

	return std::make_pair(x_range, y_range);
}

}

/// Score a human pose against another one.

/// @param[in]  human1      The human pose to be scored.
/// @param[in]  human2      The baseline human pose.
/// @return                 A map mapping body connections to their scores. The
///                         score is within the range [0, 128]. Higher is
///                         better.
inline std::unique_ptr<std::map<std::pair<
		libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
	uint8_t>>
score(const libaction::Human &human1, const libaction::Human &human2)
{
	float x_range1, y_range1, x_range2, y_range2;
	std::tie(x_range1, y_range1) = detail::sig_range(human1);
	std::tie(x_range2, y_range2) = detail::sig_range(human2);

	auto scores = std::unique_ptr<std::map<std::pair<
		libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
	uint8_t>>(
		new std::map<std::pair<
			libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
		uint8_t>());

	for (auto &connection: detail::score_connections()) {
		auto human1_from_it = human1.body_parts().find(connection.first);
		if (human1_from_it == human1.body_parts().end())
			continue;
		auto human1_to_it = human1.body_parts().find(connection.second);
		if (human1_to_it == human1.body_parts().end())
			continue;
		auto human2_from_it = human2.body_parts().find(connection.first);
		if (human2_from_it == human2.body_parts().end())
			continue;
		auto human2_to_it = human2.body_parts().find(connection.second);
		if (human2_to_it == human2.body_parts().end())
			continue;

		auto a = detail::angle_score(
			human1_from_it->second, human1_to_it->second,
			x_range1, y_range1,
			human2_from_it->second, human2_to_it->second,
			x_range2, y_range2);
		auto d = detail::distance_score(
			human1_from_it->second, human1_to_it->second,
			x_range1, y_range1, human2_from_it->second, human2_to_it->second,
			x_range2, y_range2);

		(*scores)[std::make_pair(connection.first, connection.second)]
			= static_cast<uint8_t>(128) -
				static_cast<uint8_t>(((a + d) / 2.0f) * 128.0f);
	}

	return scores;
}

}
}
}
}

#endif
