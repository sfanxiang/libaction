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
#include "score/detail.hpp"

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

/// Score a human pose against another one.

/// @param[in]  human1      The human pose to be scored.
/// @param[in]  human2      The baseline human pose.
/// @return                 A map mapping body connections to their scores. The
///                         score is within the range [0, 128]. Higher is
///                         better.
inline std::unique_ptr<std::map<std::pair<
		libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
	std::uint8_t>>
score(const libaction::Human &human1, const libaction::Human &human2)
{
	float x_range1, y_range1, x_range2, y_range2;
	std::tie(x_range1, y_range1) = detail::sig_range(human1);
	std::tie(x_range2, y_range2) = detail::sig_range(human2);

	auto scores = std::unique_ptr<std::map<std::pair<
		libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
	std::uint8_t>>(
		new std::map<std::pair<
			libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex>,
		std::uint8_t>());

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
			= static_cast<std::uint8_t>(128) -
				static_cast<std::uint8_t>(((a + d) / 2.0f) * 128.0f);
	}

	return scores;
}

}
}
}
}

#endif
