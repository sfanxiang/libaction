/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__STILL__SINGLE__DETAIL__POSENET_PARTS_HPP_
#define LIBACTION__STILL__SINGLE__DETAIL__POSENET_PARTS_HPP_

#include "libaction/body_part.hpp"

#include <array>
#include <utility>

namespace libaction
{
namespace still
{
namespace single
{
namespace detail
{
namespace posenet_parts
{

enum class Part
{
	nose = 0,
	eye_l,
	eye_r,
	ear_l,
	ear_r,
	shoulder_l,
	shoulder_r,
	elbow_l,
	elbow_r,
	wrist_l,
	wrist_r,
	hip_l,
	hip_r,
	knee_l,
	knee_r,
	ankle_l,
	ankle_r,
	end
};

inline libaction::BodyPart::PartIndex to_libaction_part_index(Part part)
{
	if (static_cast<int>(part) >= static_cast<int>(Part::end))
		return libaction::BodyPart::PartIndex::end;

	using idx = libaction::BodyPart::PartIndex;

	return std::array<libaction::BodyPart::PartIndex, 18>{
		idx::nose, idx::eye_l, idx::eye_r, idx::ear_l, idx::ear_r,
		idx::shoulder_l, idx::shoulder_r, idx::elbow_l, idx::elbow_r,
		idx::wrist_l, idx::wrist_r, idx::hip_l, idx::hip_r, idx::knee_l,
		idx::knee_r, idx::ankle_l, idx::ankle_r, idx::end
	}[static_cast<int>(part)];
}

}
}
}
}
}

#endif
