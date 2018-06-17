/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__DETAIL__CONNECTIONS_HPP_
#define LIBACTION__MOTION__DETAIL__CONNECTIONS_HPP_

#include "../../body_part.hpp"

#include <vector>

namespace libaction
{
namespace motion
{
namespace detail
{
namespace connections
{

inline std::vector<std::vector<libaction::BodyPart::PartIndex>> get_fuzz_end(
	libaction::BodyPart::PartIndex end)
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
		return { { index::eye_l, index::nose } };
		break;
	case index::eye_l:
		return { { index::eye_r, index::nose } };
		break;
	case index::ear_r:
		return { { index::ear_l, index::nose } };
		break;
	case index::ear_l:
		return { { index::ear_r, index::nose } };
		break;
	case index::end:
		return {};
		break;
	default:
		return {};
		break;
	}
}

}
}
}
}

#endif
