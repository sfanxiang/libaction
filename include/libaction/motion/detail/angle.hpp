/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__DETAIL__ANGLE_HPP_
#define LIBACTION__MOTION__DETAIL__ANGLE_HPP_

#include <cmath>

namespace libaction
{
namespace motion
{
namespace detail
{

inline constexpr float angle(float x1, float y1, float x2, float y2)
{
	return std::abs(std::atan(y1 / x1) - std::atan(y2 / x2));
}

}
}
}

#endif
