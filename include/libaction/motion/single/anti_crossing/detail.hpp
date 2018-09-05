/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__SINGLE__ANTI_CROSSING__DETAIL_HPP_
#define LIBACTION__MOTION__SINGLE__ANTI_CROSSING__DETAIL_HPP_

#include "../../../body_part.hpp"
#include "../../../human.hpp"

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
namespace detail
{

template<typename T>
inline T hypot(T x, T y)
{
	return std::sqrt(x * x + y * y);
}

template<typename T>
inline T dist(T x1, T y1, T x2, T y2)
{
	return hypot(x1 - x2, y1 - y2);
}

inline auto dist(const libaction::BodyPart &x, const libaction::BodyPart &y)
	-> decltype(dist(x.x(), x.y(), y.x(), y.y()))
{
	return dist(x.x(), x.y(), y.x(), y.y());
}

inline auto hdist(const libaction::BodyPart &x, const libaction::BodyPart &y)
	-> decltype(x.y())
{
	return std::abs(x.y() - y.y());
}

}
}
}
}
}

#endif
