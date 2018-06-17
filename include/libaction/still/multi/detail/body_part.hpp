/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__STILL__MULTI__DETAIL__BODY_PART_HPP_
#define LIBACTION__STILL__MULTI__DETAIL__BODY_PART_HPP_

#include <cstddef>

namespace libaction
{
namespace still
{
namespace multi
{
namespace detail
{

class BodyPart
{
public:
	inline BodyPart() {}

	inline BodyPart(size_t part_idx, float x, float y, float score)
	:
	part_idx_(part_idx), x_(x), y_(y), score_(score)
	{}

	inline size_t part_idx() const { return part_idx_; }
	inline float x() const { return x_; }
	inline float y() const { return y_; }
	inline float score() const { return score_; }

private:
	size_t part_idx_{};
	float x_{}, y_{}, score_{};
};

}
}
}
}

#endif
