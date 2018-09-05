/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__STILL__SINGLE__ZOOM__DETAIL_HPP_
#define LIBACTION__STILL__SINGLE__ZOOM__DETAIL_HPP_

#include "../../../body_part.hpp"
#include "../../../human.hpp"
#include "../../../detail/image.hpp"

#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>

namespace libaction
{
namespace still
{
namespace single
{
namespace zoom
{
namespace detail
{

// translate from cropped coordinates to the original image's
inline std::pair<float, float> coord_translate(
	float x, float y,
	size_t original_height, size_t original_width,
	size_t crop_x, size_t crop_y,
	size_t crop_height, size_t crop_width
) {
	if (crop_height == 0 || crop_width == 0)
		throw std::runtime_error("crop_height == 0 || crop_width == 0");
	if (original_height == 0 || original_width == 0)
		throw std::runtime_error("original_height == 0 || original_width == 0");

	size_t x2 = static_cast<size_t>(static_cast<float>(crop_height) * x);
	size_t y2 = static_cast<size_t>(static_cast<float>(crop_width) * y);

	x2 = std::min(x2, crop_height - 1);
	y2 = std::min(y2, crop_width - 1);

	x2 += crop_x;
	y2 += crop_y;

	x2 = std::min(x2, original_height - 1);
	y2 = std::min(y2, original_width - 1);

	float x3 = static_cast<float>(x2) / static_cast<float>(original_height);
	float y3 = static_cast<float>(y2) / static_cast<float>(original_width);

	return {x3, y3};
}

}
}
}
}
}

#endif
