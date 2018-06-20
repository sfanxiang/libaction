/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__DETAIL__ZOOM_HPP_
#define LIBACTION__MOTION__DETAIL__ZOOM_HPP_

#include "../../human.hpp"
#include "../../detail/image.hpp"

#include <algorithm>

namespace libaction
{
namespace motion
{
namespace detail
{
namespace zoom
{

namespace
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

template<typename ImagePtr, typename HumanPtr, typename Image>
inline libaction::Human zoom_estimate(
	const ImagePtr &image,
	const libaction::Human &human,
	const std::function<HumanPtr(const Image &image)> &estimator_callback
) {
	if (image->num_dimensions() != 3)
		throw std::runtime_error("image must have 3 dimensions");

	if (image->shape()[0] == 0 || image->shape()[1] == 0)
		return human;

	if (human.body_parts().empty())
		return human;

	float x1 = human.body_parts().begin()->second.x();
	float x2 = x1;
	float y1 = human.body_parts().begin()->second.y();
	float y2 = y1;

	for (auto &part: human.body_parts()) {
		x1 = std::min(x1, part.second.x());
		x2 = std::max(x2, part.second.x());
		y1 = std::min(y1, part.second.y());
		y2 = std::max(y2, part.second.y());
	}

	float x_expand = x2 - x1;
	float y_expand = y2 - y1;

	x1 -= x_expand;
	x2 += x_expand;
	y1 -= y_expand;
	y2 += y_expand;

	x1 = std::max(x1, +0.0f);
	x2 = std::min(x2, 1.0f);
	y1 = std::max(y1, +0.0f);
	y2 = std::min(y2, 1.0f);

	size_t x1_i = static_cast<size_t>(x1 * static_cast<float>(image->shape[0]));
	size_t x2_i = static_cast<size_t>(x2 * static_cast<float>(image->shape[0]));
	size_t y1_i = static_cast<size_t>(y1 * static_cast<float>(image->shape[1]));
	size_t y2_i = static_cast<size_t>(y2 * static_cast<float>(image->shape[1]));

	x1_i = std::min(x1_i, image->shape[0] - 1);
	x2_i = std::max(std::min(x2_i, image->shape[0] - 1), x1_i);
	y1_i = std::min(y1_i, image->shape[1] - 1);
	y2_i = std::max(std::min(y2_i, image->shape[1] - 1), y1_i);

	if (x1_i == x2_i) {
		size_t change = image->shape()[0] / 3;
		if (x1_i >= change)
			x1_i -= change;
		else
			x1_i = 0;
		x2_i += change;
	}
	if (y1_i == y2_i) {
		size_t change = image->shape()[1] / 3;
		if (y1_i >= change)
			y1_i -= change;
		else
			y1_i = 0;
		y2_i += change;
	}

	x1_i = std::min(x1_i, image->shape[0] - 1);
	x2_i = std::max(std::min(x2_i, image->shape[0] - 1), x1_i);
	y1_i = std::min(y1_i, image->shape[1] - 1);
	y2_i = std::max(std::min(y2_i, image->shape[1] - 1), y1_i);

	if (x1_i == x2_i || y1_i == y2_i)
		return human;

	x2_i++;
	y2_i++;

	auto cropped = libaction::detail::image::crop(*image,
		x1_i, y1_i, x2_i - x1_i, y2_i - y1_i);

	if (cropped->shape()[0] == 0 || cropped->shape()[1] == 0)
		return human;

	auto cropped_human = estimator_callback(*cropped);

	auto new_human = human;
	// use parts in cropped_human to update new_human
	for (auto &part_pair: cropped_human.body_parts()) {
		auto &part = part_pair.second;

		auto find = new_human.body_parts().find(part.part_index());

		if (find == new_human.body_parts().end()) {
			auto coord = coord_translate(
				part.x(), part.y(),
				image->shape()[0], image->shape()[1],
				x1_i, y1_i,
				cropped->shape()[0], cropped->shape()[1]);

			new_human.body_parts()[part.part_index()] = libaction::BodyPart(
				part.part_index(),
				coord.first, coord.second,
				part.score()
			);
		} else if (find->second.score() <= part.score()) {
			auto coord = coord_translate(
				part.x(), part.y(),
				image->shape()[0], image->shape()[1],
				x1_i, y1_i,
				cropped->shape()[0], cropped->shape()[1]);

			find->second = libaction::BodyPart(
				part.part_index(),
				coord.first, coord.second,
				part.score()
			);
		}
	}

	return new_human;
}

}
}
}
}

#endif
