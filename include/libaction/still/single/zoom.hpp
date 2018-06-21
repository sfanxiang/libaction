/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__STILL__SINGLE__ZOOM_HPP_
#define LIBACTION__STILL__SINGLE__ZOOM_HPP_

#include "../../body_part.hpp"
#include "../../human.hpp"
#include "../../detail/image.hpp"

#include <algorithm>
#include <memory>

namespace libaction
{
namespace still
{
namespace single
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

/// Estimate from a known estimation with zoom-in reestimation.

/// @param[in]  image       The full image for estimation.
/// @param[in]  human       The result from a previous estimation. Only a single
///                         human (with at least one body part) is supported.
/// @param[in]  human_hints Hints of the location of the human, usually results
///                         from the frames around this image (in a video).
/// @param[in]  estimator_callback  Callback which, when called, returns the
///                         same person as `human`, as found in the given image.
/// @return                 A human inferred from the image.
/// @exception              std::runtime_error
template<typename Image1, typename HumanPtr1, typename HumanPtr2, typename Image2>
inline std::unique_ptr<libaction::Human> zoom_estimate(
	const Image1 &image,
	const libaction::Human &human,
	const std::vector<HumanPtr1> &human_hints,
	const std::function<HumanPtr2(const Image2 &image)> &estimator_callback
) {
	if (image.num_dimensions() != 3)
		throw std::runtime_error("image must have 3 dimensions");

	if (image.shape()[0] == 0 || image.shape()[1] == 0)
		return std::unique_ptr<libaction::Human>(new libaction::Human(human));

	if (human.body_parts().empty())
		return std::unique_ptr<libaction::Human>(new libaction::Human(human));

	float x1 = human.body_parts().begin()->second.x();
	float x2 = x1;
	float y1 = human.body_parts().begin()->second.y();
	float y2 = y1;

	float mid_x = 0.0f, mid_y = 0.0f;

	for (auto &part: human.body_parts()) {
		x1 = std::min(x1, part.second.x());
		x2 = std::max(x2, part.second.x());
		y1 = std::min(y1, part.second.y());
		y2 = std::max(y2, part.second.y());

		mid_x += part.second.x() / static_cast<float>(human.body_parts().size());
		mid_y += part.second.y() / static_cast<float>(human.body_parts().size());
	}

	float height = 0.0f, width = 0.0f;

	for (auto &hint: human_hints) {
		if (!hint)
			continue;
		if (hint->body_parts().empty())
			continue;

		float x1 = hint->body_parts().begin()->second.x();
		float x2 = x1;
		float y1 = hint->body_parts().begin()->second.y();
		float y2 = y1;

		for (auto &part: hint->body_parts()) {
			x1 = std::min(x1, part.second.x());
			x2 = std::max(x2, part.second.x());
			y1 = std::min(y1, part.second.y());
			y2 = std::max(y2, part.second.y());
		}

		height = std::max(height, x2 - x1);
		width = std::max(width, y2 - y1);
	}

	float bound_x1 = std::min(x1, std::min(x2 - height, mid_x - height / 2.0f));
	float bound_x2 = std::max(x2, std::max(x1 + height, mid_x + height / 2.0f));
	float bound_y1 = std::min(y1, std::min(y2 - width, mid_y - width / 2.0f));
	float bound_y2 = std::max(y2, std::max(y1 + width, mid_y + width / 2.0f));

	std::tie(x1, x2, y1, y2) = std::tie(bound_x1, bound_x2, bound_y1, bound_y2);

	float x_expand = (x2 - x1) / 4.0f;
	float y_expand = (y2 - y1) / 4.0f;

	x1 -= x_expand;
	x2 += x_expand;
	y1 -= y_expand;
	y2 += y_expand;

	x1 = std::max(x1, +0.0f);
	x2 = std::min(x2, 1.0f);
	y1 = std::max(y1, +0.0f);
	y2 = std::min(y2, 1.0f);

	size_t x1_i = static_cast<size_t>(x1 * static_cast<float>(image.shape[0]));
	size_t x2_i = static_cast<size_t>(x2 * static_cast<float>(image.shape[0]));
	size_t y1_i = static_cast<size_t>(y1 * static_cast<float>(image.shape[1]));
	size_t y2_i = static_cast<size_t>(y2 * static_cast<float>(image.shape[1]));

	x1_i = std::min(x1_i, image.shape[0] - 1);
	x2_i = std::max(std::min(x2_i, image.shape[0] - 1), x1_i);
	y1_i = std::min(y1_i, image.shape[1] - 1);
	y2_i = std::max(std::min(y2_i, image.shape[1] - 1), y1_i);

	if (x1_i == x2_i) {
		size_t change = image.shape()[0] / 3;
		if (x1_i >= change)
			x1_i -= change;
		else
			x1_i = 0;
		x2_i += change;
	}
	if (y1_i == y2_i) {
		size_t change = image.shape()[1] / 3;
		if (y1_i >= change)
			y1_i -= change;
		else
			y1_i = 0;
		y2_i += change;
	}

	x1_i = std::min(x1_i, image.shape[0] - 1);
	x2_i = std::max(std::min(x2_i, image.shape[0] - 1), x1_i);
	y1_i = std::min(y1_i, image.shape[1] - 1);
	y2_i = std::max(std::min(y2_i, image.shape[1] - 1), y1_i);

	if (x1_i == x2_i || y1_i == y2_i)
		return std::unique_ptr<libaction::Human>(new libaction::Human(human));

	// Turn x2_i and y2_i into past-the-end indices.
	x2_i++;
	y2_i++;

	auto cropped = libaction::detail::image::crop(image,
		x1_i, y1_i, x2_i - x1_i, y2_i - y1_i);

	if (cropped->shape()[0] == 0 || cropped->shape()[1] == 0)
		return std::unique_ptr<libaction::Human>(new libaction::Human(human));

	auto cropped_human = estimator_callback(*cropped);

	if (!cropped_human)
		return std::unique_ptr<libaction::Human>(new libaction::Human(human));

	auto new_human = std::unique_ptr<libaction::Human>(new libaction::Human(
		human));
	// use parts in cropped_human to update new_human
	for (auto &part_pair: cropped_human->body_parts()) {
		auto &part = part_pair.second;

		auto find = new_human->body_parts().find(part.part_index());

		if (find == new_human->body_parts().end()) {
			auto coord = coord_translate(
				part.x(), part.y(),
				image.shape()[0], image.shape()[1],
				x1_i, y1_i,
				cropped->shape()[0], cropped->shape()[1]);

			new_human->body_parts()[part.part_index()] = libaction::BodyPart(
				part.part_index(),
				coord.first, coord.second,
				part.score()
			);
		} else if (find->second.score() <= part.score()) {
			auto coord = coord_translate(
				part.x(), part.y(),
				image.shape()[0], image.shape()[1],
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
