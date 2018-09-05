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
#include "zoom/detail.hpp"

#include <boost/multi_array.hpp>
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

/// Get the recommended range for zoom estimation.

/// @param[in]  pos         The current index of the frame, starting from 0.
/// @param[in]  length      The total number of frames. Must be greater than
///                         `pos`.
/// @param[in]  zoom_range  The range of images used for zoom reestimation.
///                         The left and right bound will be `zoom_range`
///                         frames away from `pos`, if the result is a valid
///                         frame number.
/// @return                 The left and the right bound, inclusively.
/// @exception              std::runtime_error
inline std::pair<size_t, size_t>
get_zoom_lr(size_t pos, size_t length, size_t zoom_range)
{
	if (length == 0)
		throw std::runtime_error("length == 0");
	if (length <= pos)
		throw std::runtime_error("length <= pos");

	size_t l = (pos >= zoom_range ? pos - zoom_range : 0);
	size_t r = (length - pos > zoom_range ? pos + zoom_range : length - 1);

	return std::make_pair(l, r);
}

/// Estimate from a known estimation with zoom-in reestimation.

/// @param[in]  image       The full image for estimation, which should conform
///                         to the Boost.MultiArray concept.
/// @param[in]  human       The result from a previous estimation. Only a single
///                         human (with at least one body part) is supported.
/// @param[in]  human_hints Hints of the location of the human, usually results
///                         from the frames within the range returned by
///                         get_zoom_lr(), except for `human`.
/// @param[in]  estimator_callback  Callback which, when called, returns the
///                         same person as `human`, as found in the given image.
/// @return                 A human inferred from the image.
/// @exception              std::runtime_error
template<typename Image, typename HumanPtr1, typename HumanPtr2>
inline std::unique_ptr<libaction::Human> zoom_estimate(
	const Image &image,
	const libaction::Human &human,
	const std::vector<HumanPtr1> &human_hints,
	const std::function<HumanPtr2(
		const boost::multi_array<typename Image::element, 3> &image
	)> &estimator_callback
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

	float size = std::max(height, width);

	float bound_x1 = std::min(x1, std::min(x2 - size, mid_x - size / 2.0f));
	float bound_x2 = std::max(x2, std::max(x1 + size, mid_x + size / 2.0f));
	float bound_y1 = std::min(y1, std::min(y2 - size, mid_y - size / 2.0f));
	float bound_y2 = std::max(y2, std::max(y1 + size, mid_y + size / 2.0f));

	std::tie(x1, x2, y1, y2) = std::tie(bound_x1, bound_x2, bound_y1, bound_y2);

	x1 -= (x2 - x1) / 5.0f;
	x2 += (x2 - x1) / 5.0f;
	y1 -= (y2 - y1) / 5.0f;
	y2 += (y2 - y1) / 5.0f;

	x1 = std::max(x1, +0.0f);
	x2 = std::min(x2, 1.0f);
	y1 = std::max(y1, +0.0f);
	y2 = std::min(y2, 1.0f);

	size_t x1_i = static_cast<size_t>(x1 * static_cast<float>(image.shape()[0]));
	size_t x2_i = static_cast<size_t>(x2 * static_cast<float>(image.shape()[0]));
	size_t y1_i = static_cast<size_t>(y1 * static_cast<float>(image.shape()[1]));
	size_t y2_i = static_cast<size_t>(y2 * static_cast<float>(image.shape()[1]));

	x1_i = std::min(x1_i, image.shape()[0] - 1);
	x2_i = std::max(std::min(x2_i, image.shape()[0] - 1), x1_i);
	y1_i = std::min(y1_i, image.shape()[1] - 1);
	y2_i = std::max(std::min(y2_i, image.shape()[1] - 1), y1_i);

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

	x1_i = std::min(x1_i, image.shape()[0] - 1);
	x2_i = std::max(std::min(x2_i, image.shape()[0] - 1), x1_i);
	y1_i = std::min(y1_i, image.shape()[1] - 1);
	y2_i = std::max(std::min(y2_i, image.shape()[1] - 1), y1_i);

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
			auto coord = detail::coord_translate(
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
			auto coord = detail::coord_translate(
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
