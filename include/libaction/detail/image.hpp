/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__DETAIL__IMAGE_HPP_
#define LIBACTION__DETAIL__IMAGE_HPP_

#include <boost/multi_array.hpp>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>

namespace libaction
{
namespace detail
{
/// Image manipulation utilities.
namespace image
{

/// Resize an image.

/// @param[in]  image       The input image conforming to the Boost.MultiArray
///                         concept. The image must have 3 non-empty
///                         dimensions of height, width, and channels.
/// @param[in]  target_height   The target height (>0).
/// @param[in]  target_width    The target width (>0).
/// @return                 The resized image.
/// @exception              std::runtime_error
// TODO: Resize is the second bottleneck. Optimize this function.
template<typename Input>
std::unique_ptr<boost::multi_array<typename Input::element, 3>> resize(
	const Input &image, std::size_t target_height, std::size_t target_width)
{
	if (image.num_dimensions() != 3 ||
			image.shape()[0] == 0 || image.shape()[1] == 0 ||
			image.shape()[2] == 0 ||
			target_height == 0 || target_width == 0)
		throw std::runtime_error("invalid image parameters");

	auto height = image.shape()[0];
	auto width = image.shape()[1];
	auto channels = image.shape()[2];

	auto target_image = std::unique_ptr<
		boost::multi_array<typename Input::element, 3>>(
			new boost::multi_array<typename Input::element, 3>(
				boost::extents[target_height][target_width][channels]));

	float x_ratio = static_cast<float>(height) / target_height;
	float y_ratio = static_cast<float>(width) / target_width;

	for (std::size_t i = 0; i < target_height; i++) {
		for (std::size_t j = 0; j < target_width; j++) {
			std::size_t x = height * i / target_height;
			std::size_t y = width * j / target_width;
			float x_diff = (x_ratio * i) - x;
			float y_diff = (y_ratio * j) - y;

			if (x + 1 < height && y + 1 < width) {
				for (std::size_t k = 0; k < channels; k++) {
					(*target_image)[i][j][k] =
						image[x][y][k] * (1 - x_diff) * (1 - y_diff) +
						image[x][y + 1][k] * (1 - x_diff) * y_diff +
						image[x + 1][y][k] * x_diff * (1 - y_diff) +
						image[x + 1][y + 1][k] * x_diff * y_diff;
				}
			} else if (x + 1 < height) {
				for (std::size_t k = 0; k < channels; k++) {
					(*target_image)[i][j][k] =
						image[x][y][k] * (1 - x_diff) +
						image[x + 1][y][k] * x_diff;
				}
			} else if (y + 1 < width) {
				for (std::size_t k = 0; k < channels; k++) {
					(*target_image)[i][j][k] =
						image[x][y][k] * (1 - y_diff) +
						image[x][y + 1][k] * y_diff;
				}
			} else {
				for (std::size_t k = 0; k < channels; k++) {
					(*target_image)[i][j][k] = image[x][y][k];
				}
			}
		}
	}

	return target_image;
}

template<typename Input>
std::unique_ptr<boost::multi_array<typename Input::element, 3>> crop(
	const Input &image,
	std::size_t x, std::size_t y,
	std::size_t target_height, std::size_t target_width)
{
	if (image.num_dimensions() != 3)
		throw std::runtime_error("invalid image");

	auto height = image.shape()[0];
	auto width = image.shape()[1];
	auto channels = image.shape()[2];

	std::size_t x1 = std::min(x, height);
	std::size_t y1 = std::min(y, width);
	std::size_t x2 = std::min(x1 + target_height, height);
	std::size_t y2 = std::min(y1 + target_width, width);

	auto target_image = std::unique_ptr<
		boost::multi_array<typename Input::element, 3>>(
			new boost::multi_array<typename Input::element, 3>(
				boost::extents[x2 - x1][y2 - y1][channels]));

	for (std::size_t i = x1; i < x2; i++) {
		for (std::size_t j = y1; j < y2; j++) {
			for (std::size_t k = 0; k < channels; k++) {
				(*target_image)[i - x1][j - y1][k] = image[i][j][k];
			}
		}
	}

	return target_image;
}

}
}
}

#endif
