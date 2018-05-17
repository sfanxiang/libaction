#ifndef LIBACTION_IMAGE_HPP_
#define LIBACTION_IMAGE_HPP_

#include <boost/multi_array.hpp>
#include <cstdint>
#include <memory>
#include <stdexcept>

namespace libaction
{
namespace image
{

template<typename Input>
std::unique_ptr<boost::multi_array<typename Input::element, 3>> resize(
	const Input &image, size_t target_height, size_t target_width)
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

	for (size_t i = 0; i < target_height; i++) {
		for (size_t j = 0; j < target_width; j++) {
			size_t x = height * i / target_height;
			size_t y = width * j / target_width;
			float x_diff = (x_ratio * i) - x;
			float y_diff = (y_ratio * j) - y;

			if (x + 1 < height && y + 1 < width) {
				for (size_t k = 0; k < channels; k++) {
					(*target_image)[i][j][k] =
						image[x][y][k] * (1 - x_diff) * (1 - y_diff) +
						image[x][y + 1][k] * (1 - x_diff) * y_diff +
						image[x + 1][y][k] * x_diff * (1 - y_diff) +
						image[x + 1][y + 1][k] * x_diff * y_diff;
				}
			} else if (x + 1 < height) {
				for (size_t k = 0; k < channels; k++) {
					(*target_image)[i][j][k] =
						image[x][y][k] * (1 - x_diff) +
						image[x + 1][y][k] * x_diff;
				}
			} else if (y + 1 < width) {
				for (size_t k = 0; k < channels; k++) {
					(*target_image)[i][j][k] =
						image[x][y][k] * (1 - y_diff) +
						image[x][y + 1][k] * y_diff;
				}
			} else {
				for (size_t k = 0; k < channels; k++) {
					(*target_image)[i][j][k] = image[x][y][k];
				}
			}
		}
	}

	return target_image;
}

}
}

#endif
