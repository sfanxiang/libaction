#ifndef LIBACTION_IMAGE_HPP_
#define LIBACTION_IMAGE_HPP_

#include <cstdint>
#include <memory>
#include <stdexcept>

namespace libaction
{
namespace image
{

inline constexpr size_t index(
	size_t height, size_t width, size_t channels,
	size_t x, size_t y, size_t c)
{
	return ((x * width) + y) * channels + c;
}

inline constexpr size_t size(
	size_t height, size_t width, size_t channels)
{
	return height * width * channels;
}

template<typename Input, typename Output>
std::unique_ptr<Output[]> resize(
	const Input *image, size_t height, size_t width, size_t channels,
	size_t target_height, size_t target_width)
{
	if (height == 0 || width == 0 || channels == 0
			|| target_height == 0 || target_width == 0)
		throw std::runtime_error("invalid image parameters");

	auto target_image = std::unique_ptr<Output[]>(
		new Output[size(target_height, target_width, channels)]);

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
					target_image[index(target_height, target_width, channels, i, j, k)] =
						image[index(height, width, channels, x, y, k)] * (1 - x_diff) * (1 - y_diff) +
						image[index(height, width, channels, x, y + 1, k)] * (1 - x_diff) * y_diff +
						image[index(height, width, channels, x + 1, y, k)] * x_diff * (1 - y_diff) +
						image[index(height, width, channels, x + 1, y + 1, k)] * x_diff * y_diff;
				}
			} else if (x + 1 < height) {
				for (size_t k = 0; k < channels; k++) {
					target_image[index(target_height, target_width, channels, i, j, k)] =
						image[index(height, width, channels, x, y, k)] * (1 - x_diff) +
						image[index(height, width, channels, x + 1, y, k)] * x_diff;
				}
			} else if (y + 1 < width) {
				for (size_t k = 0; k < channels; k++) {
					target_image[index(target_height, target_width, channels, i, j, k)] =
						image[index(height, width, channels, x, y, k)] * (1 - y_diff) +
						image[index(height, width, channels, x, y + 1, k)] * y_diff;
				}
			} else {
				for (size_t k = 0; k < channels; k++) {
					target_image[index(target_height, target_width, channels, i, j, k)] =
						image[index(height, width, channels, x, y, k)];
				}
			}
		}
	}

	return target_image;
}

}
}

#endif
