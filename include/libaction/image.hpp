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

std::unique_ptr<uint8_t[]> resize(
	const uint8_t *image, size_t height, size_t width, size_t channels,
	size_t target_height, size_t target_width)
{
	if (height == 0 || width == 0 || channels == 0
			|| target_height == 0 || target_width == 0)
		throw std::runtime_error("invalid image parameters");

	auto target_image = std::unique_ptr<uint8_t[]>(
		new uint8_t[size(target_height, target_width, channels)]);

	float x_ratio = (static_cast<float>(height - 1)) / target_height;
	float y_ratio = (static_cast<float>(width - 1)) / target_width;

	for (size_t i = 0; i < target_height; i++) {
		for (size_t j = 0; j < target_width; j++) {
			size_t x = (height - 1) * i / target_height;
			size_t y = (width - 1) * j / target_width;
			float x_diff = (x_ratio * i) - x;
			float y_diff = (y_ratio * j) - y;

			auto *a = &image[index(height, width, channels, x, y, 0)];
			auto *b = &image[index(height, width, channels, x, y + 1, 0)];
			auto *c = &image[index(height, width, channels, x + 1, y, 0)];
			auto *d = &image[index(height, width, channels, x + 1, y + 1, 0)];

			auto *t = &target_image[index(
				target_height, target_width, channels, i, j, 0)];

			for (size_t k = 0; k < channels; k++) {
				t[k] = a[k]*(1-x_diff)*(1-y_diff) + b[k]*(1-x_diff)*(y_diff) +
					   c[k]*(x_diff)*(1-y_diff)   + d[k]*(x_diff*y_diff);
			}
		}
	}

	return target_image;
}

}
}

#endif
