/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace libaction
{
namespace detail
{
namespace float_binary
{

std::vector<uint8_t> to_binary(float value)
{
	// no nan or inf allowed

	bool sign = std::signbit(value);

	if (std::isnan(value)) {
		return {
			static_cast<uint8_t>((sign ? 0x80U : 0U) | 0x7fU),
			0xc0U,
			0U, 0U
		};
	}
	if (!std::isfinite(value)) {
		return {
			static_cast<uint8_t>((sign ? 0x80U : 0U) | 0x7fU),
			0x80U,
			0U, 0U
		};
	}
	if (value == 0.0f)
		return { static_cast<uint8_t>(sign ? 0x80U : 0U), 0U, 0U, 0U };

	value = std::copysign(value, +1.0f);

	int exp_int;
	value = std::frexp(value, &exp_int);

	if (exp_int >= 0xff || value > 1.0f) {
		// infinity
		return {
			static_cast<uint8_t>((sign ? 0x80U : 0U) | 0x7fU),
			0x80U,
			0U, 0U
		};
	}

	value = std::ldexp(value, 24);
	exp_int += 126;

	if (!std::isfinite(value) || exp_int >= 0xff) {
		// infinity
		return {
			static_cast<uint8_t>((sign ? 0x80U : 0U) | 0x7fU),
			0x80U,
			0U, 0U
		};
	}
	if (exp_int < 0) {
		// zero
		return { static_cast<uint8_t>(sign ? 0x80U : 0U), 0U, 0U, 0U };
	}

	uint8_t exp = exp_int;
	uint32_t mant = static_cast<uint32_t>(value);

	return {
		static_cast<uint8_t>((sign ? 0x80U : 0U) | exp >> 1),
		static_cast<uint8_t>(((exp << 7) & 0x80U) | ((mant >> 16) & 0x7fU)),
		static_cast<uint8_t>((mant >> 8) & 0xffU),
		static_cast<uint8_t>(mant & 0xffU)
	};
}

template<typename Bytes>
float to_float(const Bytes &bytes)
{
	if (bytes.size() != 4)
		throw std::runtime_error("bytes.size() != 4");

	uint32_t num = (static_cast<uint32_t>(bytes[0]) << 24) |
		(static_cast<uint32_t>(bytes[1]) << 16) |
		(static_cast<uint32_t>(bytes[2]) << 8) |
		static_cast<uint32_t>(bytes[3]);

	bool sign = ((bytes[0] & 0x80U) != 0);

	if ((num & 0x7f800000U) == 0x7f800000U) {
		if ((num & 0x7fffffU) != 0U) {
			// nan
			return std::copysign(
				std::numeric_limits<float>::has_quiet_NaN ?
				std::numeric_limits<float>::quiet_NaN() :
				0.0f,
				(sign ? -1.0f : +1.0f));
		} else {
			// infinity
			return std::copysign(
				HUGE_VALF,
				(sign ? -1.0f : +1.0f));
		}
	} else {
		uint8_t exp = (num & 0x7f800000U) >> 23;
		int exp_int = static_cast<int>(exp) - 126 - 24;
		float mant = static_cast<float>((num & 0x7fffffU) | 0x800000U);
		return std::copysign(
			std::ldexp(mant, exp_int),
			(sign ? -1.0f : +1.0f));
	}
}

}
}
}
