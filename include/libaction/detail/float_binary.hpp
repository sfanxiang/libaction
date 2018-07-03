/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#include <cmath>
#include <cstdint>
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
		return { static_cast<uint8_t>(sign ? 0x80U : 0U), 0U, 0U, 0U };
	}
	if (std::isinf(value)) {
		return {
			static_cast<uint8_t>((sign ? 0x80U : 0U) | 0x7fU),
			static_cast<uint8_t>(0x7fU),
			0xffU, 0xffU
		};
	}
	if (value == 0.0f)
		return { static_cast<uint8_t>(sign ? 0x80U : 0U), 0U, 0U, 0U };

	value = std::copysign(value, +1.0f);

	int exp_int;
	value = std::ldexp(std::frexp(value, &exp_int), 24);
	exp_int += 126;

	if (exp_int < 0) {
		// zero
		return { static_cast<uint8_t>(sign ? 0x80U : 0U), 0U, 0U, 0U };
	} else if (exp_int >= 0xff) {
		// max
		return {
			static_cast<uint8_t>((sign ? 0x80U : 0U) | 0x7fU),
			static_cast<uint8_t>(0x7fU),
			0xffU, 0xffU
		};
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

}
}
}
