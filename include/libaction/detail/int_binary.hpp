/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__DETAIL__INT_BINARY_HPP_
#define LIBACTION__DETAIL__INT_BINARY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace libaction
{
namespace detail
{
namespace int_binary
{

template<typename Integral>
typename std::enable_if<
	std::is_integral<Integral>::value &&
		!std::is_same<
			typename std::remove_reference<
				typename std::remove_cv<Integral>::type
			>::type,
			bool
		>::value,
	std::vector<uint8_t>
>::type
to_binary(Integral value)
{
	typename std::make_unsigned<typename std::remove_reference<
		typename std::remove_cv<Integral>::type>::type>::type
	uvalue = value;

	size_t size = sizeof(value);
	std::vector<uint8_t> result;

	for (size_t i = 0; i < size; i++) {
		result.push_back((uvalue >> ((size - i - 1) * 8)) & 0xff);
	}
	return result;
}

template<typename Integral, typename Bytes>
typename std::enable_if<
	std::is_integral<Integral>::value &&
		!std::is_same<
			typename std::remove_reference<
				typename std::remove_cv<Integral>::type
			>::type,
			bool
		>::value,
	typename std::remove_reference<
		typename std::remove_cv<Integral>::type
	>::type
>::type
to_int(const Bytes &bytes)
{
	typename std::make_unsigned<typename std::remove_reference<
		typename std::remove_cv<Integral>::type>::type>::type
	uvalue = 0;

	size_t size = sizeof(Integral);

	if (bytes.size() != size)
		throw std::runtime_error("bytes.size() != size");

	for (size_t i = 0; i < size; i++) {
		uvalue |= (static_cast<decltype(uvalue)>(static_cast<uint8_t>(bytes[i]))
			<< ((size - i - 1) * 8));
	}

	if (std::is_signed<typename std::remove_reference<
			typename std::remove_cv<Integral>::type>::type>::value) {
		bool sign = ((uvalue & (1 << (size * 8 - 1))) != 0);
		if (sign) {
			uvalue = (~uvalue) + 1;
			decltype(uvalue) max = std::numeric_limits<Integral>::min();
			max = (~max) + 1;
			max = std::min(max, static_cast<decltype(uvalue)>(
				std::numeric_limits<Integral>::max()));
			if (uvalue > max)
				uvalue = max;

			typename std::remove_reference<
				typename std::remove_cv<Integral>::type>::type
			value = uvalue;
			value = -value;
			return value;
		} else {
			decltype(uvalue) max = std::numeric_limits<Integral>::max();
			if (uvalue > max)
				uvalue = max;

			return uvalue;
		}
	} else {
		return uvalue;
	}
}

}
}
}

#endif
