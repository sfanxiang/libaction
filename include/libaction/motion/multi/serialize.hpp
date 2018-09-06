/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__MULTI__SERIALIZE_HPP_
#define LIBACTION__MOTION__MULTI__SERIALIZE_HPP_

#include "../../body_part.hpp"
#include "../../human.hpp"
#include "../../detail/float_bytes.hpp"
#include "../../detail/int_bytes.hpp"
#include "serialize/detail.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

namespace libaction
{
namespace motion
{
namespace multi
{
namespace serialize
{

/// Serialize action data into bytes.

/// @param[in]  action      Action data of the format
///                         List<Map<Index, libaction::Human>>, where
///                         `List` is an iterable of `Map`, `Map` is an
///                         iterable of `std::pair<Index, libaction::Human>`,
///                         and `Index` is an unsigned integral identifying
///                         each person.
/// @param[in]  magic       Whether the magic number should be included.
/// @return                 Serialized bytes.
/// @exception              std::runtime_error
template<typename Action>
inline std::unique_ptr<std::vector<std::uint8_t>>
serialize(const Action &action, bool magic = true)
{
	auto data = std::unique_ptr<std::vector<std::uint8_t>>(
		new std::vector<std::uint8_t>());

	if (magic) {
		data->push_back(static_cast<std::uint8_t>('A'));
		data->push_back(static_cast<std::uint8_t>('C'));
		data->push_back(static_cast<std::uint8_t>('T'));
		data->push_back(0);
	}

	if (action.size() >= detail::max)
		throw std::runtime_error("too many items");

	detail::write_int(static_cast<std::uint32_t>(action.size()), *data);

	for (auto &human_map: action) {
		detail::write_human_map(human_map, *data);
	}

	return data;
}

}
}
}
}

#endif
