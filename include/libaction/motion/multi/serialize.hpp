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

namespace detail
{

constexpr size_t max = 0x20000000;

inline void write_float(float value, std::vector<uint8_t> &output)
{
	auto bytes = libaction::detail::float_bytes::to_bytes(value);
	output.insert(output.end(), bytes.begin(), bytes.end());
}

template<typename Integral>
inline void write_int(Integral value, std::vector<uint8_t> &output)
{
	auto bytes = libaction::detail::int_bytes::to_bytes(value);
	output.insert(output.end(), bytes.begin(), bytes.end());
}

inline void write_body_parts_bitmap(const libaction::Human &human,
	std::vector<uint8_t> &output)
{
	static_assert(static_cast<int>(libaction::BodyPart::PartIndex::end) < 32,
		"static_cast<int>(libaction::BodyPart::PartIndex::end) < 32");

	uint32_t bitmap = 0;
	for (int i = 0; i < static_cast<int>(libaction::BodyPart::PartIndex::end);
			i++) {
		if (human.body_parts().find(static_cast<libaction::BodyPart::PartIndex>
				(i)) != human.body_parts().end()) {
			bitmap |= (1UL << (31 - i));
		}
	}

	write_int(bitmap, output);
}

inline void write_human(const libaction::Human &human,
	std::vector<uint8_t> &output)
{
	write_body_parts_bitmap(human, output);

	static_assert(static_cast<int>(libaction::BodyPart::PartIndex::end) < 32,
		"static_cast<int>(libaction::BodyPart::PartIndex::end) < 32");

	for (int i = 0; i < static_cast<int>(libaction::BodyPart::PartIndex::end);
			i++) {
		auto part_it =
			human.body_parts().find(static_cast<libaction::BodyPart::PartIndex>
				(i));
		if (part_it != human.body_parts().end()) {
			write_float(part_it->second.x(), output);
			write_float(part_it->second.y(), output);
			write_float(part_it->second.score(), output);
		}
	}
}

template<typename HumanMap>
inline void write_human_map(const HumanMap &human_map,
	std::vector<uint8_t> &output)
{
	if (human_map.size() >= max)
		throw std::runtime_error("too many items");

	write_int(static_cast<uint32_t>(human_map.size()), output);

	for (auto &human_pair: human_map) {
		uint32_t index = human_pair.first;
		if (index > max)
			index = max;
		write_int(index, output);
		write_human(human_pair.second, output);
	}
}

}

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
inline std::unique_ptr<std::vector<uint8_t>>
serialize(const Action &action, bool magic = true)
{
	auto data = std::unique_ptr<std::vector<uint8_t>>(
		new std::vector<uint8_t>());

	if (magic) {
		data->push_back(static_cast<uint8_t>('A'));
		data->push_back(static_cast<uint8_t>('C'));
		data->push_back(static_cast<uint8_t>('T'));
		data->push_back(0);
	}

	if (action.size() >= detail::max)
		throw std::runtime_error("too many items");

	detail::write_int(static_cast<uint32_t>(action.size()), *data);

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
