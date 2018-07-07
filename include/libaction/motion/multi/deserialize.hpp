/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__MULTI__DESERIALIZE_HPP_
#define LIBACTION__MOTION__MULTI__DESERIALIZE_HPP_

#include "../../body_part.hpp"
#include "../../human.hpp"
#include "../../detail/float_bytes.hpp"
#include "../../detail/int_bytes.hpp"

#include <cmath>
#include <cstdint>
#include <list>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace libaction
{
namespace motion
{
namespace multi
{
namespace deserialize
{

namespace detail
{

constexpr size_t max = 0x20000000;

template<typename Iterator>
inline std::vector<uint8_t> read_vector(Iterator &it, Iterator end, size_t size)
{
	std::vector<uint8_t> v;
	for (size_t i = 0; i < size; i++) {
		if (it == end)
			throw std::runtime_error("out of bound");
		v.push_back(*it);
		it++;
	}
	return v;
}

template<typename Iterator>
inline float read_float(Iterator &it, Iterator end)
{
	auto v = read_vector(it, end, 4);
	return libaction::detail::float_bytes::to_float(v);
}

template<typename Integral, typename Iterator>
inline Integral read_int(Iterator &it, Iterator end)
{
	auto v = read_vector(it, end, sizeof(Integral));
	return libaction::detail::int_bytes::to_int<Integral>(v);
}

template<typename Iterator>
inline std::vector<libaction::BodyPart::PartIndex>
read_body_parts_bitmap(Iterator &it, Iterator end)
{
	std::vector<libaction::BodyPart::PartIndex> ind;

	auto bitmap = read_int<uint32_t>(it, end);

	static_assert(static_cast<int>(libaction::BodyPart::PartIndex::end) < 32,
		"static_cast<int>(libaction::BodyPart::PartIndex::end) < 32");

	for (int i = 0; i < static_cast<int>(libaction::BodyPart::PartIndex::end);
			i++) {
		if ((bitmap & (1UL << (31 - i))) != 0)
			ind.push_back(static_cast<libaction::BodyPart::PartIndex>(i));
	}

	return ind;
}

template<typename Iterator>
inline libaction::Human
read_human(Iterator &it, Iterator end)
{
	auto ind = read_body_parts_bitmap(it, end);

	std::list<libaction::BodyPart> body_parts;

	for (auto part_index: ind) {
		float x = read_float(it, end);
		float y = read_float(it, end);
		float score = read_float(it, end);

		if (std::isnan(x) || std::isnan(y) || std::isnan(score))
			continue;

		body_parts.push_back(libaction::BodyPart(
			part_index, x, y, score));
	}

	return libaction::Human(body_parts);
}

template<typename Iterator>
inline std::unordered_map<std::size_t, libaction::Human>
read_human_map(Iterator &it, Iterator end)
{
	auto human_map_size = read_int<uint32_t>(it, end);
	if (human_map_size >= max)
		throw std::runtime_error("too many items");

	std::unordered_map<std::size_t, libaction::Human> human_map;
	for (uint32_t i = 0; i < human_map_size; i++) {
		auto index = read_int<uint32_t>(it, end);
		if (index > max)
			index = max;

		auto human = read_human(it, end);

		human_map.insert(std::make_pair(index, std::move(human)));
	}

	return human_map;
}

}

/// Deserialize action data from bytes.

/// @param[in]  data        Action data in bytes.
/// @param[in]  magic       Whether the magic number is included in `data`.
/// @return                 Deserialized action data.
/// @exception              std::runtime_error
template<typename Data>
inline std::unique_ptr<std::list<std::unordered_map<
	std::size_t, libaction::Human>>>
deserialize(const Data &data, bool magic = true)
{
	typename Data::const_iterator it = data.begin();

	if (magic)
		detail::read_int<uint32_t>(it, data.end());	// ignore 4 bytes

	auto action_size = detail::read_int<uint32_t>(it, data.end());
	if (action_size >= detail::max)
		throw std::runtime_error("too many items");

	auto action = std::unique_ptr<std::list<std::unordered_map<
		std::size_t, libaction::Human>>>(
			new std::list<std::unordered_map<std::size_t, libaction::Human>>());

	for (uint32_t i = 0; i < action_size; i++) {
		action->push_back(detail::read_human_map(it, data.end()));
	}

	return action;
}

}
}
}
}

#endif
