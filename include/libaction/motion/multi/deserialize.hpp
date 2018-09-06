/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__MULTI__DESERIALIZE_HPP_
#define LIBACTION__MOTION__MULTI__DESERIALIZE_HPP_

#include "../../human.hpp"
#include "deserialize/detail.hpp"

#include <cstdint>
#include <list>
#include <memory>
#include <stdexcept>
#include <unordered_map>

namespace libaction
{
namespace motion
{
namespace multi
{
namespace deserialize
{

/// Deserialize action data from bytes.

/// @param[in]  data        Action data in bytes in a standard container.
/// @param[in]  magic       Whether the magic number is included in `data`.
/// @return                 Deserialized action data as a frame list of indexed
///                         humans. Index starts from 0.
/// @exception              std::runtime_error
template<typename Data>
inline std::unique_ptr<std::list<std::unordered_map<
	std::size_t, libaction::Human>>>
deserialize(const Data &data, bool magic = true)
{
	typename Data::const_iterator it = data.begin();

	if (magic)
		detail::read_int<std::uint32_t>(it, data.end());	// ignore 4 bytes

	auto action_size = detail::read_int<std::uint32_t>(it, data.end());
	if (action_size >= detail::max)
		throw std::runtime_error("too many items");

	auto action = std::unique_ptr<std::list<std::unordered_map<
		std::size_t, libaction::Human>>>(
			new std::list<std::unordered_map<std::size_t, libaction::Human>>());

	for (std::uint32_t i = 0; i < action_size; i++) {
		action->push_back(detail::read_human_map(it, data.end()));
	}

	return action;
}

}
}
}
}

#endif
