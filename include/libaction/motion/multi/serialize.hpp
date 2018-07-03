/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__MULTI__SERIALIZE_HPP_
#define LIBACTION__MOTION__MULTI__SERIALIZE_HPP_

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

namespace
{

template<typename HumanMap>
inline void serialize_human_map(const HumanMap &human_map,
	std::vector<uint8_t> &output)
{

}

template<typename Action>
inline std::unique_ptr<std::vector<uint8_t>>
serialize(const Action &action, bool magic = true)
{
	std::unique_ptr<std::vector<uint8_t>> data =
		std::unique_ptr<std::vector<uint8_t>>(new std::vector<uint8_t>());

	if (magic) {
		data->push_back(static_cast<uint8_t>('A'));
		data->push_back(static_cast<uint8_t>('C'));
		data->push_back(static_cast<uint8_t>('T'));
		data->push_back(0);
	}

	if (action.size() >= 0x7fffffff)
		throw std::runtime_error("too many items");

	// TODO

}

}
}
}
}

#endif
