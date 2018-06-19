/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__HUMAN_HPP_
#define LIBACTION__HUMAN_HPP_

#include "body_part.hpp"

#include <unordered_map>

namespace libaction
{

/// Describe a human pose.
class Human
{
public:
	/// Construct from a list of BodyPart.

	/// @param[in]  parts       An iterable representing a list of BodyPart.
	template<typename Parts>
	Human(const Parts &parts)
	{
		for (auto &part: parts) {
			body_parts_[part.part_index()] = part;
		}
	}

	/// Body parts.

	/// @return                 An unordered map mapping part index to its
	///                         respective body part.
	/// @sa                     BodyPart::PartIndex and BodyPart
	inline const std::unordered_map<BodyPart::PartIndex, BodyPart> &body_parts()
	const
	{
		return body_parts_;
	}

	/// Body parts.

	/// @return                 An unordered map mapping part index to its
	///                         respective body part.
	/// @sa                     BodyPart::PartIndex and BodyPart
	inline std::unordered_map<BodyPart::PartIndex, BodyPart> &body_parts()
	{
		return body_parts_;
	}

private:
	std::unordered_map<BodyPart::PartIndex, BodyPart> body_parts_{};
};

}

#endif
