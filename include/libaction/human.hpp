#ifndef LIBACTION_HUMAN_HPP_
#define LIBACTION_HUMAN_HPP_

#include "body_part.hpp"

#include <map>

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

	/// @return                 A map from part index to its respective body
	///                         part.
	/// @sa                     BodyPart::PartIndex and BodyPart
	inline const std::map<BodyPart::PartIndex, BodyPart> &body_parts() const
	{
		return body_parts_;
	}

private:
	std::map<BodyPart::PartIndex, BodyPart> body_parts_{};
};

}

#endif
