#ifndef LIBACTION_HUMAN_HPP_
#define LIBACTION_HUMAN_HPP_

#include "body_part.hpp"

#include <map>

namespace libaction
{

class Human
{
public:
	template<typename Parts>
	Human(const Parts &p)
	{
		for (auto &part: p) {
			body_parts[part.part_index()] = part;
		}
	}

	inline const std::map<BodyPart::PartIndex, BodyPart> &get_body_parts() const
	{
		return body_parts;
	}

private:
	std::map<BodyPart::PartIndex, BodyPart> body_parts{};
};

}

#endif
