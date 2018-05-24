#ifndef LIBACTION_MULTI_DETAIL_COCO_PARTS_HPP_
#define LIBACTION_MULTI_DETAIL_COCO_PARTS_HPP_

#include "../../body_part.hpp"

#include <array>
#include <utility>

namespace libaction
{
namespace multi
{
namespace detail
{
namespace coco_parts
{

enum class Part
{
	nose = 0,
	neck,
	shoulder_r,
	elbow_r,
	wrist_r,
	shoulder_l,
	elbow_l,
	wrist_l,
	hip_r,
	knee_r,
	ankle_r,
	hip_l,
	knee_l,
	ankle_l,
	eye_r,
	eye_l,
	ear_r,
	ear_l,
	background,
	end
};

inline constexpr std::array<std::pair<size_t, size_t>, 19> pairs()
{
	return std::array<std::pair<size_t, size_t>, 19>{
		std::make_pair(1, 2), std::make_pair(1, 5), std::make_pair(2, 3),
		std::make_pair(3, 4), std::make_pair(5, 6), std::make_pair(6, 7),
		std::make_pair(1, 8), std::make_pair(8, 9), std::make_pair(9, 10),
		std::make_pair(1, 11), std::make_pair(11, 12), std::make_pair(12, 13),
		std::make_pair(1, 0), std::make_pair(0, 14), std::make_pair(14, 16),
		std::make_pair(0, 15), std::make_pair(15, 17), std::make_pair(2, 16),
		std::make_pair(5, 17)};
}

inline constexpr std::array<std::pair<size_t, size_t>, 19> pairs_network()
{
	return std::array<std::pair<size_t, size_t>, 19>{
		std::make_pair(12, 13), std::make_pair(20, 21), std::make_pair(14, 15),
		std::make_pair(16, 17), std::make_pair(22, 23), std::make_pair(24, 25),
		std::make_pair(0, 1), std::make_pair(2, 3), std::make_pair(4, 5),
		std::make_pair(6, 7), std::make_pair(8, 9), std::make_pair(10, 11),
		std::make_pair(28, 29), std::make_pair(30, 31), std::make_pair(34, 35),
		std::make_pair(32, 33), std::make_pair(36, 37), std::make_pair(18, 19),
		std::make_pair(26, 27)};
}

inline libaction::BodyPart::PartIndex to_libaction_part_index(Part part)
{
	if (static_cast<int>(part) >= static_cast<int>(Part::end))
		return libaction::BodyPart::PartIndex::end;
	return static_cast<libaction::BodyPart::PartIndex>(static_cast<int>(part));
}

}
}
}
}

#endif
