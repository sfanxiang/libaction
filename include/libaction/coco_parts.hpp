#ifndef LIBACTION_COCO_PARTS_HPP_
#define LIBACTION_COCO_PARTS_HPP_

#include <array>
#include <utility>

namespace libaction
{
namespace coco_parts
{

enum class Part
{
	nose = 0,
	neck = 1,
	shoulder_r = 2,
	elbow_r = 3,
	wrist_r = 4,
	shoulder_l = 5,
	elbow_l = 6,
	wrist_l = 7,
	hip_r = 8,
	knee_r = 9,
	ankle_r = 10,
	hip_l = 11,
	knee_l = 12,
	ankle_l = 13,
	eye_r = 14,
	eye_l = 15,
	ear_r = 16,
	ear_l = 17,
	background = 18
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

}
}

#endif
