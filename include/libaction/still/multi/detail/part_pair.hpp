/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__STILL__MULTI__DETAIL__PART_PAIR_HPP_
#define LIBACTION__STILL__MULTI__DETAIL__PART_PAIR_HPP_

#include <cstddef>
#include <utility>

namespace libaction
{
namespace still
{
namespace multi
{
namespace detail
{

class PartPair
{
public:
	inline PartPair() {}

	inline PartPair(float sc, size_t pi1, size_t pi2, size_t i1, size_t i2,
		std::pair<float, float> cd1, std::pair<float, float> cd2,
		float sc1, float sc2)
	:
		score_(sc), part_idx1_(pi1), part_idx2_(pi2), idx1_(i1), idx2_(i2),
		coord1_(cd1), coord2_(cd2), score1_(sc1), score2_(sc2)
	{}

	inline float score() const { return score_; }
	inline size_t part_idx1() const { return part_idx1_; }
	inline size_t part_idx2() const { return part_idx2_; }
	inline size_t idx1() const { return idx1_; }
	inline size_t idx2() const { return idx2_; }
	inline std::pair<float, float> coord1() const { return coord1_; }
	inline std::pair<float, float> coord2() const { return coord2_; }
	inline float score1() const { return score1_; }
	inline float score2() const { return score2_; }

private:
	float score_{};
	size_t part_idx1_{}, part_idx2_{};
	size_t idx1_{}, idx2_{};
	std::pair<float, float> coord1_{}, coord2_{};
	float score1_{}, score2_{};
};

}
}
}
}

#endif
