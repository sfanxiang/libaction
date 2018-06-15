/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__STILL__MULTI__DETAIL__HUMAN_HPP_
#define LIBACTION__STILL__MULTI__DETAIL__HUMAN_HPP_

#include "body_part.hpp"
#include "part_pair.hpp"

#include <map>
#include <set>
#include <utility>
#include <vector>

namespace libaction
{
namespace still
{
namespace multi
{
namespace detail
{

class Human
{
public:
	template<typename Pairs>
	Human(const Pairs &p)
	{
		for (auto &pair: p) {
			add_pair(pair);
		}
	}

	inline const std::map<coco_parts::Part, BodyPart> &body_parts() const
	{
		return body_parts_;
	}

	inline bool is_connected(const Human &other) const
	{
		const auto &set1 = uidx_set.size() <= other.uidx_set.size() ?
			uidx_set : other.uidx_set;
		const auto &set2 = uidx_set.size() <= other.uidx_set.size() ?
			other.uidx_set : uidx_set;
		for (const auto &uidx: set1) {
			if (set2.find(uidx) != set2.end())
				return true;
		}
		return false;
	}

	inline void merge(const Human &other)
	{
		for (auto &part: other.body_parts_) {
			body_parts_[part.first] = part.second;
		}
		for (auto &uidx: other.uidx_set) {
			uidx_set.insert(uidx);
		}
	}

	inline size_t part_count() const
	{
		return body_parts_.size();
	}

	inline float max_score() const
	{
		float max = 0.0f;
		for (auto &x: body_parts_) {
			if (x.second.score() > max)
				max = x.second.score();
		}
		return max;
	}

private:
	std::set<std::pair<size_t, size_t>> uidx_set{};	// part_idx, idx
	std::map<coco_parts::Part, BodyPart> body_parts_{};

	inline void add_pair(const PartPair &pair)
	{
		if (pair.part_idx1() >= static_cast<size_t>(coco_parts::Part::end) ||
				pair.part_idx2() >= static_cast<size_t>(coco_parts::Part::end))
			return;

		body_parts_[static_cast<coco_parts::Part>(pair.part_idx1())] =
			BodyPart(pair.part_idx1(), pair.coord1().first,
				pair.coord1().second, pair.score());
		body_parts_[static_cast<coco_parts::Part>(pair.part_idx2())] =
			BodyPart(pair.part_idx2(), pair.coord2().first,
				pair.coord2().second, pair.score());
		uidx_set.insert(std::make_pair(pair.part_idx1(), pair.idx1()));
		uidx_set.insert(std::make_pair(pair.part_idx2(), pair.idx2()));
	}
};

}
}
}
}

#endif
