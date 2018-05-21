#ifndef LIBACTION_MULTI_HUMAN_HPP_
#define LIBACTION_MULTI_HUMAN_HPP_

#include "body_part.hpp"
#include "part_pair.hpp"

#include <map>
#include <set>
#include <utility>
#include <vector>

namespace libaction
{
namespace multi
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

	inline const std::map<coco_parts::Part, BodyPart> &get_body_parts() const
	{
		return body_parts;
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
		for (auto &part: other.body_parts) {
			body_parts[part.first] = part.second;
		}
		for (auto &uidx: other.uidx_set) {
			uidx_set.insert(uidx);
		}
	}

	inline size_t part_count() const
	{
		return body_parts.size();
	}

	inline float max_score() const
	{
		float max = 0.0f;
		for (auto &x: body_parts) {
			if (x.second.score() > max)
				max = x.second.score();
		}
		return max;
	}

private:
	std::set<std::pair<size_t, size_t>> uidx_set{};	// part_idx, idx
	std::map<coco_parts::Part, BodyPart> body_parts{};

	inline void add_pair(const PartPair &pair)
	{
		if (pair.part_idx1() >= static_cast<size_t>(coco_parts::Part::end) ||
				pair.part_idx2() >= static_cast<size_t>(coco_parts::Part::end))
			return;

		body_parts[static_cast<coco_parts::Part>(pair.part_idx1())] =
			BodyPart(pair.part_idx1(), pair.coord1().first,
				pair.coord1().second, pair.score());
		body_parts[static_cast<coco_parts::Part>(pair.part_idx2())] =
			BodyPart(pair.part_idx2(), pair.coord2().first,
				pair.coord2().second, pair.score());
		uidx_set.insert(std::make_pair(pair.part_idx1(), pair.idx1()));
		uidx_set.insert(std::make_pair(pair.part_idx2(), pair.idx2()));
	}
};

}
}

#endif