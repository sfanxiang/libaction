/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__SINGLE__ESTIMATOR_HPP_
#define LIBACTION__MOTION__SINGLE__ESTIMATOR_HPP_

#include "libaction/body_part.hpp"
#include "libaction/human.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>

namespace libaction
{
namespace motion
{
namespace single
{

/// Single-person motion estimator.
class Estimator
{
public:
	/// Constructor.
	inline Estimator();

	template<typename StillEstimator, typename ImagePtr>
	inline std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
	estimate(
		size_t pos, size_t length, size_t rate, size_t fuzz_range,
		StillEstimator &still_estimator,
		std::unique_ptr<std::function<ImagePtr(size_t pos)>> callback
	) {
		if (length == 0) {
			throw std::runtime_error("length == 0");
		}

		auto pose_pos = poses.find(pos);
		if (pose_pos != poses.end()) {
			// return existing pose
			if (*pose_pos) {
				return std::unique_ptr<std::unordered_map<size_t, libaction::Human>>(
					new std::unordered_map<size_t, libaction::Human>>(
						{ 0, **pose_pos }));
			} else {
				return std::unique_ptr<std::unordered_map<size_t, libaction::Human>>(
					new std::unordered_map<size_t, libaction::Human>>());
			}
		}

		auto pose_initial = poses.find(0);
		if (pose_initial == poses.end()) {
			// the initial frame has not been processed
			pose_initial = process_initial_frame(still_estimator, callback);
		}
	}

	inline void reset()
	{
	}

private:
	std::unordered_map<size_t, std::unique_ptr<libaction::Human>> poses{};
	std::unordered_set<libaction::BodyPart::PartIndex> avail{};

	/// Process the unprocessed initial frame.
	void process_initial_frame(StillEstimator &still_estimator,
		std::unique_ptr<std::function<ImagePtr(size_t pos)>> callback)
	{
		auto humans = still_estimator.estimate((*callback)(pos));
		std::unique_ptr<libaction::Human> human;

		if (!humans->empty()) {
			human = std::unique_ptr<libaction::Human>(
				new libaction::Human(std::move(*humans->begin())));
		}

		auto pose_initial = poses.insert(std::make_pair(0, std::move(human)))
			.first;

		// body parts present in the initial frame are considered available
		avail.reset();
		if (*pose_initial) {
			for (auto &part: (*pose_initial)->body_parts()) {
				avail.insert(part.first);
			}
		}

		return pose_initial;
	}
};

}
}
}

#endif
