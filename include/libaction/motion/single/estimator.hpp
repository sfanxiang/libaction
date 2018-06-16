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
#include "libaction/motion/detail/angle.hpp"

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

	/// TODO.
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
			return human_pose_at(pose_pos);
		}

		auto pose_initial = poses.find(0);
		if (pose_initial == poses.end()) {
			// the initial frame has not been processed
			pose_initial = process_initial_frame(still_estimator, callback);
		}

		// estimate the pose at pos
		pose_pos = estimate_pose_still(pos, still_estimator, callback);

		return human_pose_at(pose_pos);
	}

	/// Reset the status of Estimator.

	///	This is necessary when the stream or parameter is changed.
	inline void reset()
	{
		poses.clear();
		avail.clear();
	}

private:
	std::unordered_map<size_t, std::unique_ptr<libaction::Human>> poses{};
	std::unordered_set<libaction::BodyPart::PartIndex> avail{};

	/// Process the unprocessed initial frame.
	decltype(poses)::iterator process_initial_frame(
		StillEstimator &still_estimator,
		std::unique_ptr<std::function<ImagePtr(size_t pos)>> callback)
	{
		// estimate the pose at 0
		auto pose_initial = estimate_pose_still(0, still_estimator, callback);

		// body parts present in the initial frame are considered available
		avail.reset();
		if (*pose_initial) {
			for (auto &part: (*pose_initial)->body_parts()) {
				avail.insert(part.first);
			}
		}

		return pose_initial;
	}

	/// Estimate the pose at a position using a still estimator.
	decltype(poses)::iterator estimate_pose_still(size_t pos,
		StillEstimator &still_estimator,
		std::unique_ptr<std::function<ImagePtr(size_t pos)>> callback)
	{
		auto humans = still_estimator.estimate((*callback)(pos));
		std::unique_ptr<libaction::Human> human;

		if (!humans->empty()) {
			human = std::unique_ptr<libaction::Human>(
				new libaction::Human(std::move(*humans->begin())));
		}

		pose_pos = poses.insert(std::make_pair(pos, std::move(human))).first;

		return pose_pos;
	}

	/// Get the processed human pose to return to the user.
	std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
	human_pose_at(decltype(poses)::iterator it)
	{
		if (it->second) {
			return std::unique_ptr<std::unordered_map<size_t, libaction::Human>>(
				new std::unordered_map<size_t, libaction::Human>>(
					{ 0, *it->second }));
		} else {
			return std::unique_ptr<std::unordered_map<size_t, libaction::Human>>(
				new std::unordered_map<size_t, libaction::Human>>());
		}
	}
};

}
}
}

#endif