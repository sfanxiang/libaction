/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__SINGLE__ESTIMATOR_HPP_
#define LIBACTION__MOTION__SINGLE__ESTIMATOR_HPP_

#include "../../body_part.hpp"
#include "../../human.hpp"
#include "../detail/angle.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
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
	inline Estimator()
	{}

	/// TODO.
	template<typename StillEstimator, typename ImagePtr>
	inline std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
	estimate(
		size_t pos, size_t length, size_t fuzz_range, size_t fuzz_rate,
		StillEstimator &still_estimator,
		std::function<ImagePtr(size_t pos)> &callback
	) {
		if (length == 0) {
			throw std::runtime_error("length == 0");
		}

		// the initial pose
		auto initial_it = still_poses.find(0);
		if (initial_it == still_poses.end()) {
			// the initial frame has not been processed
			initial_it = process_initial_frame(still_estimator, callback);
		}

		// the pose at pos
		decltype(still_poses)::iterator it;
		if (pos == 0) {
			it = initial_it;
		} else {
			it = still_poses.find(pos);
			if (it == still_poses.end()) {
				// estimate the pose at pos
				it = estimate_still_pose(pos, still_estimator, callback);
			}
		}



		return get_human_pose(*it);
	}

	/// Reset the status of Estimator.

	///	This is necessary when the stream or parameter is changed.
	inline void reset()
	{
		still_poses.clear();
		avail.clear();
	}

private:
	std::unordered_map<size_t, std::unique_ptr<libaction::Human>> still_poses{};
	std::unordered_set<libaction::BodyPart::PartIndex> avail{};

	/// Process the unprocessed initial frame.
	template<typename StillEstimator, typename ImagePtr>
	inline decltype(still_poses)::iterator process_initial_frame(
		StillEstimator &still_estimator,
		std::function<ImagePtr(size_t pos)> &callback)
	{
		// estimate the pose at 0
		auto initial_it = estimate_pose_still(0, still_estimator, callback);

		// body parts present in the initial frame are considered available
		avail.clear();
		if (initial_it->second) {
			for (auto &part: initial_it->second->body_parts()) {
				avail.insert(part.first);
			}
		}

		return initial_it;
	}

	/// Estimate the pose at a position using a still estimator.
	template<typename StillEstimator, typename ImagePtr>
	inline decltype(still_poses)::iterator estimate_still_pose(size_t pos,
		StillEstimator &still_estimator,
		std::function<ImagePtr(size_t pos)> &callback)
	{
		auto image = callback(pos);
		auto humans = still_estimator.estimate(*image);
		image.reset();

		std::unique_ptr<libaction::Human> human;

		if (!humans->empty()) {
			human = std::unique_ptr<libaction::Human>(
				new libaction::Human(std::move(*humans->begin())));
		}

		auto it = still_poses.insert(std::make_pair(pos, std::move(human)))
			.first;

		return it;
	}

	/// Get the processed human pose to return to the user.
	inline std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
	get_human_pose(const std::unique_ptr<libaction::Human> &human)
	{
		if (human) {
			return std::unique_ptr<std::unordered_map<size_t, libaction::Human>>(
				new std::unordered_map<size_t, libaction::Human>(
					{ std::make_pair(0, *human) }));
		} else {
			return std::unique_ptr<std::unordered_map<size_t, libaction::Human>>(
				new std::unordered_map<size_t, libaction::Human>());
		}
	}
};

}
}
}

#endif
