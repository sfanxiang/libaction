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
#include "../detail/fuzz.hpp"

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
	template<typename Needed, typename StillEstimator, typename ImagePtr>
	inline std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
	estimate(
		size_t pos, size_t length, size_t fuzz_range, size_t fuzz_rate,
		const Needed &needed,
		StillEstimator &still_estimator,
		std::function<ImagePtr(size_t pos)> &callback
	) {
		if (length == 0) {
			throw std::runtime_error("length == 0");
		}
		if (length <= pos) {
			throw std::runtime_error("length <= pos");
		}

		std::function<libaction::Human*(size_t, bool)> fuzz_cb =
			[pos, length, &still_estimator, &callback, this]
				(size_t offset, bool left) -> libaction::Human* {
			if (left) {
				if (offset > pos) {
					return nullptr;
				} else {
					auto it = still_poses.find(pos - offset);
					if (it == still_poses.end()) {
						it = estimate_still_pose(pos - offset, still_estimator, callback);
					}
					return it->second.get();
				}
			} else {
				if (offset >= length - pos) {
					return nullptr;
				} else {
					auto it = still_poses.find(pos + offset);
					if (it == still_poses.end()) {
						it = estimate_still_pose(pos + offset, still_estimator, callback);
					}
					return it->second.get();
				}
			}
		};

		auto human = detail::fuzz::fuzz(fuzz_range, fuzz_rate, needed, fuzz_cb);

		return get_human_pose(human);
	}

	/// Reset the status of Estimator.

	///	This is necessary when the stream or parameter is changed.
	inline void reset()
	{
		still_poses.clear();
	}

private:
	std::unordered_map<size_t, std::unique_ptr<libaction::Human>> still_poses{};

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
