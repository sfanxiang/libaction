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

	/// Estimate for a single frame from a series of motion images.

	/// @param[in]  pos         The current index of the frame, starting from 0.
	/// @param[in]  length      The total number of frames. Must be greater than
	///                         `pos`.
	/// @param[in]  fuzz_range  The range of images used for fuzz estimation.
	///                         The distance between the right frame and the
	///                         left frame is at most `fuzz_range`.
	/// @param[in]  fuzz_rate   The stride used for fuzz estimation. Must be
	///                         greater than 0.
	/// @param[in]  still_estimator An initialized human pose estimator.
	/// @param[in]  callback    A callback function allowing random access to
	///                         the image frame at `pos`. The callback should
	///                         return a valid pointer to the image, which can
	///                         be passed to `still_estimator.estimate()`.
	/// @return                 A map of humans from their index numbers.
	/// @exception              std::runtime_error
	/// @sa                     still::single::Estimator
	template<typename StillEstimator, typename ImagePtr>
	inline std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
	estimate(
		size_t pos, size_t length, size_t fuzz_range, size_t fuzz_rate,
		StillEstimator &still_estimator,
		const std::function<ImagePtr(size_t pos)> &callback
	) {
		if (length == 0) {
			throw std::runtime_error("length == 0");
		}
		if (length <= pos) {
			throw std::runtime_error("length <= pos");
		}
		if (fuzz_rate == 0) {
			throw std::runtime_error("fuzz_rate == 0");
		}

		std::function<std::pair<bool, const libaction::Human *>(size_t, bool)> fuzz_cb
			= [pos, length, &still_estimator, &callback, this]
				(size_t offset, bool left) -> std::pair<bool, const libaction::Human *>
		{
			if (left) {
				if (offset > pos) {
					return std::make_pair(false, nullptr);
				} else {
					auto it = still_poses.find(pos - offset);
					if (it == still_poses.end()) {
						it = estimate_still_pose(pos - offset, still_estimator, callback);
					}
					return std::make_pair(true, it->second.get());
				}
			} else {
				if (offset >= length - pos) {
					return std::make_pair(false, nullptr);
				} else {
					auto it = still_poses.find(pos + offset);
					if (it == still_poses.end()) {
						it = estimate_still_pose(pos + offset, still_estimator, callback);
					}
					return std::make_pair(true, it->second.get());
				}
			}
		};

		auto human = detail::fuzz::fuzz(fuzz_range, fuzz_rate, fuzz_cb);

		return get_human_pose(human);
	}

	/// Reset the status of Estimator.

	///	This is necessary when the image stream is changed.
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
		const std::function<ImagePtr(size_t pos)> &callback)
	{
		decltype(still_estimator.estimate(*callback(pos))) humans;

		{
			auto image = callback(pos);
			if (!image)
				throw std::runtime_error("callback returned an empty pointer");
			humans = still_estimator.estimate(*image);
		}

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
