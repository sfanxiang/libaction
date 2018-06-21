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
#include "../../still/single/zoom.hpp"
#include "../detail/fuzz.hpp"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <type_traits>
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
	/// @param[in]  zoom_range  The range of images used for zoom reestimation.
	/// @param[in]  zoom_rate   The stride used for zoom reestimation. Must be
	///                         greater than 0.
	/// @param[in]  still_estimator An initialized human pose estimator.
	/// @param[in]  callback    A callback function allowing random access to
	///                         the image frame at `pos`. The callback should
	///                         return a valid pointer to the image, which can
	///                         be passed to `still_estimator.estimate()`.
	/// @return                 A map of humans from their index numbers.
	/// @exception              std::runtime_error
	/// @sa                     still::single::Estimator and
	///                         still::single::zoom::zoom_estimate
	template<typename StillEstimator, typename ImagePtr>
	inline std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
	estimate(
		size_t pos, size_t length,
		size_t fuzz_range, size_t fuzz_rate,
		size_t zoom_range, size_t zoom_rate,
		StillEstimator &still_estimator,
		const std::function<ImagePtr(size_t pos)> &callback
	) {
		if (length == 0)
			throw std::runtime_error("length == 0");
		if (length <= pos)
			throw std::runtime_error("length <= pos");
		if (fuzz_rate == 0)
			throw std::runtime_error("fuzz_rate == 0");
		if (zoom_rate == 0)
			throw std::runtime_error("zoom_rate == 0");

		std::function<std::pair<bool, const libaction::Human *>(size_t, bool)> fuzz_cb
			= [pos, length, zoom_range, zoom_rate, &still_estimator, &callback, this]
				(size_t offset, bool left) -> std::pair<bool, const libaction::Human *>
		{
			size_t pos = pos;

			if (pos >= length)
				return std::make_pair(false, nullptr);

			// get the real pos
			if (left) {
				if (offset > pos)
					return std::make_pair(false, nullptr);
				else
					pos -= offset;
			} else {
				if (offset >= length - pos)
					return std::make_pair(false, nullptr);
				else
					pos += offset;
			}

			{
				// Does pos already exist in still_poses?
				auto it = still_poses.find(pos);
				if (it != still_poses.end())
					return std::make_pair(true, it->second.get());
			}

			// pos does not exist in still_poses. We need to estimate it.

			if (needs_zoom(pos, zoom_rate)) {
				// the image at pos needs to be zoomed

				ImagePtr image;	// image at pos

				// make sure that pos exists in unzoomed_still_poses
				auto unzoomed_it = unzoomed_still_poses.find(pos);
				if (unzoomed_it == unzoomed_still_poses.end()) {
					std::tie(image, unzoomed_it) =
						estimate_still_pose_from_callback_on(
							pos, callback, still_estimator,
							unzoomed_still_poses
						);
				}

				if (unzoomed_it->second) {
					// human found in the unzoomed image

					// now prepare the hints for a zoomed estimation

					size_t l = (pos >= zoom_range ? pos - zoom_range : 0);
					size_t r = (length - pos > zoom_range ? pos + zoom_range : length - 1);
					std::vector<const libaction::Human *> hints;

					// make all unzoomed estimations available within zoom_range
					for (size_t i = l; i <= r; i++) {
						if (i == pos)
							continue;

						decltype(still_poses)::iterator it;
						if (needs_zoom(i, zoom_rate)) {
							// unzoomed estimations for images which should be
							// zoomed go to unzoomed_still_poses

							it = unzoomed_still_poses.find(i);
							if (it == unzoomed_still_poses.end()) {
								it = estimate_still_pose_from_callback_on(
									i, callback, still_estimator,
									unzoomed_still_poses
								).second;
							}
						} else {
							// estimations for images which should not be zoomed
							// go to still_poses

							it = still_poses.find(i);
							if (it == still_poses.end()) {
								it = estimate_still_pose_from_callback_on(
									i, callback, still_estimator,
									still_poses
								).second;
							}
						}

						if (it->second)	// a useful hint
							hints.push_back(it->second.get());
					}

					if (!image) {
						// image hasn't been retrieved from callback
						image = get_image_from_callback(pos, callback);
					}

					// zoom estimate
					auto human = libaction::still::single::zoom::zoom_estimate(
						*image, *unzoomed_it->second, hints,
						std::bind(
							&Estimator::estimate_still_pose_from_image,
							std::placeholders::_1,
							still_estimator
						));

					// zoomed estimations for images which should be zoomed go
					// to still_poses
					still_poses.insert(std::make_pair(pos, human));

					return std::make_pair(true, human.get());
				} else {
					// no human found in unzoomed image
					// impossible to do zoomed estimation
					still_poses.insert(std::make_pair(pos, std::unique_ptr<libaction::Human>()));

					return std::make_pair(true, nullptr);
				}
			} else {
				// the image at pos does not need to be zoomed

				auto it = estimate_still_pose_from_callback_on(
					pos, callback, still_estimator, still_poses).second;

				return std::make_pair(true, it->second.get());
			}
		};

		auto human = detail::fuzz::fuzz(fuzz_range, fuzz_rate, fuzz_cb);

		return get_human_pose(human);
	}

	/// Reset the status of Estimator.

	///	This is necessary when the image stream is changed.
	inline void reset()
	{
		unzoomed_still_poses.clear();
		still_poses.clear();
	}

private:
	// poses which should be zoomed, estimated on their unzoomed image
	std::unordered_map<size_t, std::unique_ptr<libaction::Human>> unzoomed_still_poses{};

	// poses estimated on their zoomed image if they should be zoomed,
	// otherwise poses estimated on their unzoomed image
	std::unordered_map<size_t, std::unique_ptr<libaction::Human>> still_poses{};

	static constexpr inline bool needs_zoom(size_t pos, size_t zoom_rate)
	{
		return (zoom_rate != 0) && (pos % zoom_rate == 0);
	}

	template<typename ImagePtr>
	static inline ImagePtr get_image_from_callback(
		size_t pos, const std::function<ImagePtr(size_t pos)> &callback
	) {
		auto image = callback(pos);
		if (!image)
			throw std::runtime_error("image callback returned null");
		return image;
	}

	template<typename StillEstimator, typename Image>
	static inline std::unique_ptr<libaction::Human>
	estimate_still_pose_from_image(
		const Image &image,
		StillEstimator &still_estimator)
	{
		auto humans = still_estimator.estimate(image);

		std::unique_ptr<libaction::Human> human;
		if (!humans->empty()) {
			human = std::unique_ptr<libaction::Human>(
				new libaction::Human(std::move(*humans->begin())));
		}

		return human;
	}

	template<typename StillEstimator, typename Image>
	static inline auto estimate_still_pose_from_image_on(
		size_t pos,
		const Image &image,
		StillEstimator &still_estimator,
		std::unordered_map<size_t, std::unique_ptr<libaction::Human>> &poses)
	-> std::remove_reference<decltype(poses)>::type::iterator
	{
		auto human = estimate_still_pose_from_image(image, still_estimator);
		auto it = poses.insert(std::make_pair(pos, std::move(human)))
			.first;

		return it;
	}

	template<typename StillEstimator, typename ImagePtr>
	static inline auto estimate_still_pose_from_callback_on(
		size_t pos,
		const std::function<ImagePtr(size_t pos)> &callback,
		StillEstimator &still_estimator,
		std::unordered_map<size_t, std::unique_ptr<libaction::Human>> &poses)
	-> std::pair<ImagePtr, std::remove_reference<decltype(poses)>::type::iterator>
	{
		auto image = get_image_from_callback(pos, callback);

		auto it = estimate_still_pose_from_image_on(pos, *image,
			still_estimator, poses);

		return std::make_pair(image, it);
	}

	/// Get the processed human pose to return to the user.
	inline static std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
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
