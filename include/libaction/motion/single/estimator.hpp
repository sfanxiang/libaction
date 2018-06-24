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

#include <boost/multi_array.hpp>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

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
	///                         left frame is at most `fuzz_range`. To turn off
	///                         fuzz estimation, set `fuzz_range` to 0.
	/// @param[in]  zoom        Whether zoom estimation should be enabled.
	/// @param[in]  zoom_range  The range of images used for zoom reestimation.
	///                         If `zoom_range` is 0, or no useful image is
	///                         found within the range, then a prebuilt zoom
	///                         parameter is used.
	/// @param[in]  zoom_rate   The stride used for zoom reestimation. Must be
	///                         greater than 0.
	/// @param[in]  still_estimators A vector of one or more initialized human
	///                              pose estimators, whose `estimate()` method
	///                              must accept any image conforming to the
	///                              Boost.MultiArray concept.
	/// @param[in]  callback    A callback function allowing random access to
	///                         the image frame at `pos`. The callback should
	///                         return a valid pointer to the image, which must
	///                         conform to the Boost.MultiArray concept.
	/// @return                 A map of humans from their index numbers.
	/// @exception              std::runtime_error
	/// @sa                     still::single::Estimator and
	///                         still::single::zoom::zoom_estimate
	template<typename StillEstimator, typename ImagePtr>
	inline std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
	estimate(
		size_t pos, size_t length,
		size_t fuzz_range,
		bool zoom, size_t zoom_range, size_t zoom_rate,
		const std::vector<StillEstimator*> &still_estimators,
		const std::function<ImagePtr(size_t pos)> &callback
	) {
		if (length == 0)
			throw std::runtime_error("length == 0");
		if (length <= pos)
			throw std::runtime_error("length <= pos");
		if (zoom_rate == 0)
			throw std::runtime_error("zoom_rate == 0");
		if (still_estimators.empty())
			throw std::runtime_error("still_estimators is empty");

		if (still_estimators.size() > 1) {
			// multi-thread support (preprocessing)

			// prioriry order:
			std::unordered_set<size_t>
				frames_unzoomed_1, frames_zoomed, frames_unzoomed_2;

			size_t fuzz_l, fuzz_r;
			std::tie(fuzz_l, fuzz_r) =
				detail::fuzz::get_fuzz_lr(pos, length, fuzz_range);

			for (size_t i = fuzz_l; i <= fuzz_r; i++) {
				frames_unzoomed_2.insert(i);
			}

			if (zoom) {
				for (size_t i = fuzz_l; i <= fuzz_r; i++) {
					if (needs_zoom(zoom, i, zoom_rate)) {
						size_t zoom_l, zoom_r;
						std::tie(zoom_l, zoom_r) = libaction::still::single
							::zoom::get_zoom_lr(pos, length, zoom_range);

						for (size_t j = zoom_l; j <= zoom_r; j++) {
							frames_unzoomed_2.erase(j);
							frames_unzoomed_1.insert(j);
						}

						frames_zoomed.insert(i);
					}
				}
			}

			std::mutex mutex;
			// TODO
		}

		std::function<std::pair<bool, const libaction::Human *>(size_t, bool)> fuzz_cb
			= [pos, length, zoom, zoom_range, zoom_rate, &still_estimators, &callback, this]
				(size_t offset, bool left) -> std::pair<bool, const libaction::Human *>
		{
			return fuzz_callback(pos, length, zoom, zoom_range, zoom_rate,
				still_estimators, callback, offset, left);
		};

		auto human = detail::fuzz::fuzz(fuzz_range, fuzz_cb);

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

	static constexpr inline bool needs_zoom(bool zoom, size_t pos, size_t zoom_rate)
	{
		return zoom && (zoom_rate != 0) && (pos % zoom_rate == 0);
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

		return std::make_pair(std::move(image), it);
	}

	template<typename StillEstimator, typename ImagePtr>
	void concurrent_preestimate(
		size_t zoom_range,
		const std::vector<StillEstimator*> &still_estimators,
		const std::function<ImagePtr(size_t pos)> &callback,
		std::unordered_set<size_t> frames_unzoomed_1,
		std::unordered_set<size_t> frames_zoomed,
		std::unordered_set<size_t> frames_unzoomed_2,
		std::mutex mutex)
	{

	}

	template<typename StillEstimator, typename ImagePtr>
	std::pair<bool, const libaction::Human *>
	fuzz_callback(size_t pos, size_t length,
		bool zoom, size_t zoom_range, size_t zoom_rate,
		const std::vector<StillEstimator*> &still_estimators,
		const std::function<ImagePtr(size_t pos)> &callback,
		size_t offset, bool left)
	{
		auto &still_estimator = **still_estimators.begin();

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

		// single-thread support

		// pos does not exist in still_poses. We need to estimate it.

		if (needs_zoom(zoom, pos, zoom_rate)) {
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

				size_t l, r;
				std::tie(l, r) = libaction::still::single::zoom::get_zoom_lr(
					pos, length, zoom_range);

				std::vector<const libaction::Human *> hints;

				// make all unzoomed estimations available within zoom_range
				for (size_t i = l; i <= r; i++) {
					if (i == pos)
						continue;

					decltype(still_poses)::iterator it;
					if (needs_zoom(zoom, i, zoom_rate)) {
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
				using zoom_cb_arg = boost::multi_array<typename
					std::remove_reference<decltype(*image)>::type::element,
					3
				>;
				std::function<std::unique_ptr<libaction::Human>
					(const zoom_cb_arg&)> zoom_cb =
				[&still_estimator] (const zoom_cb_arg &image_to_estimate) {
					return estimate_still_pose_from_image(image_to_estimate,
						still_estimator);
				};
				auto human = libaction::still::single::zoom::zoom_estimate(
					*image, *unzoomed_it->second, hints, zoom_cb);

				// zoomed estimations for images which should be zoomed go
				// to still_poses
				auto it = still_poses.insert(
					std::make_pair(pos, std::move(human))
				).first;

				return std::make_pair(true, it->second.get());
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
