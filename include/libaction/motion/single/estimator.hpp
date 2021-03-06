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
#include "anti_crossing.hpp"
#include "fuzz.hpp"

#include <boost/multi_array.hpp>
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>
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

/// @warning This class is not thread safe, although it contains multithread
///          features.
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
	///                         To turn off fuzz estimation, set `fuzz_range`
	///                         to 0.
	/// @param[in]  max_lengths Maximum lengths of body connections. The tuple
	///                         has a format of (from, to, length). If a
	///                         pair of body part (from, to) has a distance
	///                         longer than the length specified, the "to" part
	///                         will be removed before fuzz estimation.
	/// @param[in]  anti_crossing   Whether to enable anti crossing.
	/// @param[in]  zoom        Whether to enable zoom reestimation.
	/// @param[in]  zoom_range  The range of images used for zoom reestimation.
	/// @param[in]  zoom_rate   The stride used for zoom reestimation. Must be
	///                         greater than 0.
	/// @param[in]  still_estimators    A vector of one or more initialized
	///                         human pose estimators, whose `estimate` method
	///                         must accept any image conforming to the
	///                         Boost.MultiArray concept. If `still_estimators`
	///                         has multiple elements, the same number of
	///                         threads will be created.
	/// @param[in]  zoom_still_estimators   Estimators for zoom estimation. Can
	///                         be identical to `still_estimators`. Must have
	///                         the same number of elements as
	///                         `still_estimators` does. See the description of
	///                         `still_estimators`.
	/// @param[in]  callback    A callback function allowing random access to
	///                         the image frame at `pos`. `last_image_access`
	///                         indicates whether the image at `pos` is no
	///                         longer needed (if no error occurs). The same
	///                         image at `pos` may be retrieved multiple times.
	///                         The callback should return a valid pointer to
	///                         the image, which must conform to the
	///                         Boost.MultiArray concept.
	/// @warning                `callback` may be called concurrently from
	///                         different threads if `still_estimators` has
	///                         more than one element.
	/// @return                 A map of indexed humans. Index starts from 0.
	/// @exception              std::runtime_error
	/// @sa                     anti_crossing, fuzz, still::single::Estimator
	///                         and still::single::zoom
	template<typename StillEstimator, typename ZoomStillEstimator,
		typename ImagePtr>
	inline std::unique_ptr<std::unordered_map<std::size_t, libaction::Human>>
	estimate(
		std::size_t pos, std::size_t length,
		std::size_t fuzz_range,
		const std::vector<std::tuple<
			libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex,
			float>> &max_lengths,
		bool anti_crossing,
		bool zoom, std::size_t zoom_range, std::size_t zoom_rate,
		const std::vector<StillEstimator*> &still_estimators,
		const std::vector<ZoomStillEstimator*> &zoom_still_estimators,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback
	) {
		if (length == 0)
			throw std::runtime_error("length == 0");
		if (length <= pos)
			throw std::runtime_error("length <= pos");
		if (zoom_rate == 0)
			throw std::runtime_error("zoom_rate == 0");
		if (still_estimators.empty())
			throw std::runtime_error("still_estimators is empty");
		if (still_estimators.size() != zoom_still_estimators.size())
			throw std::runtime_error("still_estimators and "
				"zoom_still_estimators have different sizes");

		if (still_estimators.size() > 1) {
			// multi-thread support (preprocessing)

			// zoomed_used only tracks zoomed tasks of frames which should be
			// zoomed. unzoomed_used tracks unzoomed tasks of all frames.
			std::unordered_set<std::size_t> unzoomed_used, zoomed_used;	// pos

			// A task is removed from the queue only if it is certain to finish
			// without dependending on any unfinished task. If a task is in the
			// queue, then it has not been claimed yet.
			std::list<std::pair<std::size_t, bool>> queue;	// pos, zoomed

			// Populate the queue: this should cover every still estimation
			// possibly required for generating the return value.
			std::size_t range_l, range_r;
			std::tie(range_l, range_r) =
				fuzz::get_fuzz_lr(pos, length, fuzz_range);
			if (anti_crossing && range_l > 0)
				range_l--;
			if (anti_crossing && range_r < length - 1)
				range_r++;

			for (std::size_t i = range_l; i <= range_r; i++) {
				if (needs_zoom(zoom, i, zoom_rate) &&
						still_poses.find(i) == still_poses.end() &&
						zoomed_used.find(i) == zoomed_used.end()) {
					std::size_t zoom_l, zoom_r;
					std::tie(zoom_l, zoom_r) = libaction::still::single
						::zoom::get_zoom_lr(i, length, zoom_range);

					for (std::size_t j = zoom_l; j <= zoom_r; j++) {
						if (needs_zoom(zoom, j, zoom_rate)) {
							if (unzoomed_still_poses.find(j) == unzoomed_still_poses.end() &&
									unzoomed_used.find(j) == unzoomed_used.end()) {
								unzoomed_used.insert(j);
								queue.push_back({ j, false });
							}
						} else {
							if (still_poses.find(j) == still_poses.end() &&
									unzoomed_used.find(j) == unzoomed_used.end()) {
								unzoomed_used.insert(j);
								queue.push_back({ j, false });
							}
						}
					}

					zoomed_used.insert(i);
					queue.push_back({ i, true });
				} else if (!needs_zoom(zoom, i, zoom_rate) &&
						still_poses.find(i) == still_poses.end() &&
						unzoomed_used.find(i) == unzoomed_used.end()) {
					unzoomed_used.insert(i);
					queue.push_back({ i, false });
				}
			}

			std::list<std::pair<std::size_t, bool>> extra_queue;	// pos, zoomed

			// Add extra tasks to make multithread truly effective.
			for (std::size_t i = range_r + 1; ; ) {
				// if the queue is empty, we don't need extra
				if (queue.empty())
					break;

				if (i >= length) {
					if (range_l == 0)
						break;
					else
						i = range_l - 1;
				}

				if (needs_zoom(zoom, i, zoom_rate) &&
						still_poses.find(i) == still_poses.end() &&
						zoomed_used.find(i) == zoomed_used.end()) {
					std::size_t zoom_l, zoom_r;
					std::tie(zoom_l, zoom_r) = libaction::still::single
						::zoom::get_zoom_lr(i, length, zoom_range);

					for (std::size_t j = zoom_l; j <= zoom_r; j++) {
						if (needs_zoom(zoom, j, zoom_rate)) {
							if (unzoomed_still_poses.find(j) == unzoomed_still_poses.end() &&
									unzoomed_used.find(j) == unzoomed_used.end()) {
								unzoomed_used.insert(j);
								extra_queue.push_back({ j, false });
							}
						} else {
							if (still_poses.find(j) == still_poses.end() &&
									unzoomed_used.find(j) == unzoomed_used.end()) {
								unzoomed_used.insert(j);
								extra_queue.push_back({ j, false });
							}
						}
					}

					zoomed_used.insert(i);
					extra_queue.push_back({ i, true });
				} else if (!needs_zoom(zoom, i, zoom_rate) &&
						still_poses.find(i) == still_poses.end() &&
						unzoomed_used.find(i) == unzoomed_used.end()) {
					unzoomed_used.insert(i);
					extra_queue.push_back({ i, false });
				}

				// increment
				if (i > range_r) {
					i++;
					if (i >= length) {
						if (range_l == 0)
							break;
						else
							i = range_l - 1;
					}
				} else {
					if (i == 0)
						break;
					i--;
				}
			}

			if (!queue.empty()) {
				// start the threads

				std::mutex mutex;
				std::condition_variable cv;
				std::vector<std::unique_ptr<bool>> statuses;
				bool end = false;

				std::vector<std::thread> threads;

				auto it = still_estimators.begin();
				auto zoom_it = zoom_still_estimators.begin();

				for (; it != still_estimators.end()
						&& zoom_it != zoom_still_estimators.end();
					it++, zoom_it++)
				{
					statuses.push_back(std::unique_ptr<bool>(
						new bool(false)));

					threads.push_back(std::thread(
						std::bind(
							&Estimator::concurrent_preestimate<
								StillEstimator, ZoomStillEstimator, ImagePtr>,
							this,
							length, zoom, zoom_range, zoom_rate,
							std::ref(**it), std::ref(**zoom_it),
							std::cref(callback),
							std::ref(queue), std::ref(extra_queue),
							std::ref(mutex), std::ref(cv),
							std::ref(*statuses.back()), std::ref(end))));
				}

				std::unique_lock<std::mutex> lock(mutex);

				while (true) {
					// statuses are all false

					for (std::size_t i = 0; i < threads.size(); i++) {
						bool &status = *statuses[i];
						cv.wait(lock, [&status] { return status; });
					}

					// statuses are all true

					if (queue.empty())
						end = true;

					for (std::size_t i = 0; i < threads.size(); i++) {
						bool &status = *statuses[i];
						status = false;
					}

					// statuses are all false

					lock.unlock();
					cv.notify_all();
					lock.lock();

					if (end)
						break;
				}

				lock.unlock();

				for (auto &thread: threads)
					thread.join();
			}

			// TODO: Handle errors from other threads. If they are not handled,
			//       it's a waste of time to continue single-threadedly.
		}

		std::function<
				std::pair<bool, std::unique_ptr<const libaction::Human>>
				(std::size_t, bool)>
		fuzz_cb = [pos, length, &max_lengths, anti_crossing, zoom, zoom_range, zoom_rate, &still_estimators, &zoom_still_estimators, &callback, this]
			(std::size_t offset, bool left)
				-> std::pair<bool, std::unique_ptr<const libaction::Human>>
		{
			return fuzz_callback(pos, length, max_lengths, anti_crossing,
				zoom, zoom_range, zoom_rate,
				**still_estimators.begin(), **zoom_still_estimators.begin(),
				callback, offset, left);
		};

		auto human = fuzz::fuzz(fuzz_range, fuzz_cb);

		return get_human_pose(human);
	}

	/// Reset the status of Estimator.

	///	This is necessary when the stream is changed.
	inline void reset()
	{
		unzoomed_still_poses.clear();
		still_poses.clear();
	}

private:
	// poses which should be zoomed, estimated on their unzoomed image
	std::unordered_map<std::size_t, std::unique_ptr<libaction::Human>> unzoomed_still_poses{};

	// poses estimated on their zoomed image if they should be zoomed,
	// otherwise poses estimated on their unzoomed image
	std::unordered_map<std::size_t, std::unique_ptr<libaction::Human>> still_poses{};

	static inline constexpr bool needs_zoom(bool zoom, std::size_t pos, std::size_t zoom_rate)
	{
		return zoom && (zoom_rate != 0) && (pos % zoom_rate == 0);
	}

	inline bool zoom_estimation_possible(
		std::size_t pos, std::size_t length, std::size_t zoom_range, std::size_t zoom_rate)
	{
		if (pos >= length)
			return false;

		std::size_t l, r;
		std::tie(l, r) = libaction::still::single::zoom::get_zoom_lr(
			pos, length, zoom_range);

		for (std::size_t i = l; i <= r; i++) {
			if (needs_zoom(true, i, zoom_rate) &&
					unzoomed_still_poses.find(i) == unzoomed_still_poses.end())
				return false;
			if (!needs_zoom(true, i, zoom_rate) &&
					still_poses.find(i) == still_poses.end())
				return false;
		}

		return true;
	}

	template<typename ImagePtr>
	static inline ImagePtr get_image_from_callback(
		std::size_t pos, bool last_image_access,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback
	) {
		auto image = callback(pos, last_image_access);
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
		std::size_t pos,
		const Image &image,
		StillEstimator &still_estimator,
		std::unordered_map<std::size_t, std::unique_ptr<libaction::Human>> &poses)
	-> std::remove_reference<decltype(poses)>::type::iterator
	{
		auto human = estimate_still_pose_from_image(image, still_estimator);
		auto it = poses.insert(std::make_pair(pos, std::move(human)))
			.first;

		return it;
	}

	template<typename StillEstimator, typename ImagePtr>
	static inline auto estimate_still_pose_from_callback_on(
		std::size_t pos, bool last_image_access,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback,
		StillEstimator &still_estimator,
		std::unordered_map<std::size_t, std::unique_ptr<libaction::Human>> &poses)
	-> std::pair<ImagePtr, std::remove_reference<decltype(poses)>::type::iterator>
	{
		auto image = get_image_from_callback(pos, last_image_access, callback);

		auto it = estimate_still_pose_from_image_on(pos, *image,
			still_estimator, poses);

		return std::make_pair(std::move(image), it);
	}

	/// Try doing one task for concurrent(multithread) estimation. Returns
	/// (did_task, did_work).
	template<typename StillEstimator, typename ZoomStillEstimator,
		typename ImagePtr>
	std::pair<bool, bool> concurrent_preestimate_try_one(
		std::size_t length, bool zoom, std::size_t zoom_range, std::size_t zoom_rate,
		StillEstimator &still_estimator,
		ZoomStillEstimator &zoom_still_estimator,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback,
		std::list<std::pair<std::size_t, bool>> &queue,
		std::list<std::pair<std::size_t, bool>> &extra_queue,
		std::unique_lock<std::mutex> &lock)
	{
		std::size_t pos = 0;
		bool zoomed = false;

		// try to find the first possible task
		bool found = false;
		for (auto it = queue.begin(); it != queue.end(); it++) {
			if (it->second) {
				if (zoom_estimation_possible(it->first, length, zoom_range, zoom_rate)) {
					std::tie(pos, zoomed) = *it;
					found = true;
					queue.erase(it);
					break;
				}
			} else {
				std::tie(pos, zoomed) = *it;
				found = true;
				queue.erase(it);
				break;
			}
		}
		if (!found) {
			for (auto it = extra_queue.begin(); it != extra_queue.end(); it++) {
				if (it->second) {
					if (zoom_estimation_possible(it->first, length, zoom_range, zoom_rate)) {
						std::tie(pos, zoomed) = *it;
						found = true;
						extra_queue.erase(it);
						break;
					}
				} else {
					std::tie(pos, zoomed) = *it;
					found = true;
					extra_queue.erase(it);
					break;
				}
			}
		}
		if (!found)
			return std::make_pair(false, false);

		if (zoomed) {
			auto unzoomed_it = unzoomed_still_poses.find(pos);
			if (unzoomed_it == unzoomed_still_poses.end())
				throw std::runtime_error("cannot find frame in unzoomed_still_poses");

			if (unzoomed_it->second) {
				// human found in the unzoomed image

				// now prepare the hints for a zoomed estimation

				std::size_t l, r;
				std::tie(l, r) = libaction::still::single::zoom::get_zoom_lr(
					pos, length, zoom_range);

				std::vector<const libaction::Human *> hints;

				for (std::size_t i = l; i <= r; i++) {
					if (i == pos)
						continue;

					decltype(still_poses)::iterator it;
					if (needs_zoom(zoom, i, zoom_rate)) {
						// unzoomed estimations for images which should be
						// zoomed go to unzoomed_still_poses

						it = unzoomed_still_poses.find(i);
						if (it == unzoomed_still_poses.end())
							throw std::runtime_error("cannot find frame in unzoomed_still_poses");
					} else {
						// estimations for images which should not be zoomed
						// go to still_poses

						it = still_poses.find(i);
						if (it == still_poses.end())
							throw std::runtime_error("cannot find frame in still_poses");
					}

					if (it->second)	// a useful hint
						hints.push_back(it->second.get());
				}

				auto image = get_image_from_callback(pos, true, callback);

				// zoom estimate
				using zoom_cb_arg = boost::multi_array<typename
					std::remove_reference<decltype(*image)>::type::element,
					3
				>;
				std::function<std::unique_ptr<libaction::Human>
					(const zoom_cb_arg&)> zoom_cb =
				[&zoom_still_estimator, &lock]
				(const zoom_cb_arg &image_to_estimate) {
					// unlock and estimate
					lock.unlock();
					try {
						auto result = estimate_still_pose_from_image(
							image_to_estimate, zoom_still_estimator);
						lock.lock();
						return result;
					} catch (const std::runtime_error &) {
						lock.lock();
						throw;
					}
				};
				auto human = libaction::still::single::zoom::zoom_estimate(
					*image, *unzoomed_it->second, hints, zoom_cb);

				// zoomed estimations for images which should be zoomed go
				// to still_poses
				still_poses.insert(std::make_pair(pos, std::move(human)));

				return std::make_pair(true, true);
			} else {
				// no human found in unzoomed image
				// impossible to do zoomed estimation

				// a little bit wasteful, but we need to inform callback that
				// the image at pos is no longer needed
				get_image_from_callback(pos, true, callback);

				still_poses.insert(std::make_pair(pos, std::unique_ptr<libaction::Human>()));

				// finished one task, but no work is done
				return std::make_pair(true, false);
			}
		} else {
			bool eventually_zoom = needs_zoom(zoom, pos, zoom_rate);
			auto image = get_image_from_callback(pos, !eventually_zoom,
				callback);

			std::unique_ptr<libaction::Human> human;

			// unlock and estimate
			lock.unlock();
			try {
				human = estimate_still_pose_from_image(*image, still_estimator);
				lock.lock();
			} catch (const std::runtime_error &) {
				lock.lock();
				throw;
			}

			(eventually_zoom ? unzoomed_still_poses : still_poses)
				.insert(std::make_pair(pos, std::move(human)));

			return std::make_pair(true, true);
		}
	}

	// Do tasks until one unit of work is done for concurrent(multithread)
	// estimation.
	template<typename StillEstimator, typename ZoomStillEstimator,
		typename ImagePtr>
	void concurrent_preestimate_one_ensure_work(
		std::size_t length, bool zoom, std::size_t zoom_range, std::size_t zoom_rate,
		StillEstimator &still_estimator,
		ZoomStillEstimator &zoom_still_estimator,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback,
		std::list<std::pair<std::size_t, bool>> &queue,
		std::list<std::pair<std::size_t, bool>> &extra_queue,
		std::unique_lock<std::mutex> &lock)
	{
		while (true) {
			auto result = concurrent_preestimate_try_one(length,
				zoom, zoom_range, zoom_rate,
				still_estimator, zoom_still_estimator, callback,
				queue, extra_queue, lock);

			// bail out if either some work is done or no task is available
			if (result.second || !result.first)
				break;
		}
	}

	template<typename StillEstimator, typename ZoomStillEstimator,
		typename ImagePtr>
	void concurrent_preestimate_loop(
		std::size_t length, bool zoom, std::size_t zoom_range, std::size_t zoom_rate,
		StillEstimator &still_estimator,
		ZoomStillEstimator &zoom_still_estimator,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback,
		std::list<std::pair<std::size_t, bool>> &queue,
		std::list<std::pair<std::size_t, bool>> &extra_queue,
		std::mutex &mutex,
		std::condition_variable &cv,
		bool &status, bool &end)
	{
		while (true) {
			// status is false

			{
				std::unique_lock<std::mutex> lock(mutex);

				concurrent_preestimate_one_ensure_work(
					length, zoom, zoom_range, zoom_rate,
					still_estimator, zoom_still_estimator, callback,
					queue, extra_queue, lock
				);

				status = true;
			}

			// status is true

			cv.notify_all();

			{
				std::unique_lock<std::mutex> lock(mutex);

				cv.wait(lock, [&status] { return !status; });
				if (end)
					return;
			}

			// status is false
		}
	}

	template<typename StillEstimator, typename ZoomStillEstimator,
		typename ImagePtr>
	void concurrent_preestimate(
		std::size_t length, bool zoom, std::size_t zoom_range, std::size_t zoom_rate,
		StillEstimator &still_estimator,
		ZoomStillEstimator &zoom_still_estimator,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback,
		std::list<std::pair<std::size_t, bool>> &queue,
		std::list<std::pair<std::size_t, bool>> &extra_queue,
		std::mutex &mutex,
		std::condition_variable &cv,
		bool &status, bool &end)
	{
		try {
			concurrent_preestimate_loop(length, zoom, zoom_range, zoom_rate,
				still_estimator, zoom_still_estimator,
				callback, queue, extra_queue,
				mutex, cv, status, end);
		} catch (...) {
			{
				std::unique_lock<std::mutex> lock(mutex);

				queue.clear();
				extra_queue.clear();

				status = true;
			}
			cv.notify_all();
		}
	}

	template<typename StillEstimator, typename ImagePtr>
	std::pair<bool, const libaction::Human *>
	fuzz_callback_before_anti_crossing(std::size_t pos, std::size_t length,
		bool zoom, std::size_t zoom_range, std::size_t zoom_rate,
		StillEstimator &still_estimator,
		StillEstimator &zoom_still_estimator,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback,
		std::size_t offset, bool left)
	{
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

			// make sure that pos exists in unzoomed_still_poses
			auto unzoomed_it = unzoomed_still_poses.find(pos);
			if (unzoomed_it == unzoomed_still_poses.end()) {
				std::tie(std::ignore, unzoomed_it) =
					estimate_still_pose_from_callback_on(
						pos, false, callback, still_estimator,
						unzoomed_still_poses
					);
			}

			if (unzoomed_it->second) {
				// human found in the unzoomed image

				// now prepare the hints for a zoomed estimation

				std::size_t l, r;
				std::tie(l, r) = libaction::still::single::zoom::get_zoom_lr(
					pos, length, zoom_range);

				std::vector<const libaction::Human *> hints;

				// make all unzoomed estimations available within zoom_range
				for (std::size_t i = l; i <= r; i++) {
					if (i == pos)
						continue;

					decltype(still_poses)::iterator it;
					if (needs_zoom(zoom, i, zoom_rate)) {
						// unzoomed estimations for images which should be
						// zoomed go to unzoomed_still_poses

						it = unzoomed_still_poses.find(i);
						if (it == unzoomed_still_poses.end()) {
							it = estimate_still_pose_from_callback_on(
								i, false, callback, still_estimator,
								unzoomed_still_poses
							).second;
						}
					} else {
						// estimations for images which should not be zoomed
						// go to still_poses

						it = still_poses.find(i);
						if (it == still_poses.end()) {
							it = estimate_still_pose_from_callback_on(
								i, true, callback, still_estimator,
								still_poses
							).second;
						}
					}

					if (it->second)	// a useful hint
						hints.push_back(it->second.get());
				}

				auto image = get_image_from_callback(pos, true, callback);

				// zoom estimate
				using zoom_cb_arg = boost::multi_array<typename
					std::remove_reference<decltype(*image)>::type::element,
					3
				>;
				std::function<std::unique_ptr<libaction::Human>
					(const zoom_cb_arg&)> zoom_cb =
				[&zoom_still_estimator] (const zoom_cb_arg &image_to_estimate) {
					return estimate_still_pose_from_image(image_to_estimate,
						zoom_still_estimator);
				};
				auto human = libaction::still::single::zoom::zoom_estimate(
					*image, *unzoomed_it->second, hints, zoom_cb);

				// zoomed estimations for images which should be zoomed go
				// to still_poses
				auto it = still_poses.insert(
					std::make_pair(pos, std::move(human))).first;

				return std::make_pair(true, it->second.get());
			} else {
				// no human found in unzoomed image
				// impossible to do zoomed estimation

				// a little bit wasteful, but we need to inform callback that
				// the image at pos is no longer needed
				get_image_from_callback(pos, true, callback);

				still_poses.insert(std::make_pair(pos, std::unique_ptr<libaction::Human>()));

				return std::make_pair(true, nullptr);
			}
		} else {
			// the image at pos does not need to be zoomed

			auto it = estimate_still_pose_from_callback_on(
				pos, true, callback, still_estimator, still_poses).second;

			return std::make_pair(true, it->second.get());
		}
	}

	template<typename StillEstimator, typename ImagePtr>
	std::pair<
		bool,
		std::unique_ptr<const libaction::Human,
			std::function<void(const libaction::Human *)>>
		>
	fuzz_callback_before_max_lengths(std::size_t pos, std::size_t length,
		bool anti_crossing,
		bool zoom, std::size_t zoom_range, std::size_t zoom_rate,
		StillEstimator &still_estimator,
		StillEstimator &zoom_still_estimator,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback,
		std::size_t offset, bool left)
	{
		auto result = fuzz_callback_before_anti_crossing(pos, length,
			zoom, zoom_range, zoom_rate,
			still_estimator, zoom_still_estimator, callback,
			offset, left);

		if (anti_crossing && result.first && result.second) {
			std::pair<bool, const libaction::Human *> left_human, right_human;
			{
				// left
				std::size_t offset2 = offset;
				bool left2 = left;
				if (left2) {
					offset2++;
				} else {
					if (offset2 > 0) {
						offset2--;
					} else {
						offset2 = 1;
						left2 = true;
					}
				}
				left_human = fuzz_callback_before_anti_crossing(pos, length,
					zoom, zoom_range, zoom_rate,
					still_estimator, zoom_still_estimator, callback,
					offset2, left2);
			}
			{
				// right
				std::size_t offset2 = offset;
				bool left2 = left;
				if (!left2) {
					offset2++;
				} else {
					if (offset2 > 0) {
						offset2--;
					} else {
						offset2 = 1;
						left2 = false;
					}
				}
				right_human = fuzz_callback_before_anti_crossing(pos, length,
					zoom, zoom_range, zoom_rate,
					still_estimator, zoom_still_estimator, callback,
					offset2, left2);
			}

			if (!left_human.first)
				left_human.second = nullptr;
			if (!right_human.first)
				right_human.second = nullptr;

			auto anti_crossing_result = anti_crossing::anti_crossing(
				*result.second, left_human.second, right_human.second);

			std::unique_ptr<const libaction::Human,
				std::function<void(const libaction::Human *)>> human_ptr(
					anti_crossing_result.release(),
					[] (const libaction::Human *ptr) { delete ptr; });

			return std::make_pair(true, std::move(human_ptr));
		}

		std::unique_ptr<const libaction::Human,
			std::function<void(const libaction::Human *)>> human_ptr;
		if (result.first && result.second) {
			human_ptr = std::unique_ptr<const libaction::Human,
				std::function<void(const libaction::Human *)>>(
					result.second,
					[] (const libaction::Human *) {});
		}

		return std::make_pair(result.first, std::move(human_ptr));
	}

	template<typename StillEstimator, typename ImagePtr>
	std::pair<bool, std::unique_ptr<const libaction::Human>>
	fuzz_callback(std::size_t pos, std::size_t length,
		const std::vector<std::tuple<
			libaction::BodyPart::PartIndex, libaction::BodyPart::PartIndex,
			float>> &max_lengths,
		bool anti_crossing,
		bool zoom, std::size_t zoom_range, std::size_t zoom_rate,
		StillEstimator &still_estimator,
		StillEstimator &zoom_still_estimator,
		const std::function<ImagePtr(std::size_t pos, bool last_image_access)>
			&callback,
		std::size_t offset, bool left)
	{
		auto result = fuzz_callback_before_max_lengths(pos, length,
			anti_crossing,
			zoom, zoom_range, zoom_rate,
			still_estimator, zoom_still_estimator, callback,
			offset, left);

		std::pair<bool, std::unique_ptr<libaction::Human>> ret{};
		ret.first = result.first;
		if (result.first && result.second) {
			ret.second = std::unique_ptr<libaction::Human>(
				new libaction::Human(*result.second));
		}

		if (ret.first && ret.second) {
			for (auto &arg: max_lengths) {
				auto from = ret.second->body_parts().find(std::get<0>(arg));
				if (from == ret.second->body_parts().end())
					continue;
				auto to = ret.second->body_parts().find(std::get<1>(arg));
				if (to == ret.second->body_parts().end())
					continue;

				if (std::sqrt((from->second.x() - to->second.x()) * (from->second.x() - to->second.x())
						+ (from->second.y() - to->second.y()) * (from->second.y() - to->second.y()))
						> std::get<2>(arg)) {
					ret.second->body_parts().erase(to);
				}
			}
		}

		return std::move(ret);
	}

	/// Get the processed human pose to return to the user.
	inline static std::unique_ptr<std::unordered_map<std::size_t, libaction::Human>>
	get_human_pose(const std::unique_ptr<libaction::Human> &human)
	{
		if (human) {
			return std::unique_ptr<std::unordered_map<std::size_t, libaction::Human>>(
				new std::unordered_map<std::size_t, libaction::Human>(
					{ std::make_pair(0, *human) }));
		} else {
			return std::unique_ptr<std::unordered_map<std::size_t, libaction::Human>>(
				new std::unordered_map<std::size_t, libaction::Human>());
		}
	}
};

}
}
}

#endif
