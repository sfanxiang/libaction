/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__STILL__SINGLE__ESTIMATOR_HPP_
#define LIBACTION__STILL__SINGLE__ESTIMATOR_HPP_

#include "../../body_part.hpp"
#include "../../human.hpp"
#include "../../detail/image.hpp"
#include "../detail/array.hpp"
#include "detail/error_reporter.hpp"
#include "detail/posenet_parts.hpp"

#include <boost/multi_array.hpp>
#include <tensorflow/contrib/lite/kernels/register.h>
#include <tensorflow/contrib/lite/model.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <list>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace libaction
{
namespace still
{
namespace single
{

/// Single-person pose estimator.

/// @tparam     Value       The input value type specific to the model, usually
///                         `float`.
template<typename Value>
class Estimator
{
private:
	/// Initialize the class and check for error.
	inline void initialize(int threads)
	{
		if (model_height < 8 || model_width < 8 || model_channels == 0
				|| model_height % output_stride != 0
				|| model_width % output_stride != 0)
			throw std::runtime_error("invalid model parameters");

		if (!model)
			throw std::runtime_error("failed to build model");

		tflite::InterpreterBuilder(*model, resolver)(&interpreter);

		if (!interpreter)
			throw std::runtime_error("failed to build interpreter");

		if (threads > 0)
			interpreter->SetNumThreads(threads);

		if (interpreter->AllocateTensors() != kTfLiteOk)
			throw std::runtime_error("AllocateTensors failed");
	}

public:
	/// Construct from a file.

	/// @param[in]  graph_path  The path to the graph file.
	/// @param[in]  threads     Threads used when invoking the model, or 0 for
	///                         default.
	/// @param[in]  height      The height of the model.
	/// @param[in]  width       The width of the model.
	/// @param[in]  channels    The number of color channels, usually 3.
	/// @exception              std::runtime_error
	inline Estimator(
		const std::string &graph_path, int threads,
		std::size_t height, std::size_t width, std::size_t channels) :
	model_height(height), model_width(width), model_channels(channels),
	model(tflite::FlatBufferModel::BuildFromFile(
		graph_path.c_str(), &error_reporter))
	{
		initialize(threads);
	}

	/// Construct from a buffer.

	/// @param[in]  graph_buffer    The buffer containing the graph. The ownership
	///                         of the buffer is not transferred and it should
	///                         remain valid until Estimator is destroyed.
	/// @param[in]  buffer_size The size of the buffer.
	/// @param[in]  threads     Threads used when invoking the model, or 0 for
	///                         default.
	/// @param[in]  height      The height of the model.
	/// @param[in]  width       The width of the model.
	/// @param[in]  channels    The number of color channels, usually 3.
	/// @exception              std::runtime_error
	inline Estimator(
		const void *graph_buffer, std::size_t buffer_size, int threads,
		std::size_t height, std::size_t width, std::size_t channels) :
	model_height(height), model_width(width), model_channels(channels),
	model(tflite::FlatBufferModel::BuildFromBuffer(
		static_cast<const char *>(graph_buffer), buffer_size, &error_reporter))
	{
		initialize(threads);
	}

	/// Estimate from an image.

	/// @param[in]  image       The input image conforming to the
	///                         Boost.MultiArray concept. The image must have 3
	///                         non-empty dimensions of height, width, and
	///                         channels. The image will be automatically
	///                         resized to match the model height and width.
	/// @return                 A list of humans inferred from the image.
	/// @exception              std::runtime_error
	template<typename Image>
	inline std::unique_ptr<std::list<libaction::Human>> estimate(
		const Image &image)
	{
		namespace libaction_array = libaction::still::detail::array;
		namespace libaction_image = libaction::detail::image;

		if (image.num_dimensions() != 3)
			throw std::runtime_error("wrong number of dimensions");

		auto height = image.shape()[0];
		auto width = image.shape()[1];
		auto channels = image.shape()[2];

		if (channels != model_channels)
			throw std::runtime_error("bad number of channels");
		if (height == 0 || width == 0)
			throw std::runtime_error("invalid image parameters");

		auto resized_image = libaction_image::resize(
			image, model_height + 1, model_width + 1);

		std::copy(resized_image->data(),
			resized_image->data() + resized_image->num_elements(),
			get_input());

		if (interpreter->Invoke() != kTfLiteOk)
			throw std::runtime_error("Invoke failed");

		boost::multi_array_ref<float, 3> heatmap_scores(get_output(0),
			boost::extents[model_height / output_stride + 1][model_width / output_stride + 1][keypoints_size]);
		boost::multi_array_ref<float, 3> offsets(get_output(1),
			boost::extents[model_height / output_stride + 1][model_width / output_stride + 1][keypoints_size * 2]);

		auto heatmap_coords = libaction_array::argmax_2d(heatmap_scores);

		auto points = get_offset_points(*heatmap_coords, offsets);
		auto scores = get_points_confidence(heatmap_scores, *heatmap_coords);

		std::list<libaction::BodyPart> parts;
		for (std::size_t i = 0; i < keypoints_size; i++) {
			if ((*scores)[i] >= part_score_threshold) {
				parts.push_back(libaction::BodyPart(
					detail::posenet_parts::to_libaction_part_index(
						static_cast<detail::posenet_parts::Part>(i)),
					(*points)[i].first,
					(*points)[i].second,
					(*scores)[i]
				));
			}
		}

		auto humans = std::unique_ptr<std::list<libaction::Human>>(
			new std::list<libaction::Human>());
		if (parts.size() >= part_count_threshold)
			humans->push_back(libaction::Human(parts));

		return humans;
	}

	/// Set score threshold to default.
	void set_score_threshold()
	{
		part_score_threshold = default_score_threshold;
	}

	/// Set score threshold.

	/// @param[in]  threshold   Score threshold.
	void set_score_threshold(float threshold)
	{
		part_score_threshold = threshold;
	}

private:
	const std::size_t output_stride = 16;
	const std::size_t keypoints_size = 17;
	const std::size_t part_count_threshold = 3;
	const float default_score_threshold = 0.5f;

	float part_score_threshold{default_score_threshold};

	std::size_t model_height, model_width, model_channels;

	detail::ErrorReporter error_reporter{};
	std::unique_ptr<tflite::FlatBufferModel> model;
	tflite::ops::builtin::BuiltinOpResolver resolver{};
	std::unique_ptr<tflite::Interpreter> interpreter{};

	inline float *get_input()
	{
		return interpreter->typed_input_tensor<float>(0);
	}

	inline float *get_output(int index)
	{
		return interpreter->typed_output_tensor<float>(index);
	}

	template<typename Offsets>
	std::unique_ptr<std::vector<std::pair<float, float>>> get_offset_points(
		const std::vector<std::pair<std::size_t, std::size_t>> &heatmap_coords,
		const Offsets &offsets)
	{
		auto points = std::unique_ptr<std::vector<std::pair<float, float>>>(
			new std::vector<std::pair<float, float>>());

		std::size_t keypoint = 0;
		for (auto &coord: heatmap_coords) {
			auto x = offsets[coord.first][coord.second][keypoint];
			auto y = offsets[coord.first][coord.second][keypoint + keypoints_size];
			x = coord.first * output_stride + x;
			y = coord.second * output_stride + y;
			points->push_back(std::make_pair(
				x / static_cast<float>(model_height + 1),
				y / static_cast<float>(model_width + 1)));

			keypoint++;
			if (keypoint >= keypoints_size)
				break;
		}

		return points;
	}

	template<typename HeatmapScores>
	std::unique_ptr<std::vector<float>> get_points_confidence(
		const HeatmapScores &scores,
		const std::vector<std::pair<std::size_t, std::size_t>> &heatmap_coords)
	{
		auto res = std::unique_ptr<std::vector<float>>(
			new std::vector<float>());

		std::size_t keypoint = 0;
		for (auto &coord: heatmap_coords) {
			res->push_back(scores[coord.first][coord.second][keypoint]);

			keypoint++;
			if (keypoint >= keypoints_size)
				break;
		}

		return res;
	}
};

}
}
}

#endif
