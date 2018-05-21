#ifndef LIBACTION_SINGLE_ESTIMATOR_HPP_
#define LIBACTION_SINGLE_ESTIMATOR_HPP_

#include "../array.hpp"
#include "../body_part.hpp"
#include "../human.hpp"
#include "../image.hpp"
#include "posenet_parts.hpp"

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

namespace libaction
{
namespace single
{

namespace
{
class ErrorReporter: public tflite::ErrorReporter
{
	inline int Report(const char *, va_list)
	{
		// ignored
		return 0;
	}
};
}

template<typename Value>
class Estimator
{
public:
	inline Estimator(
		const std::string &graph_path, int num_threads,
		size_t height, size_t width, size_t channels) :
	model_height(height), model_width(width), model_channels(channels),
	model(tflite::FlatBufferModel::BuildFromFile(
		graph_path.c_str(), &error_reporter))
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

		if (num_threads > 0)
			interpreter->SetNumThreads(num_threads);

		if (interpreter->AllocateTensors() != kTfLiteOk)
			throw std::runtime_error("AllocateTensors failed");
	}

	template<typename Image>
	inline std::unique_ptr<std::list<libaction::Human>> estimate(
		const Image &image)
	{
		if (image.num_dimensions() != 3)
			throw std::runtime_error("wrong number of dimensions");

		auto height = image.shape()[0];
		auto width = image.shape()[1];
		auto channels = image.shape()[2];

		if (channels != model_channels)
			throw std::runtime_error("bad number of channels");
		if (height == 0 || width == 0)
			throw std::runtime_error("invalid image parameters");

		auto resized_image = image::resize(image,
			model_height + 1, model_width + 1);

		std::copy(resized_image->data(),
			resized_image->data() + resized_image->num_elements(),
			get_input());

		if (interpreter->Invoke() != kTfLiteOk)
			throw std::runtime_error("Invoke failed");

		boost::multi_array_ref<float, 3> heatmap_scores(get_output(0),
			boost::extents[model_height / output_stride + 1][model_width / output_stride + 1][keypoints_size]);
		boost::multi_array_ref<float, 3> offsets(get_output(1),
			boost::extents[model_height / output_stride + 1][model_width / output_stride + 1][keypoints_size * 2]);

		auto heatmap_coords = array::argmax_2d(heatmap_scores);

		auto points = get_offset_points(*heatmap_coords, offsets);
		auto scores = get_points_confidence(heatmap_scores, *heatmap_coords);

		std::list<libaction::BodyPart> parts;
		for (size_t i = 0; i < keypoints_size; i++) {
			if ((*scores)[i] >= part_score_threshold) {
				parts.push_back(libaction::BodyPart(
					posenet_parts::to_libaction_part_index(
						static_cast<posenet_parts::Part>(i)),
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

private:
	const size_t output_stride = 16;
	const size_t keypoints_size = 17;
	const size_t part_count_threshold = 3;
	const float part_score_threshold = 0.86;

	size_t model_height, model_width, model_channels;

	ErrorReporter error_reporter{};
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
		const std::vector<std::pair<size_t, size_t>> &heatmap_coords,
		const Offsets &offsets)
	{
		auto points = std::unique_ptr<std::vector<std::pair<float, float>>>(
			new std::vector<std::pair<float, float>>());

		size_t keypoint = 0;
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
		const std::vector<std::pair<size_t, size_t>> &heatmap_coords)
	{
		auto res = std::unique_ptr<std::vector<float>>(
			new std::vector<float>());

		size_t keypoint = 0;
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

#endif