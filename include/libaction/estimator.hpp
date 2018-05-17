#ifndef LIBACTION_ESTIMATOR_HPP_
#define LIBACTION_ESTIMATOR_HPP_

#include "array.hpp"
#include "image.hpp"

#include <tensorflow/contrib/lite/kernels/register.h>
#include <tensorflow/contrib/lite/model.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

// TODO: temp
#include <iostream>

namespace libaction
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
		if (model_height < 8 || model_width < 8 || model_channels == 0)
			throw std::runtime_error("invalid model parameters");

		if (!model)
			throw std::runtime_error("failed to build model");

		tflite::InterpreterBuilder(*model, resolver)(&interpreter);

		if (!interpreter)
			throw std::runtime_error("failed to build interpreter");

		if (num_threads > 0)
			interpreter->SetNumThreads(num_threads);

		if (interpreter->ResizeInputTensor(0, {
				1, static_cast<int>(model_height),
				static_cast<int>(model_width),
				static_cast<int>(model_channels)}) != kTfLiteOk)
			throw std::runtime_error("ResizeInputTensor failed");

		if (interpreter->AllocateTensors() != kTfLiteOk)
			throw std::runtime_error("AllocateTensors failed");
	}

	template<typename Image>
	inline void estimate(
		const Image *image, size_t height, size_t width, size_t channels)
	{
		if (channels != model_channels)
			throw std::runtime_error("bad number of channels");
		if (height == 0 || width == 0)
			throw std::runtime_error("invalid image parameters");

		auto resized_image = image::resize<Image, float>(
			image, height, width, channels, model_height, model_width);

		Value *input = get_input();
		std::copy(resized_image.get(), resized_image.get() +
			image::size(model_height, model_width, model_channels),
			input);

		if (interpreter->Invoke() != kTfLiteOk)
			throw std::runtime_error("Invoke failed");

		Value *output = get_output();

		const size_t output_shape[] = {model_height / 8, model_width / 8, heat_mat_shape_0 + paf_mat_shape_0};
		const size_t heat_mat_shape[] = {heat_mat_shape_0, model_height / 8, model_width / 8};
		const size_t paf_mat_shape[] = {paf_mat_shape_0, model_height / 8, model_width / 8};
		auto heat_mat = std::unique_ptr<Value[]>(new Value[
			heat_mat_shape[0] * heat_mat_shape[1] * heat_mat_shape[2]]);
		auto paf_mat = std::unique_ptr<Value[]>(new Value[
			paf_mat_shape[0] * paf_mat_shape[1] * paf_mat_shape[2]]);

		for (size_t i = 0; i < heat_mat_shape[0]; i++) {
			for (size_t j = 0; j < heat_mat_shape[1]; j++) {
				for (size_t k = 0; k < heat_mat_shape[2]; k++) {
					heat_mat[array::index(3, heat_mat_shape, {i, j, k})]
						= output[array::index(3, output_shape, {j, k, i})];
				}
			}
		}
		for (size_t i = 0; i < paf_mat_shape[0]; i++) {
			for (size_t j = 0; j < paf_mat_shape[1]; j++) {
				for (size_t k = 0; k < paf_mat_shape[2]; k++) {
					paf_mat[array::index(3, paf_mat_shape, {i, j, k})]
						= output[array::index(3, output_shape, {j, k, i + heat_mat_shape[0]})];
				}
			}
		}

		std::vector<std::unique_ptr<std::vector<std::pair<size_t, size_t>>>>
			coords;
		for (size_t i = 0; i < heat_mat_shape[0] - 1; i++) {
			auto s1 = array::suppress_threshold<Value>(&heat_mat[array::index(3, heat_mat_shape, {i, 0, 0})],
				heat_mat_shape[1], heat_mat_shape[2], nms_threshold);
			auto s2 = array::suppress_non_max<Value>(s1.get(), heat_mat_shape[1], heat_mat_shape[2], nms_window, nms_window);
			auto coord = array::where_not_less(s2.get(), heat_mat_shape[1], heat_mat_shape[2], nms_threshold);
			coords.push_back(std::move(coord));
		}


	}

private:
	struct PartPair
	{
		float score;
		size_t part_idx1, part_idx2;
		size_t idx1, idx2;
		size_t coord1, coord2;
		float score1, score2;

		PartPair(float sc, size_t pi1, size_t pi2, size_t i1, size_t i2,
			size_t cd1, size_t cd2, float sc1, float sc2)
		:
			score(sc), part_idx1(pi1), part_idx2(pi2), idx1(i1), idx2(i2),
			coord1(cd1), coord2(cd2), score1(sc1), score2(sc2)
		{}
	};

	const size_t heat_mat_shape_0 = 19, paf_mat_shape_0 = 38;
	const float nms_threshold = 0.15;
	const size_t nms_window = 5;
	const size_t paf_num_inter = 10;
	const float local_paf_threshold = 0.2;
	const size_t paf_count_threshold = 5;

	size_t model_height, model_width, model_channels;

	ErrorReporter error_reporter{};
	std::unique_ptr<tflite::FlatBufferModel> model;
	tflite::ops::builtin::BuiltinOpResolver resolver{};
	std::unique_ptr<tflite::Interpreter> interpreter{};

	inline Value *get_input()
	{
		return interpreter->typed_input_tensor<Value>(0);
	}

	inline Value *get_output()
	{
		return interpreter->typed_output_tensor<Value>(0);
	}

	std::unique_ptr<std::vector<PartPair>> score_pairs(
		size_t part_idx1, size_t part_idx2,
		const std::vector<std::pair<size_t, size_t>> &coord_list1,
		const std::vector<std::pair<size_t, size_t>> &coord_list2,
		Value *paf_mat_x, Value *paf_mat_y, Value *heatmap,
		float rescale1, float rescale2)
	{
		size_t idx1 = 0;
		for (auto coord1 = coord_list1.begin();
				coord1 != coord_list1.end();
				++coord1, ++idx1) {
			auto y1 = coord1->first;
			auto x1 = coord1->second;

			size_t idx2 = 0;
			for (auto coord2 = coord_list2.begin();
					coord2 != coord_list2.end();
					++coord2, ++idx2) {
				auto y2 = coord2->first;
				auto x2 = coord2->second;

				auto score_count = get_score(x1, y1, x2, y2, paf_mat_x, paf_mat_y);
				auto &score = score_count.first;
				auto &count = score_count.second;

				if (count < paf_count_threshold || score <= 0.0f)
					continue;

				// TODO
			}
		}
	}

	/* TODO: REMOVE:

    def score_pairs(part_idx1, part_idx2, coord_list1, coord_list2, paf_mat_x, paf_mat_y, heatmap, rescale=(1.0, 1.0)):
        connection_temp = []

        for idx1, (y1, x1) in enumerate(zip(coord_list1[0], coord_list1[1])):
            for idx2, (y2, x2) in enumerate(zip(coord_list2[0], coord_list2[1])):
                score, count = PoseEstimator.get_score(x1, y1, x2, y2, paf_mat_x, paf_mat_y)
                if count < PoseEstimator.PAF_Count_Threshold or score <= 0.0:
                    continue
                connection_temp.append(PoseEstimator.PartPair(
                    score=score,
                    part_idx1=part_idx1, part_idx2=part_idx2,
                    idx1=idx1, idx2=idx2,
                    coord1=(x1 * rescale[0], y1 * rescale[1]),
                    coord2=(x2 * rescale[0], y2 * rescale[1]),
                    score1=heatmap[part_idx1][y1][x1],
                    score2=heatmap[part_idx2][y2][x2],
                ))

        connection = []
        used_idx1, used_idx2 = set(), set()
        for candidate in sorted(connection_temp, key=lambda x: x.score, reverse=True):
            # check not connected
            if candidate.idx1 in used_idx1 or candidate.idx2 in used_idx2:
                continue
            connection.append(candidate)
            used_idx1.add(candidate.idx1)
            used_idx2.add(candidate.idx2)

        return connection

	*/

	std::pair<float, size_t> get_score(
		size_t x1, size_t y1, size_t x2, size_t y2,
		Value *paf_mat_x, Value *paf_mat_y)
	{
		auto dx = x2 - x1;
		auto dy = y2 - y1;
		auto norm_vec = hypotf(static_cast<float>(dx), static_cast<float>(dy));

		if (norm_vec < 1e-4f)
			return std::make_pair(0.0f, 0);

		float vx = static_cast<float>(dx) / norm_vec;
		float vy = static_cast<float>(dy) / norm_vec;

		float score = 0.0f;
		size_t count = 0;
		for (size_t i = 0; i < paf_num_inter; i++) {
			size_t x = x1 + (dx * i + paf_num_inter / 2) / paf_num_inter;
			size_t y = y1 + (dy * i + paf_num_inter / 2) / paf_num_inter;

			float curr = paf_mat_x[y][x] * vx + paf_mat_y[y][x] * vy;
			if (curr > local_paf_threshold) {
				score += curr;
				count++;
			}
		}

		return std::make_pair(score, count);
	}
};

}

#endif
