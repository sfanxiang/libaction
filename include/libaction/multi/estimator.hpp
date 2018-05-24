#ifndef LIBACTION_MULTI_ESTIMATOR_HPP_
#define LIBACTION_MULTI_ESTIMATOR_HPP_

#include "../array.hpp"
#include "../body_part.hpp"
#include "../human.hpp"
#include "../image.hpp"
#include "detail/coco_parts.hpp"
#include "detail/human.hpp"
#include "detail/part_pair.hpp"

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
namespace multi
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

		auto resized_image = image::resize(image, model_height, model_width);

		std::copy(resized_image->data(),
			resized_image->data() + resized_image->num_elements(),
			get_input());

		if (interpreter->Invoke() != kTfLiteOk)
			throw std::runtime_error("Invoke failed");

		boost::multi_array_ref<float, 3> output(get_output(),
			boost::extents[model_height / 8][model_width / 8][heat_mat_shape_0 + paf_mat_shape_0]);

		boost::multi_array<float, 3> heat_mat(
			boost::extents[heat_mat_shape_0][model_height / 8][model_width / 8]);
		boost::multi_array<float, 3> paf_mat(
			boost::extents[paf_mat_shape_0][model_height / 8][model_width / 8]);

		for (size_t i = 0; i < heat_mat.shape()[0]; i++) {
			for (size_t j = 0; j < heat_mat.shape()[1]; j++) {
				for (size_t k = 0; k < heat_mat.shape()[2]; k++) {
					heat_mat[i][j][k] = output[j][k][i];
				}
			}
		}
		for (size_t i = 0; i < paf_mat.shape()[0]; i++) {
			for (size_t j = 0; j < paf_mat.shape()[1]; j++) {
				for (size_t k = 0; k < paf_mat.shape()[2]; k++) {
					paf_mat[i][j][k] = output[j][k][i + heat_mat_shape_0];
				}
			}
		}

		std::vector<std::unique_ptr<std::vector<std::pair<size_t, size_t>>>>
			coords;
		for (size_t i = 0; i < heat_mat.shape()[0] - 1; i++) {
			auto s1 = array::suppress_threshold(heat_mat[i], nms_threshold);
			auto s2 = array::suppress_non_max(*s1, nms_window, nms_window);
			auto coord = array::where_not_less(*s2, nms_threshold);
			coords.push_back(std::move(coord));
		}

		std::vector<detail::PartPair> pairs_by_conn;
		auto coco_pairs = detail::coco_parts::pairs();
		auto coco_pairs_network = detail::coco_parts::pairs_network();
		for (size_t i = 0; i < std::min(coco_pairs.size(), coco_pairs_network.size()); i++) {
			auto part_idx1 = coco_pairs[i].first;
			auto part_idx2 = coco_pairs[i].second;
			auto paf_x_idx = coco_pairs_network[i].first;
			auto paf_y_idx = coco_pairs_network[i].second;
			auto pairs = score_pairs(
				part_idx1, part_idx2,
				*coords[part_idx1], *coords[part_idx2],
				paf_mat[paf_x_idx], paf_mat[paf_y_idx],
				heat_mat,
				1.0f / static_cast<float>(heat_mat.shape()[1]),
				1.0f / static_cast<float>(heat_mat.shape()[2])
			);
			pairs_by_conn.insert(pairs_by_conn.end(), pairs->begin(), pairs->end());
		}

		auto humans = std::unique_ptr<std::list<detail::Human>>(
			new std::list<detail::Human>());
		for (auto &pair: pairs_by_conn)
			humans->push_back(detail::Human(std::array<detail::PartPair, 1>{pair}));

		for (auto i = humans->begin(); i != humans->end(); ) {
			bool merged = false;
			auto j = i; ++j;
			for (; j != humans->end(); j++) {
				if (i->is_connected(*j)) {
					merged = true;
					i->merge(*j);
					humans->erase(j);
					break;
				}
			}
			if (!merged)
				++i;
		}

		for (auto i = humans->begin(); i != humans->end(); ) {
			if (i->part_count() < part_count_threshold
					|| i->max_score() < part_score_threshold)
				i = humans->erase(i);
			else
				++i;
		}

		auto res_humans = std::unique_ptr<std::list<libaction::Human>>(
			new std::list<libaction::Human>());
		for (auto &x: *humans) {
			std::list<libaction::BodyPart> parts;
			for (auto &y: x.get_body_parts()) {
				parts.push_back(libaction::BodyPart(
					detail::coco_parts::to_libaction_part_index(
						static_cast<detail::coco_parts::Part>(
							y.second.part_idx())),
					y.second.x(), y.second.y(), y.second.score()));
			}
			res_humans->push_back(libaction::Human(parts));
		}

		return res_humans;
	}

private:
	const size_t heat_mat_shape_0 = 19, paf_mat_shape_0 = 38;
	const float nms_threshold = 0.15;
	const size_t nms_window = 5;
	const size_t paf_num_inter = 10;
	const float local_paf_threshold = 0.2;
	const size_t paf_count_threshold = 5;
	const size_t part_count_threshold = 4;
	const float part_score_threshold = 4.5;

	size_t model_height, model_width, model_channels;

	ErrorReporter error_reporter{};
	std::unique_ptr<tflite::FlatBufferModel> model;
	tflite::ops::builtin::BuiltinOpResolver resolver{};
	std::unique_ptr<tflite::Interpreter> interpreter{};

	inline float *get_input()
	{
		return interpreter->typed_input_tensor<float>(0);
	}

	inline float *get_output()
	{
		return interpreter->typed_output_tensor<float>(0);
	}

	template<typename PafMatX, typename PafMatY, typename Heatmap>
	std::unique_ptr<std::vector<detail::PartPair>> score_pairs(
		size_t part_idx1, size_t part_idx2,
		const std::vector<std::pair<size_t, size_t>> &coord_list1,
		const std::vector<std::pair<size_t, size_t>> &coord_list2,
		const PafMatX &paf_mat_x, const PafMatY &paf_mat_y,
		const Heatmap &heatmap, float rescale1, float rescale2)
	{
		if (paf_mat_x.num_dimensions() != 2 ||
				paf_mat_y.num_dimensions() != 2 ||
				heatmap.num_dimensions() != 3)
			throw std::runtime_error("wrong number of dimensions");

		std::vector<detail::PartPair> connection_temp;

		size_t idx1 = 0;
		for (auto coord1 = coord_list1.begin();
				coord1 != coord_list1.end();
				++coord1, ++idx1) {
			auto x1 = coord1->first;
			auto y1 = coord1->second;

			size_t idx2 = 0;
			for (auto coord2 = coord_list2.begin();
					coord2 != coord_list2.end();
					++coord2, ++idx2) {
				auto x2 = coord2->first;
				auto y2 = coord2->second;

				auto score_count = get_score(x1, y1, x2, y2, paf_mat_x, paf_mat_y);
				auto &score = score_count.first;
				auto &count = score_count.second;

				if (count < paf_count_threshold || score <= 0.0f)
					continue;

				if (heatmap.shape()[0] <= part_idx1 || heatmap.shape()[0] <= part_idx2 ||
						heatmap.shape()[1] <= x1 || heatmap.shape()[1] <= x2 ||
						heatmap.shape()[2] <= y1 || heatmap.shape()[2] <= y2)
					throw std::runtime_error("out of bound");

				connection_temp.push_back(detail::PartPair(
					score,
					part_idx1, part_idx2,
					idx1, idx2,
					std::make_pair(x1 * rescale1, y1 * rescale2),
					std::make_pair(x2 * rescale1, y2 * rescale2),
					heatmap[part_idx1][x1][y1],
					heatmap[part_idx2][x2][y2]
				));
			}
		}

		std::sort(connection_temp.begin(), connection_temp.end(),
			[] (const detail::PartPair &x, const detail::PartPair &y)
			{ return x.score() > y.score(); });

		auto connection = std::unique_ptr<std::vector<detail::PartPair>>(
			new std::vector<detail::PartPair>());
		std::set<size_t> used_idx1, used_idx2;
		for (auto &candidate: connection_temp) {
			if (used_idx1.find(candidate.idx1()) != used_idx1.end() ||
					used_idx2.find(candidate.idx2()) != used_idx2.end())
				continue;
			connection->push_back(candidate);
			used_idx1.insert(candidate.idx1());
			used_idx2.insert(candidate.idx2());
		}

		return connection;
	}

	template<typename PafMatX, typename PafMatY>
	std::pair<float, size_t> get_score(
		ptrdiff_t x1, ptrdiff_t y1, ptrdiff_t x2, ptrdiff_t y2,
		const PafMatX &paf_mat_x, const PafMatY &paf_mat_y)
	{
		if (paf_mat_x.num_dimensions() != 2 || paf_mat_y.num_dimensions() != 2)
			throw std::runtime_error("wrong number of dimensions");
		if (x1 >= static_cast<ptrdiff_t>(paf_mat_x.shape()[0]) ||
				x2 >= static_cast<ptrdiff_t>(paf_mat_y.shape()[0]) ||
				y1 >= static_cast<ptrdiff_t>(paf_mat_x.shape()[1]) ||
				y2 >= static_cast<ptrdiff_t>(paf_mat_y.shape()[1]))
			throw std::runtime_error("out of bound");

		auto dx = x2 - x1;
		auto dy = y2 - y1;
		auto norm_vec = hypotf(static_cast<float>(dx), static_cast<float>(dy));

		if (norm_vec < 1e-4f)
			return std::make_pair(0.0f, 0);

		float vx = static_cast<float>(dx) / norm_vec;
		float vy = static_cast<float>(dy) / norm_vec;

		float score = 0.0f;
		size_t count = 0;
		for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(paf_num_inter); i++) {
			ptrdiff_t x = x1 + (dx * i + static_cast<ptrdiff_t>(paf_num_inter / 2)) / static_cast<ptrdiff_t>(paf_num_inter);
			ptrdiff_t y = y1 + (dy * i + static_cast<ptrdiff_t>(paf_num_inter / 2)) / static_cast<ptrdiff_t>(paf_num_inter);

			float curr = paf_mat_x[x][y] * vy + paf_mat_y[x][y] * vx;
			if (curr > local_paf_threshold) {
				score += curr;
				count++;
			}
		}

		return std::make_pair(score, count);
	}
};

}
}

#endif
