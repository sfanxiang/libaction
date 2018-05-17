#ifndef LIBACTION_ESTIMATOR_HPP_
#define LIBACTION_ESTIMATOR_HPP_

#include "array.hpp"
#include "image.hpp"

#include <tensorflow/contrib/lite/kernels/register.h>
#include <tensorflow/contrib/lite/model.h>
#include <algorithm>
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

		std::vector<std::pair<size_t, size_t>> coords;
		for (size_t i = 0; i < heat_mat_shape[0] - 1; i++) {
			auto s1 = array::suppress_threshold<Value>(&heat_mat[array::index(3, heat_mat_shape, {i, 0, 0})],
				heat_mat_shape[1], heat_mat_shape[2], nms_threshold);
			auto s2 = array::suppress_non_max<Value>(s1.get(), heat_mat_shape[1], heat_mat_shape[2], nms_window, nms_window);
			auto coord = array::where_not_less(s2.get(), heat_mat_shape[1], heat_mat_shape[2], nms_threshold);
			for (auto &c: *coord) {
				coords.push_back(c);
			}
		}
		for (auto &c: coords) {
			std::cout << c.first << ' ' << c.second << std::endl;
		}
	}

private:
	const Value nms_threshold = 0.15;
	const size_t nms_window = 5;
	const size_t heat_mat_shape_0 = 19, paf_mat_shape_0 = 38;

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
};

}

#endif
