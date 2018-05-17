#ifndef LIBACTION_ESTIMATOR_HPP_
#define LIBACTION_ESTIMATOR_HPP_

#include "image.hpp"
#include "process.hpp"

#include <tensorflow/contrib/lite/kernels/register.h>
#include <tensorflow/contrib/lite/model.h>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

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

	inline void estimate(
		const uint8_t *image, size_t height, size_t width, size_t channels)
	{
		if (channels != model_channels)
			throw std::runtime_error("bad number of channels");
		if (height == 0 || width == 0)
			throw std::runtime_error("invalid image parameters");

		std::unique_ptr<uint8_t[]> resized_image;
		if (height != model_height || width != model_width) {
			resized_image = image::resize(image, height, width, channels,
				model_height, model_width);
			image = resized_image.get();
		}

		Value *input = get_input();
		std::copy(image, image +
			image::size(model_height, model_width, model_channels),
			input);

		if (interpreter->Invoke() != kTfLiteOk)
			throw std::runtime_error("Invoke failed");

		Value *output = get_output();
		const size_t output_shape[] = {model_height / 8, model_width / 8, 19 + 38};
		const size_t heat_mat_shape[] = {19, model_height / 8, model_width / 8};
		const size_t paf_mat_shape[] = {38, model_height / 8, model_width / 8};
		auto heat_mat = std::unique_ptr<Value[]>(new Value[
			heat_mat_shape[0] * heat_mat_shape[1] * heat_mat_shape[2]]);
		auto paf_mat = std::unique_ptr<Value[]>(new Value[
			paf_mat_shape[0] * paf_mat_shape[1] * paf_mat_shape[2]]);

		for (size_t i = 0; i < heat_mat_shape[0]; i++) {
			for (size_t j = 0; j < heat_mat_shape[1]; j++) {
				for (size_t k = 0; k < heat_mat_shape[2]; k++) {
					heat_mat[(i * heat_mat_shape[1] + j) * heat_mat_shape[2] + k]
						= output[(j * output_shape[1] + k) * output_shape[2] + i];
				}
			}
		}
		for (size_t i = 0; i < paf_mat_shape[0]; i++) {
			for (size_t j = 0; j < paf_mat_shape[1]; j++) {
				for (size_t k = 0; k < paf_mat_shape[2]; k++) {
					paf_mat[(i * paf_mat_shape[1] + j) * paf_mat_shape[2] + k]
						= output[(j * output_shape[1] + k) * output_shape[2] + i + heat_mat_shape[0]];
				}
			}
		}

		for (size_t i = 0; i < heat_mat_shape[0] - 1; i++) {
			auto s1 = process::suppress_threshold<Value>(&heat_mat[i], heat_mat_shape[0], heat_mat_shape[1], 0.15f /* TODO: NMS_threshold */);
			auto s2 = process::suppress_non_max<Value>(&heat_mat[i], heat_mat_shape[0], heat_mat_shape[1], 5, 5);
			// TODO
			s2[0] = 1;
		}
	}

private:
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
