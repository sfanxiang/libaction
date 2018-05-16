#ifndef LIBACTION_ESTIMATOR_HPP_
#define LIBACTION_ESTIMATOR_HPP_

#include <tensorflow/contrib/lite/kernels/register.h>
#include <tensorflow/contrib/lite/model.h>
#include <cstddef>
#include <memory>

namespace libaction
{

class Estimator
{
public:
	typedef float value_type;

public:
	Estimator(const char *graph_path, int num_threads,
		int height, int width, int channels);

	value_type *get_input();
	void estimate();
	value_type *get_output();

	inline constexpr static size_t index(
		size_t height, size_t width, unsigned channels,
		size_t x, size_t y, size_t c)
	{
		return ((x * width) + y) * channels + c;
	}

private:
	std::unique_ptr<tflite::FlatBufferModel> model;
	tflite::ops::builtin::BuiltinOpResolver resolver{};
	std::unique_ptr<tflite::Interpreter> interpreter{};
};

}

#endif
