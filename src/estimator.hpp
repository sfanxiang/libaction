#ifndef LIBACTION_ESTIMATOR_HPP_
#define LIBACTION_ESTIMATOR_HPP_

#include <tensorflow/contrib/lite/kernels/register.h>
#include <tensorflow/contrib/lite/model.h>
#include <memory>

class Estimator
{
public:
	Estimator(const char *graph_path, int num_threads);

private:
	std::unique_ptr<tflite::FlatBufferModel> model;
	tflite::ops::builtin::BuiltinOpResolver resolver{};
	std::unique_ptr<tflite::Interpreter> interpreter{};
};

#endif
