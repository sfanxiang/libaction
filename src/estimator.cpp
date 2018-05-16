#include "estimator.hpp"

#include <tensorflow/contrib/lite/kernels/register.h>
#include <tensorflow/contrib/lite/model.h>
//#include <tensorflow/contrib/lite/string_util.h>
#include <tensorflow/contrib/lite/tools/mutable_op_resolver.h>
#include <memory>

// temp:
#include <iostream>

namespace libaction
{

Estimator::Estimator(
	const char *graph_path, int num_threads,
	int height, int width, int channels)
: model(tflite::FlatBufferModel::BuildFromFile(graph_path))
{
	if (!model)
		throw std::runtime_error("failed to build model");

	tflite::InterpreterBuilder(*model, resolver)(&interpreter);

	if (!interpreter)
		throw std::runtime_error("failed to build interpreter");

	if (num_threads > 0)
		interpreter->SetNumThreads(num_threads);

	if (interpreter->ResizeInputTensor(0, {1, height, width, channels})
		!= kTfLiteOk)
		throw std::runtime_error("ResizeInputTensor failed");

	if (interpreter->AllocateTensors() != kTfLiteOk)
		throw std::runtime_error("AllocateTensors failed");
}

Estimator::value_type *Estimator::get_input()
{
	return interpreter->typed_input_tensor<value_type>(0);
}

void Estimator::estimate()
{
	if (interpreter->Invoke() != kTfLiteOk)
		throw std::runtime_error("Invoke failed");
}

Estimator::value_type *Estimator::get_output()
{
	return interpreter->typed_output_tensor<value_type>(0);
}

}
