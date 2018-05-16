#include "estimator.hpp"

#include <tensorflow/contrib/lite/kernels/register.h>
#include <tensorflow/contrib/lite/model.h>
//#include <tensorflow/contrib/lite/string_util.h>
#include <tensorflow/contrib/lite/tools/mutable_op_resolver.h>
#include <memory>

// temp:
#include <iostream>

Estimator::Estimator(const char *graph_path, int num_threads)
: model(tflite::FlatBufferModel::BuildFromFile(graph_path))
{
	if (!model)
		throw std::runtime_error("failed to build model");

	tflite::InterpreterBuilder(*model, resolver)(&interpreter);

	if (!interpreter)
		throw std::runtime_error("failed to build interpreter");

	if (num_threads > 1)
		interpreter->SetNumThreads(num_threads);
}
