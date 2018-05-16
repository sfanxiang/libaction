#include "libaction.h"
#include "estimator.hpp"

LibactionEstimator *libaction_new_estimator(
	const char *graph_path, int num_threads)
{
	try {
		return reinterpret_cast<LibactionEstimator*>(
			new Estimator(graph_path, num_threads));
	} catch (...) {
		return nullptr;
	}
}

void libaction_delete_estimator(LibactionEstimator *estimator)
{
	try {
		if (estimator)
			delete reinterpret_cast<Estimator*>(estimator);
	} catch (...) {
	}
}
