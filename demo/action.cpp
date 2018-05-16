#include <libaction.h>

#include <cstdlib>
#include <iostream>
#include <memory>

int main()
{
	std::unique_ptr<LibactionEstimator, decltype(&libaction_delete_estimator)>
		estimator(libaction_new_estimator(
			"graph_tflite.tflite", 1), &libaction_delete_estimator);

	if (!estimator) {
		std::cerr << "failed to create estimator" << std::endl;
		return EXIT_FAILURE;
	}

	return 0;
}
