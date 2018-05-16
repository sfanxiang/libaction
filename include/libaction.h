#ifndef LIBACTION_LIBACTION_H_
#define LIBACTION_LIBACTION_H_

#ifdef __cplusplus
extern "C" {
#endif

struct LibactionEstimator;

extern LibactionEstimator *libaction_new_estimator(
	const char *graph_path, int num_threads);
extern void libaction_delete_estimator(LibactionEstimator *estimator);

#ifdef __cplusplus
}
#endif


#endif
