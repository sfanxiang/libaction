#ifndef LIBACTION__MOTION__SINGLE__ESTIMATOR_HPP_
#define LIBACTION__MOTION__SINGLE__ESTIMATOR_HPP_

#include "libaction/human.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>

namespace libaction
{
namespace motion
{
namespace single
{

class Estimator
{
public:
	inline Estimator();

	template<typename StillEstimator, typename ImagePtr>
	inline std::unique_ptr<std::unordered_map<size_t, libaction::Human>>
	estimate(
		size_t pos, size_t length, size_t rate, StillEstimator &still_estimator
		std::unique_ptr<std::function<ImagePtr(size_t pos)>> callback
	) {
	}

	inline void reset()
	{
	}

private:

};

}
}
}

#endif
