/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

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
