/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__STILL__SINGLE__DETAIL__ERROR_REPORTER_HPP_
#define LIBACTION__STILL__SINGLE__DETAIL__ERROR_REPORTER_HPP_

#include <tensorflow/contrib/lite/kernels/register.h>
#include <tensorflow/contrib/lite/model.h>

namespace libaction
{
namespace still
{
namespace single
{
namespace detail
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
}
}
}

#endif
