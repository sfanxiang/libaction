/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__MOTION__SINGLE__MISSED_MOVES__DETAIL_HPP_
#define LIBACTION__MOTION__SINGLE__MISSED_MOVES__DETAIL_HPP_

#include <tuple>

namespace libaction
{
namespace motion
{
namespace single
{
namespace missed_moves
{
namespace detail
{

template<typename Track, typename Record, typename TrackIterator>
inline TrackIterator track_to_record(Track &track, Record &record,
	const TrackIterator &item)
{
	record[std::make_tuple(std::get<0>(item->second),
			std::get<1>(item->second),
			std::get<2>(item->second))]
		= item->first;
	return track.erase(item);
}

}
}
}
}
}

#endif
