#ifndef LIBACTION_ARRAY_HPP_
#define LIBACTION_ARRAY_HPP_

#include <memory>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace libaction
{
namespace array
{

template<typename Input>
std::unique_ptr<boost::multi_array<typename Input::element, 2>>
suppress_threshold(const Input &array, typename Input::element threshold)
{
	if (array.num_dimensions() != 2)
		throw std::runtime_error("wrong number of dimensions");

	auto res = std::unique_ptr<boost::multi_array<typename Input::element, 2>>(
		new boost::multi_array<typename Input::element, 2>(
			boost::extents[array.shape()[0]][array.shape()[1]]));
	for (size_t i = 0; i < array.shape()[0]; i++) {
		for (size_t j = 0; j < array.shape()[1]; j++) {
			if (array[i][j] >= threshold)
				(*res)[i][j] = array[i][j];
			else
				(*res)[i][j] = static_cast<typename Input::element>(0);
		}
	}

	return res;
}

template<typename Input>
std::unique_ptr<boost::multi_array<typename Input::element, 2>> max_filter(
	const Input &array, size_t window_x, size_t window_y)
{
	if (array.num_dimensions() != 2)
		throw std::runtime_error("wrong number of dimensions");
	if (array.shape()[0] == 0 || array.shape()[1] == 0)
		throw std::runtime_error("invalid shape");
	if (window_x == 0 || window_y == 0 ||
			window_x > array.shape()[0] || window_y > array.shape()[1])
		throw std::runtime_error("invalid window size");

	auto x = array.shape()[0];
	auto y = array.shape()[1];

	boost::multi_array<typename Input::element, 2> temp(boost::extents[x][y]);

	for (size_t i = 0; i < x; i++) {
		std::deque<typename Input::element> dq;
		for (size_t j = 0; j < y; j++) {
			while (!dq.empty() && dq.back() < array[i][j]) {
				dq.pop_back();
			}

			if (j >= window_y && !dq.empty() &&
					dq.front() == array[i][j - window_y]) {
				dq.pop_front();
			}

			dq.push_back(array[i][j]);
			if (j >= (window_y - 1) / 2)
				temp[i][j - (window_y - 1) / 2] = dq.front();
		}
		for (size_t j = y; j < y + (window_y - 1) / 2; j++) {
			if (!dq.empty() && dq.front() == array[i][j - window_y]) {
				dq.pop_front();
			}

			if (dq.empty())
				throw std::runtime_error("queue is empty");

			temp[i][j - (window_y - 1) / 2] = dq.front();
		}
	}

	auto res = std::unique_ptr<boost::multi_array<typename Input::element, 2>>(
		new boost::multi_array<typename Input::element, 2>(
			boost::extents[x][y]));

	for (size_t j = 0; j < y; j++) {
		std::deque<typename Input::element> dq;
		for (size_t i = 0; i < x; i++) {
			while (!dq.empty() && dq.back() < temp[i][j]) {
				dq.pop_back();
			}

			if (i >= window_x && !dq.empty() &&
					dq.front() == temp[i - window_x][j]) {
				dq.pop_front();
			}

			dq.push_back(temp[i][j]);
			if (i >= (window_x - 1) / 2)
				(*res)[i - (window_x - 1) / 2][j] = dq.front();
		}
		for (size_t i = x; i < x + (window_x - 1) / 2; i++) {
			if (!dq.empty() && dq.front() == temp[i - window_x][j]) {
				dq.pop_front();
			}

			if (dq.empty())
				throw std::runtime_error("queue is empty");

			(*res)[i - (window_x - 1) / 2][j] = dq.front();
		}
	}

	return res;
}

template<typename Input>
std::unique_ptr<boost::multi_array<typename Input::element, 2>>
suppress_non_max(const Input &array, size_t window_x, size_t window_y)
{
	if (array.num_dimensions() != 2)
		throw std::runtime_error("wrong number of dimensions");

	auto filter = max_filter(array, window_x, window_y);
	auto res = std::unique_ptr<boost::multi_array<typename Input::element, 2>>(
		new boost::multi_array<typename Input::element, 2>(
			boost::extents[array.shape()[0]][array.shape()[1]]));
	for (size_t i = 0; i < array.shape()[0]; i++) {
		for (size_t j = 0; j < array.shape()[1]; j++) {
			if (array[i][j] == (*filter)[i][j])
				(*res)[i][j] = array[i][j];
			else
				(*res)[i][j] = static_cast<typename Input::element>(0);
		}
	}

	return res;
}

template<typename T>
std::unique_ptr<std::vector<std::pair<size_t, size_t>>> where_not_less(
	const T &array, typename T::element comp)
{
	if (array.num_dimensions() != 2)
		throw std::runtime_error("wrong number of dimensions");

	auto res = std::unique_ptr<std::vector<std::pair<size_t, size_t>>>(
		new std::vector<std::pair<size_t, size_t>>());
	for (size_t i = 0; i < array.shape()[0]; i++) {
		for (size_t j = 0; j < array.shape()[1]; j++) {
			if (array[i][j] >= comp) {
				res->push_back({i, j});
			}
		}
	}
	return res;
}

}
}

#endif
