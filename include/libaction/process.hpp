#ifndef LIBACTION_PROCESS_HPP_
#define LIBACTION_PROCESS_HPP_

#include <queue>
#include <stdexcept>

namespace libaction
{
namespace process
{

template<typename T>
std::unique_ptr<T[]> max_filter(const T *array, size_t x, size_t y,
	size_t window_x, size_t window_y)
{
	if (x == 0 || y == 0)
		throw std::runtime_error("invalid size");
	if (window_x == 0 || window_y == 0 || window_x > x || window_y > y)
		throw std::runtime_error("invalid window size");

	auto temp = std::unique_ptr<T[]>(new T[x * y]);

	for (size_t i = 0; i < x; i++) {
		std::deque<T> dq;
		for (size_t j = 0; j < y; j++) {
			auto index = i * y + j;

			while (!dq.empty() && dq.back() < array[index]) {
				dq.pop_back();
			}

			if (j >= window_y && !dq.empty() &&
					dq.front() == array[index - window_y]) {
				dq.pop_front();
			}

			dq.push_back(array[index]);
			if (j >= (window_y - 1) / 2)
				temp[index - (window_y - 1) / 2] = dq.front();
		}
		for (size_t j = y; j < y + (window_y - 1) / 2; j++) {
			auto index = i * y + j;

			if (!dq.empty() && dq.front() == array[index - window_y]) {
				dq.pop_front();
			}

			if (dq.empty())
				throw std::runtime_error("queue is empty");

			temp[index - (window_y - 1) / 2] = dq.front();
		}
	}

	auto res = std::unique_ptr<T[]>(new T[x * y]);

	for (size_t j = 0; j < y; j++) {
		std::deque<T> dq;
		for (size_t i = 0; i < x; i++) {
			auto index = i * y + j;

			while (!dq.empty() && dq.back() < temp[index]) {
				dq.pop_back();
			}

			if (i >= window_x && !dq.empty() &&
					dq.front() == temp[index - window_x * y]) {
				dq.pop_front();
			}

			dq.push_back(temp[index]);
			if (i >= (window_x - 1) / 2)
				res[index - (window_x - 1) / 2 * y] = dq.front();
		}
		for (size_t i = x; i < x + (window_x - 1) / 2; i++) {
			auto index = i * y + j;

			if (!dq.empty() && dq.front() == temp[index - window_x * y]) {
				dq.pop_front();
			}

			if (dq.empty())
				throw std::runtime_error("queue is empty");

			res[index - (window_x - 1) / 2 * y] = dq.front();
		}
	}

	return res;
}

template<typename T>
std::unique_ptr<T[]> suppress_threshold(const T *array, size_t x, size_t y,
	T threshold)
{
	auto res = std::unique_ptr<T[]>(new T[x * y]);
	for (size_t i = 0; i < x; i++) {
		for (size_t j = 0; j < y; j++) {
			auto index = i * y + j;

			if (array[index] >= threshold)
				res[index] = array[index];
			else
				res[index] = static_cast<T>(0);
		}
	}

	return res;
}

template<typename T>
std::unique_ptr<T[]> suppress_non_max(const T *array, size_t x, size_t y,
	size_t window_x, size_t window_y)
{
	auto filter = max_filter(array, x, y, window_x, window_y);
	auto res = std::unique_ptr<T[]>(new T[x * y]);
	for (size_t i = 0; i < x; i++) {
		for (size_t j = 0; j < y; j++) {
			auto index = i * y + j;

			if (array[index] == filter[index])
				res[index] = array[index];
			else
				res[index] = static_cast<T>(0);
		}
	}

	return res;
}

}
}

#endif
