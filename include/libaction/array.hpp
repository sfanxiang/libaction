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

inline size_t size(size_t dims, const size_t *shape)
{
	if (dims == 0)
		return 0;

	size_t res = 1;
	for (size_t i = 0; i < dims; i++) {
		res *= shape[i];
	}
	return res;
}

inline size_t size(size_t dims, const std::vector<size_t> &shape)
{
	if (shape.size() < dims)
		throw std::runtime_error("shape.size() < dims");
	return size(dims, shape.data());
}

inline size_t index(size_t dims, const size_t *shape, const size_t *indices)
{
	size_t res = 0;
	for (size_t i = 0; i < dims; i++) {
		res *= shape[i];
		res += indices[i];
	}
	return res;
}

inline size_t index(size_t dims, const std::vector<size_t> &shape,
	const size_t *indices)
{
	if (shape.size() < dims)
		throw std::runtime_error("shape.size() < dims");
	return index(dims, shape.data(), indices);
}

inline size_t index(size_t dims, const size_t *shape,
	const std::vector<size_t> &indices)
{
	if (indices.size() < dims)
		throw std::runtime_error("indices.size() < dims");
	return index(dims, shape, indices.data());
}

inline size_t index(size_t dims, const std::vector<size_t> &shape,
	const std::vector<size_t> &indices)
{
	if (shape.size() < dims)
		throw std::runtime_error("shape.size() < dims");
	if (indices.size() < dims)
		throw std::runtime_error("indices.size() < dims");
	return index(dims, shape.data(), indices.data());
}

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
			auto ind = index(2, {x, y}, {i, j});

			while (!dq.empty() && dq.back() < array[ind]) {
				dq.pop_back();
			}

			if (j >= window_y && !dq.empty() &&
					dq.front() == array[ind - window_y]) {
				dq.pop_front();
			}

			dq.push_back(array[ind]);
			if (j >= (window_y - 1) / 2)
				temp[ind - (window_y - 1) / 2] = dq.front();
		}
		for (size_t j = y; j < y + (window_y - 1) / 2; j++) {
			auto ind = index(2, {x, y}, {i, j});

			if (!dq.empty() && dq.front() == array[ind - window_y]) {
				dq.pop_front();
			}

			if (dq.empty())
				throw std::runtime_error("queue is empty");

			temp[ind - (window_y - 1) / 2] = dq.front();
		}
	}

	auto res = std::unique_ptr<T[]>(new T[x * y]);

	for (size_t j = 0; j < y; j++) {
		std::deque<T> dq;
		for (size_t i = 0; i < x; i++) {
			auto ind = index(2, {x, y}, {i, j});

			while (!dq.empty() && dq.back() < temp[ind]) {
				dq.pop_back();
			}

			if (i >= window_x && !dq.empty() &&
					dq.front() == temp[ind - window_x * y]) {
				dq.pop_front();
			}

			dq.push_back(temp[ind]);
			if (i >= (window_x - 1) / 2)
				res[ind - (window_x - 1) / 2 * y] = dq.front();
		}
		for (size_t i = x; i < x + (window_x - 1) / 2; i++) {
			auto ind = index(2, {x, y}, {i, j});

			if (!dq.empty() && dq.front() == temp[ind - window_x * y]) {
				dq.pop_front();
			}

			if (dq.empty())
				throw std::runtime_error("queue is empty");

			res[ind - (window_x - 1) / 2 * y] = dq.front();
		}
	}

	return res;
}

template<typename T>
std::unique_ptr<T[]> suppress_threshold(const T *array, size_t x, size_t y,
	T threshold)
{
	auto res = std::unique_ptr<T[]>(new T[size(2, {x, y})]);
	for (size_t i = 0; i < x; i++) {
		for (size_t j = 0; j < y; j++) {
			auto ind = index(2, {x, y}, {i, j});

			if (array[ind] >= threshold)
				res[ind] = array[ind];
			else
				res[ind] = static_cast<T>(0);
		}
	}

	return res;
}

template<typename T>
std::unique_ptr<T[]> suppress_non_max(const T *array, size_t x, size_t y,
	size_t window_x, size_t window_y)
{
	auto filter = max_filter(array, x, y, window_x, window_y);
	auto res = std::unique_ptr<T[]>(new T[size(2, {x, y})]);
	for (size_t i = 0; i < x; i++) {
		for (size_t j = 0; j < y; j++) {
			auto ind = index(2, {x, y}, {i, j});

			if (array[ind] == filter[ind])
				res[ind] = array[ind];
			else
				res[ind] = static_cast<T>(0);
		}
	}

	return res;
}

template<typename T>
std::unique_ptr<std::vector<std::pair<size_t, size_t>>> where_not_less(
	const T* array, size_t x, size_t y, T comp)
{
	auto res = std::unique_ptr<std::vector<std::pair<size_t, size_t>>>(
		new std::vector<std::pair<size_t, size_t>>());
	for (size_t i = 0; i < x; i++) {
		for (size_t j = 0; j < y; j++) {
			if (array[index(2, {x, y}, {i, j})] >= comp) {
				res->push_back({i, j});
			}
		}
	}
	return res;
}

}
}

#endif
