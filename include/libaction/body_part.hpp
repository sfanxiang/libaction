#ifndef LIBACTION_BODY_PART_HPP_
#define LIBACTION_BODY_PART_HPP_

namespace libaction
{

class BodyPart
{
public:
	inline BodyPart() {}

	inline BodyPart(size_t part_idx, float x, float y, float score)
	:
	part_idx_(part_idx), x_(x), y_(y), score_(score)
	{}

	inline size_t part_idx() const { return part_idx_; }
	inline float x() const { return x_; }
	inline float y() const { return y_; }
	inline float score() const { return score_; }

private:
	size_t part_idx_{};
	float x_{}, y_{}, score_{};
};

}

#endif
