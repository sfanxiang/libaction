#ifndef LIBACTION_BODY_PART_HPP_
#define LIBACTION_BODY_PART_HPP_

namespace libaction
{

class BodyPart
{
public:
	enum class PartIndex
	{
		nose = 0,
		neck,
		shoulder_r,
		elbow_r,
		wrist_r,
		shoulder_l,
		elbow_l,
		wrist_l,
		hip_r,
		knee_r,
		ankle_r,
		hip_l,
		knee_l,
		ankle_l,
		eye_r,
		eye_l,
		ear_r,
		ear_l,
		end
	};

	inline BodyPart() {}

	inline BodyPart(PartIndex part_index, float x, float y, float score)
	:
	part_index_(part_index), x_(x), y_(y), score_(score)
	{}

	inline PartIndex part_index() const { return part_index_; }
	inline float x() const { return x_; }
	inline float y() const { return y_; }
	inline float score() const { return score_; }

private:
	PartIndex part_index_{PartIndex::end};
	float x_{}, y_{}, score_{};
};

}

#endif
