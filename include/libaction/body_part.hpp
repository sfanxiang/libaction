#ifndef LIBACTION_BODY_PART_HPP_
#define LIBACTION_BODY_PART_HPP_

namespace libaction
{

/// Describe a keypoint of the human body.
class BodyPart
{
public:
	/// Enumeration of body parts.
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

	/// Construct with empty values.

	/// `part_index` is initialized to `PartIndex::end`. `x`, `y`, and `score`
	/// are initialized to 0.
	inline BodyPart() {}

	/// Construct from arguments.

	/// @param[in]  part_index  Index of the body part.
	/// @param[in]  x           X-coordinate (top-down).
	/// @param[in]  y           Y-coordinate (left-right).
	/// @param[in]  score       Confidence of the estimation.
	inline BodyPart(PartIndex part_index, float x, float y, float score)
	:
	part_index_(part_index), x_(x), y_(y), score_(score)
	{}

	/// Index of the body part.

	/// @return                 Index of the body part.
	inline PartIndex part_index() const { return part_index_; }

	/// X-coordinate (top-down).

	/// @return                 X-coordinate (top-down).
	inline float x() const { return x_; }

	/// Y-coordinate (left-right).

	/// @return                 Y-coordinate (left-right).
	inline float y() const { return y_; }

	/// Confidence of the estimation.

	/// @return                 Confidence of the estimation.
	inline float score() const { return score_; }

private:
	PartIndex part_index_{PartIndex::end};
	float x_{}, y_{}, score_{};
};

}

#endif
