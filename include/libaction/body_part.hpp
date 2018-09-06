/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * This Source Code Form is "Incompatible With Secondary Licenses", as
 * defined by the Mozilla Public License, v. 2.0. */

#ifndef LIBACTION__BODY_PART_HPP_
#define LIBACTION__BODY_PART_HPP_

#include <functional>

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
		neck = 1,
		shoulder_r = 2,
		elbow_r = 3,
		wrist_r = 4,
		shoulder_l = 5,
		elbow_l = 6,
		wrist_l = 7,
		hip_r = 8,
		knee_r = 9,
		ankle_r = 10,
		hip_l = 11,
		knee_l = 12,
		ankle_l = 13,
		eye_r = 14,
		eye_l = 15,
		ear_r = 16,
		ear_l = 17,
		end
	};

	/// Construct with empty values.

	/// `part_index` is initialized to PartIndex::end. `x`, `y`, and `score`
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

	/// X-coordinate (top-down, [0.0, 1.0)).

	/// @return                 X-coordinate (top-down, [0.0, 1.0)).
	inline float x() const { return x_; }

	/// Y-coordinate (left-right, [0.0, 1.0)).

	/// @return                 Y-coordinate (left-right, [0.0, 1.0)).
	inline float y() const { return y_; }

	/// Confidence of the estimation.

	/// @return                 Confidence of the estimation.
	inline float score() const { return score_; }

private:
	PartIndex part_index_{PartIndex::end};
	float x_{}, y_{}, score_{};
};

}

namespace std {

/// Hash of libaction::BodyPart::PartIndex.
template<>
struct hash<libaction::BodyPart::PartIndex>
{
	/// Hash of libaction::BodyPart::PartIndex.

	/// @return                 Hash of libaction::BodyPart::PartIndex.
	std::size_t operator()(const libaction::BodyPart::PartIndex &index)
	const noexcept
	{
		return static_cast<std::size_t>(index);
	}
};

}

#endif
