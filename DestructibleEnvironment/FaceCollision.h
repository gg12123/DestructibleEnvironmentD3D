#pragma once
#include <array>
#include "Face.h"
#include "Vector3.h"
#include "UpdatableBound.h"
#include "MathU.h"
#include "Shape.h"

class VectorDirectionWithMagnitude
{
public:
	VectorDirectionWithMagnitude() = default;

	VectorDirectionWithMagnitude(float mag, Vector3 dir) : m_Mag(mag), m_Dir(dir)
	{
	}

	float GetMag() const
	{
		return m_Mag;
	}

	const auto& GetDir() const
	{
		return m_Dir;
	}

private:
	float m_Mag;
	Vector3 m_Dir;
};

class FaceCollision
{
private:
	class WorldSpaceFace
	{
	private:
		UpdatableBound<float> CalculateBounds(const Vector3& sepNormal) const
		{
			UpdatableBound<float> bounds;

			for (auto i = 0; i < 3; i++)
				bounds.Update(Vector3::Dot(m_Points[i], sepNormal));

			return bounds;
		}

	public:
		WorldSpaceFace(const Face& f)
		{
			auto& t = f.GetOwnerShape().GetTransform();

			auto& points = f.GetCachedPoints();
			assert(points.size() == 3u);

			for (auto i = 0u; i < points.size(); i++)
				m_Points[i] = t.ToWorldPosition(points[i]);

			m_Normal = t.ToWorldDirection(f.GetNormal());
		}

		// This face is the one being moved
		VectorDirectionWithMagnitude CalculateRequiredSeperation(const WorldSpaceFace& other, const Vector3& sepNormal, bool allowNegative = true) const
		{
			auto selfBounds = CalculateBounds(sepNormal);
			auto othersBounds = other.CalculateBounds(sepNormal);

			auto distReqForNeg = selfBounds.GetMax() - othersBounds.GetMin();
			auto distReqForPos = othersBounds.GetMax() - selfBounds.GetMin();

			if (allowNegative)
			{
				return distReqForNeg < distReqForPos ?
					VectorDirectionWithMagnitude(distReqForNeg, -sepNormal) :
					VectorDirectionWithMagnitude(distReqForPos, sepNormal);
			}
			return VectorDirectionWithMagnitude(distReqForPos, sepNormal);
		}

		Vector3 GetNormal() const
		{
			return m_Normal;
		}

		const auto& GetPoints() const
		{
			return m_Points;
		}

	private:
		std::array<Vector3, 3> m_Points;
		Vector3 m_Normal;
	};

	void CalculateEdgeCrossSeperationVectors(const WorldSpaceFace& faceToMove,
		const WorldSpaceFace& other,
		int edgeIndexInToMove,
		int start)
	{
		auto& movePoints = faceToMove.GetPoints();
		auto& othersPoints = other.GetPoints();

		auto edge0 = (movePoints[(edgeIndexInToMove + 1) % 3] - movePoints[edgeIndexInToMove]).Normalized();

		for (auto i = 0; i < 3; i++)
		{
			auto edge1 = (othersPoints[(i + 1) % 3] - othersPoints[i]).Normalized();

			auto cross = Vector3::Cross(edge0, edge1);
			auto mag = cross.Magnitude();

			auto axis = mag > 0.0f ? cross / mag : Vector3::Right();

			m_SeperationVectors[start + i] = faceToMove.CalculateRequiredSeperation(other, axis);
		}
	}

	VectorDirectionWithMagnitude SmallestSeperationVector()
	{
		auto indexOfSmallest = 0u;
		auto smallestMag = MathU::Infinity;

		for (auto i = 0u; i < m_SeperationVectors.size(); i++)
		{
			auto mag = m_SeperationVectors[i].GetMag();
			if (mag < smallestMag)
			{
				smallestMag = mag;
				indexOfSmallest = i;
			}
		}
		return m_SeperationVectors[indexOfSmallest];
	}

public:
	FaceCollision() = default;

	FaceCollision(const Face& faceA, const Face& faceB)
	{
		m_FaceA = &faceA;
		m_FaceB = &faceB;
	}

	// Returns the best (i.e. smallest)
	VectorDirectionWithMagnitude CalculateSeperationVectors(const Shape& shapeToMove)
	{
		auto movingA = &shapeToMove == &m_FaceA->GetOwnerShape();

		auto faceToMove = WorldSpaceFace(movingA ? *m_FaceA : *m_FaceB);
		auto otherFace = WorldSpaceFace(movingA ? *m_FaceB : *m_FaceA);

		m_SeperationVectors[0] = faceToMove.CalculateRequiredSeperation(otherFace, otherFace.GetNormal(), false);
		m_SeperationVectors[1] = faceToMove.CalculateRequiredSeperation(otherFace, -faceToMove.GetNormal(), false);

		auto j = 2;
		for (auto i = 0; i < 3; i++)
		{
			CalculateEdgeCrossSeperationVectors(faceToMove, otherFace, i, j);
			j += 3;
		}

		return SmallestSeperationVector();
	}

	float DistRequiredToSeperate(const VectorDirectionWithMagnitude& sepVector, const Vector3& moveDir)
	{
		auto cosAlpha = Vector3::Dot(sepVector.GetDir(), moveDir);
		return cosAlpha <= 0.0f ? MathU::Infinity : sepVector.GetMag() / cosAlpha;
	}

	float CalculateRequiredSeperation(const Vector3& moveDir)
	{
		auto minReqSep = MathU::Infinity;
		for (auto i = 0u; i < m_SeperationVectors.size(); i++)
		{
			auto dist = DistRequiredToSeperate(m_SeperationVectors[i], moveDir);

			if (dist < minReqSep)
				minReqSep = dist;
		}
		return minReqSep;
	}

private:
	std::array<VectorDirectionWithMagnitude, 11> m_SeperationVectors;

	const Face* m_FaceA;
	const Face* m_FaceB;
};

static_assert(std::is_trivially_copyable<FaceCollision>::value, "Face collision should be trivially copyable.");