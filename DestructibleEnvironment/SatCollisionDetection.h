#pragma once
#include "Shape.h"
#include "CollisionData.h"
#include "CompoundShape.h"
#include "Face.h"
#include "UpdatableBound.h"

class SatCollisionDetection
{
private:
	void UpdateCurrentContactPlane(const Vector3& p0, const Vector3& n, float seperation, const Transform& activeTran)
	{
		auto pen = MathU::Abs(seperation);
		if (pen < m_CurrentContactPlane.GetPeneration())
			m_CurrentContactPlane = ContactPlane(activeTran.ToWorldPosition(p0), activeTran.ToWorldDirection(n), pen);
	}

	float CalculateSeperation(const Shape& s, const Vector3& p0, const Vector3& n, Vector3& closestPoint)
	{
		auto minVal = MathU::Infinity;

		for (auto& p : s.GetCachedPoints())
		{
			auto val = Vector3::Dot(p - p0, n);
			if (val < minVal)
			{
				minVal = val;
				closestPoint = p;
			}
		}
		return minVal;
	}

	bool CheckFaceNormalsForSep(const Shape& shapeFaces, const Shape& shapeOther)
	{
		auto& tFaces = shapeFaces.GetOwner().GetTransform();
		auto& tOther = shapeOther.GetOwner().GetTransform();

		for (auto f : shapeFaces.GetFaces())
		{
			// TODO - use pre-computed matrices
			auto p0 = tOther.ToLocalPosition(tFaces.ToWorldPosition(f->GetPlaneP0()));
			auto n = tOther.ToLocalDirection(tFaces.ToWorldDirection(f->GetNormal()));

			Vector3 closestPoint;
			auto sep = CalculateSeperation(shapeOther, p0, n, closestPoint);
			if (sep > 0.0f)
				return true;

			UpdateCurrentContactPlane(closestPoint, n, sep, tOther);
		}
		return false;
	}

	UpdatableBound<float> CalculateBound(const Vector3& n, const SimdStdVector<Vector3>& points) const
	{
		auto b = UpdatableBound<float>();

		for (auto& p : points)
			b.Update(Vector3::Dot(n, p));

		return b;
	}

	bool CheckEdgeNormalsForSep(const Shape& shapeA, const Shape& shapeB)
	{
		auto& edgesA = shapeA.GetEdgeIndexes();
		auto& edgesB = shapeB.GetEdgeIndexes();

		auto& pointsA = shapeA.GetCachedPoints();
		auto& pointsB = shapeB.GetCachedPoints();

		auto& tA = shapeA.GetOwner().GetTransform();
		auto& tB = shapeB.GetOwner().GetTransform();

		auto aSpaceToBSpace = tB.GetWorldToLocalMatrix() * tA.GetLocalToWorldMatrix();

		static SimdStdVector<Vector3> pointsATransformed;

		pointsATransformed.clear();
		for (auto& p : pointsA)
			pointsATransformed.emplace_back(aSpaceToBSpace * p);

		for (auto iA = 0u; iA < edgesA.size(); iA += 2u)
		{
			auto& p0A = pointsATransformed[edgesA[iA]];
			auto& p1A = pointsATransformed[edgesA[iA + 1u]];

			for (auto iB = 0u; iB < edgesB.size(); iB += 2u)
			{
				auto& p0B = pointsB[edgesB[iB]];
				auto& p1B = pointsB[edgesB[iB + 1u]];

				auto n = Vector3::Cross(p1A - p0A, p1B - p0B);
				auto mag = n.Magnitude();

				if (mag < 0.0001f)
					continue;

				n /= mag;

				auto aBounds = CalculateBound(n, pointsATransformed);
				auto bBounds = CalculateBound(n, pointsB);

				if (!aBounds.Overlaps(bBounds))
					return true;

				auto sep = -MathU::Min(aBounds.GetMax() - bBounds.GetMin(), bBounds.GetMax() - aBounds.GetMin());

				Vector3 closest1, closest2;
				Vector3::ClosestPointsBetweenLines(p0A, p1A, p0B, p1B, closest1, closest2);
				auto contactPoint = (closest1 + closest2) / 2.0f;
				UpdateCurrentContactPlane(contactPoint, n, sep, tB);
			}
		}
		return false;
	}

public:
	bool AreColliding(const Shape& shapeA, const Shape& shapeB)
	{
		m_CurrentContactPlane = ContactPlane(Vector3::Zero(), Vector3::Zero(), MathU::Infinity);

		if (CheckFaceNormalsForSep(shapeA, shapeB))
			return false;

		if (CheckFaceNormalsForSep(shapeB, shapeA))
			return false;

		if (CheckEdgeNormalsForSep(shapeA, shapeB))
			return false;

		return true;
	}

	const auto& GetCurrentContactPlane() const
	{
		return m_CurrentContactPlane;
	}

private:
	ContactPlane m_CurrentContactPlane;
};
