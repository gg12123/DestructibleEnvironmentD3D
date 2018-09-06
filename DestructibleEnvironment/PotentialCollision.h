#pragma once
#include "Vector3.h"
#include "Transform.h"
#include "ArrayWrapper.h"
#include "Constants.h"

class PotentialCollision
{
public:
	float CalculateRequiredSeperation() const
	{
		auto points = m_OthersTransformedPoints->GetData();
		auto count = m_OthersTransformedPoints->GetCurrCount();

		auto mostInside = MathUtils::Infinity;

		for (auto i = 0; i < count; i++)
		{
			auto comp = Vector3::Dot(points[i] - m_PointOnOwner, m_Normal);

			if (comp < mostInside)
				mostInside = comp;
		}

		return mostInside;
	}

	Vector3 GetNormalWorldSpace() const
	{
		return m_OwnerTransform->ToWorldDirection(m_Normal);
	}

	Vector3 GetPointWorldSpace() const
	{
		return m_OwnerTransform->ToWorldPosition(m_PointOnOwner);
	}

	void Init(const Vector3& pointLocal, const Vector3& normalLocal,
			 ArrayWrapper<Vector3, Constants::MaxNumPoints>& otherPoints, Transform& activeTransform)
	{
		m_PointOnOwner = pointLocal;
		m_Normal = normalLocal;
		m_OthersTransformedPoints = &otherPoints;
		m_OwnerTransform = &activeTransform;
	}

private:
	Vector3 m_PointOnOwner;
	Vector3 m_Normal;

	ArrayWrapper<Vector3, Constants::MaxNumPoints>* m_OthersTransformedPoints;
	Transform* m_OwnerTransform;
};
