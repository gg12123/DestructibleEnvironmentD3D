#pragma once
#include <vector>
#include "GjkCollisionDetection.h"
#include "EpaContact.h"
#include "CollisionData.h"
#include "CompoundShape.h"
#include "Shape.h"
#include "SatCollisionDetection.h"

#define USE_SAT_COLLISION_DETECTION

class CollisionDetector
{
private:
	ContactPlane CalculateCantact(const GjkInputShape& shape1, const GjkInputShape& shape2)
	{
		auto localContact = m_ContactFinder.FindContact(shape1, shape2, m_Detector.GetQ());

		return ContactPlane(m_ActiveTransform->ToWorldPosition(localContact.GetPoint()),
			m_ActiveTransform->ToWorldDirection(localContact.GetNormal()),
			localContact.GetPeneration());
	}

	GjkInputShape TransformToShape1sSpace(const Shape& shape2)
	{
		// TODO - Gjk can run without having all the points transformed upfront.
		// It coud be done lazily.

		m_TransformedPoints.clear();

		for (auto& p : shape2.GetCachedPoints())
			m_TransformedPoints.emplace_back(m_ToShape1sSpace * p);

		return GjkInputShape(m_TransformedPoints, m_ToShape1sSpace * shape2.GetCentre());
	}

	void InitTransformMatrix(const CompoundShape& shape1, const CompoundShape& shape2)
	{
		auto& t1 = shape1.GetTransform();
		auto& t2 = shape2.GetTransform();

		m_ToShape1sSpace = t1.GetWorldToLocalMatrix() * t2.GetLocalToWorldMatrix();
		m_ActiveTransform = &t1;
	}

public:
	bool FindContact(const Shape& shape1, const Shape& shape2, ContactPlane& contact)
	{
#ifdef USE_SAT_COLLISION_DETECTION

		if (m_SatDetector.AreColliding(shape1, shape2))
		{
			contact = m_SatDetector.GetCurrentContactPlane();
			return true;
		}
		return false;
#else
		InitTransformMatrix(shape1.GetOwner(), shape2.GetOwner());

		auto gjkShape1 = GjkInputShape(shape1.GetCachedPoints(), shape1.GetCentre());
		auto gjkShape2 = TransformToShape1sSpace(shape2);

		if (m_Detector.Run(gjkShape1, gjkShape2))
		{
			contact = CalculateCantact(gjkShape1, gjkShape2);
			return true;
		}
		return false;
#endif
	}

private:
	std::vector<Vector3> m_TransformedPoints;
	GjkCollisionDetection m_Detector;
	SatCollisionDetection m_SatDetector;
	EpaContact m_ContactFinder;
	Matrix4 m_ToShape1sSpace;
	const Transform* m_ActiveTransform;
};