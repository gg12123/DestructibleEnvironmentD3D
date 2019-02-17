#pragma once
#include <vector>
#include "GjkCollisionDetection.h"
#include "CollisionData.h"
#include "CompoundShape.h"
#include "Shape.h"

class CollisionDetector
{
private:
	ContactManifold CalculateCantact(const GjkInputShape& shape1, const GjkInputShape& shape2)
	{
		assert(false);
		return ContactManifold();
	}

	GjkInputShape TransformToShape1sSpace(const Shape& shape2)
	{
		m_TransformedPoints.clear();

		for (auto p : shape2.GetCachedPoints())
			m_TransformedPoints.emplace_back(m_ToShape1sSpace * p);

		return GjkInputShape(m_TransformedPoints, m_ToShape1sSpace * shape2.GetCentre());
	}

	bool FindContact(const Shape& shape1, const Shape& shape2, ContactManifold& contact)
	{
		auto gjkShape1 = GjkInputShape(shape1.GetCachedPoints(), shape1.GetCentre());
		auto gjkShape2 = TransformToShape1sSpace(shape2);

		if (m_Detector.Run(gjkShape1, gjkShape2) <= 0.0f)
		{
			contact = CalculateCantact(gjkShape1, gjkShape2);
			return true;
		}
		return false;
	}

	void InitTransformMatrix(CompoundShape& shape1, CompoundShape& shape2)
	{
		auto& t1 = shape1.GetTransform();
		auto& t2 = shape2.GetTransform();

		m_ToShape1sSpace = t1.GetWorldToLocalMatrix() * t2.GetLocalToWorldMatrix();
	}

public:
	void FindContacts(CompoundShape& shape1, CompoundShape& shape2, std::vector<ContactManifold>& contacts1To2)
	{
		InitTransformMatrix(shape1, shape2);

		ContactManifold contact;
		for (auto subShape1 : shape1.GetSubShapes())
		{
			for (auto subShape2 : shape2.GetSubShapes())
			{
				if (FindContact(*subShape1, *subShape2, contact))
					contacts1To2.emplace_back(contact);
			}
		}
	}

private:
	std::vector<Vector3> m_TransformedPoints;
	GjkCollisionDetection m_Detector;
	Matrix4 m_ToShape1sSpace;
};