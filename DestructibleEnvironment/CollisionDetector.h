#pragma once
#include <vector>
#include "CollisionData.h"
#include "CompoundShape.h"
#include "Shape.h"
#include "SatOptimisedCollisionDetection.h"
#include "Debug.h"

class CollisionDetector
{
private:
	struct CollisionContext
	{
		Vector3 Vector = Vector3(1.0f, 0.0f, 0.0f);
		bool SeperatedOnPrevTick = false;
	};

	SatInputShape TransformToShape1sSpace(const Shape& shape2)
	{
		m_TransformedPoints.clear();

		for (auto& p : shape2.GetCachedPoints())
			m_TransformedPoints.emplace_back(m_ToShape1sSpace * p);

		
	}

	void InitTransformMatrixForShape1Space(const CompoundShape& shape1, const CompoundShape& shape2)
	{
		auto& t1 = shape1.GetTransform();
		auto& t2 = shape2.GetTransform();

		m_ToShape1sSpace = t1.GetWorldToLocalMatrix() * t2.GetLocalToWorldMatrix();
		m_ActiveTransform = &t1;
	}

	CollisionContext& GetCollisionContext(const Shape& shape1, const Shape& shape2)
	{

	}

public:
	bool FindContact(const Shape& shape1In, const Shape& shape2In, ContactPlane& contact)
	{
		auto shape1 = &shape1In;
		auto shape2 = &shape2In;

		auto numPoints1 = shape1In.GetCachedPoints().size();
		auto numPoints2 = shape2In.GetCachedPoints().size();

		// Ensure shape1 is the shape with more points. If both shapes have equal
		// point count, ensure shap1 is the shape with greater ID.
		// So if this is called multiple times for the same two shapes, shape1 and
		// shape2 will be assigned the same each time.
		if ((numPoints1 < numPoints2) ||
			(numPoints1 == numPoints2 && shape1->GetShapeId() < shape2->GetShapeId()))
		{
			shape1 = &shape2In;
			shape2 = &shape1In;
		}

		auto context = GetCollisionContext(*shape1, *shape2);

		if (context.SeperatedOnPrevTick)
		{
			// Use CW first
			// If seperation detected, update the vector in the context and return false
		}

		// Either CW failed to find seperation or the shapes were in contact
		// on prev tick so use SAT

		InitTransformMatrixForShape1Space(shape1->GetOwner(), shape2->GetOwner());

		auto satShape1 = SatInputShape();
		auto satShape2 = TransformToShape1sSpace(*shape2);

		if (m_SatDetector.DetectCollision(satShape1, satShape2))
		{
			auto localContact = m_SatDetector.GetContactPlane();

			contact = ContactPlane(m_ActiveTransform->ToWorldPosition(localContact.GetPoint()),
				m_ActiveTransform->ToWorldDirection(localContact.GetNormal()),
				localContact.GetPeneration());

			context.SeperatedOnPrevTick = false;
			context.Vector = localContact.GetNormal();

			return true;
		}

		// No collision so use the seperation vector found in SAT to init CW
		// for next tick.
		context.SeperatedOnPrevTick = true;
		context.Vector = m_SatDetector.GetSeperationDirection();
		return false;
	}

private:
	std::vector<Vector3> m_TransformedPoints;
	std::vector<Vector3> m_TransformedNormals;
	Matrix4 m_ToShape1sSpace;
	const Transform* m_ActiveTransform;
	SatOptimisedCollisionDetection m_SatDetector;
};