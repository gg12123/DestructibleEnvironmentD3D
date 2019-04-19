#pragma once
#include <vector>
#include "CollisionData.h"
#include "CompoundShape.h"
#include "Shape.h"
#include "SatOptimisedCollisionDetection.h"
#include "Debug.h"
#include "CwSeperationFinder.h"
#include "DynamicTriangleArray.h"

class CollisionDetector
{
private:
	struct CollisionContext
	{
		Vector3 Vector = Vector3(1.0f, 0.0f, 0.0f);
		bool SeperatedOnPrevTick = false;
	};

	void TransformToShape1sSpace(const Shape& shape2)
	{
		m_TransformedPoints.clear();
		m_TransformedNormals.clear();

		for (auto& p : shape2.GetCachedPoints())
			m_TransformedPoints.emplace_back(m_ToShape1sSpace * p);

		for (auto& n : shape2.GetCachedFaceNormals())
			m_TransformedNormals.emplace_back(m_DirToShape1sSpace.RotateV(n));
	}

	void InitTransformMatrices(const CompoundShape& shape1, const CompoundShape& shape2)
	{
		auto& t1 = shape1.GetTransform();
		auto& t2 = shape2.GetTransform();

		m_ToShape1sSpace = t1.GetWorldToLocalMatrix() * t2.GetLocalToWorldMatrix();
		m_DirToShape1sSpace = t1.GetWorldToLocalRotation() * t2.GetLocalToWorldRotation();
		m_ToShape2sSpace = t2.GetWorldToLocalMatrix() * t1.GetLocalToWorldMatrix();
		m_DirToShape2sSpace = m_DirToShape1sSpace.Conj();
		m_ActiveTransform = &t1;
	}

	CollisionContext& GetCollisionContext(const Shape& shape1, const Shape& shape2)
	{
		return m_CollisionContexts.Get(shape1.GetShapeId(), shape2.GetShapeId());
	}

public:
	bool FindContact(const Shape& shape1In, const Shape& shape2In, ContactPlane& contact, std::vector<Vector3>& contactPoints)
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

		InitTransformMatrices(shape1->GetOwner(), shape2->GetOwner());

		auto& context = GetCollisionContext(*shape1, *shape2);

		if (context.SeperatedOnPrevTick)
		{
			auto cwShapeA = CwInputShapeA(shape1->GetCachedPoints());
			auto cwShapeB = CwInputShapeB(shape2->GetCachedPoints(), m_DirToShape2sSpace, m_ToShape1sSpace);

			// The sep finder will write the seperation vector back into the context vector
			if (m_CwSepFinder.FindSeperation(cwShapeA, cwShapeB, context.Vector))
				return false;
		}

		// Either CW failed to find seperation or the shapes were in contact
		// on prev tick so use SAT
		TransformToShape1sSpace(*shape2);

		auto satShape1 = SatInputShape(shape1->GetEdgeIndexesPoints(), shape1->GetEdgeIndexsFaces(),
			shape1->GetCachedPoints(), shape1->GetCachedFaceNormals(), shape1->GetFaceP0Indexes(), shape1->GetCentre());

		auto satShape2 = SatInputShape(shape2->GetEdgeIndexesPoints(), shape2->GetEdgeIndexsFaces(),
			m_TransformedPoints, m_TransformedNormals, shape2->GetFaceP0Indexes(), m_ToShape1sSpace * shape2->GetCentre());

		if (m_SatDetector.DetectCollision(satShape1, satShape2))
		{
			auto localContact = m_SatDetector.GetContactPlane();

			contact = ContactPlane(localContact.GetContactMin(), localContact.GetContactMax(),
				m_ActiveTransform->ToWorldDirection(localContact.GetNormal()));

			for (auto& p : m_ContactPointFinder.FindContactPoints(satShape1, satShape2, localContact))
				contactPoints.emplace_back(m_ActiveTransform->ToWorldPosition(p));

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
	Matrix4 m_ToShape2sSpace;
	Quaternion m_DirToShape1sSpace;
	Quaternion m_DirToShape2sSpace;
	const Transform* m_ActiveTransform;
	SatOptimisedCollisionDetection m_SatDetector;
	ContactPointFinder m_ContactPointFinder;
	CwSeperationFinder m_CwSepFinder;
	DynamicTriangleArray<CollisionContext> m_CollisionContexts;
};