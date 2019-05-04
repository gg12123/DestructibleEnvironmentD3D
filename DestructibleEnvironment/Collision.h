#pragma once
#include <memory>
#include "CollisionDetector.h"
#include "CollisionData.h"
#include "Rigidbody.h"
#include "StaticBody.h"
#include "HGrid.h"
#include "ContactManifold.h"
#include "ContactContexts.h"

class Collision
{
private:
	void AddBodyToPartition(const Rigidbody& body)
	{
		// This would be the time to clear any collision related data
		// associated with this body - such as contacts by body.

		for (auto s : body.GetSubShapes())
			m_DynamicsPartition.AddObject(*s);
	}

public:
	Collision() : m_DynamicsPartition(2.0f, 0.2f)
	{
	}

	void RunNarrowPhaseCheckForCollision(const Shape& shapeA, const Shape& shapeB)
	{
		if (&shapeA.GetOwner() == &shapeB.GetOwner())
			return;

		auto shape1 = &shapeA;
		auto shape2 = &shapeB;

		auto numPoints1 = shapeA.GetCachedPoints().size();
		auto numPoints2 = shapeB.GetCachedPoints().size();

		// Ensure shape1 is the shape with more points. If both shapes have equal
		// point count, ensure shap1 is the shape with greater ID.
		// So if this is called multiple times for the same two shapes (across different frames),
		// shape1 and shape2 will be assigned the same each time.
		if ((numPoints1 < numPoints2) ||
			(numPoints1 == numPoints2 && shape1->GetShapeId() < shape2->GetShapeId()))
		{
			shape1 = &shapeB;
			shape2 = &shapeA;
		}

		auto& c = m_Contexts.InitContext(*shape1, *shape2);
		auto inContact = false;

		ContactPlane contactPlane;
		m_ContactPoints.clear();
		if (m_Detector.FindContact(*shape1, *shape2, c, contactPlane, m_ContactPoints))
		{
			m_ManifoldInit.InitManifold(*shape1, *shape2, m_ContactPoints, contactPlane, c);
			inContact = m_ContactPoints.size() > 0u;
		}

		c.SetInContact(inContact);
	}

	void FindContacts(const std::vector<std::unique_ptr<StaticBody>>& staticBodies,
		const std::vector<std::unique_ptr<Rigidbody>>& dynamicBodies)
	{
		m_Detector.PrepareToFindContacts();
		m_Contexts.OnContactFindingStart();

		for (auto& db : dynamicBodies)
			AddBodyToPartition(*db);

		// Handle collision between dynamic objects
		m_DynamicsPartition.Run(*this);

		// Handle collisions between static objects
		// TODO - Partition the statics.
		auto dynamicCount = dynamicBodies.size();
		auto staticCount = staticBodies.size();

		for (auto i = 0U; i < dynamicCount; i++)
		{
			auto& dynamBody = *dynamicBodies[i];
			for (auto j = 0U; j < staticCount; j++)
			{
				auto& staticBody = *staticBodies[j];
				for (auto shapeDynam : dynamBody.GetSubShapes())
				{
					for (auto shapeStat : staticBody.GetSubShapes())
						RunNarrowPhaseCheckForCollision(*shapeDynam, *shapeStat);
				}
			}
		}
	}

	auto& GetContactConstraints()
	{
		return m_ManifoldInit.GetNormalContactConstraints();
	}

	auto& GetManifolds()
	{
		return m_ManifoldInit.GetManifolds();
	}

	void StoreAccImpulses()
	{
		m_ManifoldInit.StoreAccumulatedImpulsesForNextTick(m_Contexts);
	}

private:
	CollisionDetector m_Detector;
	HGrid<Shape, Collision> m_DynamicsPartition;
	ManifoldInitializer m_ManifoldInit;
	SimdStdVector<Vector3> m_ContactPoints;
	ContactContexts m_Contexts;
};
