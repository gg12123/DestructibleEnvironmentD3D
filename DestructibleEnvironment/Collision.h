#pragma once
#include <memory>
#include "CollisionDetector.h"
#include "CollisionData.h"
#include "Rigidbody.h"
#include "StaticBody.h"
#include "HGrid.h"
#include "ContactPoints.h"

class Collision
{
public:
	Collision() : m_DynamicsPartition(2.0f, 0.2f)
	{
	}

	void RunNarrowPhaseCheckForCollision(const Shape& shapeA, const Shape& shapeB)
	{
		ContactPlane contact;
		if (m_Detector.FindContact(shapeA, shapeB, contact))
		{
			m_ContactPointFinder.Find(m_ContactConstraints, m_Manifolds, shapeA, shapeB, contact);
		}
	}

	void AddObject(Rigidbody& body)
	{
		for (auto s : body.GetSubShapes())
			m_DynamicsPartition.AddObject(*s);
	}

	void FindContacts(const std::vector<std::unique_ptr<StaticBody>>& staticBodies,
		const std::vector<std::unique_ptr<Rigidbody>>& dynamicBodies)
	{
		m_ContactConstraints.clear();
		m_Manifolds.clear();

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
		return m_ContactConstraints;
	}

	auto& GetManifolds()
	{
		return m_Manifolds;
	}

private:
	CollisionDetector m_Detector;
	HGrid<Shape, Collision> m_DynamicsPartition;
	ContactPointFinder m_ContactPointFinder;
	std::vector<NormalContactConstraint> m_ContactConstraints;
	std::vector<ContactManifold> m_Manifolds;
};
