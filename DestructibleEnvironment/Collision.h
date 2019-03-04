#pragma once
#include <memory>
#include "CollisionDetector.h"
#include "CollisionData.h"
#include "CollisionResponder.h"
#include "Rigidbody.h"
#include "StaticBody.h"
#include "HGrid.h"

class Collision
{
public:
	Collision() : m_DynamicsPartition(2.0f, 0.2f)
	{
	}

	void RunNarrowPhaseCheckForCollision(Shape& shapeA, Shape& shapeB)
	{
		ContactManifold contact;
		if (m_Detector.FindContact(shapeA, shapeB, contact))
			m_Responder.CalculateResponse(contact, *shapeA.GetOwner().ToPhysicsObject(), *shapeB.GetOwner().ToPhysicsObject());
	}

	void AddObject(Rigidbody& body)
	{
		for (auto s : body.GetSubShapes())
			m_DynamicsPartition.AddObject(*s);
	}

	void DetectAndRespond(const std::vector<std::unique_ptr<StaticBody>>& staticBodies,
		const std::vector<std::unique_ptr<Rigidbody>>& dynamicBodies)
	{
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

private:
	CollisionDetector m_Detector;
	CollisionResponder m_Responder;
	HGrid<Shape, Collision> m_DynamicsPartition;
};
