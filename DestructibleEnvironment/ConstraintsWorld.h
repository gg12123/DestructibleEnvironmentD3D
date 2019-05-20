#pragma once
#include <memory>
#include "CollisionDetector.h"
#include "CollisionData.h"
#include "Rigidbody.h"
#include "StaticBody.h"
#include "HGrid.h"
#include "ContactManifold.h"
#include "ContactContexts.h"
#include "Islands.h"
#include "Joints.h"
#include "PhysicsTypes.h"

class ConstraintsWorld
{
private:
	void AddToPartition(const CompoundShape& obj)
	{
		for (auto s : obj.GetSubShapes())
			m_DynamicsPartition.AddObject(*s);
	}

	bool NarrowCollisionCheckRbPhysical(const Shape& shapeA, const Shape& shapeB, ContactContext& c)
	{
		auto inContact = false;

		auto& body1 = shapeA.GetOwner();
		auto& body2 = shapeB.GetOwner();

		if (!body1.IsAwake() && !body2.IsAwake())
		{
			if (c.TestedOnPrevTick)
				m_Detector.SaveSimplexForNextTick(shapeA, shapeB, c);

			if (c.InContactOnPrevTick())
			{
				m_ManifoldInit.InitManifoldUsingPrevContactPoints(shapeA, shapeB, c);
				inContact = true;
			}
		}
		else
		{
			ContactPlane contactPlane;
			m_ContactPoints.clear();
			if (m_Detector.FindContact(shapeA, shapeB, c, contactPlane, m_ContactPoints))
			{
				m_ManifoldInit.InitManifold(shapeA, shapeB, m_ContactPoints, contactPlane, c);
				inContact = m_ContactPoints.size() > 0u;
			}
		}

		if (inContact)
			m_Islands.RegisterManifold(shapeA.GetOwner(), shapeB.GetOwner());

		return inContact;
	}

	bool NarrowCollisionCheckContainsTrigger(const Shape& shapeA, const Shape& shapeB, ContactContext& c)
	{
		if (m_Detector.FindContact(shapeA, shapeB, c))
		{
			m_Islands.RegisterNonPhysicsContact(shapeA.GetOwner(), shapeB.GetOwner());
			return true;
		}
		return false;
	}

	bool NarrowCollisionCheckCharCtlCharCtl(const Shape& shapeA, const Shape& shapeB, ContactContext& c)
	{
		ContactPlane contactPlane;
		if (m_Detector.FindContact(shapeA, shapeB, c, contactPlane))
		{
			m_Islands.RegisterNonPhysicsContact(shapeA.GetOwner(), shapeB.GetOwner());
			// use the contact plane to constrain the char controllers somehow
			return true;
		}
		return false;
	}

	bool NarrowCollisionCheckCharCtlStatic(const Shape& shapeACharCtl, const Shape& shapeBStatic, ContactContext& c)
	{
		ContactPlane contactPlane;
		if (m_Detector.FindContact(shapeACharCtl, shapeBStatic, c, contactPlane))
		{
			// Project the char ctl out of the static
			return true;
		}
		return false;
	}

	PhysicsObjectComparisonType GetComparisionType(const CompoundShape& a, const CompoundShape& b)
	{

	}

	template<class Tdynamic>
	void TestAgainstStatics(const std::vector<std::unique_ptr<StaticBody>>& staticBodies,
		const std::vector<std::unique_ptr<Tdynamic>>& dynamicBodies)
	{
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

public:
	ConstraintsWorld() : m_DynamicsPartition(2.0f, 0.2f)
	{
	}

	void RunNarrowPhaseCheckForCollision(const Shape& shapeA, const Shape& shapeB)
	{
		auto& shapeAOwner = shapeA.GetOwner();
		auto& shapeBOwner = shapeB.GetOwner();

		if (&shapeAOwner == &shapeBOwner)
			return;

		auto shape1 = &shapeA;
		auto shape2 = &shapeB;

		if (shape1->GetShapeId() < shape2->GetShapeId())
		{
			shape1 = &shapeB;
			shape2 = &shapeA;
		}

		auto& c = m_Contexts.InitContext(*shape1, *shape2);
		auto inContact = false;

		switch (GetComparisionType(shapeAOwner, shapeBOwner))
		{
		case PhysicsObjectComparisonType::CharControllerStatic:
		{
			inContact = NarrowCollisionCheckCharCtlStatic(*shape1, *shape2, c);
			break;
		}
		case PhysicsObjectComparisonType::StaticCharController:
		{
			inContact = NarrowCollisionCheckCharCtlStatic(*shape2, *shape1, c);
			break;
		}
		default:
			break;
		}

		c.SetInContact(inContact);
	}

	void FindConstraints(const std::vector<std::unique_ptr<StaticBody>>& staticBodies,
		const std::vector<std::unique_ptr<Rigidbody>>& dynamicBodies)
	{
		m_Detector.PrepareToFindContacts();
		m_Contexts.OnContactFindingStart();
		m_Islands.Resize(dynamicBodies.size() + staticBodies.size());

		for (auto& db : dynamicBodies)
		{
			m_Islands.ClearCollisonData(*db);
			AddToPartition(*db);
		}

		// Add triggers and char controllers

		for (auto& sb : staticBodies)
		{
			m_Islands.ClearCollisonData(*sb);
		}

		// First project the char controllers out of static geometry

		// Handle collision between dynamic objects
		m_DynamicsPartition.Run(*this);

		// Handle collisions against static objects
		TestAgainstStatics(staticBodies, dynamicBodies);
		// Also the triggers

		// Register joints with the islands
		auto& joints = m_Joints.GetJoints();
		for (auto it = joints.Begin(); it != joints.End(); it++)
		{
			auto& j = *it;
			j.UpdateWorldTransform();
			m_Islands.RegisterJoint(j.GetAnchorObj(), j.GetOtherObj());
		}

		m_Islands.ReCalculateIslands(dynamicBodies);
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

	const auto GetIslands() const
	{
		return m_Islands.GetIslands();
	}

	auto& GetJoints()
	{
		return m_Joints.GetJoints().GetValues();
	}

	void AddJoint(const Joint& j)
	{
		m_Joints.AddJoint(j);
	}

private:
	CollisionDetector m_Detector;
	HGrid<Shape, ConstraintsWorld> m_DynamicsPartition;
	ManifoldInitializer m_ManifoldInit;
	SimdStdVector<Vector3> m_ContactPoints;
	ContactContexts m_Contexts;
	Islands m_Islands;
	Joints m_Joints;
};
