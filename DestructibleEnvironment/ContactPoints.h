#pragma once
#include "PhysicsObject.h"
#include "Shape.h"
#include "CollisionData.h"
#include "CollisionResponder.h"

class ContactPoint
{
public:
	ContactPoint(PhysicsObject& b1, PhysicsObject& b2, const Vector3& point, const Vector3& normal12) :
		m_Body1(&b1), m_Body2(&b2), m_Point(point), m_Normal1To2(normal12), m_AccumulatedImpulse(0.0f)
	{
	}

	void ApplyImpulse(float J) const
	{
		m_Body1->ApplyImpulse(Impulse(-J * m_Normal1To2, m_Point, J));
		m_Body2->ApplyImpulse(Impulse(J * m_Normal1To2, m_Point, J));
	}

	float GetAccumulatedImpulse() const
	{
		return m_AccumulatedImpulse;
	}

	void SetAccumulatedImpulse(float val)
	{
		m_AccumulatedImpulse = val;
	}

	float CalculateImpulse() const
	{
		return CollisionResponder::CalculateImpulse(ContactPlane(m_Point, m_Normal1To2), *m_Body1, *m_Body2);
	}

private:
	PhysicsObject * m_Body1;
	PhysicsObject * m_Body2;
	Vector3 m_Point;
	Vector3 m_Normal1To2;
	float m_AccumulatedImpulse;
};

class ContactPointFinder
{
public:
	void Find(std::vector<ContactPoint>& points, const Shape& shape1, const Shape& shape2, const ContactPlane& contactPlane)
	{
		points.emplace_back(ContactPoint(*shape1.GetOwner().ToPhysicsObject(),
			*shape2.GetOwner().ToPhysicsObject(),
			contactPlane.GetPoint(),
			contactPlane.GetNormal()));
	}
};