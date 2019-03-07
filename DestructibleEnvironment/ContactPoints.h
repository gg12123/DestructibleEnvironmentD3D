#pragma once
#include "PhysicsObject.h"
#include "Shape.h"
#include "CollisionData.h"

class ContactPoint
{
public:
	ContactPoint(PhysicsObject& b1, PhysicsObject& b2, const Vector3& point, const Vector3& normal12) :
		Body1(&b1), Body2(&b2), Point(point), Normal1To2(normal12)
	{
	}

	PhysicsObject * Body1;
	PhysicsObject * Body2;
	Vector3 Point;
	Vector3 Normal1To2;
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