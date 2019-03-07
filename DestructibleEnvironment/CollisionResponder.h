#pragma once
#include <memory>
#include <vector>
#include "Vector3.h"
#include "EdgeFaceIntersection.h"
#include "CollisionData.h"

class PhysicsObject;

class CollisionResponder
{
public:
	void CalculateResponse(const ContactPlane& contact1To2, PhysicsObject& body1, PhysicsObject& body2);

private:
	bool CalculateCollisionPoint(const std::vector<EdgeFaceIntersection>& inters, const Vector3& normal1To2, Vector3& point);

	PhysicsObject * m_Body1 = nullptr;
	PhysicsObject * m_Body2 = nullptr;
};
