#pragma once
#include <memory>
#include "Vector3.h"
#include "FaceCollision.h"
#include "EdgeFaceIntersection.h"

class PhysicsObject;

class CollisionResponder
{
public:
	void CalculateResponse(std::vector<FaceCollision>& faceColls,
		const std::vector<EdgeFaceIntersection>& inters,
		PhysicsObject& body1, PhysicsObject& body2);

private:
	bool CalculateSeperation1To2(std::vector<FaceCollision>& faceColls, Vector3& sep1To2) const;
	bool CalculateCollisionPoint(const std::vector<EdgeFaceIntersection>& inters, const Vector3& normal1To2, Vector3& point);

	PhysicsObject * m_Body1 = nullptr;
	PhysicsObject * m_Body2 = nullptr;
};
