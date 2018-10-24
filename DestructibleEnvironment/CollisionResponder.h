#pragma once
#include <memory>
#include "Vector3.h"
#include "PotentialCollision.h"

class PhysicsObject;

class CollisionResponder
{
public:
	void CalculateResponse(const std::vector<PotentialCollision>& potColls, PhysicsObject& body1, PhysicsObject& body2);

private:
	bool CalculateCollisionPointAndNormal(const std::vector<PotentialCollision>& potColls, Vector3& normal1To2, Vector3& point);
	float CalculateRequiredSeperation(const std::vector<PotentialCollision>& potColls, const Vector3& finalNormal1To2);
	Vector3 CalculateAverageNormal(const std::vector<PotentialCollision>& potColls);

	PhysicsObject * m_Body1 = nullptr;
	PhysicsObject * m_Body2 = nullptr;
};
