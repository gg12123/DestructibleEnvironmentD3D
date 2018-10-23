#pragma once
#include <vector>
#include <memory>
#include "Vector3.h"
#include "PotentialCollision.h"
#include "IntersectionFinder.h"

class Shape;
class Transform;

class CollisionDetector
{
public:
	bool FindCollision(Shape& shape1, Shape& shape2, std::vector<PotentialCollision>& detectedColls);

private:
	PotentialCollision ToPotentialCollision(const FaceEdgeIntersection<Vector3>& inter);

	IntersectionFinder m_IntersectionFinder;
	std::vector<FaceEdgeIntersection<Vector3>> m_FoundIntersections;

	Shape* m_Shape1;
	Shape* m_Shape2;
};