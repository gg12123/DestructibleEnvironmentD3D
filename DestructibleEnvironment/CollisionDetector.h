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
	// TODO - the shapes should be const
	bool FindCollision(Shape& shape1, Shape& shape2, std::vector<PotentialCollision>& detectedColls);

private:
	PotentialCollision ToPotentialCollision(const EdgeFaceIntersection& inter, const Shape& shape1);

	IntersectionFinder m_IntersectionFinder;
	std::vector<EdgeFaceIntersection> m_FoundIntersections;
};