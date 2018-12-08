#include "pch.h"
#include "CollisionDetector.h"
#include "Shape.h"
#include "MathU.h"
#include "Transform.h"
#include "ShapeEdge.h"

PotentialCollision CollisionDetector::ToPotentialCollision(const EdgeFaceIntersection& inter, const Shape& shape1)
{
	auto& edge = inter.GetEdge();
	auto& shapeEdges = edge.GetFace1().GetShape();

	auto& transformedPoints = &shapeEdges == &shape1 ?
		m_IntersectionFinder.GetShape1sTransformedPoints() :
		m_IntersectionFinder.GetShape2sTransformedPoints();

	auto& edgeP0 = transformedPoints[edge.GetP0().GetIndexInShape()];
	auto& edgeP1 = transformedPoints[edge.GetP1().GetIndexInShape()];

	return PotentialCollision(edgeP0, edgeP1, inter.GetIntPoint(), inter.GetFace());
}

bool CollisionDetector::FindCollision(Shape& shape1, Shape& shape2, std::vector<PotentialCollision>& detectedColls)
{
	detectedColls.clear();

	m_FoundIntersections.clear();
	m_IntersectionFinder.FindEdgeFaceIntersections(shape1, shape2, m_FoundIntersections);

	if (m_FoundIntersections.size() > 0U)
	{
		for (auto it = m_FoundIntersections.begin(); it != m_FoundIntersections.end(); it++)
			detectedColls.emplace_back(ToPotentialCollision(*it, shape1));

		return true;
	}
	return false;
}