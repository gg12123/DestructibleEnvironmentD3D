#include "pch.h"
#include "CollisionDetector.h"
#include "Shape.h"
#include "MathU.h"
#include "Transform.h"

PotentialCollision CollisionDetector::ToPotentialCollision(const FaceEdgeIntersection<Vector3>& inter)
{
	auto& faceEdges = *inter.OtherFace;

	auto& transformedPoints = &faceEdges.GetShape() == m_Shape1 ?
		m_IntersectionFinder.GetShape1sTransformedPoints() :
		m_IntersectionFinder.GetShape2sTransformedPoints();

	auto& indexes = faceEdges.GetSharedPoints();

	auto i = inter.PiercingEdge;
	auto& edgeP0 = transformedPoints[indexes[i]];
	auto& edgeP1 = transformedPoints[indexes[(i + 1) % indexes.size()]];

	return PotentialCollision(edgeP0, edgeP1, inter.Position, *inter.PiercedFace);
}

bool CollisionDetector::FindCollision(Shape& shape1, Shape& shape2, std::vector<PotentialCollision>& detectedColls)
{
	m_FoundIntersections.clear();
	m_IntersectionFinder.FindFaceEdgeIntersections(shape1, shape2, m_FoundIntersections);

	if (m_FoundIntersections.size() > 0U)
	{
		for (auto it = m_FoundIntersections.begin(); it != m_FoundIntersections.end(); it++)
			detectedColls.emplace_back(ToPotentialCollision(*it));

		return true;
	}
	return false;
}