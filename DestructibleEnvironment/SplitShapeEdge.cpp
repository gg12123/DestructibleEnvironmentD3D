#include "pch.h"
#include "SplitShapeEdge.h"
#include "ShapeEdge.h"
#include "ShapePoint.h"
#include "FaceOrderingComparator.h"

ShapePoint & SplitShapeEdge::GetP0() const
{
	return m_Edge->GetP0();
}

ShapePoint & SplitShapeEdge::GetP1() const
{
	return m_Edge->GetP1();
}

static bool Element0IsBeforeElement1(const Vector3& rayOrigin, const Vector3& rayDir, const CutPathElement& element0, const CutPathElement& element1)
{
	auto dist0 = (rayOrigin - element0.GetIntPoint()).MagnitudeSqr();
	auto dist1 = (rayOrigin - element1.GetIntPoint()).MagnitudeSqr();

	// No need running an expensive algorithm if the situation is well defined.
	if (MathU::Abs(dist0 - dist1) > 0.001f)
		return dist0 < dist1;

	FaceOrderingComparator comparer;
	return comparer.FaceAIsBeforeFaceB(element0.GetPiercedFace(), element1.GetPiercedFace(), rayDir);
}

static ShapePoint& RemoveClosest(std::vector<CutPathElement>& elements, const Vector3& rayOrigin, const Vector3& rayDir)
{
	auto itOfClosest = elements.begin();

	for (auto it = elements.begin() + 1; it != elements.end(); it++)
	{
		if (Element0IsBeforeElement1(rayOrigin, rayDir, *it, *itOfClosest))
			itOfClosest = it;
	}

	auto& closestPoint = itOfClosest->GetPoint();
	elements.erase(itOfClosest);
	return closestPoint;
}

void SplitShapeEdge::OnAllElementsAdded()
{
	auto& p0 = GetP0();

	auto origin = p0.GetPoint();
	auto dir = m_Edge->GetDirFromP0ToP1();

	// n^2 sort but there will usually only be 1 or 2 elements to sort.

	m_PointsSortedFromP0.emplace_back(&p0);

	while (m_Elements.size() > 0U)
		m_PointsSortedFromP0.emplace_back(&RemoveClosest(m_Elements, origin, dir));

	m_PointsSortedFromP0.emplace_back(&GetP1());
}